/*
wifi 遥控、RC遥控处理

code by:ZhuHenCai
*/



//UDP 遥控
void setupWiFiUDP() {
    Serial.println("设置 WiFi AP 和 UDP...");
    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP 地址: ");
    Serial.println(myIP);
    udp.begin(localUdpPort);
    Serial.print("UDP 监听端口: ");
    Serial.println(localUdpPort);
}

// --- 处理 UDP 指令 
void processUDP() {
    int packetSize = udp.parsePacket(); 
    if (packetSize) {
        last_udp_command_time = millis(); 

        // 数据到缓冲区
        int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
        if (len > 0) {
            incomingPacket[len] = 0; 
        //    Serial.print("收到 UDP: "); 
         //   Serial.println(incomingPacket); // 调试原始字符串

            // --- 使用 ArduinoJson 解析 ---
            StaticJsonDocument<UDP_RX_JSON_BUFFER_SIZE> docIn; 
            DeserializationError error = deserializeJson(docIn, incomingPacket); 

            if (error) { 
                Serial.print(F("deserializeJson() failed: "));
                Serial.println(error.f_str());
                // 回复错误信息给 APP，或忽略
                // udp.beginPacket(udp.remoteIP(), udp.remotePort());
                // udp.print("{\"status\":\"JSON Error\"}");
                // udp.endPacket();
                return; // 出错则不继续处理
            }

            // --- 解析成功，提取指令和参数 ---
            const char* cmd = docIn["cmd"]; 

            if (cmd == nullptr) { // 如果没有 "cmd" 字段
                Serial.println("错误: UDP 指令缺少 'cmd' 字段。");
                return;
            }

            String command = String(cmd); 
            bool commandProcessed = false;
            bool isDirectionalCommand = false;

            // --- 1. 通用指令处理 ---
            if (command.equalsIgnoreCase("arm")) {
                if (currentFlightMode == FLIGHT_MODE_DISARMED) {
                    if (abs(roll) < 30.0f && abs(pitch) < 30.0f) {
                        Serial.println("解锁电机...");
                        delay(1); 
                        //将当前角度作为初始化
                        initializeAttitudeForArming();
                        currentFlightMode = FLIGHT_MODE_ARMED; 
                        rc_throttle_input = 0;
                        resetPIDIntegrals();
                        rc_raw_roll_input = 0; 
                        rc_raw_pitch_input = 0; 
                        rc_raw_yaw_input = 0;
                    } else { 
                        Serial.println("解锁失败：飞机姿态不水平！"); 
                    }
                } else {
                    Serial.println("指令忽略：飞机已解锁。"); 
                }
                commandProcessed = true;
            }
            else if (command.equalsIgnoreCase("disarm")) {
                Serial.println("锁定电机...");
                currentFlightMode = FLIGHT_MODE_DISARMED; 
                commandProcessed = true; 
            }
            else if (command.equalsIgnoreCase("turtle")) {
                if (currentFlightMode == FLIGHT_MODE_DISARMED && isFlippedOver) {
                    Serial.println("进入 Turtle 模式...");
                    currentFlightMode = FLIGHT_MODE_TURTLE;
                    writeMotorsPWM(MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM);
                }
                else if (currentFlightMode == FLIGHT_MODE_TURTLE) {
                    Serial.println("退出 Turtle 模式...");
                    currentFlightMode = FLIGHT_MODE_DISARMED;
                    writeMotorsPWM(MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM);
                } 
                else if (currentFlightMode != FLIGHT_MODE_DISARMED) {
                    Serial.println("指令忽略：必须先锁定才能进入Turtle模式。");
                }
                else {
                    Serial.println("指令忽略：飞机未倒地，无法进入Turtle模式。");
                } 
                commandProcessed = true; 
            }
            else if (command.equalsIgnoreCase("status")) {
                commandProcessed = true; // 标记已处理，会触发下面的状态发送
            }

            //// 标记已处理，开始下面的状态发送，APP那里油门跟 方指令是分开  delay(5);发送的
            // --- 2. 正常飞行模式 (ARMED) 指令处理 ---
            else if (currentFlightMode == FLIGHT_MODE_ARMED) {
                if (command.equalsIgnoreCase("th")) { // 油门指令
                    // 检查 "val" 字段是否存在且为数字
                    if (docIn["val"].is<int>() || docIn["val"].is<float>()) {
                        rc_throttle_input = constrainf(docIn["val"].as<float>(), 0.0f, 950.0f); // 获取油门值并约束,满值是1000，这里先测试8成油门，APP传过来的是0-1000
                        commandProcessed = true;
                        isDirectionalCommand = false;
                    } else { Serial.println("错误: 油门指令缺少 'val' 或类型错误。"); }
                } 
                else if (command.equalsIgnoreCase("a")) { // 角度/速率指令
                    // 检查字段是否存在且为数字
                    if ((docIn["r"].is<int>() || docIn["r"].is<float>()) &&
                        (docIn["p"].is<int>() || docIn["p"].is<float>()) &&
                        (docIn["y"].is<int>() || docIn["y"].is<float>()))
                    {
                        rc_raw_roll_input = constrainf(docIn["r"].as<float>(), -1.0f, 1.0f);
                        rc_raw_pitch_input = constrainf(docIn["p"].as<float>(), -1.0f, 1.0f);
                        rc_raw_yaw_input = constrainf(docIn["y"].as<float>(), -1.0f, 1.0f);
                       /*
                        Serial.print("收到 rc_raw_roll_input: "); 
                        Serial.println(rc_raw_roll_input); 

                        Serial.print("收到 rc_raw_pitch_input: "); 
                        Serial.println(rc_raw_pitch_input); 

                        Serial.print("收到 rc_raw_yaw_input: "); 
                        Serial.println(rc_raw_yaw_input); 
                      */

                        commandProcessed = true;
                        isDirectionalCommand = true;
                    } else {
                        Serial.println("错误: 角度指令缺少 r/p/y 或类型错误。");
                    }
                } 
                else if (command.startsWith("flip_")) { 
                    if (abs(roll) < 15.0f && abs(pitch) < 15.0f) { 
                        FlipDirection requestedFlip = FLIP_NONE;
                        if (command.equalsIgnoreCase("flip_f")) requestedFlip = FLIP_FRONT;
                        else if (command.equalsIgnoreCase("flip_b")) requestedFlip = FLIP_BACK;
                        else if (command.equalsIgnoreCase("flip_l")) requestedFlip = FLIP_LEFT;
                        else if (command.equalsIgnoreCase("flip_r")) requestedFlip = FLIP_RIGHT;
                        if (requestedFlip != FLIP_NONE) {
                            Serial.print("开始翻转: ");
                            Serial.println(command); 
                            currentFlightMode = FLIGHT_MODE_FLIPPING;
                            currentFlipDirection = requestedFlip; 
                            flipStartTime = millis();
                            resetPIDIntegrals(); 
                        }
                    } else {
                        Serial.println("无法翻转: 姿态不稳或高度不足。"); 
                    }
                    commandProcessed = true; 
                }

                // 如果在 ARMES 模式下，没有收到有效的方向或油门指令 但收到了其他有效指令 alt on/off)
                if (!isDirectionalCommand && commandProcessed && !command.equalsIgnoreCase("th")) {
                    rc_raw_roll_input = 0.0f;
                    rc_raw_pitch_input = 0.0f;
                    rc_raw_yaw_input = 0.0f; 
                }
            }

            // --- 3. Turtle 模式指令处理  地上翻机之后，在地上尝试翻起来 ---
            else if (currentFlightMode == FLIGHT_MODE_TURTLE) {
                // Turtle 模式的 push 指令是简单字符串，不需要 JSON 解析特定参数
                writeMotorsPWM(MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM); // 先停止
                turtleMotorPulseTime = 0;
                int m1=0, m2=0, m3=0, m4=0;
                bool pulse = false;
                if (command.equalsIgnoreCase("push_f")) { 
                    m3 = m4 = TURTLE_MOTOR_POWER_PWM;
                    pulse = true; 
                }
                else if (command.equalsIgnoreCase("push_b")) {
                    m1 = m2 = TURTLE_MOTOR_POWER_PWM;
                    pulse = true; 
                }
                else if (command.equalsIgnoreCase("push_l")) {
                    m2 = m3 = TURTLE_MOTOR_POWER_PWM; pulse = true;
                }
                else if (command.equalsIgnoreCase("push_r")) { 
                    m1 = m4 = TURTLE_MOTOR_POWER_PWM; pulse = true; 
                }
                if (pulse) {
                    writeMotorsPWM(m1, m2, m3, m4); 
                    turtleMotorPulseTime = millis();
                    Serial.print("Turtle 脉冲: "); 
                    Serial.println(command);
                }
                commandProcessed = true;
            }

            // --- 4. 指令处理完毕 ---
            if (commandProcessed) {
                 // *** 无论处理了什么指令，都回复当前状态给 APP ***
                 sendStatusUpdate();
            } else {
              //  Serial.print("未知指令或在当前模式下无效: "); Serial.println(command);
                // 对于无效指令，可以选择回复错误或不回复
                // udp.beginPacket(udp.remoteIP(), udp.remotePort());
                // udp.print("{\"status\":\"Invalid Cmd\"}");
                // udp.endPacket();
            }

        } // end if len > 0

    } else { 
        // 本次循环没有收到 UDP 指令
        // 如果飞机已解锁且处于正常飞行模式，则将摇杆输入归零以实现自稳悬停
        if (currentFlightMode == FLIGHT_MODE_ARMED) {
            //rc_raw_roll_input = 0.0f;
           // rc_raw_pitch_input = 0.0f;
           // rc_raw_yaw_input = 0.0f;
            // 油门 rc_throttle_input 保持上一次的值
        }
    }
}


// --- 发送状态更新给 APP 
void sendStatusUpdate() {
    // 检查是否知道回复地址
    if (udp.remoteIP() == IPAddress(0,0,0,0)) {
        // Serial.println("无法发送状态：未知远程地址。"); // 可能在 APP 连接前收到指令
        return;
    }

    // 1. 创建 JSON 文档
    StaticJsonDocument<UDP_TX_JSON_BUFFER_SIZE> docOut; 
    // 2. 填充 JSON 数据
    docOut["mode"] = (int)currentFlightMode; 
   // 将浮点数四舍五入到一位小数
    float roll_rounded = roundf(roll * 10.0f) / 10.0f;
    float pitch_rounded = roundf(pitch * 10.0f) / 10.0f;
    float yaw_rounded = roundf(yaw * 10.0f) / 10.0f;


    // 直接将处理后的 float 值赋给 JSON 对象
    docOut["roll"] = roll_rounded;
    docOut["pitch"] = pitch_rounded;
    docOut["yaw"] = yaw_rounded;
    docOut["alt"] = 0; 
    docOut["thr_in"] = (int)rc_throttle_input; 

    docOut["m1"] = m1_pwm;
    docOut["m2"] = m2_pwm;
    docOut["m3"] = m3_pwm;
    docOut["m4"] = m4_pwm;
    // 3. 序列化 JSON 并发送
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    serializeJson(docOut, udp); 
    udp.endPacket(); 


}




// --------------ELRS/CRSF 控制-------------------

//初始化ELRS串口并检测信号，以决定使用ELRS还是WiFi模式。
void setupELRSAndDetermineControlMode() {
  Serial.println("正在检测ELRS接收机...");
  CRSF_SERIAL_PORT.begin(CRSF_BAUDRATE, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);
  
  // 等待一小段时间，看是否有CRSF信号进来
  long detection_start_time = millis();
  bool elrs_detected = false;
  while (millis() - detection_start_time < 9500) { // 检测1.5秒
    if (CRSF_SERIAL_PORT.available()) {
      elrs_detected = true;
      break;
    }
    delay(10);
  }

  if (elrs_detected) {
    currentControlMode = CONTROL_MODE_ELRS;
    Serial.println("成功检测到ELRS接收机！进入ELRS遥控模式。");
    // 清空串口缓冲区，准备接收
    while(CRSF_SERIAL_PORT.available()) CRSF_SERIAL_PORT.read();
  } else {
    currentControlMode = CONTROL_MODE_WIFI;
    CRSF_SERIAL_PORT.end();
    Serial.println("未检测到ELRS接收机，将回退到WiFi APP模式。");
  }
}

//读取和解析CRSF数据包。默认读取16个通道，实际只用到8个通道,冗余设计
void readCRSF() {
    uint8_t buffer[64];
    while (CRSF_SERIAL_PORT.available()) {
        if (CRSF_SERIAL_PORT.read() == CRSF_SYNC_BYTE) {
            if (CRSF_SERIAL_PORT.available()) {
                uint8_t frame_len = CRSF_SERIAL_PORT.read();
                if (frame_len > 1 && frame_len <= sizeof(buffer)) {
                    uint8_t bytes_to_read = frame_len - 1; 
                    CRSF_SERIAL_PORT.readBytes(buffer, bytes_to_read);
                    uint8_t frame_type = buffer[0];

                    if (frame_type == CRSF_FRAME_TYPE_RC_CHANNELS) {
                        // 22 bytes for 16 channels, each channel is 11 bits
                        // Unpack the 11-bit channel data
                        crsf_channel_data[0]  = (buffer[1] | buffer[2] << 8) & 0x07FF;
                        crsf_channel_data[1]  = (buffer[2] >> 3 | buffer[3] << 5) & 0x07FF;
                        crsf_channel_data[2]  = (buffer[3] >> 6 | buffer[4] << 2 | buffer[5] << 10) & 0x07FF;
                        crsf_channel_data[3]  = (buffer[5] >> 1 | buffer[6] << 7) & 0x07FF;
                        crsf_channel_data[4]  = (buffer[6] >> 4 | buffer[7] << 4) & 0x07FF;
                        crsf_channel_data[5]  = (buffer[7] >> 7 | buffer[8] << 1 | buffer[9] << 9) & 0x07FF;
                        crsf_channel_data[6]  = (buffer[9] >> 2 | buffer[10] << 6) & 0x07FF;
                        crsf_channel_data[7]  = (buffer[10] >> 5 | buffer[11] << 3) & 0x07FF;
                        crsf_channel_data[8]  = (buffer[12] | buffer[13] << 8) & 0x07FF;
                        crsf_channel_data[9]  = (buffer[13] >> 3 | buffer[14] << 5) & 0x07FF;
                        crsf_channel_data[10] = (buffer[14] >> 6 | buffer[15] << 2 | buffer[16] << 10) & 0x07FF;
                        crsf_channel_data[11] = (buffer[16] >> 1 | buffer[17] << 7) & 0x07FF;
                        crsf_channel_data[12] = (buffer[17] >> 4 | buffer[18] << 4) & 0x07FF;
                        crsf_channel_data[13] = (buffer[18] >> 7 | buffer[19] << 1 | buffer[20] << 9) & 0x07FF;
                        crsf_channel_data[14] = (buffer[20] >> 2 | buffer[21] << 6) & 0x07FF;
                        crsf_channel_data[15] = (buffer[21] >> 5 | buffer[22] << 3) & 0x07FF;
                        
                        last_crsf_packet_time = millis();
                        crsf_failsafe = false;
                        processCRSFChannels();
                    }
                }
            }
        }
    }
    // 检测失控保护
    if (millis() - last_crsf_packet_time > 500) { // 超过500ms没收到信号
        crsf_failsafe = true;
    }
}



//处理CRSF通道数据，更新RC输入和飞行模式。
void processCRSFChannels() {
    // CRSF通道值范围是 172-1811。映射到需要的范围。
    // 172 -> 1000us, 992 -> 1500us, 1811 -> 2000us
    // 油门 (通道1): 映射到 0-1000
    rc_throttle_input = mapf(crsf_channel_data[0], 172, 1811, 0.0f, 1000.0f);
    rc_throttle_input = constrainf(rc_throttle_input, 0.0f, 950.0f);

    // 横滚 (通道2): 映射到 -1.0 到 1.0
    rc_raw_roll_input = mapf(crsf_channel_data[1], 172, 1811, -1.0f, 1.0f);
    if (fabs(rc_raw_roll_input) < 0.02f) rc_raw_roll_input = 0.0f;
    // 俯仰 (通道3): 映射到 -1.0 到 1.0,取负值，遥控方向与无人机陀螺仪方向一致
    rc_raw_pitch_input = -mapf(crsf_channel_data[2], 172, 1811, -1.0f, 1.0f);
 if (fabs(rc_raw_pitch_input) < 0.02f) rc_raw_pitch_input = 0.0f;
    // 偏航 (通道4): 映射到 -1.0 到 1.0 取负值，让遥控方向与现有无人机陀螺仪方向一致
    rc_raw_yaw_input =- mapf(crsf_channel_data[3], 172, 1811, -1.0f, 1.0f);
 if (fabs(rc_raw_yaw_input) < 0.02f) rc_raw_yaw_input = 0.0f;
    // 解锁/上锁开关 (通道5 - AUX1)
    // 大于1500us (中间值992) 认为是解锁
    bool arm_switch = (crsf_channel_data[4] > 1000); 

    if (arm_switch) {
        if (currentFlightMode == FLIGHT_MODE_DISARMED && rc_throttle_input < 50.0f) {
            if (abs(roll) < 30.0f && abs(pitch) < 30.0f) {
                Serial.println("ELRS 解锁电机...");
                initializeAttitudeForArming();
                currentFlightMode = FLIGHT_MODE_ARMED;
            }
        }
    } else {
        if (currentFlightMode == FLIGHT_MODE_ARMED) {
            Serial.println("ELRS 锁定电机...");
            currentFlightMode = FLIGHT_MODE_DISARMED;
        }
    }
}
