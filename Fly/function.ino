/*
一些通用函数的实现

code by:ZhuHenCai
*/



//设置电机 PWM 通道
//ESP32 有 16 个 PWM 通道（编号从 0 到 15）
//PWM_FREQ频率1000，PWM_RESOLUTION分辨率
void setupPWM() {
    Serial.println("设置 PWM 通道...");
    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR1_PIN, 0);
    ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR2_PIN, 1);
    ledcSetup(2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR3_PIN, 2);
    ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(MOTOR4_PIN, 3);
    //初始定为停机
    writeMotorsPWM(MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM);
    Serial.println("PWM 设置完成.");
}


// 油门指数曲线处理0.6，0.7
float applyThrottleCurve(float throttleInput) {
    if (throttleInput < 1.0f)
        return 0.0f;
    float normalizedInput = throttleInput / 1000.0f;
    float normalizedOutput = THROTTLE_EXPO * normalizedInput + (1.0f - THROTTLE_EXPO) * normalizedInput * normalizedInput * normalizedInput;
    return constrainf(normalizedOutput * 1000.0f, 0.0f, 1000.0f);
}


// 摇杆曲线
float applyRCExpo(float input, float expoFactor) {
    if (abs(input) < 1e-6f)
        return 0.0f;
    float output = expoFactor * input + (1.0f - expoFactor) * input * input * input;
    return constrainf(output, -1.0f, 1.0f);
}


// 安全检查函数
void safetyCheck() {
    if (currentControlMode == CONTROL_MODE_WIFI) {
        // UDP 通信超时
        if (currentFlightMode != FLIGHT_MODE_DISARMED && millis() - last_udp_command_time > UDP_TIMEOUT_MS) {
            Serial.println("严重警告: UDP 通信超时! 强制锁定电机!");
            currentFlightMode = FLIGHT_MODE_DISARMED;
        }
    } else { 
        // ELRS 失控
        if (crsf_failsafe && currentFlightMode != FLIGHT_MODE_DISARMED) {
            Serial.println("严重警告: ELRS 信号丢失! 强制锁定电机!");
            currentFlightMode = FLIGHT_MODE_DISARMED;
        }
    }
	
    // 2. 极端姿态角
    if (currentFlightMode == FLIGHT_MODE_ARMED && (abs(roll) > 55.0f || abs(pitch) > 55.0f)) {
        Serial.println("严重警告: 检测到极端姿态角! 强制锁定电机!");
        currentFlightMode = FLIGHT_MODE_DISARMED;
    }
   // 3. 确保锁定状态下电机停止 (双重保险)
    if (currentFlightMode == FLIGHT_MODE_DISARMED) {
        writeMotorsPWM(MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM);
        rc_throttle_input = 0;

    }

}


// 移动平均滤波器函数
float movingAverageFilter(float newValue, std::vector<float>& history, float& sum, int windowSize) {
    if (windowSize <= 0)
        return newValue;
    if (history.size() >= windowSize) {
        sum -= history[0];
        history.erase(history.begin());
    }

    history.push_back(newValue);
    sum += newValue;

    return sum / history.size();
}


//归一化处理
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    if (abs(in_max - in_min) < 1e-6f) {
        return out_min;
    }
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//限制值
float constrainf(float val, float min_val, float max_val) {

    if (val < min_val)
        return min_val;
    if (val > max_val)
        return max_val;
    return val;
}

//禁用低电压复位功能,懒人处理
//有精力的增加ADC电路读取电池电压，户外飞行一定要注意这个，避免砸机或伤人
void disableBrownOut() {
 REG_WRITE(RTC_CNTL_BROWN_OUT_REG, 0);
}

//查看死机原因
void print_reset_reason_detailed() {
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.print("Reset reason: ");
  switch (reason) {
    case ESP_RST_UNKNOWN:    Serial.println("Unknown"); break;
    case ESP_RST_POWERON:    Serial.println("Power on"); break;
    case ESP_RST_EXT:        Serial.println("External pin"); break;
    case ESP_RST_SW:         Serial.println("Software"); break;
    case ESP_RST_PANIC:      Serial.println("Panic/exception"); break;
    case ESP_RST_INT_WDT:    Serial.println("Interrupt watchdog"); break;
    case ESP_RST_TASK_WDT:   Serial.println("Task watchdog"); break;
    case ESP_RST_WDT:        Serial.println("Other watchdog"); break;
    case ESP_RST_DEEPSLEEP:  Serial.println("Exiting deep sleep"); break;
    case ESP_RST_BROWNOUT:   Serial.println("Brownout event!"); break; // 重点关注
    case ESP_RST_SDIO:       Serial.println("SDIO"); break;
    default:                 Serial.println("Other: " + String(reason)); break;
  }
}

const int LED_PIN = 2;  // LED脚
//LED闪
void flashLED(int count, int interval) {
  for (int i = 0; i < count; i++) {
    digitalWrite(LED_PIN, HIGH);  
    delay(interval);           
    digitalWrite(LED_PIN, LOW); 
    delay(interval); 
  }
}