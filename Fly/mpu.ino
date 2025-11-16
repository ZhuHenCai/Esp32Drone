/*
传感器相关实现

code by:ZhuHenCai
*/


//  传感器初始化 
void setupSensors() {
    mpu_initialized = false;
    if (!mpu.begin(MPU6050_ADDR)) {
        Serial.println("错误: 未找到 MPU6050 芯片！程序停止。");
        while (1)
        delay(10);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);//加速器量程，2.4，8，16，求稳，
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);//陀螺仪量程,较准时，由于是静止，其实可以设置为灵敏度高一点250,角速度转成度时，要根据这个值去计算
    mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);//原94，MPU6050_BAND_184_HZ提高MPU自带的低通融合MPU6050_BAND_21_HZ MPU6050_BAND_44_HZ MPU6050_BAND_94_HZ ，尝试5HZ，延迟17.8ms  原MPU6050_BAND_10_HZ
    mpu_initialized = true;
    Serial.println("MPU6050 初始化成功。");
    Serial.println("传感器初始化完成。");
}


// 向 MPU6050 寄存器写入数据 (I2C)
void writeMPU6050Register(uint8_t reg, uint8_t data) {

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}


// 从 MPU6050 寄存器读取数据 (I2C)
uint8_t readMPU6050Register(uint8_t reg) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}

//  陀螺仪校准 每次开机都执行，在计算角度前在calculateAttitude中进行修正
void calibrateGyro() {
	
	if (!mpu_initialized) {
		Serial.println("跳过陀螺仪校准：MPU6050 未初始化。");
		return; 
	} 
	Serial.println("开始校准陀螺仪... 请保持无人机绝对静止！");
	const int numReadings = 3000; 
	float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f; 
	sensors_event_t a, g, temp; 
	Serial.println("正在采样...");
	for (int i = 0; i < numReadings; i++) {
		if (mpu.getEvent(&a, &g, &temp)) {
				gx_sum += g.gyro.x; 
				gy_sum += g.gyro.y;
				gz_sum += g.gyro.z;
			} else { 
				Serial.println("警告: 校准时读取 MPU 失败！"); i--; delay(1);
			}
				delay(2); 
			} 
			//这里的偏移是 弧度率
			gyroXoffset_rad = gx_sum / numReadings; 
			gyroYoffset_rad = gy_sum / numReadings; 
			gyroZoffset_rad = gz_sum / numReadings; 
			Serial.println("陀螺仪校准完成。"); 
			Serial.print("X 轴偏移 (deg/s): ");
			Serial.println(gyroXoffset_rad * RAD_TO_DEG);
			Serial.print("Y 轴偏移 (deg/s): ");
			Serial.println(gyroYoffset_rad * RAD_TO_DEG); 
			Serial.print("Z 轴偏移 (deg/s): ");
			Serial.println(gyroZoffset_rad * RAD_TO_DEG); 
 
}


//===========地面水平偏差问题，每次解锁后，立即把当前姿态计算出来作为无人机当前角度，让无人机起飞后以0为目标摆平机身====

void initializeAttitudeForArming() {
    Serial.println("解锁程序启动...同步姿态滤波器至当前真实角度...");
    
    // 1: 采样加速度计，获取当前最准确的“原始”地面倾斜角
    const int numReadings = 500;
    long ax_sum = 0, ay_sum = 0,az_sum=0;
    for (int i = 0; i < numReadings; i++) {
        int16_t raw_ax, raw_ay, raw_az;
        readRawAccelData(&raw_ax, &raw_ay, &raw_az);
        ax_sum += raw_ax;
        ay_sum += raw_ay;
        az_sum+=raw_az;
        delay(1); 
    }
    float ax_avg = (float)ax_sum / numReadings;
    float ay_avg = (float)ay_sum / numReadings;
    float az_avg = (float)az_sum / numReadings;
    
    // 2: 应用已保存的六面校准数据，计算出最准确的“真实”姿态
    float calibrated_ax = (ax_avg - accel_x_offset_saved) * accel_x_scale_saved;
    float calibrated_ay = (ay_avg - accel_y_offset_saved) * accel_y_scale_saved;
    float calibrated_az = (az_avg - accel_z_offset_saved) * accel_z_scale_saved;

    float true_surface_tilt_roll = atan2(-calibrated_ax, calibrated_az);
    float true_surface_tilt_pitch = atan2(calibrated_ay, sqrt(calibrated_ax * calibrated_ax + calibrated_az * calibrated_az));  
    
    //3: 用这个最准确的“真实”姿态，强行覆盖并重置滤波器的当前状态
    angle_roll = true_surface_tilt_roll;
    angle_pitch = true_surface_tilt_pitch;
    roll = true_surface_tilt_roll;
    pitch = true_surface_tilt_pitch;
    yaw = 0.0f;

    // 4: 重置所有PID积分项和历史状态
    resetPIDIntegrals();

    Serial.println("姿态同步完成，起飞目标为：真实水平面(0,0)。");
}


//传感器读取 ，减去6面矫正偏差
void readSensors() {
    if (mpu_initialized) {
        // --- 1. 手动处理加速度计 ---
        int16_t raw_ax, raw_ay, raw_az;
        readRawAccelData(&raw_ax, &raw_ay, &raw_az);

        float calibrated_ax = ( (float)raw_ax - accel_x_offset_saved) * accel_x_scale_saved;
        float calibrated_ay = ( (float)raw_ay - accel_y_offset_saved) * accel_y_scale_saved;
        float calibrated_az = ( (float)raw_az - accel_z_offset_saved) * accel_z_scale_saved;
        
        const float G_SENSITIVITY = 8192.0f; 
        accX = (calibrated_ax / G_SENSITIVITY) * SENSORS_GRAVITY_STANDARD;
        accY = (calibrated_ay / G_SENSITIVITY) * SENSORS_GRAVITY_STANDARD;
        accZ = (calibrated_az / G_SENSITIVITY) * SENSORS_GRAVITY_STANDARD;


        sensors_event_t a_dummy, g, temp_mpu;
        mpu.getEvent(&a_dummy, &g, &temp_mpu); 
        
        gyroX_rad = g.gyro.x; 
        gyroY_rad = g.gyro.y;
        gyroZ_rad = g.gyro.z;
        
    } else { 
	}
}


// 
//直接读取加速度计原始ADC值
void readRawAccelData(int16_t* ax, int16_t* ay, int16_t* az) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B); // MPU6050_RA_ACCEL_XOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true); // 读取6个寄存器
  
    // 读取并合并高低字节
    *ax = (int16_t)(Wire.read() << 8 | Wire.read());
    *ay = (int16_t)(Wire.read() << 8 | Wire.read());
    *az = (int16_t)(Wire.read() << 8 | Wire.read());
}


// 六面校准函数 同时计算出安装偏差

void performSixPointCalibration() {
    long ax_max = -32768, ax_min = 32767;
    long ay_max = -32768, ay_min = 32767;
    long az_max = -32768, az_min = 32767;

    const int SAMPLES_PER_SIDE = 512;

    Serial.println("\n\n===== 开始六面加速度计校准 =====");
    Serial.println("您需要将无人机按6个方向静置。");
    Serial.println("在每个方向静置好后，在下方输入任意字符再按发送键继续...");

    for (int side = 1; side <= 6; side++) {
        switch (side) {
            case 1: Serial.print("\n步骤 1/6: 请将无人机 [水平放置]，然后发送任意字符..."); break;
            case 2: Serial.print("\n步骤 2/6: 请将无人机 [倒置（底朝天）]，然后发送任意字符..."); break;
            case 3: Serial.print("\n步骤 3/6: 请将无人机 [机头朝下]，然后发送任意字符..."); break;
            case 4: Serial.print("\n步骤 4/6: 请将无人机 [机头朝上]，然后发送任意字符..."); break;
            case 5: Serial.print("\n步骤 5/6: 请将无人机 [左侧立]，然后发送任意字符..."); break;
            case 6: Serial.print("\n步骤 6/6: 请将无人机 [右侧立]，然后发送任意字符..."); break;
        }

        while (Serial.available() == 0) { delay(100); }
        while (Serial.available() > 0) { Serial.read(); } // 清空缓冲区
        
        Serial.println(" 正在采样...");
        delay(1000); // 等待1秒让飞机稳定

        long ax_sum = 0, ay_sum = 0, az_sum = 0;
        for (int i = 0; i < SAMPLES_PER_SIDE; i++) {
            int16_t raw_ax, raw_ay, raw_az;
            readRawAccelData(&raw_ax, &raw_ay, &raw_az);
            ax_sum += raw_ax;
            ay_sum += raw_ay;
            az_sum += raw_az;
            delay(4);
        }
        long ax_avg = ax_sum / SAMPLES_PER_SIDE;
        long ay_avg = ay_sum / SAMPLES_PER_SIDE;
        long az_avg = az_sum / SAMPLES_PER_SIDE;

        if (ax_avg > ax_max) ax_max = ax_avg;
        if (ax_avg < ax_min) ax_min = ax_avg;
        if (ay_avg > ay_max) ay_max = ay_avg;
        if (ay_avg < ay_min) ay_min = ay_avg;
        if (az_avg > az_max) az_max = az_avg;
        if (az_avg < az_min) az_min = az_avg;
    }

    // 计算零点偏移 (Bias) 和刻度 (Scale)
    float accel_x_offset = (float)(ax_max + ax_min) / 2.0f;
    float accel_y_offset = (float)(ay_max + ay_min) / 2.0f;
    float accel_z_offset = (float)(az_max + az_min) / 2.0f;
    
    float avg_delta_x = (float)(ax_max - ax_min) / 2.0f;
    float avg_delta_y = (float)(ay_max - ay_min) / 2.0f;
    float avg_delta_z = (float)(az_max - az_min) / 2.0f;
    float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3.0f;

    float accel_x_scale = avg_delta / avg_delta_x;
    float accel_y_scale = avg_delta / avg_delta_y;
    float accel_z_scale = avg_delta / avg_delta_z;

    // --- 将校准结果永久保存到闪存 ---
    preferences.begin("drone-calib", false);
    preferences.putFloat("accelXOff", accel_x_offset);
    preferences.putFloat("accelYOff", accel_y_offset);
    preferences.putFloat("accelZOff", accel_z_offset);
    preferences.putFloat("accelXScale", accel_x_scale);
    preferences.putFloat("accelYScale", accel_y_scale);
    preferences.putFloat("accelZScale", accel_z_scale);
    preferences.end();
    
    Serial.println("\n\n===== 六面校准完成并已永久保存！ =====");
    Serial.println("请重新启动无人机以应用新的校准数据。");
    while(1) {} // 暂停程序
}
