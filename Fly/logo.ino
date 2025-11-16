/*
飞行日志

code by:ZhuHenCai
*/





// 日志缓冲区结构
struct LogEntry {
    float loop_time_s;
    float roll, pitch, yaw;
    
    // 俯仰角PID相关数据
    float pitch_pid_output;
    float target_pitch_rate;
    float gyro_x_filtered;
    float pitchKP;
    float pitchKI;
    float pitcKD;
    
    // 横滚角PID相关数据
    float roll_pid_output;
    float target_roll_rate;
    float gyro_y_filtered;
    float rollKP;
    float rollKI;
    float rollKD;
    
    // 偏航角PID相关数据
    float yaw_pid_output;
    float target_yaw_rate;
    float gyro_z_filtered;
    float yawKP;
    float yawKI;
    float yawKD;
    
    // 电机PWM输出
    float m1_pwm, m2_pwm, m3_pwm, m4_pwm;
};

LogEntry logBuffer[LOG_SIZE];  // 日志缓冲区



void dumpLog() {
    // 打印CSV标题行
    Serial.println("loop_time_s,"
                   "roll,pitch,yaw,"
                   "pitch_pid_output,target_pitch_rate,gyro_x_filtered,pitchKP,pitchKI,pitcKD,"
                   "roll_pid_output,target_roll_rate,gyro_y_filtered,rollKP,rollKI,rollKD,"
                   "yaw_pid_output,target_yaw_rate,gyro_z_filtered,yawKP,yawKI,yawKD,"
                   "M1,M2,M3,M4");
    
    // 打印数据行
    for (int i = 0; i < logIndex; i++) {
        const LogEntry& entry = logBuffer[i];
        
        Serial.print(entry.loop_time_s, 5); Serial.print(",");
        
        Serial.print(entry.roll, 1); Serial.print(",");
        Serial.print(entry.pitch, 1); Serial.print(",");
        Serial.print(entry.yaw, 1); Serial.print(",");
        
        Serial.print(entry.pitch_pid_output, 2); Serial.print(",");
        Serial.print(entry.target_pitch_rate, 2); Serial.print(",");
        Serial.print(entry.gyro_x_filtered, 2); Serial.print(",");
        Serial.print(entry.pitchKP, 2); Serial.print(",");
        Serial.print(entry.pitchKI, 2); Serial.print(",");
        Serial.print(entry.pitcKD, 2); Serial.print(",");
        
        Serial.print(entry.roll_pid_output, 2); Serial.print(",");
        Serial.print(entry.target_roll_rate, 2); Serial.print(",");
        Serial.print(entry.gyro_y_filtered, 2); Serial.print(",");
        Serial.print(entry.rollKP, 2); Serial.print(",");
        Serial.print(entry.rollKI, 2); Serial.print(",");
        Serial.print(entry.rollKD, 2); Serial.print(",");
        
        Serial.print(entry.yaw_pid_output, 2); Serial.print(",");
        Serial.print(entry.target_yaw_rate, 2); Serial.print(",");
        Serial.print(entry.gyro_z_filtered, 2); Serial.print(",");
        Serial.print(entry.yawKP, 2); Serial.print(",");
        Serial.print(entry.yawKI, 2); Serial.print(",");
        Serial.print(entry.yawKD, 2); Serial.print(",");
        
        Serial.print(entry.m1_pwm, 1); Serial.print(",");
        Serial.print(entry.m2_pwm, 1); Serial.print(",");
        Serial.print(entry.m3_pwm, 1); Serial.print(",");
        Serial.println(entry.m4_pwm, 1);
    }
}
