/*
PID计算

code by:ZhuHenCai
*/
const float ONE_G = 9.80665;
float level_pitch_offset=0.0f;
float level_roll_offset=0.0f;
//姿态计算
float complimentary_filter_alpha = 0.98f; //原0.991，看到行业内别人都是0.997，0.98目标觉得是合适的
static int accValidCount = 0;
const int REQUIRED_VALID_COUNT = 5;

void calculateAttitude() {

    // 1. 读取并应用校准值，得到物理传感器的干净数据
    float acc_phy_x =  accX;
    float acc_phy_y = accY;
    float acc_phy_z =  accZ; 
    
    float gyro_phy_x = gyroX_rad - gyroXoffset_rad;
    float gyro_phy_y = gyroY_rad - gyroYoffset_rad;
    float gyro_phy_z = gyroZ_rad - gyroZoffset_rad;

    gyro_x_filtered = gyro_phy_x* RAD_TO_DEG;
    gyro_y_filtered =gyro_phy_y* RAD_TO_DEG;
    gyro_z_filtered =gyro_phy_z* RAD_TO_DEG;

    // 1. 从加速度计计算 Roll 和 Pitch 
    float accel_roll_deg = 0.0f;
    float accel_pitch_deg = 0.0f;
    //ONE_G=1000，当设备静止或匀速运动时，合加速度应接近重力加速度，current_valid用于判断如果处于打扛加速飞行时，不使用加速器
    float acc_norm = sqrt(acc_phy_x * acc_phy_x + acc_phy_y * acc_phy_y + acc_phy_z * acc_phy_z);
    bool current_valid = fabs(acc_norm - ONE_G) < 0.1f * ONE_G;//ONE_G=1000

    //不行就用摇杆角度来限制
    if (current_valid) {
        accValidCount++;
    } else {
        accValidCount = 0;
    }
    bool acc_is_valid = (accValidCount >= REQUIRED_VALID_COUNT);
    if (acc_is_valid) {
            accel_roll_deg = atan2(-acc_phy_x, acc_phy_z)* RAD_TO_DEG;
            accel_pitch_deg = atan2(acc_phy_y, sqrt(acc_phy_x * acc_phy_x + acc_phy_z * acc_phy_z))* RAD_TO_DEG;  
    } else {
            accel_roll_deg = angle_roll;
            accel_pitch_deg = angle_pitch;
    }


    // 2. 陀螺仪积分
    float gyro_pitch_change = gyro_x_filtered * loop_time_s;
    float gyro_roll_change = gyro_y_filtered * loop_time_s;

    
    //  3. 角速度+加速度互补滤波融合 
    // *** 修正融合关系：pitch 融合 gyroX, roll 融合 gyroY ***,陀螺仪高信任，加速度低权重 complimentary_filter_alpha 0.9996f
    angle_pitch = complimentary_filter_alpha * (angle_pitch + gyro_pitch_change) + (1.0f - complimentary_filter_alpha) * accel_pitch_deg;
    angle_roll = complimentary_filter_alpha * (angle_roll + gyro_roll_change) + (1.0f - complimentary_filter_alpha) * accel_roll_deg;
    //减去水平偏移
    pitch =angle_pitch-level_pitch_offset;
    roll = angle_roll-level_roll_offset;


    //  4. 计算 Yaw 角度 
    float gyro_yaw_change = gyro_z_filtered  * loop_time_s;
    yaw += gyro_yaw_change;
    // 限制 Yaw 范围
    if (yaw > 180.0f) yaw -= 360.0f; if (yaw < -180.0f) yaw += 360.0f;

}




const float D_output_filter_alpha = 0.5f; //1-D_output_filter_alpha=上一个D的权重// 可调参数,原0.8 例如 0.5f, 0.6f, 根据实际效果调整，原2可直飞，但抖，原0.7 可行

float  prev_roll_angle_p=0.0f;
float  prev_pitch_angle_p=0.0f;
float  prev_yaw_angle_p=0.0f;
float pitch_angle_d=0.0f,yaw_angle_d=0.0f,roll_angle_d=0.0f;
float prev_roll_angle_d=0.0f;
float prev_pitch_angle_d=0.0f;
float  prev_yaw_angle_d=0.0f;
float targetRollAngle,targetPitchAngle;
const int THROTTLE_TAKEOFF_THRESHOLD = 60; 
//PID 计算  双环
//useAnglePID: 是否启用外环角度PID (正常飞行时为true, 翻转时为false)
void runPIDControllers(float targetRollAngle1, float targetPitchAngle1, float targetYawAngle, bool useAnglePID) {

    bool targetActive = fabs(targetRollAngle1) > 0.5f || fabs(targetPitchAngle1) > 0.5f || fabs(targetYawAngle) >0.5f;

    float final_throttle_command = applyThrottleCurve(rc_throttle_input);
    if (useAnglePID) {
		//------------外环-------------
        //===1、P值======
		    //差值  手动补差偏差
        roll_angle_error = targetRollAngle1 - roll;
        pitch_angle_error = targetPitchAngle1 - pitch;
        yaw_angle_error = targetYawAngle - yaw;

        //微小的噪音抖动，不处理
        if (fabs(roll_angle_error) < 0.5f) roll_angle_error = 0.0f;
        if (fabs(pitch_angle_error) < 0.5f) pitch_angle_error = 0.0f;
        if (fabs(yaw_angle_error) < 0.5f) yaw_angle_error = 0.0f;
        //半周，则旋转
        if (yaw_angle_error > 180.0f) yaw_angle_error -= 360.0f;
        if (yaw_angle_error < -180.0f) yaw_angle_error += 360.0f;

        //低通P值
        //外环比例 P值 
        float roll_angle_p = kp_roll_angle * roll_angle_error;
        float pitch_angle_p = kp_pitch_angle * pitch_angle_error;
        float yaw_angle_p = kp_yaw_angle * yaw_angle_error;

        //P进行低通
        roll_angle_p = roll_angle_p * 0.95 + prev_roll_angle_p * 0.05;
        pitch_angle_p = pitch_angle_p * 0.95 + prev_pitch_angle_p * 0.05;
        yaw_angle_p = yaw_angle_p * 0.95 + prev_yaw_angle_p * 0.05;
        // 更新保存为上一次P值
        prev_roll_angle_p=roll_angle_p;
        prev_pitch_angle_p=pitch_angle_p;
        prev_yaw_angle_p=yaw_angle_p;





        //===2、积分值====

        //地面不积分，油门150，且误差5度以上才积分，避免起飞前原地积分，飞后偏航,偷懒，fabs(pitch_pid_output)<300 其实相位迟一步的
        if (final_throttle_command > THROTTLE_TAKEOFF_THRESHOLD && targetActive) {
          if (fabs(roll_angle_error) < 20.0f&&fabs(roll_pid_output)<300) {
             roll_angle_integral += roll_angle_error * loop_time_s *ki_roll_angle ;
            }
           if (fabs(pitch_angle_error) < 20.0f&&fabs(pitch_pid_output)<300) {
              pitch_angle_integral += pitch_angle_error * loop_time_s * ki_pitch_angle;
           }
           if (fabs(yaw_angle_error) < 20.0f&&fabs(yaw_pid_output)<300) {
               yaw_angle_integral += yaw_angle_error * loop_time_s *ki_yaw_angle ;
           }
        } else {
            // 在地面，强制清零积分，防止累积
            roll_angle_integral *=0.9f;
            pitch_angle_integral *=0.9f;
            yaw_angle_integral *=0.9f;
        }

        //积分方向与实际脱离
        if ((roll_angle_integral > 0 && roll_angle_error < 0) || (roll_angle_integral < 0 && roll_angle_error > 0)) { 
          roll_angle_integral  *= 0.98f; 
        }

        if ((pitch_angle_integral > 0 && pitch_angle_error < 0) || (pitch_angle_integral < 0 && pitch_angle_error > 0)) { 
          pitch_angle_integral  *= 0.98f; 
        }

        if ((yaw_angle_integral > 0 && yaw_angle_error < 0) || (yaw_angle_integral < 0 && yaw_angle_error > 0)) { 
          yaw_angle_integral  *= 0.98f; 
        }

        //限制积分
        roll_angle_integral = constrainf(roll_angle_integral, -100.0f, 100.0f);
        pitch_angle_integral = constrainf(pitch_angle_integral,  -100.0f, 100.0f);
		    yaw_angle_integral = constrainf(yaw_angle_integral,  -50.0f,50.0f);
 

        //===3、微分====

        // 内环角速度太跳跃了，反向取微分值 ，提前介入角速度阻尼
        roll_angle_d = -kd_roll_angle * gyro_y_filtered;
        pitch_angle_d = -kd_pitch_angle * gyro_x_filtered;
        yaw_angle_d = -kd_yaw_angle * gyro_z_filtered; 

        //更新上一次角度差
        roll_angle_prev_error = roll_angle_error;
        pitch_angle_prev_error = pitch_angle_error;
        yaw_angle_prev_error = yaw_angle_error;

        //计算出外环角度目标，并限制值
        target_roll_rate = roll_angle_p + roll_angle_integral  + roll_angle_d;
        target_pitch_rate = pitch_angle_p + pitch_angle_integral  + pitch_angle_d;
        target_yaw_rate = yaw_angle_p + yaw_angle_integral +yaw_angle_d;



    } else {
		 //  翻转等特殊模式 直接使用传入的目标角速率
        target_roll_rate = targetRollAngle1;
        target_pitch_rate = targetPitchAngle1;
        target_yaw_rate=targetYawAngle;
		// 重置角度环的积分，避免干扰
        roll_angle_integral = 0;
        pitch_angle_integral = 0;
        yaw_angle_integral=0;
    }


    
	//================内环PID begin==================

    // 0、差值
    roll_rate_error = target_roll_rate - gyro_y_filtered;
    pitch_rate_error = target_pitch_rate - gyro_x_filtered;
    yaw_rate_error = target_yaw_rate - gyro_z_filtered; 

    //对危险噪音不作响应
    if (fabs(roll_rate_error) < 0.5f) roll_rate_error = 0.0f;
    if (fabs(pitch_rate_error) < 0.5f) pitch_rate_error = 0.0f;
    if (fabs(yaw_rate_error) < 0.5f) yaw_rate_error = 0.0f;


    //1、积分，对大抖动状态，不作积分处理
    if (final_throttle_command > THROTTLE_TAKEOFF_THRESHOLD  && targetActive) {
        if (fabs(roll_rate_error) < 40.0f&&fabs(roll_pid_output)<300) { 
            roll_rate_integral += roll_rate_error * loop_time_s*ki_roll_rate;
        }
        if (fabs( pitch_rate_error ) < 40.0f&&fabs(pitch_pid_output)<300) { 
            pitch_rate_integral += pitch_rate_error * loop_time_s*ki_pitch_rate;
        }
        if (fabs(yaw_rate_error) < 40.0f&&fabs(yaw_pid_output)<300) {

            yaw_rate_integral += yaw_rate_error * loop_time_s*ki_yaw_rate;
        }
    }else{
      
        roll_rate_integral *=0.9f;
        pitch_rate_integral *=0.9f;
        yaw_rate_integral *=0.9f;
      
      }
    //积分方向相反时，降低积分
    if ((roll_rate_integral > 0 && roll_rate_error < 0) || (roll_rate_integral < 0 && roll_rate_error > 0)) { 

        roll_rate_integral  *= 0.98f; 
    }
    if ((pitch_rate_integral > 0 && pitch_rate_error < 0) || (pitch_rate_integral < 0 && pitch_rate_error > 0)) { 

        pitch_rate_integral *= 0.98f; // 衰减而非清零

    }
    if ((yaw_rate_integral > 0 && yaw_rate_error < 0) || (yaw_rate_integral < 0 && yaw_rate_error > 0)) { 

        yaw_rate_integral *= 0.98f; 
    }
    //积分饱和限制
    roll_rate_integral = constrainf(roll_rate_integral, -100.0f, 100.0f);
    pitch_rate_integral = constrainf(pitch_rate_integral, -100.0f, 100.0f);
    yaw_rate_integral = constrainf(yaw_rate_integral, -80.0f, 80.0f);


    //2、微分，抖动相对大，低通处理
    roll_rate_derivative = (roll_rate_error - roll_rate_prev_error) / loop_time_s;
    pitch_rate_derivative =(pitch_rate_error - pitch_rate_prev_error) / loop_time_s;
    yaw_rate_derivative = (yaw_rate_error - yaw_rate_prev_error) / loop_time_s;
    filtered_roll_D_output = (1.0f - D_output_filter_alpha) * filtered_roll_D_output + D_output_filter_alpha * roll_rate_derivative;
    filtered_pitch_D_output = (1.0f - D_output_filter_alpha) * filtered_pitch_D_output + D_output_filter_alpha * pitch_rate_derivative;
    filtered_yaw_D_output = (1.0f - D_output_filter_alpha) * filtered_yaw_D_output + D_output_filter_alpha * yaw_rate_derivative;

    //3、PID值sum
    roll_pid_output = kp_roll_rate * roll_rate_error + roll_rate_integral + kd_roll_rate * filtered_roll_D_output;
    pitch_pid_output = kp_pitch_rate * pitch_rate_error + pitch_rate_integral + kd_pitch_rate * filtered_pitch_D_output;
    yaw_pid_output = kp_yaw_rate * yaw_rate_error + yaw_rate_integral + kd_yaw_rate * filtered_yaw_D_output;

  if(fabs(roll_pid_output)>400){
    roll_angle_integral *= 0.995f;
    }

    // PID输出限幅
    roll_pid_output = constrainf(roll_pid_output, -400.0f, 400.0f);
    pitch_pid_output = constrainf(pitch_pid_output, -400.0f, 400.0f);
    yaw_pid_output = constrainf(yaw_pid_output, -400.0f, 400.0f);

    // 更新上一次差值
    roll_rate_prev_error = roll_rate_error;
    pitch_rate_prev_error = pitch_rate_error;
    yaw_rate_prev_error = yaw_rate_error;
    //更新角速度上一次的值
    prev_gyro_z_filtered=gyro_z_filtered;
    prev_gyro_y_filtered=gyro_y_filtered;
    prev_gyro_x_filtered=gyro_x_filtered;


    //------长期衰减，防无限累积0.999，超小

    roll_angle_integral *= 0.9995f;
    pitch_angle_integral *= 0.9995f;
    yaw_angle_integral *= 0.9995f;
    roll_rate_integral *= 0.9995f;
    pitch_rate_integral *= 0.9995f;
    yaw_rate_integral *= 0.9995f;
    

    //=======================调试Begin=====================================

 /*
    if (millis() % 100 == 0) { 

    Serial.printf("T:%.2f roll:%.2f pitch:%.2f yaw:%.2f | r_err:%.2f r_tar:%.2f r_pid:%.2f | gx:%.2f gy:%.2f gz:%.2f\n",
    (float)millis()/1000.0f, roll, pitch, yaw,
    roll_rate_error, target_roll_rate, roll_pid_output,
    gyro_x_filtered, gyro_y_filtered, gyro_z_filtered);
    Serial.println();

      
        Serial.println(" time_s:"+ String(loop_time_s, 5)); 
        Serial.print(" targetRollAngle:"); Serial.print(targetRollAngle1, 1); 
        Serial.print("  targetPitchAngle:"); Serial.println(targetPitchAngle1, 1);  
        Serial.print("  pitch:"+ String(pitch, 5)); Serial.print("  gyro_x:"+ String(gyro_x, 3)); 
        Serial.print("  roll:"+ String(roll, 5)); Serial.print("  gyro_y:"+ String(gyro_y, 3)); 
        Serial.print(" yaw:"+ String(yaw, 5));  Serial.println("  gyro_z:"+ String(gyro_z, 3)); 

        Serial.print(" M1:");
        Serial.print(m1_pwm, 1);  
        Serial.print(" M2:");
        Serial.print(m2_pwm, 1); 
        Serial.print(" M3:"); 
        Serial.print(m3_pwm, 1); 
        Serial.print(" M4:"); 
        Serial.print(m4_pwm, 1); 
        Serial.println();
       
        }
 */

  // === 记录飞行日志 的冗余临时变量 ===
  
  targetRollAngle=targetRollAngle1;
  targetPitchAngle=targetPitchAngle1;


}



//根据姿态调节各个电机PWM
void mixMotors(float throttleCommand, float rollCommand, float pitchCommand, float yawCommand) {

    float m1_speed = throttleCommand + pitchCommand + rollCommand - yawCommand;
    float m2_speed = throttleCommand + pitchCommand - rollCommand + yawCommand;
    float m3_speed = throttleCommand - pitchCommand - rollCommand - yawCommand;
    float m4_speed = throttleCommand - pitchCommand + rollCommand + yawCommand;

    //抗饱合，超值同比下调
    float max_motor = max(max(m1_speed, m2_speed), max(m3_speed, m4_speed));
    if (max_motor > MAX_PWM_SIGNAL) {
        float scale = (float)MAX_PWM_SIGNAL / max_motor;
        m1_speed *= scale;
        m2_speed *= scale;
        m3_speed *= scale;
        m4_speed *= scale;
    }

    if (rc_throttle_input < 35.0f) {
        m1_pwm = MOTOR_STOP_PWM;
        m2_pwm = MOTOR_STOP_PWM;
        m3_pwm = MOTOR_STOP_PWM;
        m4_pwm = MOTOR_STOP_PWM;
    } else {
        m1_pwm = constrainf(m1_speed, MIN_PWM_SIGNAL, MAX_PWM_SIGNAL);
        m2_pwm = constrainf(m2_speed, MIN_PWM_SIGNAL, MAX_PWM_SIGNAL);
        m3_pwm = constrainf(m3_speed, MIN_PWM_SIGNAL, MAX_PWM_SIGNAL);
        m4_pwm = constrainf(m4_speed, MIN_PWM_SIGNAL, MAX_PWM_SIGNAL);
		//日志开启
       //writeFlightLog();
    }



    /*
    if (millis() % 100 == 0) { 
        Serial.println(" time_s:"+ String(loop_time_s, 5)); 
        Serial.print("  pitch:"+ String(pitch, 5)); Serial.print("  gyro_x:"+ String(gyro_x, 3)); 
        Serial.print("  roll:"+ String(roll, 5)); Serial.print("  gyro_y:"+ String(gyro_y, 3)); 
        Serial.print(" yaw:"+ String(yaw, 5));  Serial.println("  gyro_z:"+ String(gyro_z, 3)); 
    }
    */
 writeMotorsPWM(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
}
//更新到电机各通道
void writeMotorsPWM(int m1, int m2, int m3, int m4) {

    ledcWrite(0, getMapPwm(m1));
    ledcWrite(1, getMapPwm(m2));
    ledcWrite(2, getMapPwm(m3));
    ledcWrite(3, getMapPwm(m4));
}
//最终的PWM值
int getMapPwm(float value) {
    float duty = mapf(value, 0, 1000, 0, (1 << PWM_RESOLUTION) - 1);
    // 对占空比进行取整
    return round(duty);
}


//重置各个PID
void resetPIDIntegrals() {
    //时间重置
    last_loop_time=micros();
    roll_angle_integral = 0.0f;
    pitch_angle_integral = 0.0f;
    yaw_angle_integral= 0.0f;
    roll_rate_integral = 0.0f;
    pitch_rate_integral = 0.0f;
    yaw_rate_integral = 0.0f;



    filtered_roll_D_output = 0.0f;
    filtered_pitch_D_output = 0.0f;
    filtered_yaw_D_output = 0.0f;

    gyro_x_filtered =0.0f;
    gyro_y_filtered =0.0f;
    gyro_z_filtered =0.0f;

    prev_gyro_z_filtered=0.0f;
    prev_gyro_y_filtered=0.0f;
    prev_gyro_x_filtered=0.0f;

    roll_rate_prev_error = 0.0f;
    pitch_rate_prev_error =0.0f;
    yaw_rate_prev_error = 0.0f;

    roll_angle_prev_error =  0.0f;
    pitch_angle_prev_error =  0.0f;
    yaw_angle_prev_error =  0.0f;
    
    prev_roll_angle_d=0.0f;
    prev_pitch_angle_d=0.0f;
    prev_yaw_angle_d=0.0f;

    prev_roll_angle_p = 0.0f;
    prev_pitch_angle_p = 0.0f;
    prev_yaw_angle_p = 0.0f;

    gyroXFilter.reset();
    gyroYFilter.reset();
    gyroZFilter.reset();

 
}
