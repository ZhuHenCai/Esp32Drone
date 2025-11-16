/*
控制状态逻辑管理代码
handleFlippingState  功能没实现成功，慎用，觉得思路有严重的问题
code by:ZhuHenCai
*/
//=============== 无人机状态控制===============
void runControlLoop() {
    switch (currentFlightMode) { 
        case FLIGHT_MODE_DISARMED: handleDisarmedState();//上锁
        break;
        case FLIGHT_MODE_ARMED: handleArmedState();//解锁
        break;
        case FLIGHT_MODE_FLIPPING: handleFlippingState();//翻滚，，，，没实现，砸机
        break;
        case FLIGHT_MODE_TURTLE: handleTurtleState();//倒机起身
        break;
        }
    } 


// 状态处理
void updateFlightMode() {
    isFlippedOver = (currentFlightMode == FLIGHT_MODE_DISARMED) && (abs(roll) > 85.0f || abs(pitch) > 85.0f);
    if (currentFlightMode == FLIGHT_MODE_FLIPPING) {
        if (millis() - flipStartTime > FLIP_DURATION_MS) {
                Serial.println("翻转完成。");
                currentFlightMode = FLIGHT_MODE_ARMED;
                currentFlipDirection = FLIP_NONE; 
                resetPIDIntegrals(); 
            } 
    }
    if (currentFlightMode == FLIGHT_MODE_TURTLE) {
            if (!isFlippedOver) { 
                Serial.println("已翻正，退出Turtle。");
                currentFlightMode = FLIGHT_MODE_DISARMED; 
                writeMotorsPWM(MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM); 
            }
    }

}


//=============== 状态处理函数===============
//未解锁，停四个电机，重置积分，关掉高度功能
void handleDisarmedState() {
    writeMotorsPWM(MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM);
    resetPIDIntegrals();

}
//解锁状态
void handleArmedState() {
    // 只有在非失控状态下才响应遥控输入
    if (currentControlMode == CONTROL_MODE_WIFI || (currentControlMode == CONTROL_MODE_ELRS && !crsf_failsafe)) {
            //油门线性处理
            float processed_throttle = applyThrottleCurve(rc_throttle_input);
            //对输入的目标角进行指数处理
            //输入0.1-0.7 结果为 0.8012-9.716，需要调小，否则容易翻机，原为20，设置为5
            float final_target_roll = applyRCExpo(rc_raw_roll_input, RC_RATE_EXPO) * MAX_TILT_ANGLE;
            float final_target_pitch = applyRCExpo(rc_raw_pitch_input, RC_RATE_EXPO) * MAX_TILT_ANGLE;
            float final_target_yaw = applyRCExpo(rc_raw_yaw_input, RC_RATE_EXPO) * MAX_TILT_ANGLE;
            float final_throttle_command = processed_throttle;

            //值限制
            final_throttle_command = constrainf(final_throttle_command, MIN_PWM_SIGNAL, MAX_PWM_SIGNAL);
    
              // --- 地面模式  没飞前PID不启用---
             // PID控制器完全静默，只输出怠速，并持续清零积分
            if (final_throttle_command < 50) {
                resetPIDIntegrals(); 
                roll_pid_output = 0;
                pitch_pid_output = 0;
                yaw_pid_output = 0;
            } else {
                // --- 空中模式 ---
                // PID控制器完全激活
                runPIDControllers(final_target_roll, final_target_pitch, final_target_yaw, true);
            }

            //操作电机值
            mixMotors(final_throttle_command, roll_pid_output, pitch_pid_output, yaw_pid_output);
        } else {
            // 如果是 ELRS 模式且处于失控保护状态，则电机停转
            writeMotorsPWM(MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM);
    }
}

//翻转特效，没弄成功，觉得这种思路不行，很被动
void handleFlippingState() {
    unsigned long timeSinceFlipStart = millis() - flipStartTime;
    float flip_throttle = applyThrottleCurve(rc_throttle_input);
     // 初始横滚、俯仰和偏航的目标速率为0
    float target_rate_roll = 0.0f, target_rate_pitch = 0.0f, target_rate_yaw = 0.0f;
    //总翻转时间预计为600ms，
    if (timeSinceFlipStart < FLIP_BOOST_DURATION_MS) {
        flip_throttle *= FLIP_THROTTLE_BOOST;
    } else if (timeSinceFlipStart < FLIP_BOOST_DURATION_MS + FLIP_ROTATE_DURATION_MS) {
        // 根据翻转方向，设置速率
        switch (currentFlipDirection) {
            case FLIP_FRONT:
                target_rate_pitch = FLIP_RATE_TARGET;
                break;
            case FLIP_BACK:
                target_rate_pitch = -FLIP_RATE_TARGET;
                break;
            case FLIP_LEFT:
                target_rate_roll = -FLIP_RATE_TARGET;
                break;
            case FLIP_RIGHT:
                target_rate_roll = FLIP_RATE_TARGET;
                break;
            default:
                break;
        }
        // 入翻滚时间
        unsigned long timeIntoRotation = timeSinceFlipStart - FLIP_BOOST_DURATION_MS;
        if (timeIntoRotation > FLIP_CUT_THROTTLE_START_MS && timeIntoRotation < FLIP_CUT_THROTTLE_END_MS) {
        // 翻滚中对油门下调
            flip_throttle *= FLIP_THROTTLE_CUT_FACTOR;
        }
    } else {
        //安装会有偏差，要矫正
        target_rate_roll = 0.0f;
        target_rate_pitch = 0.0f;
    }
    //限制油门
    flip_throttle = constrainf(flip_throttle, MOTOR_STOP_PWM, MAX_PWM_SIGNAL);
    runPIDControllers(target_rate_roll, target_rate_pitch, target_rate_yaw, false);
    mixMotors(flip_throttle, roll_pid_output, pitch_pid_output, yaw_pid_output);
}
//翻机后停机
void handleTurtleState() {
    if (turtleMotorPulseTime != 0 && millis() - turtleMotorPulseTime > TURTLE_PULSE_DURATION_MS) {
        writeMotorsPWM(MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM, MOTOR_STOP_PWM);
        turtleMotorPulseTime = 0;
    }
}    
