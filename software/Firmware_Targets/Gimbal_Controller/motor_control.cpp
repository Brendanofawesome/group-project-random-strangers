//Gives motor parameters and creates the SimpleFOC objects
//Also configures the intended angle-control mode and SVPWM setting
//Settings are stored but not executed by this file


#include "motor_control.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>


// ======================================
// MOTOR CONFIGURATION
// ======================================

static BLDCMotor motor = IS_PITCH ? BLDCMotor(12, 8.3f, 30.0f) : BLDCMotor(15, 14.6f, 15.0f);

// SimpleFOC Mini:
static BLDCDriver3PWM driver(PWM_A_PIN, PWM_B_PIN, PWM_C_PIN, ENABLE_PIN);

//planner
#ifdef useTrapezoidalPlanner
    #include <SimpleFOCDrivers.h>
    #include "utilities/trapezoids/TrapezoidalPlanner.h"

    TrapezoidalPlanner planner( IS_PITCH ? pitch_velocity_limit : yaw_velocity_limit,
                                IS_PITCH ? pitch_motion_acceleration : yaw_motion_acceleration, 
                                IS_PITCH ? pitch_motion_deceleration : yaw_motion_deceleration);
#endif


//commander interface
Commander* commanderPTR;
void doMotor(char* cmd) { 
    if(commanderPTR != nullptr){
        #ifdef useTrapezoidalPlanner
            if ((cmd[0] == '-' || cmd[0] == '+' || cmd[0] == '.' || isDigit(cmd[0]))) {
                float target = 0.0f;
                commanderPTR->scalar(&target, cmd); 
                planner.setTarget(target);
                return;
            }
        #endif
        commanderPTR->motor(&motor, cmd); 
    }
}

// ======================================
// INITIALIZE simpleFOC motor control system
// ======================================

bool initializeMotorControl(Sensor* sensor, Commander* commander) {
    if (sensor == nullptr) return false;
    driver.voltage_power_supply = 24;
    driver.pwm_frequency = 21000; //20kHz

    // Set the intended SimpleFOC control modes
    motor.controller = MotionControlType::angle;
    motor.torque_controller = TorqueControlType::voltage;
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // configuration parameters
    motor.voltage_limit = 24.0f;
    motor.current_limit = 1.25f;

    if(IS_PITCH){
        motor.velocity_limit = pitch_velocity_limit;
        motor.P_angle.P = pitch_P_angle_P;
        motor.PID_velocity.P = pitch_PID_velocity_P;
        motor.PID_velocity.I = pitch_PID_velocity_I;
        motor.PID_velocity.D = pitch_PID_velocity_D;
        motor.LPF_angle.Tf = pitch_LPF_angle;
        motor.LPF_velocity.Tf = pitch_LPF_velocity;
        motor.P_angle.output_ramp = pitch_P_angle_output_ramp;
        motor.P_angle.D = pitch_P_angle_D;
    } else if (IS_YAW){
        motor.velocity_limit = yaw_velocity_limit;
        motor.P_angle.P = yaw_P_angle_P;
        motor.PID_velocity.P = yaw_PID_velocity_P;
        motor.PID_velocity.I = yaw_PID_velocity_I;
        motor.PID_velocity.D = yaw_PID_velocity_D;
        motor.LPF_angle.Tf = yaw_LPF_angle;
        motor.LPF_velocity.Tf = yaw_LPF_velocity;
        motor.P_angle.output_ramp = yaw_P_angle_output_ramp;
        motor.P_angle.D = yaw_P_angle_D;
    } else return false;


    //connect sensor object to motor object
    motor.linkSensor(sensor);

    //connect driver object to motor object
    motor.linkDriver(&driver);

    //initialize the driver hardware
    if (!driver.init()) {
        return false;
    }

    if (!motor.init()) {
        driver.disable();
        return false;
    }

    if (!motor.initFOC()) {
        motor.disable();
        return false;
    }

    commanderPTR = commander;
    if(commander != nullptr){
        commander->add('M', doMotor, "Yaw Motor");
    }

    //connect planner to motor if trapezoidal planner is used
    #ifdef useTrapezoidalPlanner
        planner.linkMotor(motor);
    #endif

    return true;
}

// ======================================
// UPDATE simpleFOC motor control system
// ======================================
void updateMotorControl() {
    #ifdef useTrapezoidalPlanner
        static uint8_t downsample_counter = 0;
        if(downsample_counter++ == 10){
            float new_target = planner.run();

            //feedforward velocity
            static uint32_t previous_planner_time = micros();
            uint32_t dt = micros() - previous_planner_time;
            previous_planner_time = micros();
            if(dt > 0.0f && dt < 0.5f){
                motor.feed_forward_velocity = (new_target - motor.target) / (dt * 0.000001f);
            } else {
                motor.feed_forward_velocity = 0.0f;
            }
            
            downsample_counter = 0;
            motor.target = new_target;
        }
    #endif

    // Update the motor control system
    motor.loopFOC();
    motor.move();
}

// ======================================
// GET CURRENT ANGLE
// ======================================
float getMotorCurrentAngle() {
    return _normalizeAngle(motor.shaft_angle);
}

// ======================================
// SET TARGET ANGLE
// ======================================
void setMotorTargetAngle(const float& angle) {
    if(isfinite(angle)) {
        #ifdef useTrapezoidalPlanner
            planner.setTarget(angle);
        #else
            motor.target = angle;
        #endif
    } else {
        if(DEBUG) Serial.println("Received non-finite target angle, ignoring.");
        return;
    }
}

// ======================================
// GET TARGET ANGLE
// ======================================
float getMotorTargetAngle(){
    #ifdef useTrapezoidalPlanner
        return planner.getTarget();
    #else
        return motor.target;
    #endif
}

// ======================================
// GET MOTOR REFERENCE
// ======================================
BLDCMotor* getMotor(){
    return &motor;
}