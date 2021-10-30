#include "Copter.h"

#if MODE_GROUND_ENABLED == ENABLED

#define SERVO_OUTPUT_RANGE 18000

// Parameters standard value
#define ARMING_TIME               3500 // ms
#define MAX_THROTTLE              1800
#define MIN_THROTTLE              1025
#define MAX_BRAKE_THROTTLE        0.5
#define MAX_BRAKE_TIME            2000 // ms
#define MAX_FLAT_THROTTLE         0.35
#define MAX_TURN_THROTTLE         0.7
#define MAXIMUM_INCLINATION_ANGLE 4500 // centiDegrees
#define MINIMUM_INCLINATION_ANGLE 1000 // centiDegrees
#define MAX_INCLINED_THROTTLE     0.8

/*
 * Init and run calls for ground locomotion mode
 */

// ground_init - initialise ground controller
bool ModeGround::init(bool ignore_checks)
{
    // Don't enter the mode if sticks are not centered
    if (!is_zero(channel_pitch->norm_input_dz()) 
        || !is_zero(channel_roll->norm_input_dz())
        || !is_zero(channel_yaw->norm_input_dz()) 
        || channel_throttle->get_control_in()) {
        return false;
    }
    
    // Set up servos, which should be connected to CH_1 and CH_2
    if (!(SRV_Channels::set_aux_channel_default(SRV_Channel::k_tilt_front_motors , CH_1))
        || !(SRV_Channels::set_aux_channel_default(SRV_Channel::k_tilt_back_motors , CH_2))) {
        return false;
    }
    // Set servos to ground mode angle
    SRV_Channels::set_angle(SRV_Channel::k_tilt_front_motors, SERVO_OUTPUT_RANGE);
    SRV_Channels::set_angle(SRV_Channel::k_tilt_back_motors, SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_front_motors, -SERVO_OUTPUT_RANGE/2);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_back_motors, -SERVO_OUTPUT_RANGE/2);
    
    armed = false;
    lastState = NO_THROTTLE;
    currState = NO_THROTTLE;
    servoState = TILT_INWARDS;
    
    // Disable throttle failsafe
    g.failsafe_throttle = FS_THR_DISABLED;
    g.failsafe_gcs = FS_GCS_DISABLED;
    
    return true;
}

// ground_run - runs the main ground controller
// should be called at 100hz or more
void ModeGround::run()
{
    // If a new command hasn't been received, return
    if (!copter.ap.new_radio_frame) {
        return;
    }
    
    // Mark radio frame as consumed
    copter.ap.new_radio_frame = false;
    
    memset(motorsOutput, 0, 8);
    
    // Special inclination, while moving forward
    if (rc().channel(CH_7)->get_control_in() > 600) {
        if (servoState != TILT_FORWARD) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_front_motors, SERVO_OUTPUT_RANGE/2);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_back_motors, -SERVO_OUTPUT_RANGE/2);
            servoState = TILT_FORWARD;
        }
    // Special inclination, while moving backward
    } else if(rc().channel(CH_7)->get_control_in() < 400) {
        if (servoState != TILT_BACKWARDS) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_front_motors, -SERVO_OUTPUT_RANGE/2);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_back_motors, SERVO_OUTPUT_RANGE/2);
            servoState = TILT_BACKWARDS;
        }
    // Standard motors tilt while moving
    } else {
        if (servoState != TILT_INWARDS) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_front_motors, -SERVO_OUTPUT_RANGE/2);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_back_motors, -SERVO_OUTPUT_RANGE/2);
            servoState = TILT_INWARDS;
        }
    }
    
    // Motors should stop spinning if radio not detected, if not armed or
    // if throttle stick is down
    if ( copter.failsafe.radio || !copter.ap.rc_receiver_present || !motors->armed() 
        || !copter.channel_throttle->get_control_in()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        armed = false;
        lastState = NO_THROTTLE;
        currState = NO_THROTTLE;
        time = AP_HAL::millis();
        return;
    }
    
    if (!armed) {
        if (AP_HAL::millis() - time < ARMING_TIME) {
            return;
        }
        armed = true;
    }
    
    float throttle, yaw;
    
    throttle = copter.channel_pitch->norm_input_dz();
    yaw = copter.channel_yaw->norm_input_dz();
    
    // Check for saturation and scale back throttle and yaw proportionally
    /*const float saturationValue = fabsf(throttle) + fabsf(yaw);
    if (saturationValue > 1.0f) {
        yawScaled = yaw / saturationValue;
        throttleScaled = throttle / saturationValue;
    } else {*/
    //yawScaled = yaw;
    //throttleScaled = throttle;
    //}
    
    uint16_t throttlePWM = motors->get_pwm_output_max() - motors->get_pwm_output_min();
    uint16_t minThrottlePWM = (uint16_t)((float)(throttlePWM * fabsf(throttle) 
                                              * MAX_FLAT_THROTTLE));
    uint16_t yawPWM = (uint16_t)((float)(motors->get_pwm_output_max() - motors->get_pwm_output_min()) 
                  * MAX_TURN_THROTTLE * fabsf(yaw));
    
    // Value angle*100
    int32_t pitch = ahrs.pitch_sensor;
    
    // Reverse steering - default for Rover: "By default regular and skid-steering 
    // vehicles reverse their rotation direction when backing up"
    // Otherwise, don't change direction while reversing
    if(rc().channel(CH_8)->get_control_in() < 400){
        const float steeringDir = is_negative(throttle) ? 1.0 : -1.0;
        yaw = steeringDir * yaw;
    }
    
    // Check if we are moving on an inclined surface
    if(abs(pitch) >= MINIMUM_INCLINATION_ANGLE){
        if ((currState != INCLINED)) {
            if (currState != NO_THROTTLE) {
                lastState = currState;
            }
            currState = INCLINED;
        }
        if (((throttle > 0.0f) && (pitch < 0)) || ((throttle < 0.0f) && (pitch > 0))) {
            // Thrust depends on the square of throttle
            float factor = safe_sqrt((float)(abs(pitch) * MAX_INCLINED_THROTTLE / MAXIMUM_INCLINATION_ANGLE));
            if (factor > MAX_INCLINED_THROTTLE) {
                factor = MAX_INCLINED_THROTTLE;
            }
            throttlePWM = (uint16_t)((float)(throttlePWM * fabsf(throttle) * factor));
            if(throttlePWM < minThrottlePWM){
                throttlePWM = minThrottlePWM;
            }
        } else {
            throttlePWM = minThrottlePWM;
        }
    // Check if we want to break              
    } else if (AP_HAL::millis() - brakeTimer <= MAX_BRAKE_TIME) {
        if(currState == INCLINED){
            lastState = INCLINED;
            currState = FLAT;
        }
        if (((currState == FORWARD) && (lastState == BACKWARD)) ||
           ((currState == BACKWARD) && (lastState == FORWARD))) {
            throttlePWM = (uint16_t)((float)(throttlePWM * fabsf(throttle) 
                                            * MAX_BRAKE_THROTTLE));
        } else {
            throttlePWM = minThrottlePWM;
        }
    } else {
        if(currState == INCLINED){
            lastState = INCLINED;
            currState = FLAT;
        }
        throttlePWM = minThrottlePWM;
    }
    
    // Backward
    if (throttle > 0.0f) {
        if ((currState != BACKWARD) && (currState != INCLINED)) {
            if (currState != NO_THROTTLE) {
                lastState = currState;
            }
            currState = BACKWARD;
            // Sometimes, we can jump directly from forward to backward without 
            // passing by NO_THROTTLE state
            brakeTimer = AP_HAL::millis();
        }
        if(servoState != TILT_FORWARD){
            motorsOutput[0] = throttlePWM;
            motorsOutput[2] = throttlePWM;
        }
        if (servoState == TILT_BACKWARDS){
            motorsOutput[1] = throttlePWM;
            motorsOutput[3] = throttlePWM;
        }
    // Forward
    } else if (throttle < 0.0f) {
        if ((currState != FORWARD) && (currState != INCLINED)) {
            if (currState != NO_THROTTLE) {
                lastState = currState;
            }
            currState = FORWARD;
            // Sometimes, we can jump directly from forward to backward without 
            // passing by NO_THROTTLE state
            brakeTimer = AP_HAL::millis();
        }
        if(servoState != TILT_BACKWARDS){
            motorsOutput[1] = throttlePWM;
            motorsOutput[3] = throttlePWM;
        }
        if (servoState == TILT_FORWARD){
            motorsOutput[0] = throttlePWM;
            motorsOutput[2] = throttlePWM;
        }
    // No throttle being requested
    } else {
        // Could be added a no state timer, to avoid braking if state was no 
        // state for too long
        if ((currState != NO_THROTTLE) && (currState != INCLINED)){
            lastState = currState;
            currState = NO_THROTTLE;
        }
        brakeTimer = AP_HAL::millis();
    }
    
    // Clockwise Turn
    if (yaw > 0.0f) {
        if (servoState == TILT_INWARDS) {
            motorsOutput[0] += yawPWM;
            motorsOutput[1] += yawPWM;
        } else if (servoState == TILT_FORWARD){
            motorsOutput[1] += yawPWM;
            motorsOutput[2] += yawPWM;
        } else if (servoState == TILT_BACKWARDS){
            motorsOutput[0] += yawPWM;
            motorsOutput[3] += yawPWM;
        }
    // Counter-clockwise turn
    } else if (yaw < 0.0f) {
        if ( servoState == TILT_INWARDS) {
            motorsOutput[2] += yawPWM;
            motorsOutput[3] += yawPWM;
        } else if (servoState == TILT_FORWARD){
            motorsOutput[0] += yawPWM;
            motorsOutput[3] += yawPWM;
        } else if (servoState == TILT_BACKWARDS){
            motorsOutput[1] += yawPWM;
            motorsOutput[2] += yawPWM;
        }
    }
}

void ModeGround::output_to_motors()
{
    for (int8_t i = 0; i < 4; i++){
        uint16_t pwm = motors->get_pwm_output_min() + motorsOutput[i];
        
        if(pwm < MIN_THROTTLE){
            pwm = motors->get_pwm_output_min();
        } else if (pwm > MAX_THROTTLE) {
            pwm = MAX_THROTTLE;
        }
        
        motors->rc_write(AP_MOTORS_MOT_1 + i, pwm);
    }
}

void ModeGround::exit()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_front_motors, -SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_back_motors, -SERVO_OUTPUT_RANGE);
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    // Re-enable throtle failsafe
    g.failsafe_throttle.load();
    g.failsafe_gcs.load();
}

#endif