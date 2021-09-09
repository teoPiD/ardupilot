#include "Copter.h"

#if MODE_GROUND_ENABLED == ENABLED

#define SERVO_OUTPUT_RANGE 18000
#define MAX_SPEED_NO_BRAKE_POWER 0.2
#define MAX_SPEED_BRAKE_POWER 0.4
#define MAX_TURN_POWER 0.6
#define BRAKE_MAX_TIME 2000
#define MIN_SPIN 1025
#define MAX_SPIN 1700
#define ARMING_TIME 4000

/*
 * Init and run calls for ground locomotion mode
 */

// ground_init - initialise ground controller
bool ModeGround::init(bool ignore_checks)
{
    // Don't enter the mode if sticks are not centered
    if (!is_zero(channel_pitch->norm_input_dz()) 
        || !is_zero(channel_roll->norm_input_dz())
        || !is_zero(channel_yaw->norm_input_dz())) {
        return false;
    }
    
    // Set up servos, which should be connected to CH_1 and CH_2
    if (!(SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor_tilt , CH_1))
        || !(SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor_tilt , CH_2))) {
        return false;
    }
    // Set servos to ground mode angle
    SRV_Channels::set_angle(SRV_Channel::k_motor_tilt, SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, -SERVO_OUTPUT_RANGE/2);
    
    armed = false;
    lastState = NO_THROTTLE;
    currState = NO_THROTTLE;
    
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
    
    float throttle, throttleScaled, yaw, yawScaled;
    
    throttle = copter.channel_pitch->norm_input_dz();
    yaw = copter.channel_yaw->norm_input_dz();
    
    for (int i = 0; i < 4; i++){
        motorsOutput[i] = 0;
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
    
    // Check for saturation and scale back throttle and yaw proportionally
    const float saturationValue = fabsf(throttle) + fabsf(yaw);
    if (saturationValue > 1.0f) {
        yawScaled = yaw / saturationValue;
        throttleScaled = throttle / saturationValue;
    } else {
        yawScaled = yaw;
        throttleScaled = throttle;
    }
    
    uint16_t throttlePWM = motors->get_pwm_output_max() - motors->get_pwm_output_min();
    uint16_t yawPWM = (uint16_t)((float)(motors->get_pwm_output_max() - motors->get_pwm_output_min()) 
                  * MAX_TURN_POWER * fabsf(yawScaled));
    
    // Reverse steering - default for Rover: " By default regular and skid-steering 
    // vehicles reverse their rotation direction when backing up"
    // Otherwise, Don0t change direction while reversing
    if(rc().channel(CH_7)->get_control_in() > 1800){
        const float steeringDir = is_negative(throttleScaled) ? -1.0 : 1.0;
        yawScaled = steeringDir * yawScaled;
    }
    
    // Check if we want to break              
    if (AP_HAL::millis() - brakeTimer <= BRAKE_MAX_TIME) {
        if (((currState == FORWARD) && (lastState == BACKWARD)) ||
           ((currState == BACKWARD) && (lastState == FORWARD))) {
            throttlePWM = (uint16_t)((float)(throttlePWM * fabsf(throttleScaled) 
                                            * MAX_SPEED_BRAKE_POWER));
        } else {
            throttlePWM = (uint16_t)((float)(throttlePWM * fabsf(throttleScaled) 
                                            * MAX_SPEED_NO_BRAKE_POWER));
        }
    } else {
        throttlePWM = (uint16_t)((float)(throttlePWM * fabsf(throttleScaled) 
                                        * MAX_SPEED_NO_BRAKE_POWER));
    }
    
    // Backward
    if (throttleScaled > 0.0f) {
        if (currState != BACKWARD) {
            if (currState != NO_THROTTLE) {
                lastState = currState;
            }
            currState = BACKWARD;
            // Sometimes, we can jump directly from forward to backward without 
            // passing by NO_THROTTLE state
            brakeTimer = AP_HAL::millis();
        }
        motorsOutput[0] = throttlePWM;
        motorsOutput[2] = throttlePWM;
    // Forward
    } else if (throttleScaled < 0.0f) {
        if (currState != FORWARD) {
            if (currState != NO_THROTTLE) {
                lastState = currState;
            }
            currState = FORWARD;
            // Sometimes, we can jump directly from forward to backward without 
            // passing by NO_THROTTLE state
            brakeTimer = AP_HAL::millis();
        }
        motorsOutput[1] = throttlePWM;
        motorsOutput[3] = throttlePWM;
    // No throttle being requested
    } else {
        if (currState != NO_THROTTLE){
            lastState = currState;
            currState = NO_THROTTLE;
        }
        brakeTimer = AP_HAL::millis();
    }
                  
    if (yawScaled > 0.0f) {
        motorsOutput[0] += yawPWM;
        motorsOutput[1] += yawPWM;
    } else if (yawScaled < 0.0f) {
        motorsOutput[2] += yawPWM;
        motorsOutput[3] += yawPWM;
    }
}

void ModeGround::output_to_motors()
{
    for (int8_t i = 0; i < 4; i++){
        uint16_t pwm = motors->get_pwm_output_min() + motorsOutput[i];
        
        if(pwm < MIN_SPIN){
            pwm = motors->get_pwm_output_min();
        } else if (pwm > MAX_SPIN) {
            pwm = MAX_SPIN;
        }
        
        motors->rc_write(AP_MOTORS_MOT_1 + i, pwm);
    }
}

void ModeGround::exit()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, -SERVO_OUTPUT_RANGE);
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    // Re-enable throtle failsafe
    g.failsafe_throttle.load();
    g.failsafe_gcs.load();
}

#endif