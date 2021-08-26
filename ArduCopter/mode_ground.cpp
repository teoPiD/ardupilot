#include "Copter.h"

#if MODE_GROUND_ENABLED == ENABLED

#define SERVO_OUTPUT_RANGE 18000
#define MAX_SPEED_POWER 0.2
#define MAX_TURN_POWER 0.6
#define MIN_SPIN 1025
#define MAX_SPIN 1700

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
    
    float throttle, yaw;
    
    throttle = channel_pitch->norm_input_dz();
    yaw = channel_yaw->norm_input_dz();
    
    for (int i = 0; i < 4; i++){
        motors_output[i] = 0;
    }
    
    // Motors should stop spinning if radio not detected or if not armed
    if(copter.failsafe.radio || !copter.ap.rc_receiver_present || !motors->armed()){
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        armed = false;
        time = AP_HAL::millis();
        return;
    }
    
    if (!armed) {
        if (AP_HAL::millis() - time < 5000) {
            return;
        }
        armed = true;
    }
    
    int16_t throttle_pwm = ((motors->get_pwm_output_max() - motors->get_pwm_output_min()) 
                  * MAX_SPEED_POWER * fabsf(throttle));
    
    if (throttle > 0.0f) {
        motors_output[0] = throttle_pwm;
        motors_output[2] = throttle_pwm;
    } else if (throttle < 0.0f) {
        motors_output[1] = throttle_pwm;
        motors_output[3] = throttle_pwm;
    }
    
    int16_t yaw_pwm = ((motors->get_pwm_output_max() - motors->get_pwm_output_min()) 
                  * MAX_TURN_POWER * fabsf(yaw));
                  
    if (yaw > 0.0f) {
        motors_output[0] += yaw_pwm;
        motors_output[1] += yaw_pwm;
    } else if (yaw < 0.0f) {
        motors_output[2] += yaw_pwm;
        motors_output[3] += yaw_pwm;
    }
    
    
    /*switch (motors->get_spool_state()) {

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
            if (throttle > 0.0f) {
                motors->rc_write(AP_MOTORS_MOT_1, pwm);
                motors->rc_write(AP_MOTORS_MOT_3, pwm);
                motors->rc_write(AP_MOTORS_MOT_2, motors->get_pwm_output_min());
                motors->rc_write(AP_MOTORS_MOT_4, motors->get_pwm_output_min());
            } else if (throttle < 0.0f) {
                motors->rc_write(AP_MOTORS_MOT_2, pwm);
                motors->rc_write(AP_MOTORS_MOT_4, pwm);
                motors->rc_write(AP_MOTORS_MOT_1, motors->get_pwm_output_min());
                motors->rc_write(AP_MOTORS_MOT_3, motors->get_pwm_output_min());
            } else {
                motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            }
            break;

        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // Do nothing
            break;
    }*/
}

void ModeGround::output_to_motors()
{
    for (int i = 0; i < 4; i++){
        int16_t pwm = motors->get_pwm_output_min() + motors_output[i];
        
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