#include "Copter.h"

#if MODE_GROUND_ENABLED == ENABLED

#define SERVO_OUTPUT_RANGE 18000
#define MAX_POWER 0.3
#define MIN_SPIN 1100

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
    
    // Disable throttle failsafe
    g.failsafe_throttle = FS_THR_DISABLED;
    
    return true;
}

// ground_run - runs the main ground controller
// should be called at 100hz or more
void ModeGround::run()
{
    // If a new command hasn't been received, return
    /*if (!copter.ap.new_radio_frame) {
        return;
    }
    
    // Mark radio frame as consumed
    copter.ap.new_radio_frame = false;
    
    float throttle;//, yaw;
    
    throttle = channel_pitch->norm_input_dz();
    //yaw = channel_yaw->norm_input_dz();
    */
    // Motors should stop spinning if radio not detected or if not armed
    /*if(copter.failsafe.radio || !copter.ap.rc_receiver_present || !motors->armed()){
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        return;
    }*/
    /*
    int16_t pwm = motors->get_pwm_output_min()
        + ((motors->get_pwm_output_max() - motors->get_pwm_output_min()) * MAX_POWER * fabsf(throttle));
    pwm = (pwm < MIN_SPIN) ? MIN_SPIN : pwm;
    if(copter.failsafe.radio || !copter.ap.rc_receiver_present || !motors->armed()){
        pwm = 0;
    }
    
    if (throttle > 0.0f) {
        motors->rc_write(AP_MOTORS_MOT_1, 1600);
        motors->rc_write(AP_MOTORS_MOT_3, 1600);
        motors->rc_write(AP_MOTORS_MOT_2, motors->get_pwm_output_min());
        motors->rc_write(AP_MOTORS_MOT_4, motors->get_pwm_output_min());
    } else if (throttle < 0.0f) {
        motors->rc_write(AP_MOTORS_MOT_2, pwm);
        motors->rc_write(AP_MOTORS_MOT_4, pwm);
        motors->rc_write(AP_MOTORS_MOT_1, motors->get_pwm_output_min());
        motors->rc_write(AP_MOTORS_MOT_3, motors->get_pwm_output_min());
    } else {
        motors->rc_write(AP_MOTORS_MOT_1, motors->get_pwm_output_min());
        motors->rc_write(AP_MOTORS_MOT_2, motors->get_pwm_output_min());
        motors->rc_write(AP_MOTORS_MOT_3, motors->get_pwm_output_min());
        motors->rc_write(AP_MOTORS_MOT_4, motors->get_pwm_output_min());
    }
    */
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
    motors->rc_write(AP_MOTORS_MOT_3, 1500);
}

void ModeGround::exit()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, -SERVO_OUTPUT_RANGE);
    // Re-enable throtle failsafe
    g.failsafe_throttle.load();
}

#endif