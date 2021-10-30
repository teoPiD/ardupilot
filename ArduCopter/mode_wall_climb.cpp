#include "Copter.h"

#if MODE_WALL_CLIMB_ENABLED == ENABLED

#define SERVO_OUTPUT_RANGE 18000
#define MIN_SPIN 1025
#define MAX_SPIN 1800
#define ARMING_TIME 4000

/*
 * Init and run calls for wall climb mode
 */

// Wall climb_init
bool ModeWallClimb::init(bool ignore_checks)
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
    // Set servos to wall climb angle
    SRV_Channels::set_angle(SRV_Channel::k_tilt_front_motors, SERVO_OUTPUT_RANGE);
    SRV_Channels::set_angle(SRV_Channel::k_tilt_back_motors, SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_front_motors, (int16_t)((float)(SERVO_OUTPUT_RANGE*0.25)));
    SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_back_motors, (int16_t)((float)(-SERVO_OUTPUT_RANGE*0.25)));
    
    armed = false;
    servoState = TILT_135;
    
    // Disable throttle failsafe
    g.failsafe_throttle = FS_THR_DISABLED;
    g.failsafe_gcs = FS_GCS_DISABLED;
    
    return true;
}

// wall_climb_run - runs the main wall climb controller
// should be called at 100hz or more
void ModeWallClimb::run()
{
    // If a new command hasn't been received, return
    if (!copter.ap.new_radio_frame) {
        return;
    }
    
    // Mark radio frame as consumed
    copter.ap.new_radio_frame = false;
    
    memset(motorsOutput, 0, 8);
    
    if (rc().channel(CH_7)->get_control_in() < 400) {
        if (servoState != TILT_115) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_front_motors, (int16_t)((float)(SERVO_OUTPUT_RANGE*0.3611)));
            SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_back_motors, (int16_t)((float)(-SERVO_OUTPUT_RANGE*0.3611)));
            servoState = TILT_115;
        }
    } else if(rc().channel(CH_7)->get_control_in() > 600) {
        if (servoState != TILT_135) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_front_motors, (int16_t)((float)(SERVO_OUTPUT_RANGE*0.25)));
            SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_back_motors, (int16_t)((float)(-SERVO_OUTPUT_RANGE*0.25)));
            servoState = TILT_135;
        }
    } else {
        if (servoState != TILT_125) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_front_motors, (int16_t)((float)(SERVO_OUTPUT_RANGE*0.3056)));
            SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_back_motors, (int16_t)((float)(-SERVO_OUTPUT_RANGE*0.3056)));
            servoState = TILT_125;
        }
    }
    
    // Motors should stop spinning if radio not detected, if not armed or
    // if throttle stick is down
    if ( copter.failsafe.radio || !copter.ap.rc_receiver_present || !motors->armed() 
        || !copter.channel_throttle->get_control_in()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        armed = false;
        time = AP_HAL::millis();
        return;
    }
    
    if (!armed) {
        if (AP_HAL::millis() - time < ARMING_TIME) {
            return;
        }
        armed = true;
    }
    
    float throttle;
    
    throttle = copter.channel_pitch->norm_input_dz();
    
    uint16_t throttlePWM = (uint16_t)((float)(motors->get_pwm_output_max() - motors->get_pwm_output_min())*fabsf(throttle));

    
    if (throttle > 0.0f || throttle < 0.0f) {
            motorsOutput[0] = throttlePWM;
            motorsOutput[1] = throttlePWM;
            motorsOutput[2] = throttlePWM;
            motorsOutput[3] = throttlePWM;
    }    
}

void ModeWallClimb::output_to_motors()
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

void ModeWallClimb::exit()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_front_motors, -SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tilt_back_motors, -SERVO_OUTPUT_RANGE);
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    // Re-enable throtle failsafe
    g.failsafe_throttle.load();
    g.failsafe_gcs.load();
}

#endif