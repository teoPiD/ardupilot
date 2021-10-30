#include "Copter.h"

#if MODE_TEST_ENABLED == ENABLED

#define ARMING_TIME 4000
#define ZERO_THROTTLE_TIME 3000
#define THROTTLE_TIME 5000

/*
 * Init and run calls for the static thrust tests
 */

// Test_init - initialise 
bool ModeTest::init(bool ignore_checks)
{
    // Don't enter the mode if sticks are not centered
    if (!is_zero(channel_pitch->norm_input_dz()) 
        || !is_zero(channel_roll->norm_input_dz())
        || !is_zero(channel_yaw->norm_input_dz())) {
        return false;
    }
    
    armed = false;
    counter = 0;
    
    // Disable throttle failsafe
    g.failsafe_throttle = FS_THR_DISABLED;
    g.failsafe_gcs = FS_GCS_DISABLED;
    
    return true;
}

// Test run - runs the test
void ModeTest::run()
{
    // If a new command hasn't been received, return
    if (!copter.ap.new_radio_frame) {
        return;
    }
    
    // Mark radio frame as consumed
    copter.ap.new_radio_frame = false;
    
    motorOutput = 0;
    
    // Motors should stop spinning if radio not detected, if not armed or
    // if throttle stick is down
    if ( copter.failsafe.radio || !copter.ap.rc_receiver_present || !motors->armed()) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
        armed = false;
        counter = 0;
        time = AP_HAL::millis();
        motorTimer = AP_HAL::millis();
        return;
    }
    
    if (!armed) {
        if (AP_HAL::millis() - time < ARMING_TIME) {
            return;
        }
        armed = true;
        motorTimer = AP_HAL::millis();
        counter = 0;
    }
    
    if (counter > 20) {
        return;
    }
    
    motorOutput = motors->get_pwm_output_min() + (counter*50);
    
    if (AP_HAL::millis() - motorTimer >= THROTTLE_TIME) {
        counter++;
        motorTimer = AP_HAL::millis();
        return;
    }

}

void ModeTest::output_to_motors()
{
    if(motorOutput < motors->get_pwm_output_min()){
        motorOutput = motors->get_pwm_output_min();
    }else if(motorOutput > motors->get_pwm_output_max()){
        motorOutput = motors->get_pwm_output_max();
    }
    motors->rc_write(AP_MOTORS_MOT_1, motorOutput);
}

void ModeTest::exit()
{
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    // Re-enable throtle failsafe
    g.failsafe_throttle.load();
    g.failsafe_gcs.load();
}

#endif