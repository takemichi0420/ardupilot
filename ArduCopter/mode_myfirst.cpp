#include "Copter.h"

#if MODE_MYFIRST_ENABLED
/*
 * Init and run calls for MyFirst flight mode
 */

// myfirst_run - runs the main MyFirst controller
// should be called at 100hz or more
void ModeMyfirst::run()
{
    // apply simple mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    float target_roll_rad, target_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->lean_angle_max_rad());
    myfirst_function(); // Call the new function for MyFirst mode
    target_roll_rad = target_roll_rad * 0.5f; // Example modification for MyFirst mode
    target_pitch_rad = target_pitch_rad * 0.5f; // Example modification for MyFirst mode

    // get pilot's desired yaw rate
    float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

    // Determine desired spool state based on pilot throttle input.
    // The setter enforces that disarmed aircraft are held at SHUT_DOWN until armed.
    if (copter.ap.throttle_zero) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    float pilot_desired_throttle = get_pilot_desired_throttle();

    switch (motors->get_spool_state()) {
    case AP_Motors::SpoolState::SHUT_DOWN:
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::GROUND_IDLE:
        // Landed
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pilot_desired_throttle = 0.0f;
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        // clear landing flag above zero throttle
        if (!motors->limit.throttle_lower) {
            set_land_complete(false);
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(target_roll_rad, target_pitch_rad, target_yaw_rate_rads);

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);
}


//myfirst_function
void ModeMyfirst::myfirst_function() {
    // Example implementation of a new function for MyFirst mode
    // This function can be used to implement custom behavior specific to MyFirst mode
    // For demonstration purposes, we'll just log a message
    if (myfirst_variable >= 400) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "My First Function");
        myfirst_variable = 0;
    }
    myfirst_variable++; // Increment the variable for demonstration
}

#endif
