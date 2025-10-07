#include "Copter.h"

#if MODE_GUIDED_ENABLED



// init - initialise guided controller
bool ModeDrawStar::init(bool ignore_checks)
{
    // start in velaccel control mode
    wp_control_start();
    path_num= 0;
    genarate_path();
    return true;
}

// run - runs the guided controller
// should be called at 100hz or more
void ModeDrawStar::run()
{if (path_num < 6){
    if (wp_nav->reached_wp_destination()){
        path_num ++;
        wp_nav->set_wp_destination_NEU_m(path[path_num], false);

    }
}
    // run pause control if the vehicle is paused
    wp_control_run();
 }
void ModeDrawStar::genarate_path()
{
    float radius_m = 10.0;

    wp_nav->get_wp_stopping_point_NEU_m(path[0]);
    path[1] = path[0]+ Vector3p(1.0f , 0, 0)* radius_m;
    path[2] = path[0]+ Vector3p(-cosf(radians(36.0f)) , -sinf(radians(36.0f)), 0)* radius_m;
    path[3] = path[0]+ Vector3p(sinf(radians(18.0f)) , cosf(radians(18.0f)), 0)* radius_m;
    path[4] = path[0]+ Vector3p(sinf(radians(18.0f)) , -cosf(radians(18.0f)), 0)* radius_m;
    path[5] = path[0]+ Vector3p(-cosf(radians(36.0f)) , sinf(radians(36.0f)), 0)* radius_m;
    path[6] = path[1];

}

void ModeDrawStar::wp_control_run()
{
    
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // set motors to full range
    
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    // run waypoint controller
    
    // call z-axis position controller (wpnav should have already updated it's alt target)
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());
    pos_control->update_U_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

ModeDrawStar::SubMode ModeDrawStar::guided_mode = SubMode::TakeOff;

void ModeDrawStar::wp_control_start()
{
    // set to position control mode
    guided_mode = SubMode::WP;

    // initialise waypoint and spline controller
    wp_nav->wp_and_spline_init_m();

    // initialise wpnav to stopping point
    wp_nav->set_wp_destination_NEU_m(path[0]);

    // initialise yaw
    auto_yaw.set_mode_to_default(false);
}

bool ModeDrawStar::allows_arming(AP_Arming::Method method) const
{
    // always allow arming from the ground station or scripting
    if (AP_Arming::method_is_GCS(method) || method == AP_Arming::Method::SCRIPTING) {
        return true;
    }

    // optionally allow arming from the transmitter
    return option_is_enabled(Option::AllowArmingFromTX);
};

bool ModeDrawStar::option_is_enabled(Option option) const
{
    return (copter.g2.guided_options.get() & (uint32_t)option) != 0;
}
#endif
