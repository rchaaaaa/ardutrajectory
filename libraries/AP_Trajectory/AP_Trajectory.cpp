#include "AP_Trajectory.h"

extern const AP_HAL::HAL& hal;

bool AP_Trajectory::read_segment_from_storage(uint16_t index, Trajectory::FlightState::Segment &segment) const{
    return true;
}
bool AP_Trajectory::read_rotation_from_storage(uint16_t index, Trajectory::States::TrajAndRotsSingleSeq::RotMsg &rotation) const{
    return true;
}

void AP_Trajectory::update()
{
    // if (_flags.state != TRAJECTORY_RUNNING || _trajectory_total == 0) {
    //     return;
    // }

    // // save persistent waypoint_num for watchdog restore
    // hal.util->persistent_data.trajectory_num = _trajectory.index;

    // // check if we have an active nav command
    // if (!_flags.nav_cmd_loaded || _nav_cmd.index == AP_MISSION_CMD_INDEX_NONE) {
    //     // advance in mission if no active nav command
    //     if (!advance_current_nav_cmd()) {
    //         // failure to advance nav command means mission has completed
    //         complete();
    //         return;
    //     }
    // }else{
    //     // run the active nav command
    //     if (verify_command(_nav_cmd)) {
    //         // market _nav_cmd as complete (it will be started on the next iteration)
    //         _flags.nav_cmd_loaded = false;
    //         // immediately advance to the next mission command
    //         if (!advance_current_nav_cmd()) {
    //             // failure to advance nav command means mission has completed
    //             complete();
    //             return;
    //         }
    //     }
    // }

    // // check if we have an active do command
    // if (!_flags.do_cmd_loaded) {
    //     advance_current_do_cmd();
    // }else{
    //     // check the active do command
    //     if (verify_command(_do_cmd)) {
    //         // mark _do_cmd as complete
    //         _flags.do_cmd_loaded = false;
    //     }
    // }
}