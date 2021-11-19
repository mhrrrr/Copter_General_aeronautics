#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_GA_MAVlink.h"

extern const AP_HAL::HAL& hal;

/// Constructor
AP_BattMonitor_GA_MAVlink::AP_BattMonitor_GA_MAVlink(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    // always healthy
    _state.healthy = true;
}

// read - read the voltage and current
void AP_BattMonitor_GA_MAVlink::read()
{
    // calculate time since last current read
    uint32_t tnow = AP_HAL::micros();
    float dt = tnow - _state.last_time_micros;

    // get voltage
    _state.voltage = batt_voltage;

    // read current
    if (has_current()) {
        // read current
        _state.current_amps = batt_current;
        _state.consumed_mah = batt_consumed_mah;

        // update total energy drawn since startup
        if (_state.last_time_micros != 0 && dt < 2000000.0f) {
            // .0002778 is 1/3600 (conversion to hours)
            float mah = _state.current_amps * dt * 0.0000002778f;
            _state.consumed_wh  += 0.001f * mah * _state.voltage;
        }

        // record time
        _state.last_time_micros = tnow;
    }
}

/// return true if battery provides current info
bool AP_BattMonitor_GA_MAVlink::has_current() const
{
    return true;
}

void AP_BattMonitor_GA_MAVlink::handle_ga_mavlink_msg(int voltage, int current, int mah) 
{
    batt_voltage=voltage/10.;
    batt_current=current/10.;
    batt_consumed_mah=(int)mah;
}