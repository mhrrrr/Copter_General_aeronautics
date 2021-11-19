#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

class AP_BattMonitor_GA_MAVlink : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_GA_MAVlink(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override { return has_current(); }

    /// returns true if battery monitor provides current info
    bool has_current() const override;

    void init(void) override {}

    // handle mavlink GA_MAVLINK messages
    void handle_ga_mavlink_msg(int voltage, int current, int mah) override;

protected:
    float batt_voltage=0.0f;
    float batt_current=0.0f;
    float batt_consumed_mah=0.0f;
};