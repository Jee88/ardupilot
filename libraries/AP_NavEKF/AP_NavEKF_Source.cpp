/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_NavEKF_Source.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_NavEKF_Source::var_info[] = {

    // @Param: POSXY
    // @DisplayName: Position Horizontal Source (Primary)
    // @Description: Position Horizontal Source (Primary)
    // @Values: 0:None, 1:GPS, 2:Beacon, 4:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("POSXY", 1, AP_NavEKF_Source, _source[0].posxy, (int8_t)AP_NavEKF_Source::SourceXY::GPS),

    // @Param: VELXY
    // @DisplayName: Velocity Horizontal Source
    // @Description: Velocity Horizontal Source
    // @Values: 0:None, 1:GPS, 2:Beacon, 3:OpticalFlow, 4:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("VELXY", 2, AP_NavEKF_Source, _source[0].velxy, (int8_t)AP_NavEKF_Source::SourceXY::GPS),

    // @Param: POSZ
    // @DisplayName: Position Vertical Source
    // @Description: Position Vertical Source
    // @Values: 0:None, 1:Baro, 2:RangeFinder, 3:GPS, 4:Beacon, 5:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("POSZ", 3, AP_NavEKF_Source, _source[0].posz, (int8_t)AP_NavEKF_Source::SourceZ::BARO),

    // @Param: VELZ
    // @DisplayName: Velocity Vertical Source
    // @Description: Velocity Vertical Source
    // @Values: 0:None, 3:GPS, 4:Beacon, 5:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("VELZ", 4, AP_NavEKF_Source, _source[0].velz, (int8_t)AP_NavEKF_Source::SourceZ::GPS),

    // @Param: YAW
    // @DisplayName: Yaw Source
    // @Description: Yaw Source
    // @Values: 0:None, 1:Compass, 2:External, 3:External with Compass Fallback
    // @User: Advanced
    AP_GROUPINFO("YAW", 5, AP_NavEKF_Source, _source[0].yaw, (int8_t)AP_NavEKF_Source::SourceYaw::COMPASS),

    // @Param: POSXY2
    // @DisplayName: Position Horizontal Source (Secondary)
    // @Description: Position Horizontal Source (Secondary)
    // @Values: 0:None, 1:GPS, 2:Beacon, 4:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("POSXY2", 6, AP_NavEKF_Source, _source[1].posxy, (int8_t)AP_NavEKF_Source::SourceXY::GPS),

    // @Param: VELXY2
    // @DisplayName: Velocity Horizontal Source (Secondary)
    // @Description: Velocity Horizontal Source (Secondary)
    // @Values: 0:None, 1:GPS, 2:Beacon, 3:OpticalFlow, 4:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("VELXY2", 7, AP_NavEKF_Source, _source[1].velxy, (int8_t)AP_NavEKF_Source::SourceXY::GPS),

    // @Param: POSZ2
    // @DisplayName: Position Vertical Source (Secondary)
    // @Description: Position Vertical Source (Secondary)
    // @Values: 0:None, 1:Baro, 2:RangeFinder, 3:GPS, 4:Beacon, 5:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("POSZ2", 8, AP_NavEKF_Source, _source[1].posz, (int8_t)AP_NavEKF_Source::SourceZ::BARO),

    // @Param: VELZ2
    // @DisplayName: Velocity Vertical Source (Secondary)
    // @Description: Velocity Vertical Source (Secondary)
    // @Values: 0:None, 3:GPS, 4:Beacon, 5:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("VELZ2", 9, AP_NavEKF_Source, _source[1].velz, (int8_t)AP_NavEKF_Source::SourceZ::GPS),

    // @Param: YAW2
    // @DisplayName: Yaw Source (Secondary)
    // @Description: Yaw Source (Secondary)
    // @Values: 0:None, 1:Compass, 2:External, 3:External with Compass Fallback
    // @User: Advanced
    AP_GROUPINFO("YAW2", 10, AP_NavEKF_Source, _source[1].yaw, (int8_t)AP_NavEKF_Source::SourceYaw::COMPASS),

    AP_GROUPEND
};

AP_NavEKF_Source::AP_NavEKF_Source()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_NavEKF_Source::init()
{
    // ensure init is only run once
    if (_active_source.initialised) {
        return;
    }

    // initialise active sources
    _active_source.posxy = (SourceXY)_source[0].posxy.get();
    _active_source.velxy = (SourceXY)_source[0].velxy.get();
    _active_source.posz = (SourceZ)_source[0].posz.get();
    _active_source.velz = (SourceZ)_source[0].velz.get();
    _active_source.yaw = (SourceYaw)_source[0].yaw.get();

    _active_source.initialised = true;
}

// set position source to either 0=primary or 1=secondary
void AP_NavEKF_Source::setPosVelXYZSource(uint8_t source_idx)
{
    // ensure init has been run
    init();

    _active_source.posxy = (source_idx == 1 ? (SourceXY)_source[1].posxy.get() : (SourceXY)_source[0].posxy.get());
    _active_source.velxy = (source_idx == 1 ? (SourceXY)_source[1].velxy.get() : (SourceXY)_source[0].velxy.get());
    _active_source.posz = (source_idx == 1 ? (SourceZ)_source[1].posz.get() : (SourceZ)_source[0].posz.get());
    _active_source.velz = (source_idx == 1 ? (SourceZ)_source[1].velz.get() : (SourceZ)_source[0].velz.get());
    _active_source.yaw = (source_idx == 1 ? (SourceYaw)_source[1].yaw.get() : (SourceYaw)_source[0].yaw.get());
}

// sensor specific helper functions
bool AP_NavEKF_Source::usingGPS() const
{
    return getPosXYSource() == SourceXY::GPS ||
           getPosZSource() == SourceZ::GPS ||
           getVelXYSource() == SourceXY::GPS ||
           getVelZSource() == SourceZ::GPS;
}

// true if some parameters have been configured (used during parameter conversion)
bool AP_NavEKF_Source::params_configured_in_storage() const
{
    return _source[0].posxy.configured_in_storage() ||
           _source[0].velxy.configured_in_storage() ||
           _source[0].posz.configured_in_storage() ||
           _source[0].velz.configured_in_storage() ||
           _source[0].yaw.configured_in_storage();
}
