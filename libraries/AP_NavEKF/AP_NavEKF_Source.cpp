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
    AP_GROUPINFO("POSXY", 1, AP_NavEKF_Source, _posxy_source1, 1),

    // @Param: VELXY
    // @DisplayName: Velocity Horizontal Source
    // @Description: Velocity Horizontal Source
    // @Values: 0:None, 1:GPS, 2:Beacon, 3:OpticalFlow, 4:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("VELXY", 2, AP_NavEKF_Source, _velxy_source1, 1),

    // @Param: POSZ
    // @DisplayName: Position Vertical Source
    // @Description: Position Vertical Source
    // @Values: 0:None, 1:Baro, 2:RangeFinder, 3:GPS, 4:Beacon, 5:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("POSZ", 3, AP_NavEKF_Source, _posz_source1, 1),

    // @Param: VELZ
    // @DisplayName: Velocity Vertical Source
    // @Description: Velocity Vertical Source
    // @Values: 0:None, 1:Baro, 2:RangeFinder, 3:GPS, 4:Beacon, 5:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("VELZ", 4, AP_NavEKF_Source, _velz_source1, 3),

    // @Param: POSXY2
    // @DisplayName: Position Horizontal Source (Secondary)
    // @Description: Position Horizontal Source (Secondary)
    // @Values: 0:None, 1:GPS, 2:Beacon, 4:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("POSXY2", 6, AP_NavEKF_Source, _posxy_source2, 0),

    // @Param: VELXY2
    // @DisplayName: Velocity Horizontal Source (Secondary)
    // @Description: Velocity Horizontal Source (Secondary)
    // @Values: 0:None, 1:GPS, 2:Beacon, 3:OpticalFlow, 4:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("VELXY2", 7, AP_NavEKF_Source, _velxy_source2, 1),

    // @Param: POSZ2
    // @DisplayName: Position Vertical Source (Secondary)
    // @Description: Position Vertical Source (Secondary)
    // @Values: 0:None, 1:Baro, 2:RangeFinder, 3:GPS, 4:Beacon, 5:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("POSZ2", 8, AP_NavEKF_Source, _posz_source2, 1),

    // @Param: VELZ2
    // @DisplayName: Velocity Vertical Source (Secondary)
    // @Description: Velocity Vertical Source (Secondary)
    // @Values: 0:None, 1:Baro, 2:RangeFinder, 3:GPS, 4:Beacon, 5:ExternalNav
    // @User: Advanced
    AP_GROUPINFO("VELZ2", 9, AP_NavEKF_Source, _velz_source2, 3),

    AP_GROUPEND
};

AP_NavEKF_Source::AP_NavEKF_Source()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_NavEKF_Source::init()
{
    // ensure init is only run once
    if (_initialised) {
        return;
    }

    // initialise active sources
    _active_posxy_source = (SourceXY)_posxy_source1.get();
    _active_velxy_source = (SourceXY)_velxy_source1.get();
    _active_posz_source = (SourceZ)_posz_source1.get();
    _active_velz_source = (SourceZ)_velz_source1.get();

    _initialised = true;
}

// set position source to either 0=primary or 1=secondary
void AP_NavEKF_Source::setPosVelXYZSource(uint8_t source_idx)
{
    // ensure init has been run
    init();

    _active_posxy_source = (source_idx == 1 ? (SourceXY)_posxy_source2.get() : (SourceXY)_posxy_source1.get());
    _active_velxy_source = (source_idx == 1 ? (SourceXY)_velxy_source2.get() : (SourceXY)_velxy_source1.get());
    _active_posz_source = (source_idx == 1 ? (SourceZ)_posz_source2.get() : (SourceZ)_posz_source1.get());
    _active_velz_source = (source_idx == 1 ? (SourceZ)_velz_source2.get() : (SourceZ)_velz_source1.get());
}

// sensor specific helper functions
bool AP_NavEKF_Source::usingGPS() const
{
    return getPosXYSource() == SourceXY::GPS ||
           getPosZSource() == SourceZ::GPS ||
           getVelXYSource() == SourceXY::GPS ||
           getVelZSource() == SourceZ::GPS;
}
