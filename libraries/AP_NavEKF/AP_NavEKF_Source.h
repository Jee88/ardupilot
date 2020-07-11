#pragma once

#pragma GCC optimize("O2")

#include <AP_Param/AP_Param.h>

class AP_NavEKF_Source
{

public:
    // Constructor
    AP_NavEKF_Source();

    /* Do not allow copies */
    AP_NavEKF_Source(const AP_NavEKF_Source &other) = delete;
    AP_NavEKF_Source &operator=(const AP_NavEKF_Source&) = delete;

    enum class SourceXY {
        NONE = 0,
        GPS = 1,
        BEACON = 2,
        OPTFLOW = 3,
        EXTNAV = 4
    };

    enum class SourceZ {
        NONE = 0,
        BARO = 1,
        RANGEFINDER = 2,
        GPS = 3,
        BEACON = 4,
        EXTNAV = 5
    };

    enum class SourceYaw {
        NONE = 0,
        COMPASS = 1,
        EXTERNAL = 2,
        EXTERNAL_COMPASS_FALLBACK = 3
    };

    // initialisation
    void init();

    // get/set current position source
    SourceXY getPosXYSource() const { return _initialised ? _active_posxy_source : (SourceXY)_posxy_source1.get(); }
    SourceZ getPosZSource() const { return _initialised ? _active_posz_source : (SourceZ)_posz_source1.get() ; }

    // set position and velocity sources to either 0=primary or 1=secondary
    void setPosVelXYZSource(uint8_t source_idx);

    // get/set velocity source
    SourceXY getVelXYSource() const { return _initialised ? _active_velxy_source : (SourceXY)_velxy_source1.get(); }
    SourceZ getVelZSource() const { return _initialised ? _active_velz_source : (SourceZ)_velz_source1.get(); }
    void setVelZSource(SourceZ source) { _active_velz_source = source; }

    // get yaw source
    SourceYaw getYawSource() const { return _initialised ? _active_yaw_source : (SourceYaw)_yaw_source1.get(); }

    // sensor specific helper functions

    // true if any source is GPS
    bool usingGPS() const;

    static const struct AP_Param::GroupInfo var_info[];

private:

    // Parameters
    AP_Int8 _posxy_source1;     // primary xy position source
    AP_Int8 _velxy_source1;     // primary xy velocity source
    AP_Int8 _posz_source1;      // primary position z (aka altitude or height) source
    AP_Int8 _velz_source1;      // primary velocity z source
    AP_Int8 _yaw_source1;       // primary yaw source
    AP_Int8 _posxy_source2;     // secondary xy position source
    AP_Int8 _velxy_source2;     // secondary xy velocity source
    AP_Int8 _posz_source2;      // position z (aka altitude or height) source
    AP_Int8 _velz_source2;      // secondary velocity z source
    AP_Int8 _yaw_source2;       // secondary yaw source

    // active sources
    bool _initialised;                      // true once init has been run
    SourceXY _active_posxy_source;    // current xy position source
    SourceZ _active_posz_source;      // current z position source
    SourceXY _active_velxy_source;    // current xy velocity source
    SourceZ _active_velz_source;      // current z velocity source
    SourceYaw _active_yaw_source;     // current yaw source
};
