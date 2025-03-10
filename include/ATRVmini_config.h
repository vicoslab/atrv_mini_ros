/*  ATRVmini constants
 *  Modified from Player config
 */

#ifndef ATRVmini_CONFIG_H
#define ATRVmini_H

// Odometery Constants
// ===================
// Arbitrary units per meter
#define ODO_DISTANCE_CONVERSION 194232.294147
// Arbitrary units per radian
#define ODO_ANGLE_CONVERSION 64501.69726

// Sonar Constants
// ===============
// Arbitrary units per meter (for sonar)
#define RANGE_CONVERSION 1550
#define SONAR_ECHO_DELAY 30000
#define SONAR_PING_DELAY 0
#define SONAR_SET_DELAY  0
#define SONAR_MAX_RANGE 3000

#define BODY_INDEX 0
#define BASE_INDEX 1
const int SONARS_PER_BANK[] = {5, 8, 5, 8 };
const int SONAR_RING_BANK_BOUND[] = {0, 6, 14};
#define SONAR_MAX_PER_BANK 16

const int SONARS_PER_RING[] = {24, 24};
const float SONAR_RING_START_ANGLE[] = {180,90};
const float SONAR_RING_ANGLE_INC[] = {-15, -15};
const float SONAR_RING_DIAMETER[] = {.25, .26};
const float SONAR_RING_HEIGHT[] = {0.055, -0.06};

// Digital IO constants
// ====================
#define HEADING_HOME_ADDRESS 0x31
#define HOME_BEARING -32500

#define BUMPER_COUNT 14
#define BUMPER_ADDRESS_STYLE 0
#define BUMPER_BIT_STYLE 1
#define BUMPER_STYLE 0
const int BUMPERS_PER[] = {6,8};
const double BUMPER_ANGLE_OFFSET[] = {-1,1,-1,1};
const double BUMPER_HEIGHT_OFFSET[][4] = {{.5,.5,.05,.05},
    {.25,.25,.05,.05}
};

// IR Constants
// ============
#define IR_POSES_COUNT 0
#define IR_BASE_BANK 0
#define IR_BANK_COUNT 0
#define IR_PER_BANK 0

// Misc Constants
// ==============
#define USE_JOYSTICK 0
#define JOY_POS_RATIO 6
#define JOY_ANG_RATIO -0.01
#define POWER_OFFSET 1.82 
#define PLUGGED_THRESHOLD 27.6

#endif