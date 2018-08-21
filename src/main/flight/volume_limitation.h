/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#include "common/axis.h"

#include "pg/pg.h"

typedef enum {
    VOL_LIM_NO_LIM = 0,
    VOL_LIM_ALT_LIM = 1,
    VOL_LIM_DIST_LIM = 2,
    VOL_LIM_BOTH = 3
} volLimitaionState_e;

typedef struct volLimitation_s {
    uint16_t maxAltitude; //meters
    uint16_t altitudeAlert; //meters
    uint16_t maxDistance; //meters
    uint16_t distanceAngleSwitch; //meters
    uint16_t distanceAlert; //meters
    uint16_t throttleP, throttleI, throttleD;
    uint16_t throttleMin;
    uint16_t throttleMax;
    uint16_t throttleHover;
    uint16_t distLimP, distLimD;
    uint8_t minSats;
} volLimitationConfig_t;

PG_DECLARE(volLimitationConfig_t, volLimitationConfig);

typedef struct {
    int32_t currentAltitude;
    uint16_t distanceToHome;
    int16_t directionToHome;
    uint8_t numSat;
    float zVelocity; // Up/down movement in cm/s
    float zVelocityAvg; // Up/down average in cm/s
} volLimSensorData_s;

typedef struct {
    uint8_t altitude;
    uint8_t distance;
} volLimAlert_s;

typedef struct {
    volLimAlert_s alert;
    volLimAlert_s limit;
    volLimSensorData_s sensor;
    uint8_t angleModeDemanded;
    float angleDemand;
} volLimData_s;

void volLimitation_init(void);
void volLimitation_NewGpsData(void);
void volLimitation_SensorUpdate(void);
float volLimitation_AltitudeLim(float throttle);
float volLimitation_DistanceLimAngle(int axis, float angle_command);
uint8_t volLimitation_DistanceLimStatus(void);
volLimAlert_s getVolLimAlert(void);
