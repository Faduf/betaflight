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

#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_VOLUME_LIMITATION

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/time.h"

#include "io/gps.h"

#include "fc/config.h"
#include "fc/fc_core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"

#include "volume_limitation.h"

PG_REGISTER_WITH_RESET_TEMPLATE(volLimitationConfig_t, volLimitationConfig, PG_VOLUME_LIMITATION, 0);

PG_RESET_TEMPLATE(volLimitationConfig_t, volLimitationConfig,
    .maxAltitude = 50,
    .maxDistance = 150,
    .throttleP = 20,
    .throttleI = 30,
    .throttleD = 5,
    .distLimP = 10,
    .distLimD = 10,
    .throttleMin = 1000,
    .throttleMax = 2000,
    .throttleHover = 1400,
    .minSats = 8
);

static float volLimThrottle;

static bool newGPSData = false;
volLimData_s volLimData;

/*
 If we have new GPS data, update home heading
 if possible and applicable.
*/
void volLimitation_NewGpsData(void)
{
    if (!ARMING_FLAG(ARMED)) {
        GPS_reset_home_position();
        newGPSData = true;
    }
}

void volLimitation_init(void)
{
    volLimData.alert.altitude = 0;
    volLimData.alert.distance = 0;
    volLimData.angleModeDemanded = 0;
}

void volLimitation_SensorUpdate(void)
{
    volLimData.sensor.currentAltitude = getEstimatedAltitude();

    // Calculate altitude velocity
    static uint32_t previousTimeUs;
    static int32_t previousAltitude;

    const uint32_t currentTimeUs = micros();
    const float dTime = currentTimeUs - previousTimeUs;

    if (newGPSData) { // Calculate velocity at lowest common denominator
        volLimData.sensor.distanceToHome = GPS_distanceToHome;
        volLimData.sensor.directionToHome = GPS_directionToHome;
        volLimData.sensor.numSat = gpsSol.numSat;

        volLimData.sensor.zVelocity = (volLimData.sensor.currentAltitude - previousAltitude) * 1000000.0f / dTime;
        volLimData.sensor.zVelocityAvg = 0.8f * volLimData.sensor.zVelocityAvg + volLimData.sensor.zVelocity * 0.2f;

        previousAltitude = volLimData.sensor.currentAltitude;
        previousTimeUs = currentTimeUs;
    }
}

float volLimitation_AltitudeLim(float throttle)
{
    static float previousAltitudeError = 0;
    static float previousThrottle = 0;
    static int16_t altitudeIntegral = 0;
    int16_t throttleIterm = 0, throttleDterm = 0, throttlePterm = 0;
    float ct = cos(DECIDEGREES_TO_RADIANS(attitude.values.pitch / 10))* cos(DECIDEGREES_TO_RADIANS(attitude.values.roll / 10));
    ct = constrainf(ct,0.5f,1.0f); // Inclination coefficient limitation

    const int16_t altitudeError = volLimitationConfig()->maxAltitude - (getEstimatedAltitude() / 100); // Error in meters

    // OSD and limitaiton activation if too High
    if (altitudeError < 10) {
        volLimData.alert.altitude = 1;
        volLimData.limit.altitude = 1;
    } else {
        volLimData.alert.altitude = 0;
        volLimData.limit.altitude = 0;
    }

    // Only allow integral windup within +-10m absolute altitude error
    if ((ABS(altitudeError) < 10) && (throttle > previousThrottle)) {
        altitudeIntegral += altitudeError;
        throttleIterm = (volLimitationConfig()->throttleI * altitudeIntegral) / 8000;
        throttleIterm = constrain(throttleIterm, -250, 250);
    } else {
        altitudeIntegral = 0;
        throttleIterm = 0;
    }

    // DTerm calculation
    const int16_t altitudeDerivative = (altitudeError - previousAltitudeError) * getPidFrequency();
    throttleDterm = volLimitationConfig()->throttleD * altitudeDerivative * 20;
    throttleDterm = 0; // deactivate DTerm
    previousAltitudeError = altitudeError;

    // PTerm calculation
    throttlePterm = volLimitationConfig()->throttleP * altitudeError / 10;

    int16_t altitudeAdjustment = (throttlePterm + throttleIterm  + throttleDterm) / ct;
    int16_t hoverAdjustment = (volLimitationConfig()->throttleHover - 1000) / ct;

    volLimThrottle = constrainf(1000 + altitudeAdjustment + hoverAdjustment, volLimitationConfig()->throttleMin, volLimitationConfig()->throttleMax);

    // Limitation applies only when altitude limit is reached
    float commandedLimThrottle;
    if (volLimData.limit.altitude == 1) {
        commandedLimThrottle = scaleRangef(volLimThrottle, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);
        commandedLimThrottle = constrainf(throttle,0.0f,commandedLimThrottle);
    } else {
        commandedLimThrottle = throttle;
    }
    previousThrottle = commandedLimThrottle;
    DEBUG_SET(DEBUG_VOL_LIMITATION, 3, (int16_t)volLimThrottle);

    return commandedLimThrottle;
}

float volLimitation_DistanceLimAngle(int axis, float angle_command)
{
    static float previousDistance = 0;
    float volLimAngleD = 0, volLimAngleP, volLimAngle = 0;
    float volLimAngle_Pitch = 0, volLimAngle_Roll = 0;

    // Heading
    int16_t headingError = (attitude.values.yaw - (GPS_directionToHome * 10));
    float distanceError = (float)volLimitationConfig()->maxDistance - (float)volLimData.sensor.distanceToHome; // in meter

    // Pilot loose authority when approaching the limit
    float pilotAuthorityFac = (distanceError + 20)/20;
    pilotAuthorityFac = constrainf(pilotAuthorityFac,0,1);

    // Distance over limit
    if (volLimData.sensor.distanceToHome > volLimitationConfig()->maxDistance) {
        // DTerm calculation
        previousDistance = distanceError;
        volLimAngleD = (float)volLimitationConfig()->distLimD * (distanceError - previousDistance) * getPidFrequency()*100;
        volLimAngleD = 0;// = constrainf(volLimAngleD,-10,10);

        // PTerm caclulation
        volLimAngleP = distanceError * (float)(volLimitationConfig()->distLimP) / 10;

        volLimAngle = -1 * (volLimAngleP + volLimAngleD);
        volLimAngle = constrainf(volLimAngle,-30,30);

        volLimAngle_Pitch = volLimAngle * cos_approx(degreesToRadians(headingError/10));
        volLimAngle_Roll = -1 * volLimAngle * sin_approx(degreesToRadians(headingError/10));
    } else {
        volLimAngle_Pitch = angle_command * pilotAuthorityFac;
        volLimAngle_Roll = angle_command * pilotAuthorityFac;
    }

    if (axis == FD_ROLL) {
        volLimData.angleDemand = volLimAngle_Roll;
        volLimData.angleDemand += angle_command * pilotAuthorityFac; // Add pilot authority
    } else if (axis == FD_PITCH) {
        volLimData.angleDemand = volLimAngle_Pitch;
        volLimData.angleDemand += angle_command * pilotAuthorityFac; // Add pilot authority
    } else {
        volLimData.angleDemand = angle_command;
    }

    DEBUG_SET(DEBUG_VOL_LIMITATION, 0, (int16_t)headingError);
    DEBUG_SET(DEBUG_VOL_LIMITATION, 1, (int16_t)(volLimAngle_Pitch));
    DEBUG_SET(DEBUG_VOL_LIMITATION, 2, (int16_t)(volLimAngle_Roll));

    return volLimData.angleDemand;
}

uint8_t volLimitation_DistanceLimStatus(void)
{
    static uint8_t isStabModeSwitched = 0;
    float distanceError = (float)volLimitationConfig()->maxDistance - (float)volLimData.sensor.distanceToHome; // in meter

    if (IS_RC_MODE_ACTIVE(BOXANGLE)) {
        isStabModeSwitched = 1;
    }

    // Activate Stab mode and alert OSD if too far
    if(distanceError < 20) {
        volLimData.angleModeDemanded = 1;
        volLimData.alert.distance = 1;
        isStabModeSwitched = 0;
    } else {
        volLimData.alert.distance = 0;
        // Wait for the stab mode switch to release the stab mode
        if (isStabModeSwitched) {
            volLimData.angleModeDemanded = 0;
        }
    }
    return volLimData.angleModeDemanded;
}

volLimAlert_s getVolLimAlert(void)
{
    return volLimData.alert;
}

#endif // USE_VOLUME_LIMITATION
