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
    .altitudeAlert = 40,
    .maxDistance = 200,
    .distanceAngleSwitch = 180,
    .distanceAlert = 150,
    .throttleP = 150,
    .throttleI = 20,
    .throttleD = 50,
    .distLimP = 10,
    .distLimD = 5,
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

float volLimitation_AltitudeLim(void)
{
    static float previousAltitudeError = 0;
    static int16_t altitudeIntegral = 0;
    float ct = cos(DECIDEGREES_TO_RADIANS(attitude.values.pitch / 10))* cos(DECIDEGREES_TO_RADIANS(attitude.values.roll / 10));
    ct = constrainf(ct,0.5f,1.0f); // Inclination coefficient limitation

    const int16_t altitudeError = volLimitationConfig()->maxAltitude - (getEstimatedAltitude() / 100); // Error in meters
    const int16_t altitudeDerivative = altitudeError - previousAltitudeError;

    // OSD alert if too High
    if ((getEstimatedAltitude() / 100) > volLimitationConfig()->altitudeAlert) {
        volLimData.alert.altitude = 1;
    } else {
        volLimData.alert.altitude = 0;
    }

    if ((getEstimatedAltitude() / 100) > volLimitationConfig()->maxAltitude) {
        volLimData.limit.altitude = 1;
    } else {
        volLimData.limit.altitude = 0;
    }

    // Only allow integral windup within +-15m absolute altitude error
    if (ABS(altitudeError) < 15) {
        altitudeIntegral = constrain(altitudeIntegral + altitudeError, -250, 250);
    } else {
        altitudeIntegral = 0;
    }

    previousAltitudeError = altitudeError;

    int16_t altitudeAdjustment = (volLimitationConfig()->throttleP * altitudeError + (volLimitationConfig()->throttleI * altitudeIntegral) / 10 *  + volLimitationConfig()->throttleD * altitudeDerivative) / ct / 20;
    int16_t hoverAdjustment = (volLimitationConfig()->throttleHover - 1000) / ct;

    volLimThrottle = constrainf(1000 + altitudeAdjustment + hoverAdjustment, volLimitationConfig()->throttleMin, volLimitationConfig()->throttleMax);
    float commandedLimThrottle = scaleRangef(volLimThrottle, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);

    DEBUG_SET(DEBUG_VOL_LIMITATION, 1, altitudeError);
    DEBUG_SET(DEBUG_VOL_LIMITATION, 2, altitudeAdjustment);
    DEBUG_SET(DEBUG_VOL_LIMITATION, 3, volLimThrottle);

    return commandedLimThrottle;
}

uint8_t volLimitation_DistanceLim(void)
{
    static uint8_t isStabModeSwitched = 0;
    static float previousDistance = 0;
    float volLimAngleD = 0, volLimAngle = 0;

    // Heading
    const int16_t headingError = (GPS_directionToHome - attitude.values.yaw);
    const float distanceError = (float)volLimitationConfig()->maxDistance - ((float)volLimData.sensor.distanceToHome)/100; // in meter

    // OSD alert if too High
    if (volLimData.sensor.distanceToHome > volLimitationConfig()->distanceAlert) {
        volLimData.alert.distance = 1;
    } else {
        volLimData.alert.distance = 0;
    }

    // Activate Stab mode if too far
    if(volLimData.sensor.distanceToHome > MIN(volLimitationConfig()->maxDistance,volLimitationConfig()->distanceAngleSwitch)) {
        volLimData.angleDemanded = 1;
        isStabModeSwitched = 0;
    } else {
        // Wait for the stab mode switch to release the stab mode
        if (isStabModeSwitched) {
            volLimData.angleDemanded = 0;
        }
    }
    if(IS_RC_MODE_ACTIVE(BOXANGLE)) {
        isStabModeSwitched = 1;
    }

    // Distance over limit
    if (volLimData.sensor.distanceToHome > volLimitationConfig()->maxDistance) {
        volLimData.limit.distance = 1;
    } else {
        volLimData.limit.distance = 0;
    }

    // Derivative calculation
    previousDistance = distanceError;
    volLimAngleD = (float)volLimitationConfig()->distLimD * (distanceError - previousDistance)/50;
    volLimAngleD = constrainf(volLimAngleD,-5,5);

    volLimAngle = distanceError * (float)volLimitationConfig()->distLimP + volLimAngleD;
    volLimAngle = constrainf(volLimAngle,-30,30);

    float volLimAngle_Pitch = volLimAngle * cos_approx(degreesToRadians(headingError));
    float volLimAngle_Roll = volLimAngle * sin_approx(degreesToRadians(headingError));

    DEBUG_SET(DEBUG_VOL_LIMITATION, 0, headingError);
    return volLimData.angleDemanded;
}

volLimAlert_s getVolLimAlert(void)
{
    return volLimData.alert;
}

#endif // USE_VOLUME_LIMITATION
