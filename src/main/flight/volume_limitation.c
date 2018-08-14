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
    .maxDistance = 200,
    .throttleP = 150,
    .throttleI = 20,
    .throttleD = 50,
    .velP = 80,
    .velI = 20,
    .velD = 15,
    .yawP = 40,
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
    volLimData.state.altitude = 0;
    volLimData.state.distance = 0;
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

    // Only allow integral windup within +-15m absolute altitude error
    if (ABS(altitudeError) < 15) {
        altitudeIntegral = constrain(altitudeIntegral + altitudeError, -250, 250);
        volLimData.state.altitude = 1;
    } else {
        altitudeIntegral = 0;
        volLimData.state.altitude = 0;
    }

    previousAltitudeError = altitudeError;

    int16_t altitudeAdjustment = (volLimitationConfig()->throttleP * altitudeError + (volLimitationConfig()->throttleI * altitudeIntegral) / 10 *  + volLimitationConfig()->throttleD * altitudeDerivative) / ct;
    int16_t hoverAdjustment = (volLimitationConfig()->throttleHover - 1000) / ct;

    volLimThrottle = constrain(1000 + altitudeAdjustment + hoverAdjustment, volLimitationConfig()->throttleMin, volLimitationConfig()->throttleMax);
    float commandedLimThrottle = scaleRangef(volLimThrottle, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);

    DEBUG_SET(DEBUG_VOL_LIMITATION, 1, volLimitationConfig()->throttleP * altitudeError);
    DEBUG_SET(DEBUG_VOL_LIMITATION, 2, altitudeAdjustment);
    DEBUG_SET(DEBUG_VOL_LIMITATION, 3, volLimThrottle);

    return commandedLimThrottle;
}

uint8_t volLimitation_DistanceLim(void)
{
    static uint8_t isStabModeSwitched = 0;

    // Heading
    const int16_t headingError = (GPS_directionToHome - attitude.values.yaw);
    const int16_t distanceError = (volLimitationConfig()->maxDistance - volLimData.sensor.distanceToHome);

    // Activate Stab mode if too far
    if(distanceError < 0) {
        volLimData.state.distance = 1;
        isStabModeSwitched = 0;
    } else {
        // Wait for the stab mode switch to release the stab mode
        if (isStabModeSwitched) {
            volLimData.state.distance = 0;
        }
    }

    if(IS_RC_MODE_ACTIVE(BOXANGLE)) {
        isStabModeSwitched = 1;
    }

    DEBUG_SET(DEBUG_VOL_LIMITATION, 0, headingError);
    return volLimData.state.distance;
}

volLimState_s getVolLimStatus(void)
{
    return volLimData.state;
}

#endif // USE_VOLUME_LIMITATION
