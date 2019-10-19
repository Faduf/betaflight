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

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/time.h"

#include "io/gps.h"

#include "fc/config.h"
#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/imu.h"
#include "flight/position.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/compass.h"
#include "sensors/barometer.h"

#include "volume_limitation.h"

#define ALTHOLD_CT_FAC      1.2
#define ALTHOLD_PTERM_SCALE 1000
#define ALTHOLD_PTERM_BOOST 1.2
#define ALTHOLD_ITERM_SCALE 200000
#define ALTHOLD_DTERM_SCALE 2000

#define SAFEHOLD_YAW_RATE       10
#define SAFEHOLD_YAW_SPEED_MAX  100
#define SAFEHOLD_PITCH_P_SCALE  20000
#define SAFEHOLD_PITCH_I_SCALE  10000
#define SAFEHOLD_PITCH_D_SCALE  100
#define SAFEHOLD_ROLL_P_SCALE   200
#define SAFEHOLD_ROLL_I_SCALE   100
#define SAFEHOLD_ROLL_D_SCALE   10


PG_REGISTER_WITH_RESET_TEMPLATE(volLimitationConfig_t, volLimitationConfig, PG_VOLUME_LIMITATION, 0);

PG_RESET_TEMPLATE(volLimitationConfig_t, volLimitationConfig,
    .altitudeLimitation = false,
    .distanceLimitation = false,
    .landingAllowed = false,
    .maxAltitude = 100,
    .alertAltitudeBeforeMax = 20,
    .maxDistance = 1000,
    .alertDistanceBeforeMax = 50,
    .minSafeAltitude = 20,
    .throttleP = 120,
    .throttleI = 32,
    .throttleD = 44,
    .throttleMin = 1000,
    .throttleMax = 2000,
    .throttleHover = 1300,
    .safeHold_pitch_P = 130,
    .safeHold_pitch_I = 38,
    .safeHold_pitch_D = 130,
    .safeHold_roll_P = 140,
    .safeHold_roll_I = 62,
    .safeHold_roll_D = 120,
    .safeHoldAngleMax = 20,
    .minSats = 8,
    .armingWithoutGps = false
);

/*********** Global Variables **************/
static bool newGPSData = false;
volLimData_s volLimData;
int16_t safeHold_angle[2] = { 0, 0 };    // it's the angles that must be applied for Safe hold state

/*********** Static functions **************/
static int32_t updateHoldYaw(void);
static bool updateHoldRollPitchAngle(int32_t heading, int32_t safeDirection);

/*******************************************/

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
    volLimData.alert.safeHold = 0;
    volLimData.alert.sensorFailure = 0;
    volLimData.altitudeLimDemanded = 0;
    volLimData.safeModeDemanded = 0;
    volLimData.safeHoldState = SAFEHOLD_OFF;
}

void volLimitation_SensorUpdate(void)
{
    volLimData.sensor.currentAltitude = getEstimatedAltitudeCm();

    // Calculate altitude velocity
    static uint32_t previousTimeUs;
    static int32_t previousAltitude;

    const uint32_t currentTimeUs = micros();
    const float dTime = currentTimeUs - previousTimeUs;

    if (newGPSData) { // Calculate velocity at lowest common denominator
        volLimData.sensor.distanceToHome = GPS_distanceToHomeCM; // Distance in cm
        volLimData.sensor.directionToHome = GPS_directionToHome; // Direction in degrees
        volLimData.sensor.numSat = gpsSol.numSat;

        volLimData.sensor.zVelocity = (volLimData.sensor.currentAltitude - previousAltitude) * 1000000.0f / dTime;
        volLimData.sensor.zVelocityAvg = 0.8f * volLimData.sensor.zVelocityAvg + volLimData.sensor.zVelocity * 0.2f;

        previousAltitude = volLimData.sensor.currentAltitude;
        previousTimeUs = currentTimeUs;
    }
}

float volLimitation_AltitudeLim(float throttle)
{
    int32_t altitudeLimIntegral = 0;
    static int32_t altitudeErrorLimSum = 0;
    int32_t throttleItermLim = 0, throttleDtermLim = 0, throttlePtermLim = 0;
    static float volLimThrottle;
    float ct = cos(DECIDEGREES_TO_RADIANS(attitude.values.pitch / 10))* cos(DECIDEGREES_TO_RADIANS(attitude.values.roll / 10));
    ct = constrainf(ct,0.5f,1.0f); // Inclination coefficient limitation

    if (!volLimitationConfig()->altitudeLimitation) {
        volLimData.alert.distance = 2;
        return throttle;
    }

    const int32_t altitudeErrorLim = (volLimitationConfig()->maxAltitude * 100) - volLimData.sensor.currentAltitude; // Error in cm

    // OSD alert activation if too High
    if (altitudeErrorLim < (volLimitationConfig()->alertAltitudeBeforeMax * 100)) {
        volLimData.alert.altitude = 1;
    } else {
        volLimData.alert.altitude = 0;
    }

    // Activate limitation 20m before max and limitation is activated
    if ((altitudeErrorLim < 2000) && (volLimitationConfig()->altitudeLimitation)) {
        volLimData.altitudeLimDemanded = 1;
    } else {
        volLimData.altitudeLimDemanded = 0;
    }

    // Only allow integral windup within +-20m absolute altitude error
    if ((ABS(altitudeErrorLim) < 2000)) {
        altitudeErrorLimSum += altitudeErrorLim;
        altitudeLimIntegral = altitudeErrorLimSum / 100;
        throttleItermLim = (volLimitationConfig()->throttleI * altitudeLimIntegral) / ALTHOLD_ITERM_SCALE;
        throttleItermLim = constrain(throttleItermLim, -250, 250);
    } else {
        altitudeLimIntegral = 0;
        throttleItermLim = 0;
    }

    // DTerm calculation
    throttleDtermLim = - volLimitationConfig()->throttleD * volLimData.sensor.zVelocityAvg / ALTHOLD_DTERM_SCALE;;
    throttleDtermLim = constrain(throttleDtermLim, -250, 250);

    // PTerm calculation
    throttlePtermLim = volLimitationConfig()->throttleP * altitudeErrorLim / ALTHOLD_PTERM_SCALE;
    // Boost Pterm if too far from the altitude target
    if(ABS(altitudeErrorLim) < 500) {
        throttlePtermLim = throttlePtermLim * ALTHOLD_PTERM_BOOST;
    }

    int32_t altitudeLimAdjustment = (throttlePtermLim + throttleItermLim  + throttleDtermLim) / ct;
    int32_t hoverAdjustment = (volLimitationConfig()->throttleHover - 1000) / ct;

    volLimThrottle = constrainf(1000 + altitudeLimAdjustment + hoverAdjustment, volLimitationConfig()->throttleMin, volLimitationConfig()->throttleMax);

    // Limitation applies only when altitude limit is reached
    float commandedLimThrottle;
    if (volLimData.altitudeLimDemanded == 1) {
        commandedLimThrottle = scaleRangef(volLimThrottle, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);
        commandedLimThrottle = constrainf(throttle,0.0f,commandedLimThrottle);

        // Reset I if the pilot has decreased the throttle to go down
        if(throttle ==  commandedLimThrottle) {
            altitudeLimIntegral = 0;
            throttleItermLim = 0;
        }
    } else {
        commandedLimThrottle = throttle;
    }

    return commandedLimThrottle;
}

float volLimitation_AltitudeHold(uint8_t altholdStatus)
{
    int32_t altitudeIntegral = 0;
    static int32_t altitudeErrorSum = 0;
    static int32_t altitudeTarget = 0; // ALTITUDE IN CM
    int32_t throttleIterm = 0, throttleDterm = 0, throttlePterm = 0;
    static float volHoldThrottle;
    float commandedHoldThrottle;
    float ct = cos(DECIDEGREES_TO_RADIANS(attitude.values.pitch / 10))* cos(DECIDEGREES_TO_RADIANS(attitude.values.roll / 10));
    ct =ct * ALTHOLD_CT_FAC; // Boost CT value to avoid altitude drop when pitch/roll angle
    ct = constrainf(ct,0.5f,1.0f); // Inclination coefficient limitation

    const int32_t altitudeError = altitudeTarget - volLimData.sensor.currentAltitude; // Error in cm

    if(altholdStatus == 1) {
        // Decrease throttle progressivly if landing is requested
        if (volLimData.safeHoldState == SAFEHOLD_LAND){
            // TO DO
        }

        // Only allow integral windup within +-20m absolute altitude error
        if (ABS(altitudeError) < 2000) {
            altitudeErrorSum += altitudeError;
            altitudeIntegral = altitudeErrorSum / 100;
            throttleIterm = (volLimitationConfig()->throttleI * altitudeIntegral) / ALTHOLD_ITERM_SCALE;
            throttleIterm = constrain(throttleIterm, -250, 250);
        } else {
            altitudeIntegral = 0;
            throttleIterm = 0;
        }

        // DTerm calculation
        throttleDterm = - volLimitationConfig()->throttleD * volLimData.sensor.zVelocityAvg / ALTHOLD_DTERM_SCALE;
        throttleDterm = constrain(throttleDterm, -200, 200);

        // PTerm calculation
        throttlePterm = volLimitationConfig()->throttleP * altitudeError / ALTHOLD_PTERM_SCALE;

        // Boost Pterm if too far from the altitude target
        if(ABS(altitudeError) < 500) {
            throttlePterm = throttlePterm * ALTHOLD_PTERM_BOOST;
        }


        int32_t altitudeAdjustment = (throttlePterm + throttleIterm  + throttleDterm) / ct;
        int32_t hoverAdjustment = (volLimitationConfig()->throttleHover - 1000) / ct;

        volHoldThrottle = constrainf(1000 + altitudeAdjustment + hoverAdjustment, volLimitationConfig()->throttleMin, volLimitationConfig()->throttleMax);

        // Limitation applies only when altitude limit is reached
        commandedHoldThrottle = scaleRangef(volHoldThrottle, MAX(rxConfig()->mincheck, PWM_RANGE_MIN), PWM_RANGE_MAX, 0.0f, 1.0f);

        DEBUG_SET(DEBUG_VOL_LIM_ALTHOLD, 0, (int16_t)altitudeError);
        DEBUG_SET(DEBUG_VOL_LIM_ALTHOLD, 1, (int16_t)throttlePterm);
        DEBUG_SET(DEBUG_VOL_LIM_ALTHOLD, 2, (int16_t)throttleDterm);
        DEBUG_SET(DEBUG_VOL_LIM_ALTHOLD, 3, (int16_t)throttleIterm);
        return commandedHoldThrottle;
    } else {
        // altitude target reset
        altitudeTarget = getEstimatedAltitudeCm();
        altitudeTarget = constrain(altitudeTarget, (volLimitationConfig()->minSafeAltitude*100), altitudeTarget); // 20m minimu

        // Iterm reset
        altitudeIntegral = 0;
        throttleIterm = 0;

        return 0;
    }
}

uint8_t volLimitation_DistanceLimStatus(void)
{
    static uint8_t isStabModeSwitched = 0;
    static uint8_t initHomeDirection = 0;
    static int32_t homeDirectionSafe = 0;
    int32_t headingError = 0;
    uint8_t safePointReached = 0;

    if (!volLimitationConfig()->distanceLimitation) {
        volLimData.alert.distance = 2;
        return 0;
    }

    float distanceError = (float)(volLimitationConfig()->maxDistance) - (float)(volLimData.sensor.distanceToHome / 100); // in meters

    if (IS_RC_MODE_ACTIVE(BOXANGLE)) {
        isStabModeSwitched = 1;
    }

    // Activate alert OSD if too far
    if(distanceError < volLimitationConfig()->alertDistanceBeforeMax) {
        volLimData.alert.distance = 1;
    } else {
        volLimData.alert.distance = 0;
    }

    switch(volLimData.safeHoldState)
    {
        case SAFEHOLD_OFF:
            // No Roll/Pitch inclinations
            volLimData.alert.safeHold = 0;
            safeHold_angle[FD_PITCH] = 0;
            safeHold_angle[FD_ROLL] = 0;
            initHomeDirection = 0;
            volLimData.safeModeDemanded = 0;

            // Activate Stab mode, althold and poshold if max distance reached and limitation activated
            if((distanceError < 0) && volLimSanityCheck()) {
                volLimData.safeModeDemanded = 1;
                isStabModeSwitched = 0;
                volLimData.safeHoldState = SAFEHOLD_INIT;
            }
            if(FLIGHT_MODE(SAFE_HOLD_MODE) && volLimSanityCheck()) {
                isStabModeSwitched = 0;
                volLimData.alert.safeHold = 1;
                volLimData.safeHoldState = SAFEHOLD_INIT;
            }
            break;

        case SAFEHOLD_INIT:
            // if safe mode manually turned off, return to OFF
            if(!FLIGHT_MODE(SAFE_HOLD_MODE)){
                volLimData.safeHoldState = SAFEHOLD_OFF;
            }

            // if sensor or not healthy, allow deactivation of the SAFE HOLD MODE
            if(!volLimSanityCheck()){
                volLimData.safeHoldState = SAFEHOLD_WAIT;
            }

            // Save Home direction for further steps
            if(initHomeDirection == 0){
                homeDirectionSafe = volLimData.sensor.directionToHome;
                initHomeDirection = 1;
            }

            // Turn on yaw to face Home point
            headingError = updateHoldYaw();

            // No Roll/Pitch inclinations
            safeHold_angle[FD_PITCH] = 0;
            safeHold_angle[FD_ROLL] = 0;

            // If home point nearly faced, go to next step
            if(headingError < 10){
                volLimData.safeHoldState = SAFEHOLD_GO_SAFE;
            }
            break;

        case SAFEHOLD_GO_SAFE:
            // if safe mode manually turned off, return to OFF
            if(!FLIGHT_MODE(SAFE_HOLD_MODE)){
                volLimData.safeHoldState = SAFEHOLD_OFF;
            }
            // if sensor or not healthy, allow deactivation of the SAFE HOLD MODE
            if(!volLimSanityCheck()){
                volLimData.safeHoldState = SAFEHOLD_WAIT;
            }
            // Update Yaw rate and Roll/Pitch Angle to go to safe point
            headingError = updateHoldYaw();
            safePointReached = updateHoldRollPitchAngle(headingError,homeDirectionSafe);
            if(safePointReached == 1){
                volLimData.safeHoldState = SAFEHOLD_WAIT;
            }
            // Safe hold deactivation via ANGLE MODE Switch
            if(isStabModeSwitched == 1 && !IS_RC_MODE_ACTIVE(BOXANGLE)){
                volLimData.safeHoldState = SAFEHOLD_OFF;
                volLimData.safeModeDemanded = 0;
            }
            break;

        case SAFEHOLD_WAIT:
            // if safe mode manually turned off, return to OFF
            if(!FLIGHT_MODE(SAFE_HOLD_MODE)){
                volLimData.safeHoldState = SAFEHOLD_OFF;
            }
            // Update Yaw rate and Roll/Pitch Angle to stay to safe point
            headingError = updateHoldYaw();
            safePointReached = updateHoldRollPitchAngle(headingError,homeDirectionSafe);
            // Activate landing if allowed
            if(volLimitationConfig()->landingAllowed == true){
                volLimData.safeHoldState = SAFEHOLD_LAND;
            }
            // Safe hold deactivation via ANGLE MODE Switch
            if(isStabModeSwitched == 1 && !IS_RC_MODE_ACTIVE(BOXANGLE)){
                volLimData.safeHoldState = SAFEHOLD_OFF;
                volLimData.safeModeDemanded = 0;
            }
            break;

        case SAFEHOLD_LAND:
            // if safe mode manually turned off, return to OFF
            if(!FLIGHT_MODE(SAFE_HOLD_MODE)){
                volLimData.safeHoldState = SAFEHOLD_OFF;
            }
            // Update Yaw rate and Roll/Pitch Angle to stay to safe point
            headingError = updateHoldYaw();
            safePointReached = updateHoldRollPitchAngle(headingError,homeDirectionSafe);

            // Safe hold deactivation via ANGLE MODE Switch
            if(isStabModeSwitched == 1 && !IS_RC_MODE_ACTIVE(BOXANGLE)){
                volLimData.safeHoldState = SAFEHOLD_OFF;
                volLimData.safeModeDemanded = 0;
            }
            break;

        default:
            volLimData.safeHoldState = SAFEHOLD_OFF;
            break;
    }

    return volLimData.safeModeDemanded;
}

static int32_t updateHoldYaw(void)
{
    /* Yaw rotation calculation */
    int32_t headingError = (attitude.values.yaw / 10) - volLimData.sensor.directionToHome;
    volLimData.holdYaw = constrain(headingError * SAFEHOLD_YAW_RATE, -SAFEHOLD_YAW_SPEED_MAX, SAFEHOLD_YAW_SPEED_MAX);
    return headingError;
}

static bool updateHoldRollPitchAngle(int32_t heading, int32_t safeDirection)
{
    bool inSafeZone = false;
    int16_t safePitchPterm = 0, safeRollPterm = 0, safePitchDterm = 0, safeRollDterm = 0;
    static int32_t safePitchIterm = 0, safeRollIterm = 0;
    static float previousDistanceErrorCM = 0;
    static float distanceVelocityAvg = 0;
    static int32_t previousSafeDirectionError = 0;
    static float angularVelocityAvg = 0;

    float targetDistanceCM = (float)((volLimitationConfig()->maxDistance - volLimitationConfig()->alertDistanceBeforeMax)*100); // in cm
    float distanceErrorCM =  (float)volLimData.sensor.distanceToHome - targetDistanceCM; // Error in cm

    int32_t safeDirectionError = volLimData.sensor.directionToHome - safeDirection; // in degrees
    // Scale the direction Error to be within +/-180°
    if(safeDirectionError > 180){
        safeDirectionError -= 360;
    } else if(safeDirectionError < -180){
        safeDirectionError += 360;
    }

    // Allow Roll and Pitch correction only if facing home
    if((heading < 10) && (heading > -10)) {
        /* Pitch inclination to reach the safe distance */
        safePitchPterm = distanceErrorCM * volLimitationConfig()->safeHold_pitch_P / SAFEHOLD_PITCH_P_SCALE;
        safePitchPterm = constrain(safePitchPterm,-20,20);

        // Only allow Iterm in +/-2m
        if(ABS(distanceErrorCM) < 2000){
            safePitchIterm += volLimitationConfig()->safeHold_pitch_I * distanceErrorCM;
            safePitchIterm = safePitchIterm / SAFEHOLD_PITCH_I_SCALE;
            safePitchIterm = constrain(safePitchIterm,-10,10);
        } else {
            safePitchIterm = 0;
        }

        distanceVelocityAvg = 0.99*distanceVelocityAvg + 0.01*(distanceErrorCM - previousDistanceErrorCM);
        safePitchDterm = (int32_t)distanceVelocityAvg * volLimitationConfig()->safeHold_pitch_D / SAFEHOLD_PITCH_D_SCALE;
        safePitchDterm = constrain(safePitchDterm,-20,20);

        safeHold_angle[FD_PITCH] = constrain(safePitchPterm + (int16_t)safePitchIterm + safePitchDterm,-volLimitationConfig()->safeHoldAngleMax,volLimitationConfig()->safeHoldAngleMax);

        /* Roll inclination to reach the safe directions */
        safeRollPterm = safeDirectionError * volLimitationConfig()->safeHold_roll_P / SAFEHOLD_ROLL_P_SCALE;
        safeRollPterm = constrain(safeRollPterm,-20,20);

        // Only allow Iterm in +/-10°
        if(ABS(safeDirectionError) < 20){
            safeRollIterm += volLimitationConfig()->safeHold_roll_I * safeDirectionError;
            safeRollIterm = safeRollIterm / SAFEHOLD_ROLL_I_SCALE;
            safeRollIterm = constrain(safeRollIterm,-10,10);
        } else {
            safeRollIterm = 0;
        }

        angularVelocityAvg = 0.99*angularVelocityAvg + 0.01*(safeDirectionError - previousSafeDirectionError) * 100;
        safeRollDterm = angularVelocityAvg * volLimitationConfig()->safeHold_roll_D / SAFEHOLD_ROLL_D_SCALE;
        safeRollDterm = constrain(safeRollDterm,-20,20);

        safeHold_angle[FD_ROLL] = constrain(safeRollPterm + (int16_t)safeRollIterm + safeRollDterm,-volLimitationConfig()->safeHoldAngleMax,volLimitationConfig()->safeHoldAngleMax);
    } else {
        safeHold_angle[FD_PITCH] = 0;
        safeHold_angle[FD_ROLL] = 0;
        // Regulation cut-off => reset Iterms
        safePitchIterm = 0;
        safeRollIterm = 0;
    }

    previousDistanceErrorCM = distanceErrorCM;
    previousSafeDirectionError = safeDirectionError;

    // Check if is in safe Zone
    if(distanceErrorCM < (volLimitationConfig()->alertDistanceBeforeMax * 100)){
        inSafeZone = true;
    } else {
        inSafeZone = false;
    }

    DEBUG_SET(DEBUG_VOL_LIM_SAFEHOLD, 0, (int16_t)volLimData.safeHoldState);
    DEBUG_SET(DEBUG_VOL_LIM_SAFEHOLD, 1, (int16_t)angularVelocityAvg);
    DEBUG_SET(DEBUG_VOL_LIM_SAFEHOLD, 2, (int16_t)safeRollIterm);
    DEBUG_SET(DEBUG_VOL_LIM_SAFEHOLD, 3, (int16_t)safeRollDterm);

    return inSafeZone;
}

bool volLimSanityCheck(void)
{
    if (!gpsIsHealthy() || !STATE(GPS_FIX) || !compassIsHealthy() || !isBaroReady()) {
        volLimData.alert.sensorFailure = 1;
        return false;
    } else {
        volLimData.alert.sensorFailure = 0;
        return true;
    }
}

volLimAlert_s getVolLimAlert(void)
{
    return volLimData.alert;
}

float gpsHoldGetYawRate(void)
{
    return volLimData.holdYaw;
}

bool gpsNeededForVolLim(void)
{
    bool gpsNeededForArming = false;
    // GPS is needed before arming if distance limitation is activated or GPS wanted
    gpsNeededForArming = (!volLimitationConfig()->armingWithoutGps) | (volLimitationConfig()->distanceLimitation);
    return gpsNeededForArming;
}
