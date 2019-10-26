/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

#include "platform.h"

#include "cms/cms.h"
#include "cms/cms_types.h"
#include "cms/cms_menu_geofence.h"

#include "config/feature.h"

#include "fc/config.h"

#include "flight/volume_limitation.h"

#include "rx/rx.h"

uint8_t geofenceConfig_altitudeLimitation; // activation of the altitude limitation
uint8_t geofenceConfig_distanceLimitation; // activation of the distance limitation
uint8_t geofenceConfig_landingAllowed; // Is it allowed to laknd if pilot has not taken control back
uint16_t geofenceConfig_maxAltitude; //meters
uint8_t geofenceConfig_alertAltitudeBeforeMax; // meters
uint16_t geofenceConfig_maxDistance; //meters
uint8_t geofenceConfig_alertDistanceBeforeMax; //meters
uint16_t geofenceConfig_throttleP, geofenceConfig_throttleI, geofenceConfig_throttleD;
uint16_t geofenceConfig_throttleMin;
uint16_t geofenceConfig_throttleMax;
uint16_t geofenceConfig_throttleHover;
uint8_t geofenceConfig_safeHold_pitch_P;
uint8_t geofenceConfig_safeHold_pitch_I;
uint8_t geofenceConfig_safeHold_pitch_D;
uint8_t geofenceConfig_safeHold_roll_P;
uint8_t geofenceConfig_safeHold_roll_I;
uint8_t geofenceConfig_safeHold_roll_D;
uint8_t geofenceConfig_safeHoldAngleMax; // Angle max in deg
uint8_t geofenceConfig_minSafeAltitude; // Min altitude for althold in m
uint8_t geofenceConfig_minSats;
uint8_t geofenceConfig_armingWithoutGps; // possibility to arm without GPS

static long cmsx_Geofence_onEnter(void)
{
    geofenceConfig_altitudeLimitation = volLimitationConfig()->altitudeLimitation;
    geofenceConfig_distanceLimitation = volLimitationConfig()->distanceLimitation;
    geofenceConfig_landingAllowed = volLimitationConfig()->landingAllowed;
    geofenceConfig_maxAltitude = volLimitationConfig()->maxAltitude;
    geofenceConfig_alertAltitudeBeforeMax = volLimitationConfig()->alertAltitudeBeforeMax;
    geofenceConfig_maxDistance = volLimitationConfig()->maxDistance;
    geofenceConfig_alertDistanceBeforeMax = volLimitationConfig()->alertDistanceBeforeMax;
    geofenceConfig_throttleP = volLimitationConfig()->throttleP;
    geofenceConfig_throttleI = volLimitationConfig()->throttleI;
    geofenceConfig_throttleD = volLimitationConfig()->throttleD;
    geofenceConfig_throttleMin = volLimitationConfig()->throttleMin;
    geofenceConfig_throttleMax = volLimitationConfig()->throttleMax;
    geofenceConfig_throttleHover = volLimitationConfig()->throttleHover;
    geofenceConfig_safeHold_pitch_P = volLimitationConfig()->safeHold_pitch_P;
    geofenceConfig_safeHold_pitch_I = volLimitationConfig()->safeHold_pitch_I;
    geofenceConfig_safeHold_pitch_D = volLimitationConfig()->safeHold_pitch_D;
    geofenceConfig_safeHold_roll_P = volLimitationConfig()->safeHold_roll_P;
    geofenceConfig_safeHold_roll_I = volLimitationConfig()->safeHold_roll_I;
    geofenceConfig_safeHold_roll_D = volLimitationConfig()->safeHold_roll_D;
    geofenceConfig_safeHoldAngleMax = volLimitationConfig()->safeHoldAngleMax;
    geofenceConfig_minSafeAltitude = volLimitationConfig()->minSafeAltitude;
    geofenceConfig_minSats = volLimitationConfig()->minSats;
    geofenceConfig_armingWithoutGps = volLimitationConfig()->armingWithoutGps;
    
    return 0;
}

static long cmsx_Geofence_onExit(const OSD_Entry *self)
{
    UNUSED(self);

    volLimitationConfigMutable()->altitudeLimitation = geofenceConfig_altitudeLimitation;
    volLimitationConfigMutable()->distanceLimitation = geofenceConfig_distanceLimitation;
    volLimitationConfigMutable()->landingAllowed = geofenceConfig_landingAllowed;
    volLimitationConfigMutable()->maxAltitude = geofenceConfig_maxAltitude;
    volLimitationConfigMutable()->alertAltitudeBeforeMax = geofenceConfig_alertAltitudeBeforeMax;
    volLimitationConfigMutable()->maxDistance = geofenceConfig_maxDistance;
    volLimitationConfigMutable()->alertDistanceBeforeMax = geofenceConfig_alertDistanceBeforeMax;
    volLimitationConfigMutable()->throttleP = geofenceConfig_throttleP;
    volLimitationConfigMutable()->throttleI = geofenceConfig_throttleI;
    volLimitationConfigMutable()->throttleD = geofenceConfig_throttleD;
    volLimitationConfigMutable()->throttleMin = geofenceConfig_throttleMin;
    volLimitationConfigMutable()->throttleMax = geofenceConfig_throttleMax;
    volLimitationConfigMutable()->throttleHover = geofenceConfig_throttleHover;
    volLimitationConfigMutable()->safeHold_pitch_P = geofenceConfig_safeHold_pitch_P;
    volLimitationConfigMutable()->safeHold_pitch_I = geofenceConfig_safeHold_pitch_I;
    volLimitationConfigMutable()->safeHold_pitch_D = geofenceConfig_safeHold_pitch_D;
    volLimitationConfigMutable()->safeHold_roll_P = geofenceConfig_safeHold_roll_P;
    volLimitationConfigMutable()->safeHold_roll_I = geofenceConfig_safeHold_roll_I;
    volLimitationConfigMutable()->safeHold_roll_D = geofenceConfig_safeHold_roll_D;
    volLimitationConfigMutable()->safeHoldAngleMax = geofenceConfig_safeHoldAngleMax;
    volLimitationConfigMutable()->minSafeAltitude = geofenceConfig_minSafeAltitude;
    volLimitationConfigMutable()->minSats = geofenceConfig_minSats;
    volLimitationConfigMutable()->armingWithoutGps = geofenceConfig_armingWithoutGps;
    
    return 0;
}

static const OSD_Entry cmsx_menuGeofenceEntries[] =
{
    { "-- GEOFENCE --", OME_Label, NULL, NULL, 0},
    { "ALTITUDE",   OME_Bool,   NULL,  &geofenceConfig_altitudeLimitation, 0 },
    { "DISTANCE",   OME_Bool,   NULL,   &geofenceConfig_distanceLimitation, 0 },
    { "LANDING",    OME_Bool,   NULL,   &geofenceConfig_landingAllowed, 0 },
    { "MAX ALT",    OME_UINT16, NULL,   &(OSD_UINT16_t) {    &geofenceConfig_maxAltitude, 20, 500, 1}, 0 },
    { "ALT ALERT",  OME_UINT8,  NULL,   &(OSD_UINT8_t)  {     &geofenceConfig_alertAltitudeBeforeMax, 0, 50, 1}, 0 },
    { "MAX DIST",   OME_UINT16, NULL,   &(OSD_UINT16_t) {    &geofenceConfig_maxDistance, 50, 5000, 1}, 0 },
    { "DIST ALERT", OME_UINT8,  NULL,   &(OSD_UINT8_t)  {     &geofenceConfig_alertDistanceBeforeMax, 0, 50, 1}, 0 },
    { "MIN SATS", OME_UINT8,  NULL,   &(OSD_UINT8_t)  {     &geofenceConfig_minSats, 0, 50, 1}, 0 },
    { "ARM w/GPS",    OME_Bool,   NULL,   &geofenceConfig_armingWithoutGps, 0 },


    { "-- THROTTLE PID--",  OME_Label, NULL, NULL, 0},
    { "THR P",      OME_UINT16, NULL, &(OSD_UINT16_t)   {  &geofenceConfig_throttleP, 0, 500, 1}, 0 },
    { "THR I",     OME_UINT16, NULL, &(OSD_UINT16_t)    {   &geofenceConfig_throttleI,  0, 500, 1}, 0 },
    { "THR D",     OME_UINT16, NULL, &(OSD_UINT16_t)    {   &geofenceConfig_throttleD,  0, 500, 1}, 0 },
    { "THR MIN",   OME_UINT16, NULL, &(OSD_UINT16_t)    {   &geofenceConfig_throttleMin, 1000, 2000, 100}, 0 },
    { "THR MAX",   OME_UINT16, NULL, &(OSD_UINT16_t)    {   &geofenceConfig_throttleMax, 1000, 2000, 100}, 0 },
    { "THR HOVER",   OME_UINT16, NULL, &(OSD_UINT16_t)  { &geofenceConfig_throttleHover, 1000, 2000, 100}, 0 },

    { "-- SAFEHOLD --",  OME_Label, NULL, NULL, 0},
    { "ROLL P",      OME_UINT8, NULL, &(OSD_UINT8_t)   {  &geofenceConfig_safeHold_roll_P, 0, 200, 1}, 0 },
    { "ROLL I",     OME_UINT8, NULL, &(OSD_UINT8_t)    {   &geofenceConfig_safeHold_roll_I,  0, 200, 1}, 0 },
    { "ROLL D",     OME_UINT8, NULL, &(OSD_UINT8_t)    {   &geofenceConfig_safeHold_roll_D,  0, 200, 1}, 0 },
    { "PITCH P",      OME_UINT8, NULL, &(OSD_UINT8_t)   {  &geofenceConfig_safeHold_pitch_P, 0, 200, 1}, 0 },
    { "PITCH I",     OME_UINT8, NULL, &(OSD_UINT8_t)    {   &geofenceConfig_safeHold_pitch_I,  0, 200, 1}, 0 },
    { "PITCH D",     OME_UINT8, NULL, &(OSD_UINT8_t)    {   &geofenceConfig_safeHold_pitch_D,  0, 200, 1}, 0 },
    { "ANGLE MAX",  OME_UINT8,  NULL,   &(OSD_UINT8_t)  {     &geofenceConfig_safeHoldAngleMax, 0, 50, 1}, 0 }, 
    { "MAX ALT",  OME_UINT8,  NULL,   &(OSD_UINT8_t)  {     &geofenceConfig_minSafeAltitude, 10, 100, 1}, 0 },       
      
    { "BACK", OME_Back, NULL, NULL, 0 },
    { NULL, OME_END, NULL, NULL, 0 }
};

CMS_Menu cmsx_menuGeofence = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUGEO",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_Geofence_onEnter,
    .onExit = cmsx_Geofence_onExit,
    .entries = cmsx_menuGeofenceEntries
};
