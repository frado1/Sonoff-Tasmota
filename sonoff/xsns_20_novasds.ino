/*
  xsns_20_novasds.ino - Nova SDS011/SDS021 particle concentration sensor support for Sonoff-Tasmota

  Copyright (C) 2018  Theo Arends

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

#ifdef USE_NOVA_SDS
/*********************************************************************************************\
   Nova Fitness SDS011 (and possibly SDS021) particle concentration sensor
   For background information see http://aqicn.org/sensor/sds011/

   Hardware Serial will be selected if GPIO3 = [SDS0X01]
  \*********************************************************************************************/

#include <TasmotaSerial.h>

TasmotaSerial *NovaSdsSerial;

// States:
//  0 = Serial/SDS not ready
//  1 = Start and Read Firmware version 
//  2 = Running
//  3 = Get next valid pollution data
//  Transition: 1 -> 0 -> 1 -> 2 -> 3 -> 2 -> 3 -> .....
typedef enum {
   NOVASDS_NOT_READY,
   NOVASDS_START_FIRMWARE,
   NOVASDS_RUNNING,
   NOVASDS_GET_POLLUTION
} NovaSdsStates_t;
NovaSdsStates_t novasds_state = NOVASDS_START_FIRMWARE;

//
uint8_t novasds_valid = 0;
uint8_t novasds_running = 255;
uint8_t novasds_read_tick = 30;
uint8_t novasds_wakup_tick = 179;

uint8_t novasds_secTicker = 0;
uint8_t novasds_50mSecTicker = 0;

byte NovaSdsResponse[9];

void NovaSdsStop();
void NovaSdsStart();

struct sds011data {
  uint16_t pm100;
  uint16_t pm25;
  char     firmware[9];
  bool     awake;
} novasds_data;

void AddLogBuffer(byte loglevel, const char *text, uint8_t *buffer, int count)
{
  snprintf_P(log_data, sizeof(log_data), text);
  for (int i = 0; i < count; i++) {
    snprintf_P(log_data, sizeof(log_data), PSTR("%s %02X"), log_data, *(buffer++));
  }
  AddLog(loglevel);
}

bool NovaSdsReceiveReady()
{
  return (NovaSdsSerial->available() > 1);
}


void NovaSdsWriteData(uint8_t cmd, uint8_t mode, uint8_t data) {
  uint8_t novasds_start_cmd[] = {0xAA, 0xB4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x06, 0xAB};

  novasds_start_cmd[2] = cmd;
  novasds_start_cmd[3] = mode;
  novasds_start_cmd[4] = data;


  uint16_t checksum = 0;
  for (uint8_t i = 2; i <= 16; i++)
    checksum += novasds_start_cmd[i];

  novasds_start_cmd[17] = (uint8_t)(checksum % 256);

  AddLogBuffer(LOG_LEVEL_DEBUG_MORE, "SDS: NovaSdsWrite", novasds_start_cmd, 19);

  while (NovaSdsSerial->available())  {  // read serial if any old data is available
    NovaSdsSerial->read();
  }
  NovaSdsSerial->flush();

  NovaSdsSerial->write(novasds_start_cmd, sizeof(novasds_start_cmd));
}

bool NovaSdsReadData()
{
  if (! NovaSdsReceiveReady() ) return false;

  memset(NovaSdsResponse, '\0', sizeof(NovaSdsResponse));

  // Read garbage until start token (0xAA)
  while ((NovaSdsSerial->peek() != 0xAA) && NovaSdsSerial->available()) {
    NovaSdsSerial->read();
  }

  NovaSdsSerial->readBytes(NovaSdsResponse, sizeof(NovaSdsResponse));
  NovaSdsSerial->flush();

  if (NovaSdsResponse[8] == ((NovaSdsResponse[2] + NovaSdsResponse[3] + NovaSdsResponse[4] + NovaSdsResponse[5] + NovaSdsResponse[6] + NovaSdsResponse[7]) & 0xFF)) {
    return true;
  } else {
    AddLogBuffer(LOG_LEVEL_DEBUG_MORE, PSTR("SDS: " D_CHECKSUM_FAILURE), NovaSdsResponse, sizeof(NovaSdsResponse));
    return false;
  }

  return false;
}

void NovaSdsStart()
{
  AddLog_P(LOG_LEVEL_DEBUG, "SDS: Start");
  NovaSdsWriteData(0x06, 0x01, 0x01);
  novasds_running = 1;
}

void NovaSdsStop()
{
  AddLog_P(LOG_LEVEL_DEBUG, "SDS: Stop");
  NovaSdsWriteData(0x06, 0x01, 0x00);
  novasds_running = 0;
}

void NovaSdsGetIsAwake()
{
  AddLog_P(LOG_LEVEL_DEBUG, "SDS: GetIsAwake");
  NovaSdsWriteData(0x06, 0x00, 0x00);
}

void NovaSdsGetFirmware()
{
  AddLog_P(LOG_LEVEL_DEBUG, "SDS: GetFirmware");
  NovaSdsWriteData(0x07, 0x00, 0x00);
}

bool NovaSdsParsePolutionData()
{
  AddLog_P(LOG_LEVEL_DEBUG, PSTR("SDS: ReadPolutionData"));
  novasds_data.pm25 = (NovaSdsResponse[2] + 256 * NovaSdsResponse[3]);
  novasds_data.pm100 = (NovaSdsResponse[4] + 256 * NovaSdsResponse[5]);
  return true;
}

bool NovaSdsParseFirmware()
{
  AddLog_P(LOG_LEVEL_DEBUG, PSTR("SDS: ReadFirmware"));
  sprintf(novasds_data.firmware, "%02d.%02d.%02d", NovaSdsResponse[3], NovaSdsResponse[4], NovaSdsResponse[5]);
  return true;
}

bool NovaSdsParseAwake()
{
  AddLog_P(LOG_LEVEL_DEBUG, PSTR("SDS: ReadAwake"));
  novasds_data.awake = (NovaSdsResponse[4] == 0x01);
  return true;
}

/*********************************************************************************************/

void NovaSds50mSecond()
{
  novasds_50mSecTicker++;
  if ((novasds_50mSecTicker % 6) == 0) {     // Every 300 mSec
    bool data_ready = NovaSdsReceiveReady();

    if (data_ready && NovaSdsReadData() ) {
      switch (NovaSdsResponse[1]) {
        case 0xC5:
          AddLogBuffer(LOG_LEVEL_DEBUG_MORE, PSTR("SDS: process RESPONSE"), NovaSdsResponse, sizeof(NovaSdsResponse));
          switch (NovaSdsResponse[2] ) {
            case 0x07:
              NovaSdsParseFirmware();
              AddLog_P(LOG_LEVEL_INFO, novasds_data.firmware);
              break;

            case 0x06:
              NovaSdsParseAwake();
              novasds_running = (novasds_data.awake ? 1 : 0);
              AddLog_P(LOG_LEVEL_INFO,  (novasds_data.awake ? PSTR(D_SENSOR " " D_ON) : PSTR(D_SENSOR " " D_OFF)));
              break;
          }
          break;

        case 0xC0:
          if ( novasds_state == NOVASDS_GET_POLLUTION ) {
            // try to take a single stable reading and sleep the sensor
            AddLogBuffer(LOG_LEVEL_DEBUG_MORE, PSTR("SDS: process POLLUTION DATA"), NovaSdsResponse, sizeof(NovaSdsResponse));
            NovaSdsParsePolutionData();
            novasds_valid = 1;
            novasds_state = NOVASDS_RUNNING;
            NovaSdsStop();
          }
          break;

        default:
          break;
      }
    }
  }

  if ((novasds_50mSecTicker % 20) == 0) {     // Every 1 Sec
    novasds_50mSecTicker = 0;

    novasds_secTicker++;

//    if ( (novasds_secTicker % 10) == 0) {
//      snprintf_P(log_data, sizeof(log_data), PSTR("SDS: secTicker %d, state %d, running %d, valid %d"), novasds_secTicker, novasds_state, novasds_running, novasds_valid);
//      AddLog(LOG_LEVEL_DEBUG_MORE);
//    }

    // Read Firmware on state == 1
    if ( novasds_state == NOVASDS_START_FIRMWARE ) {
      NovaSdsGetFirmware();
      novasds_state = NOVASDS_RUNNING;      // Set state to 2, Firmware read
    }

    if (novasds_secTicker < novasds_read_tick) {
      // wake up the sensor and wait read ticks to stabalize the sensor
      if (!novasds_running) {
        NovaSdsStart();
      }
    } else if ( novasds_secTicker == novasds_read_tick) {
      // Start meassuring
      novasds_state = NOVASDS_GET_POLLUTION;
    } else if (novasds_secTicker >= novasds_wakup_tick) {
      // reset the counter
      novasds_secTicker = 0;
    } 
  }
}

/*********************************************************************************************/

void NovaSdsSecond()                 // Every second
{
}

/*********************************************************************************************/

void NovaSdsInit()
{
  novasds_state = NOVASDS_NOT_READY;
  if (pin[GPIO_SDS0X1_RX] < 99 && pin[GPIO_SDS0X1_TX] < 99) {
    NovaSdsSerial = new TasmotaSerial(pin[GPIO_SDS0X1_RX], pin[GPIO_SDS0X1_TX], 1);
    if (NovaSdsSerial->begin(9600)) {
      if (NovaSdsSerial->hardwareSerial()) {
        ClaimSerial();
      }
      novasds_state = NOVASDS_START_FIRMWARE;  // Set state to 1 to read Firmware
      NovaSdsStop();      // Start with stopped sensor
    }

    // Reset tickers
    novasds_secTicker = 0;
    novasds_50mSecTicker = 0;
  }
}

#ifdef USE_WEBSERVER
const char HTTP_SDS0X1_SNS[] PROGMEM = "%s"
                                       "{s}SDS0X1 " D_ENVIRONMENTAL_CONCENTRATION " 2.5 " D_UNIT_MICROMETER "{m}%s " D_UNIT_MICROGRAM_PER_CUBIC_METER "{e}"
                                       "{s}SDS0X1 " D_ENVIRONMENTAL_CONCENTRATION " 10 " D_UNIT_MICROMETER "{m}%s " D_UNIT_MICROGRAM_PER_CUBIC_METER "{e}"
                                       "{s}SDS0X1 " D_SENSOR "{m}%s{e}";      // {s} = <tr><th>, {m} = </th><td>, {e} = </td> </tr>
#endif  // USE_WEBSERVER

void NovaSdsShow(boolean json)
{
  if (novasds_valid) {
    char pm10[10];
    char pm2_5[10];
    float pm10f = (float)(novasds_data.pm100) / 10.0f;
    float pm2_5f = (float)(novasds_data.pm25) / 10.0f;
    dtostrfd(pm10f, 1, pm10);
    dtostrfd(pm2_5f, 1, pm2_5);
    if (json) {
      snprintf_P(mqtt_data, sizeof(mqtt_data), PSTR("%s,\"SDS0X1\":{\"PM2.5\":%s,\"PM10\":%s}"), mqtt_data, pm2_5, pm10);
#ifdef USE_DOMOTICZ
      if (0 == tele_period) {
        DomoticzSensor(DZ_VOLTAGE, pm2_5);  // PM2.5
        DomoticzSensor(DZ_CURRENT, pm10);   // PM10
      }
#endif  // USE_DOMOTICZ
#ifdef USE_WEBSERVER
    } else {
      snprintf_P(mqtt_data, sizeof(mqtt_data), HTTP_SDS0X1_SNS, mqtt_data, pm2_5, pm10, novasds_data.awake ? D_ON : D_OFF);
#endif  // USE_WEBSERVER
    }
  }
}

/*********************************************************************************************\
   Interface
  \*********************************************************************************************/

#define XSNS_20

boolean Xsns20(byte function)
{
  boolean result = false;

  if (novasds_state != NOVASDS_NOT_READY) {
    switch (function) {
      case FUNC_INIT:
        NovaSdsInit();
        break;
      case FUNC_EVERY_50_MSECOND:
        NovaSds50mSecond();
        break;
      case FUNC_EVERY_SECOND:
        NovaSdsSecond();
        break;
      case FUNC_JSON_APPEND:
        NovaSdsShow(1);
        break;
#ifdef USE_WEBSERVER
      case FUNC_WEB_APPEND:
        NovaSdsShow(0);
        break;
#endif  // USE_WEBSERVER
    }
  }
  return result;
}

#endif  // USE_NOVA_SDS
