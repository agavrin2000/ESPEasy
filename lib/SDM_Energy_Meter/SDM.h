/* Library for reading SDM 120/220/230/630 DDS238 Modbus Energy meters.
*  Reading via Hardware or Software Serial library & rs232<->rs485 converter
*  2016-2019 Reaper7 (tested on wemos d1 mini->ESP8266 with Arduino 1.9.0-beta & 2.4.1 esp8266 core)
*  crc calculation by Jaime García (https://github.com/peninquen/Modbus-Energy-Monitor-Arduino/)
*/
//------------------------------------------------------------------------------
#ifndef SDM_h
#define SDM_h
//------------------------------------------------------------------------------
#include <Arduino.h>
#include "SDM_Config_User.h"

#ifdef USE_HARDWARESERIAL
#include <HardwareSerial.h>
#else
#if defined(ESP8266) || defined(ESP32)
#include <ESPeasySerial.h>
#else
#include <SoftwareSerial.h>
#endif
#endif
//------------------------------------------------------------------------------
//DEFAULT CONFIG (DO NOT CHANGE ANYTHING!!! for changes use SDM_Config_User.h):
//------------------------------------------------------------------------------
#ifndef SDM_UART_BAUD
#define SDM_UART_BAUD 4800 //default baudrate
#endif

#ifndef DERE_PIN
#define DERE_PIN NOT_A_PIN //default digital pin for control MAX485 DE/RE lines (connect DE & /RE together to this pin)
#endif

#ifdef USE_HARDWARESERIAL

#ifndef SDM_UART_CONFIG
#define SDM_UART_CONFIG SERIAL_8N1 //default hardware uart config
#endif

#ifdef ESP8266
#ifndef SWAPHWSERIAL
#define SWAPHWSERIAL 0 //(only esp8266) when hwserial used, then swap uart pins from 3/1 to 13/15 (default not swap)
#endif
#endif

#endif

#ifndef MAX_MILLIS_TO_WAIT
#define MAX_MILLIS_TO_WAIT 500 //default max time to wait for response from SDM
#endif
//------------------------------------------------------------------------------
enum PM_Model
{
    SDM120C = 0,
    SDM220T = 1,
    SDM230 = 2,
    SDM630 = 3,
    DDS238 = 4
};
enum PM_Metric
{
    VOLTAGE,
    CURRENT,
    ACTIVE_POWER,
    APPARENT_POWER,
    REACTIVE_POWER,
    POWER_FACTOR,
    FREQUENCY,
    PHASE,
    ACTIVE_ENERGY,
    REACTIVE_ENERGY,
    VOLTAGE1,
    CURRENT1,
    ACTIVE_POWER1,
    APPARENT_POWER1,
    REACTIVE_POWER1,
    POWER_FACTOR1,
    PHASE1,
    VOLTAGE2,
    CURRENT2,
    ACTIVE_POWER2,
    APPARENT_POWER2,
    REACTIVE_POWER2,
    POWER_FACTOR2,
    PHASE2,
    VOLTAGE3,
    CURRENT3,
    ACTIVE_POWER3,
    APPARENT_POWER3,
    REACTIVE_POWER3,
    POWER_FACTOR3,
    PHASE3
};

//BYTES 3 & 4 (BELOW)
//SDM 120, 220,230 registers
#define SDM120C_VOLTAGE 0x0000                //V
#define SDM120C_CURRENT 0x0006                //A
#define SDM120C_ACTIVE_POWER 0x000C           //W
#define SDM120C_APPARENT_POWER 0x0012         //VA
#define SDM120C_REACTIVE_POWER 0x0018         //VAR
#define SDM120C_POWER_FACTOR 0x001E           //
#define SDM120C_PHASE_ANGLE 0x0024            //Degrees
#define SDM120C_FREQUENCY 0x0046              //Hz
#define SDM120C_IMPORT_ACTIVE_ENERGY 0x0048   //kWh
#define SDM120C_EXPORT_ACTIVE_ENERGY 0x004A   //kWh
#define SDM120C_IMPORT_REACTIVE_ENERGY 0x004C //kVARh
#define SDM120C_EXPORT_REACTIVE_ENERGY 0x004E //kVARh
#define SDM120C_TOTAL_ACTIVE_ENERGY 0x0156    //kWh
#define SDM120C_TOTAL_REACTIVE_ENERGY 0x0158  //kVARh

//SDM 530, 630 registers
#define SDM630_VOLTAGE1 0x0000             //V
#define SDM630_VOLTAGE2 0x0002             //V
#define SDM630_VOLTAGE3 0x0004             //V
#define SDM630_VOLTAGE_AVERAGE 0x002A      //V
#define SDM630_CURRENT1 0x0006             //A
#define SDM630_CURRENT2 0x0008             //A
#define SDM630_CURRENT3 0x000A             //A
#define SDM630_CURRENT_AVERAGE 0x002E      //A
#define SDM630_CURRENT_SUM 0x0030          //A
#define SDM630_ACTIVE_POWER1 0x000C        //W
#define SDM630_ACTIVE_POWER2 0x000E        //W
#define SDM630_ACTIVE_POWER3 0x0010        //W
#define SDM630_TOTAL_ACTIVE_POWER 0x0034   //W
#define SDM630_APPARENT_POWER1 0x0012      //VA
#define SDM630_APPARENT_POWER2 0x0014      //VA
#define SDM630_APPARENT_POWER3 0x0016      //VA
#define SDM630_TOTAL_APPARENT_POWER 0x0038 //VA
#define SDM630_REACTIVE_POWER1 0x0018      //VAr
#define SDM630_REACTIVE_POWER2 0x001A      //VAr
#define SDM630_REACTIVE_POWER3 0x001C      //VAr
#define SDM630_TOTAL_REACTIVE_POWER 0x003C //VAr
#define SDM630_POWER_FACTOR1 0x001E
#define SDM630_POWER_FACTOR2 0x0020
#define SDM630_POWER_FACTOR3 0x0022
#define SDM630_TOTAL_POWER_FACTOR 0x003E
#define SDM630_PHASE_ANGLE1 0x0024               //Degrees
#define SDM630_PHASE_ANGLE2 0x0026               //Degrees
#define SDM630_PHASE_ANGLE3 0x0028               //Degrees
#define SDM630_TOTAL_PHASE_ANGLE 0x0042          //Degrees
#define SDM630_FREQUENCY 0x0046                  //HZ
#define SDM630_IMPORT_ACTIVE_ENERGY 0x0048       //kWh
#define SDM630_EXPORT_ACTIVE_ENERGY 0x004A       //kWh
#define SDM630_IMPORT_REACTIVE_ENERGY 0x004C     //kVARh
#define SDM630_EXPORT_REACTIVE_ENERGY 0x004E     //kVARh
#define SDM630_APPARENT_ENERGY 0x0050            //kVAh
#define SDM630_LN1_TO_LN2_VOLTAGE 0x00C8         //V
#define SDM630_LN2_TO_LN3_VOLTAGE 0x00CA         //V
#define SDM630_LN3_TO_LN1_VOLTAGE 0x00CC         //V
#define SDM630_LN3_TO_LN1_VOLTAGE_AVERAGE 0x00CE //V
#define SDM630_CURRENT_NEUTRAL 0x00E0            //A
#define SDM630_TOTAL_ACTIVE_ENERGY 0x0156        //kWh
#define SDM630_TOTAL_REACTIVE_ENERGY 0x0158      //kVARh

//DDS 238 registers
#define DDS238_TOTAL_ENERGY 0x0000         //kWh
#define DDS238_EXPORT_ACTIVE_ENERGY 0x0008 //Wh
#define DDS238_IMPORT_ACTIVE_ENERGY 0x000A //Wh
#define DDS238_VOLTAGE 0x000C              //V
#define DDS238_CURRENT 0x000D              //A
#define DDS238_ACTIVE_POWER 0x000E         //VA
#define DDS238_REACTIVE_POWER 0x000F       //VAR
#define DDS238_POWER_FACTOR 0x0010         //
#define DDS238_FREQUENCY 0x0011            //Hz

//------------------------------------------------------------------------------
#define SDM_ERR_NO_ERROR 0          //no error
#define SDM_ERR_CRC_ERROR 1         //crc error
#define SDM_ERR_WRONG_BYTES 2       //bytes b0,b1 or b2 wrong
#define SDM_ERR_NOT_ENOUGHT_BYTES 3 //not enough bytes from sdm
#define SDM_ERR_TIMEOUT 4           //timeout
//------------------------------------------------------------------------------
class SDM
{
public:
#ifdef USE_HARDWARESERIAL
#if defined(ESP8266)
    SDM(HardwareSerial &serial, long baud = SDM_UART_BAUD, int dere_pin = DERE_PIN, int config = SDM_UART_CONFIG, bool swapuart = SWAPHWSERIAL);
#elif defined(ESP32)
    SDM(HardwareSerial &serial, long baud = SDM_UART_BAUD, int dere_pin = DERE_PIN, int config = SDM_UART_CONFIG, int8_t rx_pin = -1, int8_t tx_pin = -1);
#else
    SDM(HardwareSerial &serial, long baud = SDM_UART_BAUD, int dere_pin = DERE_PIN, int config = SDM_UART_CONFIG);
#endif
#else
#if defined(ESP8266) || defined(ESP32)
    SDM(ESPeasySerial& serial, long baud = SDM_UART_BAUD, int dere_pin = DERE_PIN);
#else
    SDM(SoftwareSerial &serial, long baud = SDM_UART_BAUD, int dere_pin = DERE_PIN);
#endif
#endif
    virtual ~SDM();

    void begin(void);
    uint16_t getErrCode(bool _clear = false);             //return last errorcode (optional clear this value, default flase)
    uint16_t getErrCount(bool _clear = false);            //return total errors count (optional clear this value, default flase)
    uint16_t getSuccCount(bool _clear = false);           //return total success count (optional clear this value, default false)
    void clearErrCode();                                  //clear last errorcode
    void clearErrCount();                                 //clear total errors count
    void clearSuccCount();                                //clear total success count
    float getValue(PM_Metric metric, PM_Model model, uint8_t node = 0x01);       //get PowerMeter value
    String getTitle(PM_Metric metric);
    String getUnit(PM_Metric metric);

    
private:
#ifdef USE_HARDWARESERIAL
    HardwareSerial &sdmSer;
#else
#if defined(ESP8266) || defined(ESP32)
    ESPeasySerial &sdmSer;
#else
    SoftwareSerial &sdmSer;
#endif
#endif

#ifdef USE_HARDWARESERIAL
    int _config = SDM_UART_CONFIG;
#if defined(ESP8266)
    bool _swapuart = SWAPHWSERIAL;
#elif defined(ESP32)
    int8_t _rx_pin = -1;
    int8_t _tx_pin = -1;
#endif
#endif
    long _baud = SDM_UART_BAUD;
    int _dere_pin = DERE_PIN;
    uint16_t readingerrcode = SDM_ERR_NO_ERROR; //4 = timeout; 3 = not enough bytes; 2 = number of bytes OK but bytes b0,b1 or b2 wrong, 1 = crc error
    uint16_t readingerrcount = 0;               //total errors counter
    uint32_t readingsuccesscount = 0;           //total success counter
    uint16_t calculateCRC(uint8_t *array, uint8_t num);
    float readVal(uint16_t reg, PM_Model model, uint8_t node); //read value from register = reg and from deviceId = node
};
#endif //SDM_h