/* Library for reading SDM 120/220/230/630 and DDS238 Modbus Energy meters.
*  Reading via Hardware or Software Serial library & rs232<->rs485 converter
*  2016-2019 Reaper7 (tested on wemos d1 mini->ESP8266 with Arduino 1.9.0-beta & 2.4.1 esp8266 core)
*  crc calculation by Jaime Garc�a (https://github.com/peninquen/Modbus-Energy-Monitor-Arduino/)
*/
//------------------------------------------------------------------------------
#include "SDM.h"
//------------------------------------------------------------------------------
#ifdef USE_HARDWARESERIAL
#if defined(ESP8266)
SDM::SDM(HardwareSerial &serial, long baud, int dere_pin, int config, bool swapuart) : sdmSer(serial)
{
    this->_baud = baud;
    this->_dere_pin = dere_pin;
    this->_config = config;
    this->_swapuart = swapuart;
}
#elif defined(ESP32)
SDM::SDM(HardwareSerial &serial, long baud, int dere_pin, int config, int8_t rx_pin, int8_t tx_pin) : sdmSer(serial)
{
    this->_baud = baud;
    this->_dere_pin = dere_pin;
    this->_config = config;
    this->_rx_pin = rx_pin;
    this->_tx_pin = tx_pin;
}
#else
SDM::SDM(HardwareSerial &serial, long baud, int dere_pin, int config) : sdmSer(serial)
{
    this->_baud = baud;
    this->_dere_pin = dere_pin;
    this->_config = config;
}
#endif
#else
#if defined(ESP8266) || defined(ESP32)
SDM::SDM(ESPeasySerial& serial, long baud, int dere_pin) : sdmSer(serial) 
{
  this->_baud = baud;
  this->_dere_pin = dere_pin;
}
#else
SDM::SDM(SoftwareSerial &serial, long baud, int dere_pin) : sdmSer(serial)
{
    this->_baud = baud;
    this->_dere_pin = dere_pin;
}
#endif
#endif

SDM::~SDM()
{
}

void SDM::begin(void)
{
#ifdef USE_HARDWARESERIAL
#if defined(ESP8266)
    sdmSer.begin(_baud, (SerialConfig)_config);
#elif defined(ESP32)
    sdmSer.begin(_baud, _config, _rx_pin, _tx_pin);
#else
    sdmSer.begin(_baud, _config);
#endif
#else
    sdmSer.begin(_baud);
#endif
#ifdef USE_HARDWARESERIAL
#ifdef ESP8266
    if (_swapuart)
        sdmSer.swap();
#endif
#endif
    if (_dere_pin != NOT_A_PIN)
    {
        pinMode(_dere_pin, OUTPUT);   //set output pin mode for DE/RE pin when used (for control MAX485)
        digitalWrite(_dere_pin, LOW); //set init state to receive from SDM -> DE Disable, /RE Enable (for control MAX485)
    }
}

float SDM::readVal(uint16_t reg, PM_Model model, uint8_t node)
{

    float res = NAN; //return value
    uint16_t readErr = SDM_ERR_NO_ERROR;

    uint16_t resp_size = 9;                                            //default response size for Eastron SDM-meters
    uint8_t sdmarr[9] = {node, 0x04, 0x00, 0x00, 0x00, 0x02, 0, 0, 0}; //default command template for Eastron SDM-meters

    if (model == DDS238)
    {
        sdmarr[1] = 0x03;
        if (reg == DDS238_VOLTAGE || reg == DDS238_CURRENT || reg == DDS238_ACTIVE_POWER || reg == DDS238_REACTIVE_POWER ||
            reg == DDS238_POWER_FACTOR || reg == DDS238_FREQUENCY)
        {
            sdmarr[5] = 0x01;
            resp_size = 7;
        }
    }

    sdmarr[2] = highByte(reg);
    sdmarr[3] = lowByte(reg);

    uint16_t temp = calculateCRC(sdmarr, 6); //calculate out crc only from first 6 bytes
    sdmarr[6] = lowByte(temp);
    sdmarr[7] = highByte(temp);

#ifndef USE_HARDWARESERIAL
    sdmSer.listen(); //enable softserial rx interrupt
#endif

    while (sdmSer.available() > 0)
    { //read serial if any old data is available
        sdmSer.read();
    }

    if (_dere_pin != NOT_A_PIN)
        digitalWrite(_dere_pin, HIGH); //transmit to SDM  -> DE Enable, /RE Disable (for control MAX485)

    delay(2); //fix for issue (nan reading) by sjfaustino: https://github.com/reaper7/SDM_Energy_Meter/issues/7#issuecomment-272111524

    sdmSer.write(sdmarr, 8); //send 8 bytes

    sdmSer.flush(); //clear out tx buffer

    if (_dere_pin != NOT_A_PIN)
        digitalWrite(_dere_pin, LOW); //receive from SDM -> DE Disable, /RE Enable (for control MAX485)

    unsigned long resptime = millis() + MAX_MILLIS_TO_WAIT;
    while (sdmSer.available() < resp_size)
    {
        if (resptime < millis())
        {
            readErr = SDM_ERR_TIMEOUT; //err debug (4)
            break;
        }
        yield();
    }

    if (readErr == SDM_ERR_NO_ERROR)
    { //if no timeout...
        if (sdmSer.available() >= resp_size)
        {

            for (int n = 0; n < resp_size; n++)
            {
                sdmarr[n] = sdmSer.read();
            }

            if (sdmarr[0] == node && sdmarr[2] == (resp_size - 5) &&
                ((sdmarr[1] == 0x03 && model == DDS238) || (sdmarr[1] == 0x04 && model != DDS238)))
            {

                if ((calculateCRC(sdmarr, resp_size - 2)) == ((sdmarr[resp_size - 1] << 8) | sdmarr[resp_size - 2]))
                {
                    //calculate crc from first 7 bytes and compare with received crc (bytes 7 & 8)
                    if(model == DDS238) {
                        uint32_t dds238_res;
                        if (resp_size == 9)
                        {
                            ((uint8_t *)&dds238_res)[3] = sdmarr[3];
                            ((uint8_t *)&dds238_res)[2] = sdmarr[4];
                            ((uint8_t *)&dds238_res)[1] = sdmarr[5];
                            ((uint8_t *)&dds238_res)[0] = sdmarr[6];
                        }
                        else
                        {
                            ((uint8_t *)&dds238_res)[3] = 0x00;
                            ((uint8_t *)&dds238_res)[2] = 0x00;
                            ((uint8_t *)&dds238_res)[1] = sdmarr[3];
                            ((uint8_t *)&dds238_res)[0] = sdmarr[4];
                        }
                        res = float(dds238_res);
                    }
                    else {
                        ((uint8_t *)&res)[3] = sdmarr[3];
                        ((uint8_t *)&res)[2] = sdmarr[4];
                        ((uint8_t *)&res)[1] = sdmarr[5];
                        ((uint8_t *)&res)[0] = sdmarr[6];
                    }
                }
                else
                {
                    readErr = SDM_ERR_CRC_ERROR; //err debug (1)
                }
            }
            else
            {
                readErr = SDM_ERR_WRONG_BYTES; //err debug (2)
            }
        }
        else
        {
            readErr = SDM_ERR_NOT_ENOUGHT_BYTES; //err debug (3)
        }
    }

    if (readErr != SDM_ERR_NO_ERROR)
    { //if error then copy temp error value to global val and increment global error counter
        readingerrcode = readErr;
        readingerrcount++;
    }
    else
    {
        ++readingsuccesscount;
    }

    while (sdmSer.available() > 0)
    { //read redundant serial bytes, if any
        sdmSer.read();
    }

#ifndef USE_HARDWARESERIAL
    sdmSer.end(); //disable softserial rx interrupt
#endif

    return (res);
}

uint16_t SDM::getErrCode(bool _clear)
{
    uint16_t _tmp = readingerrcode;
    if (_clear == true)
        clearErrCode();
    return (_tmp);
}

uint16_t SDM::getErrCount(bool _clear)
{
    uint16_t _tmp = readingerrcount;
    if (_clear == true)
        clearErrCount();
    return (_tmp);
}

uint16_t SDM::getSuccCount(bool _clear)
{
    uint16_t _tmp = readingsuccesscount;
    if (_clear == true)
        clearSuccCount();
    return (_tmp);
}

void SDM::clearErrCode()
{
    readingerrcode = SDM_ERR_NO_ERROR;
}

void SDM::clearErrCount()
{
    readingerrcount = 0;
}

void SDM::clearSuccCount()
{
    readingsuccesscount = 0;
}

uint16_t SDM::calculateCRC(uint8_t *array, uint8_t num)
{
    uint16_t _crc, _flag;
    _crc = 0xFFFF;
    for (uint8_t i = 0; i < num; i++)
    {
        _crc = _crc ^ array[i];
        for (uint8_t j = 8; j; j--)
        {
            _flag = _crc & 0x0001;
            _crc >>= 1;
            if (_flag)
                _crc ^= 0xA001;
        }
    }
    return _crc;
}

float SDM::getValue(PM_Metric metric, PM_Model model, uint8_t node)
{
    switch (model)
    {
    case SDM120C:
    case SDM220T:
    case SDM230:
        switch (metric)
        {
        case VOLTAGE:
            return readVal(SDM120C_VOLTAGE, model, node);
        case CURRENT:
            return readVal(SDM120C_CURRENT, model, node);
        case ACTIVE_POWER:
            return readVal(SDM120C_ACTIVE_POWER, model, node);
        case APPARENT_POWER:
            return readVal(SDM120C_APPARENT_POWER, model, node);
        case REACTIVE_POWER:
            return readVal(SDM120C_REACTIVE_POWER, model, node);
        case POWER_FACTOR:
            return readVal(SDM120C_POWER_FACTOR, model, node);
        case FREQUENCY:
            return readVal(SDM120C_FREQUENCY, model, node);
        case PHASE:
            return readVal(SDM120C_PHASE_ANGLE, model, node);
        case ACTIVE_ENERGY:
            return readVal(SDM120C_TOTAL_ACTIVE_ENERGY, model, node);
        case REACTIVE_ENERGY:
            return readVal(SDM120C_TOTAL_REACTIVE_ENERGY, model, node);
        default:
            return NAN;
        }
    case DDS238:
        switch (metric)
        {
        case VOLTAGE:
            return readVal(DDS238_VOLTAGE, model, node) / 10;
        case CURRENT:
            return readVal(DDS238_CURRENT, model, node) / 100;
        case ACTIVE_POWER:
            return readVal(DDS238_ACTIVE_POWER, model, node);
        case APPARENT_POWER:
        {
            float rp = readVal(DDS238_REACTIVE_POWER, model, node);
            float ap = readVal(DDS238_ACTIVE_POWER, model, node);
            return sqrt(rp * rp + ap * ap);
        }
        case REACTIVE_POWER:
            return readVal(DDS238_REACTIVE_POWER, model, node);
        case POWER_FACTOR:
            return readVal(DDS238_POWER_FACTOR, model, node) / 1000;
        case FREQUENCY:
            return readVal(DDS238_FREQUENCY, model, node) / 100;
        case ACTIVE_ENERGY:
            return readVal(DDS238_TOTAL_ENERGY, model, node) / 100;
        default:
            return NAN;
        }
    case SDM630:
        switch (metric)
        {
        case VOLTAGE:
            return readVal(SDM630_VOLTAGE_AVERAGE, model, node);
        case CURRENT:
            return readVal(SDM630_CURRENT_SUM, model, node);
        case ACTIVE_POWER:
            return readVal(SDM630_TOTAL_ACTIVE_POWER, model, node);
        case APPARENT_POWER:
            return readVal(SDM630_TOTAL_APPARENT_POWER, model, node);
        case REACTIVE_POWER:
            return readVal(SDM630_TOTAL_REACTIVE_POWER, model, node);
        case POWER_FACTOR:
            return readVal(SDM630_TOTAL_POWER_FACTOR, model, node);
        case FREQUENCY:
            return readVal(SDM630_FREQUENCY, model, node);
        case ACTIVE_ENERGY:
            return readVal(SDM630_TOTAL_ACTIVE_ENERGY, model, node);
        case REACTIVE_ENERGY:
            return readVal(SDM630_TOTAL_REACTIVE_ENERGY, model, node);
        case PHASE:
            return readVal(SDM630_TOTAL_PHASE_ANGLE, model, node);

        case VOLTAGE1:
            return readVal(SDM630_VOLTAGE1, model, node);
        case CURRENT1:
            return readVal(SDM630_CURRENT1, model, node);
        case ACTIVE_POWER1:
            return readVal(SDM630_ACTIVE_POWER1, model, node);
        case APPARENT_POWER1:
            return readVal(SDM630_APPARENT_POWER1, model, node);
        case REACTIVE_POWER1:
            return readVal(SDM630_REACTIVE_POWER1, model, node);
        case POWER_FACTOR1:
            return readVal(SDM630_POWER_FACTOR1, model, node);
        case PHASE1:
            return readVal(SDM630_PHASE_ANGLE1, model, node);
        case VOLTAGE2:
            return readVal(SDM630_VOLTAGE2, model, node);
        case CURRENT2:
            return readVal(SDM630_CURRENT2, model, node);
        case ACTIVE_POWER2:
            return readVal(SDM630_ACTIVE_POWER2, model, node);
        case APPARENT_POWER2:
            return readVal(SDM630_APPARENT_POWER2, model, node);
        case REACTIVE_POWER2:
            return readVal(SDM630_REACTIVE_POWER2, model, node);
        case POWER_FACTOR2:
            return readVal(SDM630_POWER_FACTOR2, model, node);
        case PHASE2:
            return readVal(SDM630_PHASE_ANGLE2, model, node);
        case VOLTAGE3:
            return readVal(SDM630_VOLTAGE3, model, node);
        case CURRENT3:
            return readVal(SDM630_CURRENT3, model, node);
        case ACTIVE_POWER3:
            return readVal(SDM630_ACTIVE_POWER3, model, node);
        case APPARENT_POWER3:
            return readVal(SDM630_APPARENT_POWER3, model, node);
        case REACTIVE_POWER3:
            return readVal(SDM630_REACTIVE_POWER3, model, node);
        case POWER_FACTOR3:
            return readVal(SDM630_POWER_FACTOR3, model, node);
        case PHASE3:
            return readVal(SDM630_PHASE_ANGLE3, model, node);
        default:
            return NAN;
        }
    default:
        return NAN;
    }
}
    String SDM::getTitle(PM_Metric metric)
    {
        switch (metric)
        {
        case VOLTAGE:
            return F("Voltage (V)");
        case CURRENT:
            return F("Current (A)");
        case ACTIVE_POWER:
            return F("Active Power (W)");
        case APPARENT_POWER:
            return F("Apparent Power (VA)");
        case REACTIVE_POWER:
            return F("Reactive Power (VAr)");
        case POWER_FACTOR:
            return F("Power Factor (cos-phi)");
        case FREQUENCY:
            return F("Frequency (Hz)");
        case ACTIVE_ENERGY:
            return F("Active Energy (kWh)");
        case REACTIVE_ENERGY:
            return F("Reactive Energy (kVArh)");
        case PHASE:
            return F("Phase (deg)");
        case VOLTAGE1:
            return F("Voltage(L1) (V)");
        case CURRENT1:
            return F("Current(L1) (A)");
        case ACTIVE_POWER1:
            return F("Active Power(L1) (W)");
        case APPARENT_POWER1:
            return F("Apparent Power(L1) (VA)");
        case REACTIVE_POWER1:
            return F("Reactive Power(L1) (VAr)");
        case POWER_FACTOR1:
            return F("Power Factor(L1) (cos-phi)");
        case PHASE1:
            return F("Phase(L1) (deg)");
        case VOLTAGE2:
            return F("Voltage(L2) (V)");
        case CURRENT2:
            return F("Current(L2) (A)");
        case ACTIVE_POWER2:
            return F("Active Power(L2) (W)");
        case APPARENT_POWER2:
            return F("Apparent Power(L2) (VA)");
        case REACTIVE_POWER2:
            return F("Reactive Power(L2) (VAr)");
        case POWER_FACTOR2:
            return F("Power Factor(L2) (cos-phi)");
        case PHASE2:
            return F("Phase(L2) (deg)");
        case VOLTAGE3:
            return F("Voltage(L3) (V)");
        case CURRENT3:
            return F("Current(L3) (A)");
        case ACTIVE_POWER3:
            return F("Active Power(L3) (W)");
        case APPARENT_POWER3:
            return F("Apparent Power(L3) (VA)");
        case REACTIVE_POWER3:
            return F("Reactive Power(L3) (VAr)");
        case POWER_FACTOR3:
            return F("Power Factor(L3) (cos-phi)");
        case PHASE3:
            return F("Phase(L3) (deg)");
        }
        return "";
    }

    String SDM::getUnit(PM_Metric metric)
    {
        switch (metric)
        {
        case VOLTAGE:
            return F("V");
        case CURRENT:
            return F("A");
        case ACTIVE_POWER:
            return F("W");
        case APPARENT_POWER:
            return F("VA");
        case REACTIVE_POWER:
            return F("VAr");
        case POWER_FACTOR:
            return F("cos-phi");
        case FREQUENCY:
            return F("Hz");
        case ACTIVE_ENERGY:
            return F("kWh");
        case REACTIVE_ENERGY:
            return F("kVArh");
        case PHASE:
            return F("deg");
        case VOLTAGE1:
            return F("V");
        case CURRENT1:
            return F("A");
        case ACTIVE_POWER1:
            return F("W");
        case APPARENT_POWER1:
            return F("VA");
        case REACTIVE_POWER1:
            return F("VAr");
        case POWER_FACTOR1:
            return F("cos-phi");
        case PHASE1:
            return F("deg");
        case VOLTAGE2:
            return F("V");
        case CURRENT2:
            return F("A");
        case ACTIVE_POWER2:
            return F("W");
        case APPARENT_POWER2:
            return F("VA");
        case REACTIVE_POWER2:
            return F("VAr");
        case POWER_FACTOR2:
            return F("cos-phi");
        case PHASE2:
            return F("deg");
        case VOLTAGE3:
            return F("V");
        case CURRENT3:
            return F("A");
        case ACTIVE_POWER3:
            return F("W");
        case APPARENT_POWER3:
            return F("VA");
        case REACTIVE_POWER3:
            return F("VAr");
        case POWER_FACTOR3:
            return F("cos-phi");
        case PHASE3:
            return F("deg");
        }
        return "";
    }