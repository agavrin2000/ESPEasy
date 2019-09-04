#ifdef USES_P222

//#######################################################################################################
//######## Plugin 222: SDM120C/220T/230/530/630 Eastron Energy Meter and Hiking DDS238-1-ZN #############
//#######################################################################################################
/*
  Plugin written by: Anton Gavrin anton.gavrin@gmail.com

  This plugin reads values from Power meters: Eastron SDM120C/SDM220T/SDM230/SDM630 Energy Meter 
  and Hiking DDS238-1-ZN.
*/

#define PLUGIN_222
#define PLUGIN_ID_222         222
#define PLUGIN_NAME_222       "Energy (AC) - Eastron SDMxx/Hiking DDSxxx [TESTING]"

#define P222_DEV_ID          PCONFIG(0)
#define P222_DEV_ID_LABEL    PCONFIG_LABEL(0)
#define P222_MODEL           PCONFIG(1)
#define P222_MODEL_LABEL     PCONFIG_LABEL(1)
#define P222_BAUDRATE        PCONFIG(2)
#define P222_BAUDRATE_LABEL  PCONFIG_LABEL(2)
#define P222_QUERY1          PCONFIG(3)
#define P222_QUERY2          PCONFIG(4)
#define P222_QUERY3          PCONFIG(5)
#define P222_QUERY4          PCONFIG(6)
#define P222_DEPIN           CONFIG_PIN3

#define P222_DEV_ID_DFLT     1
#define P222_MODEL_DFLT      SDM120C
#define P222_BAUDRATE_DFLT   1  // 9600 baud
#define P222_QUERY1_DFLT     VOLTAGE
#define P222_QUERY2_DFLT     CURRENT
#define P222_QUERY3_DFLT     ACTIVE_POWER
#define P222_QUERY4_DFLT     FREQUENCY

#define P222_NR_OUTPUT_VALUES          4
#define P222_NR_OUTPUT_OPTIONS        31
#define P222_QUERY1_CONFIG_POS  3


#include <SDM.h>    // Requires SDM library from Reaper7 - https://github.com/reaper7/SDM_Energy_Meter/
#include <ESPeasySerial.h>

// These pointers may be used among multiple instances of the same plugin,
// as long as the same serial settings are used.
ESPeasySerial* Plugin_222_SoftSerial = NULL;
SDM* Plugin_222_SDM = NULL;
boolean Plugin_222_init = false;

boolean Plugin_222(byte function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function)
  {

    case PLUGIN_DEVICE_ADD:
      {
        Device[++deviceCount].Number = PLUGIN_ID_222;
        Device[deviceCount].Type = DEVICE_TYPE_TRIPLE;     // connected through 3 datapins
        Device[deviceCount].VType = SENSOR_TYPE_QUAD;
        Device[deviceCount].Ports = 0;
        Device[deviceCount].PullUpOption = false;
        Device[deviceCount].InverseLogicOption = false;
        Device[deviceCount].FormulaOption = true;
        Device[deviceCount].ValueCount = P222_NR_OUTPUT_VALUES;
        Device[deviceCount].SendDataOption = true;
        Device[deviceCount].TimerOption = true;
        Device[deviceCount].GlobalSyncOption = true;
        break;
      }

    case PLUGIN_GET_DEVICENAME:
      {
        string = F(PLUGIN_NAME_222);
        break;
      }

    case PLUGIN_GET_DEVICEVALUENAMES:
      {
        for (byte i = 0; i < VARS_PER_TASK; ++i) {
          if ( i < P222_NR_OUTPUT_VALUES) {
            byte choice = PCONFIG(i + P222_QUERY1_CONFIG_POS);
            safe_strncpy(
              ExtraTaskSettings.TaskDeviceValueNames[i],
              Plugin_222_SDM->getUnit(PM_Metric(choice)),
              sizeof(ExtraTaskSettings.TaskDeviceValueNames[i]));
          } else {
            ZERO_FILL(ExtraTaskSettings.TaskDeviceValueNames[i]);
          }
        }
        break;
      }

    case PLUGIN_GET_DEVICEGPIONAMES:
      {
        serialHelper_getGpioNames(event);
        event->String3 = formatGpioName_output_optional("DE");
        break;
      }

    case PLUGIN_SET_DEFAULTS:
      {
        P222_DEV_ID = P222_DEV_ID_DFLT;
        P222_MODEL = P222_MODEL_DFLT;
        P222_BAUDRATE = P222_BAUDRATE_DFLT;
        P222_QUERY1 = P222_QUERY1_DFLT;
        P222_QUERY2 = P222_QUERY2_DFLT;
        P222_QUERY3 = P222_QUERY3_DFLT;
        P222_QUERY4 = P222_QUERY4_DFLT;

        success = true;
        break;
      }

    case PLUGIN_WEBFORM_LOAD:
      {
        serialHelper_webformLoad(event);

        if (P222_DEV_ID == 0 || P222_DEV_ID > 247 || P222_BAUDRATE >= 6) {
          // Load some defaults
          P222_DEV_ID = P222_DEV_ID_DFLT;
          P222_MODEL = P222_MODEL_DFLT;
          P222_BAUDRATE = P222_BAUDRATE_DFLT;
          P222_QUERY1 = P222_QUERY1_DFLT;
          P222_QUERY2 = P222_QUERY2_DFLT;
          P222_QUERY3 = P222_QUERY3_DFLT;
          P222_QUERY4 = P222_QUERY4_DFLT;
        }
        addFormNumericBox(F("Modbus Address"), P222_DEV_ID_LABEL, P222_DEV_ID, 1, 247);

        {
          String options_model[5] = { F("SDM120C"), F("SDM220T"), F("SDM230"), F("SDM630"), F("DDS238") };
          addFormSelector(F("Model Type"), P222_MODEL_LABEL, 5, options_model, NULL, P222_MODEL );
        }
        {
          String options_baudrate[6];
          for (int i = 0; i < 6; ++i) {
            options_baudrate[i] = String(p222_storageValueToBaudrate(i));
          }
          addFormSelector(F("Baud Rate"), P222_BAUDRATE_LABEL, 6, options_baudrate, NULL, P222_BAUDRATE );
        }

        if (P222_MODEL == SDM120C && P222_BAUDRATE > 3)
          addFormNote(F("<span style=\"color:red\"> SDM120 only allows up to 9600 baud with default 2400!</span>"));

        if (P222_MODEL == DDS238 && P222_BAUDRATE > 3)
          addFormNote(F("<span style=\"color:red\"> DDS238 only allows up to 9600 baud with default 9600!</span>"));

        if (P222_MODEL == 3 && P222_BAUDRATE == 0)
          addFormNote(F("<span style=\"color:red\"> SDM630 only allows 2400 to 38400 baud with default 9600!</span>"));

        if (Plugin_222_SDM != nullptr) {
          addRowLabel(F("Checksum (pass/fail)"));
          String chksumStats;
          chksumStats = Plugin_222_SDM->getSuccCount();
          chksumStats += '/';
          chksumStats += Plugin_222_SDM->getErrCount();
          addHtml(chksumStats);
        }

        {
          // In a separate scope to free memory of String array as soon as possible
          sensorTypeHelper_webformLoad_header();
          String options[P222_NR_OUTPUT_OPTIONS];
          for (int i = 0; i < P222_NR_OUTPUT_OPTIONS; ++i) {
            options[i] = Plugin_222_SDM->getTitle(PM_Metric(i));
          }
          for (byte i = 0; i < P222_NR_OUTPUT_VALUES; ++i) {
            const byte pconfigIndex = i + P222_QUERY1_CONFIG_POS;
            sensorTypeHelper_loadOutputSelector(event, pconfigIndex, i, P222_NR_OUTPUT_OPTIONS, options);
          }
        }
        success = true;
        break;
      }

    case PLUGIN_WEBFORM_SAVE:
      {
          serialHelper_webformSave(event);
          // Save output selector parameters.
          for (byte i = 0; i < P222_NR_OUTPUT_VALUES; ++i) {
            const byte pconfigIndex = i + P222_QUERY1_CONFIG_POS;
            const byte choice = PCONFIG(pconfigIndex);
            sensorTypeHelper_saveOutputSelector(event, pconfigIndex, i, Plugin_222_SDM->getUnit(PM_Metric(choice)));
          }

          P222_DEV_ID = getFormItemInt(P222_DEV_ID_LABEL);
          P222_MODEL = getFormItemInt(P222_MODEL_LABEL);
          P222_BAUDRATE = getFormItemInt(P222_BAUDRATE_LABEL);

          Plugin_222_init = false; // Force device setup next time
          success = true;
          break;
      }

    case PLUGIN_INIT:
      {
        addLog(LOG_LEVEL_INFO, "Init");
        Plugin_222_init = true;
        if (Plugin_222_SoftSerial != NULL) {
          delete Plugin_222_SoftSerial;
          Plugin_222_SoftSerial=NULL;
        }

        Plugin_222_SoftSerial = new ESPeasySerial(CONFIG_PIN1, CONFIG_PIN2);
        unsigned int baudrate = p222_storageValueToBaudrate(P222_BAUDRATE);
        Plugin_222_SoftSerial->begin(baudrate);
        addLog(LOG_LEVEL_INFO, "Initing");

        if (Plugin_222_SDM != NULL) {
          delete Plugin_222_SDM;
          Plugin_222_SDM=NULL;
        }
        Plugin_222_SDM = new SDM(*Plugin_222_SoftSerial, baudrate, P222_DEPIN);
        Plugin_222_SDM->begin();
        addLog(LOG_LEVEL_INFO, "Init done");
        success = true;
        break;
      }

    case PLUGIN_EXIT:
    {
      Plugin_222_init = false;
      if (Plugin_222_SoftSerial != NULL) {
        delete Plugin_222_SoftSerial;
        Plugin_222_SoftSerial=NULL;
      }
      if (Plugin_222_SDM != NULL) {
        delete Plugin_222_SDM;
        Plugin_222_SDM=NULL;
      }
      break;
    }

    case PLUGIN_READ:
      {
        addLog(LOG_LEVEL_INFO, "Reading");
        if (Plugin_222_init)
        {
          addLog(LOG_LEVEL_INFO, "Reading2");
          int model = P222_MODEL;
          byte dev_id = P222_DEV_ID;
          UserVar[event->BaseVarIndex]     = p222_readVal(P222_QUERY1, dev_id, model);
          UserVar[event->BaseVarIndex + 1] = p222_readVal(P222_QUERY2, dev_id, model);
          UserVar[event->BaseVarIndex + 2] = p222_readVal(P222_QUERY3, dev_id, model);
          UserVar[event->BaseVarIndex + 3] = p222_readVal(P222_QUERY4, dev_id, model);
          success = true;
          break;
        }
        break;
      }
  }
  return success;
}

float p222_readVal(byte query, byte node, unsigned int model) {

  if (Plugin_222_SDM == NULL) return 0.0;

  byte retry_count = 3;
  bool success = false;
  float _tempvar = NAN;
  while (retry_count > 0 && !success) {
    Plugin_222_SDM->clearErrCode();
    _tempvar = Plugin_222_SDM->getValue(PM_Metric(query), PM_Model(model), node);
    --retry_count;
    if (Plugin_222_SDM->getErrCode() == SDM_ERR_NO_ERROR) {
      success = true;
    }
  }
  if (loglevelActiveFor(LOG_LEVEL_INFO)) {
    String log = F("PowerMeter: (");
    log += node;
    log += ',';
    log += model;
    log += ") ";
    log += Plugin_222_SDM->getTitle(PM_Metric(query));
    log += ": ";
    log += _tempvar;
    addLog(LOG_LEVEL_INFO, log);
  }
  return _tempvar;
}

int p222_storageValueToBaudrate(byte baudrate_setting) {
  unsigned int baudrate = 9600;
  switch (baudrate_setting) {
    case 0:  baudrate = 1200; break;
    case 1:  baudrate = 2400; break;
    case 2:  baudrate = 4800; break;
    case 3:  baudrate = 9600; break;
    case 4:  baudrate = 19200; break;
    case 5:  baudrate = 38400; break;
  }
  return baudrate;
}

#endif // USES_P222
