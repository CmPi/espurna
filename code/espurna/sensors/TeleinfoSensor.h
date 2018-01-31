// -----------------------------------------------------------------------------
// Teleinfo 
// French energy counter
// ESP-01 1M board based sensor
// Uses SoftwareSerial library
// Copyright (C) 2018 by CmPi <cmpi at webe dot fr>
// -----------------------------------------------------------------------------

#if SENSOR_SUPPORT && TELEINFO_SUPPORT

#pragma once

// common includes for a sensor
#include "Arduino.h"
#include "BaseSensor.h"

// Teleinfo sensor needs also these ones
#include <SoftwareSerial.h>

// Slots for a Teleinfo sensor
#define TI_SLOT_ADCO                  0   // n° de contrat
#define TI_SLOT_OPTARIF               1   // option tarifaire, ici Heure Creuse
#define TI_SLOT_ISOUSC                2   // intensité souscrite
#define TI_SLOT_HCHC                  3   // valeur de l'index heure creuse en option tarifaire heure creuse
#define TI_SLOT_HCHP                  4   // valeur de l'index heure pleine en option tarifaire heure pleine 
#define TI_SLOT_PTEC                  5   // période tarifaire en cours, ici heure pleine
#define TI_SLOT_IINST                 6   //  valeur instantanée de l'intensité, ici 1 A 
#define TI_SLOT_MAX                   6   // 
#define TI_SLOT_COUNT                 7   // 

#define TI_STX  0X02
#define TI_ETX  0x03
#define TI_EOT  0x04
#define TI_LF   0x0A
#define TI_CR   0x0D
#define TI_SP   0x20

PROGMEM const char teleinfo_adco_topic[]    = "ADCO";
PROGMEM const char teleinfo_optarif_topic[] = "OPTARIF";
PROGMEM const char teleinfo_isousc_topic[]  = "ISOUSC";
PROGMEM const char teleinfo_hchc_topic[]    = "HCHC";
PROGMEM const char teleinfo_hchp_topic[]    = "HCHP";
PROGMEM const char teleinfo_ptec_topic[]    = "PTEC";
PROGMEM const char teleinfo_iinst_topic[]   = "IINST";

PROGMEM const char* const teleinfo_topics[] = {
    teleinfo_adco_topic, teleinfo_optarif_topic, teleinfo_isousc_topic,
    teleinfo_isousc_topic, teleinfo_hchc_topic, teleinfo_hchp_topic,
    teleinfo_ptec_topic, teleinfo_iinst_topic
};

class TeleinfoSensor : public BaseSensor {

    public:

        // ---------------------------------------------------------------------
        // Public
        // ---------------------------------------------------------------------

        TeleinfoSensor(): BaseSensor(), _data() {
            _count = TI_SLOT_COUNT;
            _sensor_id = SENSOR_TELEINFO_ID;
        }

        ~TeleinfoSensor() {
            if (_serial) delete _serial;
        }

        // ---------------------------------------------------------------------

        void setRX(unsigned char pin_rx) {
            if (_pin_rx == pin_rx) return;
            _pin_rx = pin_rx;
            _dirty = true;
        }

        void setInverted(bool inverted) {
            if (_inverted == inverted) return;
            _inverted = inverted;
            _dirty = true;
        }

        // ---------------------------------------------------------------------

        unsigned char getRX() {
            return _pin_rx;
        }

        bool getInverted() {
            return _inverted;
        }

        // ---------------------------------------------------------------------
        // Sensor API
        // ---------------------------------------------------------------------

        // Initialization method, must be idempotent
        void begin() {

            if (!_dirty) return;
            _dirty = false;

            if (_serial) delete _serial;

            _serial = new SoftwareSerial( _pin_rx, SW_SERIAL_UNUSED_PIN, _inverted, 32 );
            _serial->begin(TELEINFO_BAUDRATE);

        }

        // Descriptive name of the sensor
        String description() {
            char buffer[28];
            snprintf(buffer, sizeof(buffer), "TELEINFO @ SwSerial(%u,NULL)", _pin_rx);
            return String(buffer);
        }

        // Descriptive name of the slot # index
        String slot(unsigned char index) {
            if (index < _count) {
                _error = SENSOR_ERROR_OK;
                char buffer[36];
                if (index == TI_SLOT_ADCO)    snprintf(buffer, sizeof(buffer), teleinfo_adco_topic    );
                if (index == TI_SLOT_OPTARIF) snprintf(buffer, sizeof(buffer), teleinfo_optarif_topic );
                if (index == TI_SLOT_ISOUSC)  snprintf(buffer, sizeof(buffer), teleinfo_isousc_topic  );
 
                return String(buffer);
            }
            _error = SENSOR_ERROR_OUT_OF_RANGE;
            return description();
        };

        // Address of the sensor (it could be the GPIO or I2C address)
        String address(unsigned char index) {
            return String(_pin_rx);
        }

        // Loop-like method, call it in your main loop
        void tick() {
            _read();
        }

        // Type for slot # index
        unsigned char type(unsigned char index) {
           if (index < _count) {
                _error = SENSOR_ERROR_OK;
                if (index == TI_SLOT_ADCO)    return MAGNITUDE_DIGITAL;
                if (index == TI_SLOT_OPTARIF) return MAGNITUDE_DIGITAL;
                if (index == TI_SLOT_ISOUSC)  return MAGNITUDE_CURRENT;
                if (index == TI_SLOT_HCHC)    return MAGNITUDE_ENERGY;  // Joules
                if (index == TI_SLOT_HCHP)    return MAGNITUDE_ENERGY;  // Joules
                if (index == TI_SLOT_PTEC)    return MAGNITUDE_DIGITAL;
                if (index == TI_SLOT_IINST)   return MAGNITUDE_CURRENT;

            }
            _error = SENSOR_ERROR_OUT_OF_RANGE;
            return MAGNITUDE_NONE;
        }

        // Current value for slot # index
        double value(unsigned char index) {
            if (index == TI_SLOT_ADCO)    return _adco;
            if (index == TI_SLOT_OPTARIF) return _optarif;
            if (index == TI_SLOT_ISOUSC)  return _isousc;
            if (index == TI_SLOT_HCHC)    return _hchc;
            if (index == TI_SLOT_HCHP)    return _hchp;
            if (index == TI_SLOT_PTEC)    return 0;
            if (index == TI_SLOT_IINST)   return _iinst;
            return 0;
        }

    protected:

        // ---------------------------------------------------------------------
        // Protected
        // ---------------------------------------------------------------------

        void _read() {

            static unsigned char state = 0;
            static unsigned long last = 0;
            static bool found = false;
            static unsigned char index = 0;

            if (state == 0) {

                while (_serial->available()) {
                    _serial->flush();
                    found = true;
                    last = millis();
                }

                if (found && (millis() - last > V9261F_SYNC_INTERVAL)) {
                    _serial->flush();
                    index = 0;
                    state = 1;
                }

            } else if (state == 1) {

                while (_serial->available()) {
                    _serial->read();
                    if (index++ >= 7) {
                        _serial->flush();
                        index = 0;
                        state = 2;
                    }
                }

            } else if (state == 2) {

                while (_serial->available()) {
                    _data[index] = _serial->read();
                    if (index++ >= 19) {
                        _serial->flush();
                        last = millis();
                        state = 3;
                    }
                }

            } else if (state == 3) {

                if (_checksum()) {

                }

                last = millis();
                index = 0;
                state = 4;

            } else if (state == 4) {

                while (_serial->available()) {
                    _serial->flush();
                    last = millis();
                }

                if (millis() - last > V9261F_SYNC_INTERVAL) {
                    state = 1;
                }

            }

        }

        bool _checksum() {
            unsigned char checksum = 0;
            for (unsigned char i = 0; i < 19; i++) {
                checksum = checksum + _data[i];
            }
            checksum = ~checksum + 0x33;
            return checksum == _data[19];
        }

        // ---------------------------------------------------------------------

        unsigned int _pin_rx = TELEINFO_PIN;
        bool _inverted = TELEINFO_PIN_INVERSE;
        SoftwareSerial * _serial = NULL;

        double _adco    = 0;
        double _optarif = 0;
        double _isousc  = 0;
        double _hchc    = 0;
        double _hchp    = 0;
        double _iinst    = 0;

        unsigned char _data[24];

};

#endif // SENSOR_SUPPORT && TELEINFO_SUPPORT
