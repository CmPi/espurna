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
#define TI_SLOT_ADCO                  0   // contract number - 12 chars
#define TI_SLOT_OPTARIF               1   // option tarifaire, ici Heure Creuse - 4 chars
#define TI_SLOT_ISOUSC                2   // intensité souscrite - XX (A)
#define TI_SLOT_HCHC                  3   // valeur de l'index heure creuse en option tarifaire heure creuse
#define TI_SLOT_HCHP                  4   // valeur de l'index heure pleine en option tarifaire heure pleine 
#define TI_SLOT_PTEC                  5   // période tarifaire en cours, ici heure pleine
#define TI_SLOT_IINST                 6   //  valeur instantanée de l'intensité, ici 1 A 
#define TI_SLOT_IMAX                  7   // 
#define TI_SLOT_COUNT                 8   // 

#define TI_STX  0X02
#define TI_ETX  0x03
#define TI_EOT  0x04
#define TI_LF   0x0A  // SGR
#define TI_CR   0x0D  // EGR
#define TI_SP   0x20

#define TI_STATE_INIT       0
#define TI_STATE_WAIT_STX   1
#define TI_STATE_WAIT_ETX   2
#define TI_STATE_READY      3

#define TI_BUFSIZE          64

PROGMEM const char teleinfo_adco_topic[]    = "ADCO";
PROGMEM const char teleinfo_optarif_topic[] = "OPTARIF";
PROGMEM const char teleinfo_isousc_topic[]  = "ISOUSC";
PROGMEM const char teleinfo_hchc_topic[]    = "HCHC";
PROGMEM const char teleinfo_hchp_topic[]    = "HCHP";
PROGMEM const char teleinfo_ptec_topic[]    = "PTEC";
PROGMEM const char teleinfo_iinst_topic[]   = "IINST";
PROGMEM const char teleinfo_imax_topic[]    = "IMAX";

PROGMEM const char* const teleinfo_topics[] = {
    teleinfo_adco_topic, teleinfo_optarif_topic, teleinfo_isousc_topic,
    teleinfo_hchc_topic, teleinfo_hchp_topic, teleinfo_ptec_topic, 
    teleinfo_iinst_topic, teleinfo_imax_topic
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

            clearBuffer();
            _state = TI_STATE_INIT;

            _serial = new SoftwareSerial( _pin_rx, SW_SERIAL_UNUSED_PIN, _inverted, 64 );
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
                if (index == TI_SLOT_ISOUSC)  snprintf(buffer, sizeof(buffer), teleinfo_topics[index] );
                if (index == TI_SLOT_HCHC)    snprintf(buffer, sizeof(buffer), teleinfo_topics[index] );
                if (index == TI_SLOT_HCHP)    snprintf(buffer, sizeof(buffer), teleinfo_topics[index] );
                if (index == TI_SLOT_PTEC)    snprintf(buffer, sizeof(buffer), teleinfo_topics[index] );
                if (index == TI_SLOT_IINST)   snprintf(buffer, sizeof(buffer), teleinfo_topics[index] );
                if (index == TI_SLOT_IMAX)    snprintf(buffer, sizeof(buffer), teleinfo_topics[index] );
                /*
                snprintf(buffer, sizeof(buffer), teleinfo_topics[index] );
                */
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
                if (index == TI_SLOT_ADCO)    return MAGNITUDE_ENERGY;
                if (index == TI_SLOT_OPTARIF) return MAGNITUDE_ENERGY;
                if (index == TI_SLOT_ISOUSC)  return MAGNITUDE_CURRENT;
                if (index == TI_SLOT_HCHC)    return MAGNITUDE_ENERGY;  // Joules
                if (index == TI_SLOT_HCHP)    return MAGNITUDE_ENERGY;  // Joules
                if (index == TI_SLOT_PTEC)    return MAGNITUDE_ENERGY;
                if (index == TI_SLOT_IINST)   return MAGNITUDE_CURRENT;
                if (index == TI_SLOT_IMAX)    return MAGNITUDE_CURRENT;

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
            if (index == TI_SLOT_PTEC)    return _ptec;
            if (index == TI_SLOT_IINST)   return _iinst;
            if (index == TI_SLOT_IMAX)    return _imax;
            return 0;
        }

    protected:

        // ---------------------------------------------------------------------
        // Protected
        // ---------------------------------------------------------------------

        void clearBuffer() {
            // Clear our buffer, set index to 0
            memset(_recv_buff, 0,TI_BUFSIZE);
            _recv_idx = 0;
        }  


        void _read() {

            static unsigned char state = 0;
            static unsigned long last = 0;
            static bool found = false;
            static unsigned char index = 0;

            _adco = _adco + 1;

            char c;
            // c &= 0x7F;

            // Handle teleinfo serial
            while (_serial->available()) {
                _optarif = _optarif + 1;
                // Read Serial and process to tinfo
                c = _serial->read();
                if (c>0) {
                    _ptec = _ptec +1;
                }


                _imax = c;
                switch (c) {
                    case TI_STX: // start of transmission
                        _hchc = _hchc + 1;
                        clearBuffer();
                        if (_state == TI_STATE_INIT || _state == TI_STATE_WAIT_STX ) {
                        _state = TI_STATE_WAIT_ETX;
                        } 
                        break;
                    case TI_ETX: // end of transmission
                        _hchp = _hchp + 1;
                              // We were waiting fo this one ?
                        if (_state == TI_STATE_WAIT_ETX) {
                            _state = TI_STATE_READY;
                        } 
                        else if ( _state == TI_STATE_INIT) {
                            _state = TI_STATE_WAIT_STX ;
                        } 
                        break;
                    default: {
                        if (_state==TI_STATE_READY) {
                            _iinst = _iinst +1;
                            if ( _recv_idx < TI_BUFSIZE)
                                _recv_buff[_recv_idx++]=c;
                            else
                                clearBuffer();
                        }
                    }
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

        char            _recv_buff[TI_BUFSIZE]; // line receive buffer
        unsigned int   _recv_idx;  // index in receive buffer
        boolean         _frame_updated; // Data on the frame has been updated
    

        unsigned int _pin_rx = TELEINFO_PIN;
        bool _inverted = TELEINFO_PIN_INVERSE;
        SoftwareSerial * _serial = NULL;
        unsigned int _state = TI_STATE_INIT;
        double _adco    = 0;
        double _optarif = 0;
        double _isousc  = 0;
        double _hchc    = 0;
        double _hchp    = 0;
        double _ptec    = 0;
        double _iinst   = 0;
        double _imax    = 0;
        unsigned char _data[24];

};

#endif // SENSOR_SUPPORT && TELEINFO_SUPPORT
