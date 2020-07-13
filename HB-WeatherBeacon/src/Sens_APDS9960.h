
//---------------------------------------------------------
// Sens_APDS9960
// 2020-07-09 Gila (Creative Commons)
// https://creativecommons.org/licenses/by-nc-sa/4.0/
// You are free to Share & Adapt under the following terms:
// Give Credit, NonCommercial, ShareAlike
// +++
// AskSin++ 2016-10-31 papa Creative Commons
//---------------------------------------------------------

#ifndef _SENS_APDS9960_H_
#define _SENS_APDS9960_H_

#include <Sensors.h>
#include <Adafruit_APDS9960.h>

namespace as {

class Sens_APDS9960 : public Sensor {

    uint16_t _red;
    uint16_t _green;
    uint16_t _blue;
    uint16_t _ambient;
    uint16_t _illuminance;
    Adafruit_APDS9960 _apds;

public:
    Sens_APDS9960()
        : _red(0)
        , _green(0)
        , _blue(0)
        , _ambient(0)
    { }

    void measureRaw()
    {
        while(!_apds.colorDataReady()){
            delay(5);
        }
        _apds.getColorData(&_red, &_green, &_blue, &_ambient);
        _illuminance = _apds.calculateLux(_red, _green, _blue);
    }

    void init()
    {
        if(!_apds.begin()){
            DPRINTLN(F("Failed to initialize APDS9960! Please check your wiring."));
        }
        else {
            DPRINTLN(F("APDS9960 found"));
            _present = true;
            _apds.enableColor(true);   
        };
    }

    void measure()
    {
        _red = _green = _blue = _ambient = 0;
        if (_present == true) {
            measureRaw();
            DPRINT(F("APDS9960 Red            : "));
            DDECLN(_red);
            DPRINT(F("APDS9960 Green          : "));
            DDECLN(_green);
            DPRINT(F("APDS9960 Blue           : "));
            DDECLN(_blue);
            DPRINT(F("APDS9960 Ambient        : "));
            DDECLN(_ambient);
            DPRINT(F("APDS9960 Illuminance        : "));
            DDECLN(_illuminance);            
        }
    }

    uint16_t red()      { return _red; }
    uint16_t green()    { return _green; }
    uint16_t blue()     { return _blue; }
    uint16_t ambient()  { return _ambient; }
    uint16_t illuminance()  { return _illuminance; }
};

}

#endif