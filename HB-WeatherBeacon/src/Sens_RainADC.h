//---------------------------------------------------------
// Sens_HX711
// 2020-07-09 Gila (Creative Commons)
// https://creativecommons.org/licenses/by-nc-sa/4.0/
// You are free to Share & Adapt under the following terms:
// Give Credit, NonCommercial, ShareAlike
// +++
// AskSin++ 2016-10-31 papa Creative Commons
//---------------------------------------------------------

#ifndef _SENS_RAINADC_H_
#define _SENS_RAINADC_H_

#define RAIN_PIN A0

#include <Sensors.h>
#include <Arduino.h>

namespace as {

class Sens_RainADC : public Sensor {

    uint8_t _rain;

public:
    Sens_RainADC()
        : _rain(0)
    { }

    enum Rain
    {
        No = 0,
        Moderate = 1,
        Heavy = 2
    };

    void init()
    {
        _present = true;
    }

    void measure()
    {
        _rain = Rain::No;
        if (_present == true) {
            uint16_t val = analogRead(RAIN_PIN);  // read the input pin
            
            if(val<400) _rain = Rain::Heavy; 
            else if(val<600) _rain = Rain::Moderate;
            
            DPRINT(F("Rain                    : "));
            DDECLN(val);
        }
    }

    uint8_t  rain() { return _rain; }
};

}

#endif