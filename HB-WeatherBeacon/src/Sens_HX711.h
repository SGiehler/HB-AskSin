//---------------------------------------------------------
// Sens_HX711
// 2020-07-09 Gila (Creative Commons)
// https://creativecommons.org/licenses/by-nc-sa/4.0/
// You are free to Share & Adapt under the following terms:
// Give Credit, NonCommercial, ShareAlike
// +++
// AskSin++ 2016-10-31 papa Creative Commons
//---------------------------------------------------------

#ifndef _SENS_HX711_H_
#define _SENS_HX711_H_

#define LOADCELL_DOUT_PIN 7
#define LOADCELL_SCK_PIN 6
#define WEIGHT_THRESHOLD 10

#include <Sensors.h>
#include <HX711.h>

namespace as {

class Sens_HX711 : public Sensor {

    int32_t _weight;
    int32_t _weightBuffer;
    HX711 _hx711;

public:
    Sens_HX711()
        : _weight(0)
    { }

    void measureRaw()
    {
        _hx711.power_up();
        _weight = _hx711.get_units(5);
        _hx711.power_down();
    }

    void init(uint16_t loadcellCal)
    {
        _hx711.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
        _hx711.power_up();
        _hx711.set_scale((float)(loadcellCal / 10.f));
        _hx711.tare(5);
        DPRINTLN(F("HX711 setup"));
        _present = true;
    }

    void measure(uint16_t loadcellCal)
    {
        _weight = 0;
        if (_present == true) {
            _hx711.set_scale((float)(loadcellCal / 10.f));
            measureRaw();

            int16_t temp = _weight;
            _weight = _weight - _weightBuffer;
            _weightBuffer = temp;

            DPRINT(F("HX711 Weight            : "));
            DDECLN(_weight);

            if(_weight < WEIGHT_THRESHOLD) {
              _weight = 0;  
            }    
        }
    }

    int16_t  weight() { return _weight; }
};

}

#endif