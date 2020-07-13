//---------------------------------------------------------
// HB-UNI-Sensor1
// Version 1.18
// (C) 2018-2020 Tom Major (Creative Commons)
// https://creativecommons.org/licenses/by-nc-sa/4.0/
// You are free to Share & Adapt under the following terms:
// Give Credit, NonCommercial, ShareAlike
// +++
// AskSin++ 2016-10-31 papa Creative Commons
//---------------------------------------------------------

//---------------------------------------------------------
// !! NDEBUG sollte aktiviert werden wenn die Sensorentwicklung und die Tests abgeschlossen sind und das Gerät in den 'Produktionsmodus' geht.
// Zum Beispiel bei aktiviertem BME280 und MAX44009 werden damit ca. 2,6 KBytes Flash und 100 Bytes RAM eingespart.
// Insbesondere die RAM-Einsparungen sind wichtig für die Stabilität / dynamische Speicherzuweisungen etc.
// Dies beseitigt dann auch die mögliche Arduino-Warnung 'Low memory available, stability problems may occur'.
//
#define NDEBUG

//---------------------------------------------------------
// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>
#include <MultiChannelDevice.h>
#include <Register.h>

// number of available peers per channel
#define PEERS_PER_CHANNEL 6

#define cDEVICE_ID          { 0xA5, 0xA5, 0x03 }
#define cDEVICE_SERIAL      "GILAWB0002"
#define cDEVICE_MODEL       { 0xFD, 0x02 }
#define CCU_FIRMWARE        0x14

#define CONFIG_BUTTON_PIN   8
#define LED1_PIN            4
#define LED2_PIN            5
#define BAT_VOLT_LOW        27  // 2.7V
#define BAT_VOLT_CRITICAL   25  // 2.5V

#define BH1750_ADDR         0x23
// Gilas 
// #define LOADCELL_CAL        9207 // 920.7
#define LOADCELL_CAL 10829 // 1082.93 Peter

#define AREA_SCALE_FACTOR   4654 // 100 / pi r^2 in meters

#include "battery.h"

// Gila
// #define BAT_SENSOR BatteryResDiv<A1, 4575>
#define BAT_SENSOR BatteryResDiv<A1, 4550>
#define CLOCK sysclock
#define SAVEPWR_MODE Sleep<>

using namespace as;

#include "Sens_BME280.h"
#include "Sens_BH1750.h"
#include "Sens_HX711.h"
#include "Sens_RainADC.h"

const struct DeviceInfo PROGMEM devinfo = {
    cDEVICE_ID,
    cDEVICE_SERIAL, 
    cDEVICE_MODEL,  
    CCU_FIRMWARE,
    as::DeviceType::THSensor,    // Device Type
    { 0x01, 0x01 }               // Info Bytes
};

// Configure the used hardware
typedef AvrSPI<10, 11, 12, 13>                 SPIType;
typedef Radio<SPIType, 2>                      RadioType;
typedef DualStatusLed<LED2_PIN, LED1_PIN>       LedType;
typedef AskSin<LedType, BAT_SENSOR, RadioType> BaseHal;

class Hal : public BaseHal {
public:
    void init(const HMID& id)
    {
        BaseHal::init(id);
        // measure battery every 10 min
        battery.init(seconds2ticks(10 * 60), CLOCK);
        battery.low(BAT_VOLT_LOW);
        battery.critical(BAT_VOLT_CRITICAL);
    }

    bool runready() { return CLOCK.runready() || BaseHal::runready(); }
} hal;

class WeatherEventMsg : public Message {
public:
    void init(uint8_t msgcnt, int16_t temp, uint16_t airPressure, uint8_t humidity, uint32_t brightness, uint8_t rain, uint16_t rainweight, uint16_t batteryVoltage, bool batLow)
    {

        uint8_t t1 = (temp >> 8) & 0x7f;
        uint8_t t2 = temp & 0xff;
        if (batLow == true) {
            t1 |= 0x80;    // set bat low bit
        }

        // als Standard wird BCAST gesendet um Energie zu sparen, siehe Beschreibung unten.
        // Bei jeder 20. Nachricht senden wir stattdessen BIDI|WKMEUP, um eventuell anstehende Konfigurationsänderungen auch
        // ohne Betätigung des Anlerntaster übernehmen zu können (mit Verzögerung, worst-case 20x Sendeintervall).
        uint8_t flags = BCAST;
        if ((msgcnt % 20) == 2) {
            flags = BIDI | WKMEUP;
        }
        Message::init(23, msgcnt, 0x72, flags, t1, t2);

        // Message Length (first byte param.): 11 + payload
        //  1 Byte payload -> length 12
        // 12 Byte payload -> length 23
        // max. payload: 17 Bytes (https://www.youtube.com/watch?v=uAyzimU60jw)

        // BIDI|WKMEUP: erwartet ACK vom Empfänger, ohne ACK wird das Senden wiederholt
        // LazyConfig funktioniert, d.h. eine anstehende Conf.Änderung von der CCU wird nach dem nächsten Senden übernommen. Aber erhöhter
        // Funkverkehr wegen ACK
        //
        // BCAST: ohne ACK zu Erwarten, Standard für HM Sensoren.
        // LazyConfig funktioniert nicht, d.h. eine anstehende Conf.Änderung von der CCU muss durch den Config Button am Sensor übernommen
        // werden!!

        // papa:
        // BIDI - fordert den Empfänger auf ein Ack zu schicken. Das wird auch zwingend für AES-Handling gebraucht. BCAST - signalisiert
        // eine Broadcast-Message. Das wird z.B. verwendet, wenn mehrere Peers vor einen Sensor existieren. Es wird dann an einen Peer
        // gesndet und zusätzlich das BCAST-Flag gesetzt. So dass sich alle die Nachrricht ansehen. Ein Ack macht dann natürlich keinen Sinn
        // - es ist ja nicht klar, wer das senden soll.
        //
        // WKMEUP - wird für LazyConfig verwendet. Ist es in einer Message gesetzt, so weiss
        // die Zentrale, dass das Geräte noch kurz auf weitere Nachrichten wartet. Die Lib setzt diese Flag für die StatusInfo-Message
        // automatisch. Außerdem bleibt nach einer Kommunikation der Empfang grundsätzlich für 500ms angeschalten.

        // airPressure
        pload[0] = (airPressure >> 8) & 0xff;
        pload[1] = airPressure & 0xff;

        // humidity
        pload[2] = humidity;

        // brightness (Lux)
        pload[3] = (brightness >> 24) & 0xff;
        pload[4] = (brightness >> 16) & 0xff;
        pload[5] = (brightness >> 8) & 0xff;
        pload[6] = (brightness >> 0) & 0xff;

        // rain
        pload[7] = rain;

        // rainweight
        pload[8] = (rainweight >> 8) & 0xff;
        pload[9] = (rainweight >> 0) & 0xff;

        // batteryVoltage
        pload[10] = (batteryVoltage >> 8) & 0xff;
        pload[11] = batteryVoltage & 0xff;

    }
};

// die "freien" Register 0x20/21 werden hier als 16bit memory für das Update
// Intervall in Sek. benutzt siehe auch hb-uni-sensor1.xml, <parameter
// id="Sendeintervall"> .. ausserdem werden die Register 0x22/0x23 für den
// konf. Parameter Höhe benutzt
DEFREGISTER(Reg0, MASTERID_REGS, DREG_LEDMODE, DREG_LOWBATLIMIT, DREG_TRANSMITTRYMAX, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27)
class SensorList0 : public RegList0<Reg0> {
public:
    SensorList0(uint16_t addr)
        : RegList0<Reg0>(addr)
    {
    }

    bool     updIntervall(uint16_t value) const { return this->writeRegister(0x20, (value >> 8) & 0xff) && this->writeRegister(0x21, value & 0xff); }
    uint16_t updIntervall() const { return (this->readRegister(0x20, 0) << 8) + this->readRegister(0x21, 0); }

    bool     altitude(uint16_t value) const { return this->writeRegister(0x22, (value >> 8) & 0xff) && this->writeRegister(0x23, value & 0xff); }
    uint16_t altitude() const { return (this->readRegister(0x22, 0) << 8) + this->readRegister(0x23, 0); }

    bool     loadcellCal(uint16_t value) const { return this->writeRegister(0x24, (value >> 8) & 0xff) && this->writeRegister(0x25, value & 0xff); }
    uint16_t loadcellCal() const { return (this->readRegister(0x24, 0) << 8) + this->readRegister(0x25, 0); }

    bool     areaScaleFactor(uint16_t value) const { return this->writeRegister(0x26, (value >> 8) & 0xff) && this->writeRegister(0x27, value & 0xff); }
    uint16_t areaScaleFactor() const { return (this->readRegister(0x26, 0) << 8) + this->readRegister(0x27, 0); }
    
    void defaults()
    {
        clear();
        ledMode(1);
        lowBatLimit(BAT_VOLT_LOW);
        transmitDevTryMax(6);
        updIntervall(60);
        altitude(0);
        loadcellCal(LOADCELL_CAL);
        areaScaleFactor(AREA_SCALE_FACTOR);
    }
};

class WeatherChannel : public Channel<Hal, List1, EmptyList, List4, PEERS_PER_CHANNEL, SensorList0>, public Alarm {

    WeatherEventMsg msg;

    int16_t  temperature10;
    uint16_t airPressure10;
    uint8_t  humidity;
    uint8_t  altitude;
    uint32_t brightness100;
    uint8_t  rain;
    uint16_t rainweight;
    uint16_t batteryVoltage;
    bool     regularWakeUp;

    Sens_BME280 bme280;
    Sens_BH1750<BH1750_ADDR> bh1750;
    Sens_HX711 hx711;
    Sens_RainADC rainadc;

public:
    WeatherChannel()
        : Channel()
        , Alarm(seconds2ticks(60))
        , temperature10(0)
        , airPressure10(0)
        , humidity(0)
        , brightness100(0)
        , rain(0)
        , rainweight(0)
        , batteryVoltage(0)
        , regularWakeUp(true)
    {
    }
    virtual ~WeatherChannel() {}

    virtual void trigger(AlarmClock& clock)
    {

        measure();
        uint8_t msgcnt = device().nextcount();
        msg.init(msgcnt, temperature10, airPressure10, humidity, brightness100, rain, rainweight, batteryVoltage, device().battery().low());
        if (msg.flags() & Message::BCAST) {
            device().broadcastEvent(msg, *this);
        } else {
            device().sendPeerEvent(msg, *this);
        }
        // reactivate for next measure
        uint16_t updCycle = this->device().getList0().updIntervall();
        set(seconds2ticks(updCycle));
        clock.add(*this);

        regularWakeUp = true;
    }

    void forceSend()
    {
        CLOCK.cancel(*this);
        regularWakeUp = false;    // Verhindert enableINT in trigger()
        trigger(CLOCK);           // Messen/Senden
        delay(250);               // Verzögerung für wiederholtes Senden bzw. digitalInput Entprellen

    }

    void measure()
    {
        // BME280
        uint16_t altitude = this->device().getList0().altitude();
        bme280.measure(altitude);
        temperature10 = bme280.temperature();
        airPressure10 = bme280.pressureNN();
        humidity      = bme280.humidity();

        // BH1750
        bh1750.measure();
        brightness100 = bh1750.brightnessLux();


        //RainADC
        rainadc.measure();
        rain = rainadc.rain();

        // HX711
        if(rain != Sens_RainADC::No) {
            uint16_t areaScaleFactor = this->device().getList0().areaScaleFactor();
            hx711.measure(this->device().getList0().loadcellCal());
            rainweight = hx711.weight() * (areaScaleFactor / 100.f);
        } else {
            rainweight = 0;
        }

        batteryVoltage = device().battery().current();
    }

    void initSensors()
    {
        bme280.init();
        bh1750.init();
        rainadc.init();     
        hx711.init(this->device().getList0().loadcellCal());
    }

    void setup(Device<Hal, SensorList0>* dev, uint8_t number, uint16_t addr)
    {
        Channel::setup(dev, number, addr);
        initSensors();
        set(seconds2ticks(5));    // first message in 5 sec.
        CLOCK.add(*this);
    }

    void configChanged()
    {
        // DPRINTLN(F("Config changed: List1"));2
    }

    uint8_t status() const { return 0; }

    uint8_t flags() const { return 0; }
};

class SensChannelDevice : public MultiChannelDevice<Hal, WeatherChannel, 1, SensorList0> {
public:
    typedef MultiChannelDevice<Hal, WeatherChannel, 1, SensorList0> TSDevice;
    SensChannelDevice(const DeviceInfo& info, uint16_t addr)
        : TSDevice(info, addr)
    {
    }
    virtual ~SensChannelDevice() {}

    virtual void configChanged()
    {
        TSDevice::configChanged();
        DPRINTLN(F("Config Changed: List0"));

        uint8_t ledMode = this->getList0().ledMode();
        DPRINT(F("ledMode: "));
        DDECLN(ledMode);

        uint8_t lowBatLimit = this->getList0().lowBatLimit();
        DPRINT(F("lowBatLimit: "));
        DDECLN(lowBatLimit);
        battery().low(lowBatLimit);

        uint8_t txDevTryMax = this->getList0().transmitDevTryMax();
        DPRINT(F("transmitDevTryMax: "));
        DDECLN(txDevTryMax);

        uint16_t updCycle = this->getList0().updIntervall();
        DPRINT(F("updCycle: "));
        DDECLN(updCycle);

        uint16_t altitude = this->getList0().altitude();
        DPRINT(F("altitude: "));
        DDECLN(altitude);

        uint16_t loadcellCal = this->getList0().loadcellCal();
        DPRINT(F("loadcellCal: "));
        DDECLN(loadcellCal);

        uint16_t areaScaleFactor = this->getList0().areaScaleFactor();
        DPRINT(F("areaScaleFactor: "));
        DDECLN(areaScaleFactor);
    }
};

SensChannelDevice               sdev(devinfo, 0x20);
ConfigButton<SensChannelDevice> cfgBtn(sdev);

void setup()
{
    DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
    sdev.init(hal);
    buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
    sdev.initDone();  
}

void loop()
{
    bool worked = hal.runready();
    bool poll   = sdev.pollRadio();
    if (worked == false && poll == false) {

        // deep discharge protection
        // if we drop below critical battery level - switch off all and sleep forever
        if (hal.battery.critical()) {
            // this call will never return
            hal.activity.sleepForever(hal);
        }
        // if nothing to do - go sleep
        hal.activity.savePower<SAVEPWR_MODE>(hal);
    }
}

