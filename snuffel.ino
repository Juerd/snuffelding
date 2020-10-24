#define Sprintf(f, ...) ({ char* s; asprintf(&s, f, __VA_ARGS__); String r = s; free(s); r; })
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <WiFiSettings.h>
#include <MQTT.h>
#include <PMS.h>
#include <MHZ19.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <NeoPixelBus.h>
#include <math.h>
#include <list>
using namespace std;

const int interval = 5000;
const int buttonpin  = 15;
const int i2c_sda = 23;
const int i2c_scl = 13;
const int ledpin = 26;

NeoPixelBus<NeoGrbwFeature, Neo800KbpsMethod> led(1, ledpin);
MQTTClient mqtt;
String topic_prefix;
bool add_units;

void retain(String topic, String message) {
    Serial.printf("%s %s\n", topic.c_str(), message.c_str());
    mqtt.publish(topic, message, true, 0);
}

#define FN function<void()>
#define LM function<void(SnuffelSensor&)>
struct SnuffelSensor {
    bool    enabled;
    String  id;
    String  description;
    String  topic_suffix;
    FN      init;
    FN      prepare;
    LM      fetch;

    void publish(list<pair<const char*, String>> mapping, String value, String unit) {
        String topic = topic_prefix + topic_suffix;
        for (auto i : mapping) topic.replace(i.first, i.second);
        retain(topic, add_units ? (value + " " + unit) : value);
    }
    void publish(String value, String unit) {
        publish({ }, value, unit);
    }
};

list<SnuffelSensor> snuffels;

void setup_sensors() {
    {
        static int gpio = 4;
        static OneWire ds(gpio);
        static DallasTemperature sensors(&ds);

        struct SnuffelSensor s = {
            enabled: false,  // enabled by default via WiFiSettings
            id: "DS18B20",
            description: "temperature sensor(s)",
            topic_suffix: "temperature/{index}",
            init: []() {
                sensors.begin();
                sensors.setWaitForConversion(false);
            },
            prepare: []() {
                sensors.requestTemperatures();
            },
            fetch: [](SnuffelSensor& self) {
                unsigned int timeout = millis() + 500;
                while (millis() < timeout && !sensors.isConversionComplete()) { }

                for (int i = 0; i < 100 /* arbitrary maximum */; i++) {
                    float C = sensors.getTempCByIndex(i);
                    if (C == DEVICE_DISCONNECTED_C) break;
                    self.publish({ { "{index}", String(i) } }, String(C), "°C");
                }
            },
        };
        snuffels.push_back(s);
    }

    {
        static HardwareSerial hwserial(2);
        static MHZ19 mhz;
        static int rx = 22, tx = 21;

        struct SnuffelSensor s = {
            enabled: false,  // enabled by default via WiFiSettings
            id: "MH-Z19",
            description: "CO2 sensor",
            topic_suffix: "co2",
            init: []() {
                hwserial.begin(9600, SERIAL_8N1, rx, tx);
                mhz.begin(hwserial);
                mhz.autoCalibration();
            },
            prepare: NULL,
            fetch: [](SnuffelSensor& self) {
                int CO2 = mhz.getCO2();
                self.publish(String(CO2), "PPM");
            }
        };
        snuffels.push_back(s);
    }

    {
        static HardwareSerial hwserial(1);
        static PMS pms(hwserial);
        static PMS::DATA data;
        static int rx = 25, tx = 32;

        struct SnuffelSensor s = {
            enabled: false,  // enabled by default via WiFiSettings
            id: "PMS7003",
            description: "dust sensor",
            topic_suffix: "dust/PM{size}",
            init: []() {
                hwserial.begin(9600, SERIAL_8N1, rx, tx);
                pms.passiveMode();
            },
            prepare: []() {
                pms.requestRead();
            },
            fetch: [](SnuffelSensor& self) {
                if (! pms.readUntil(data)) return;
                self.publish({ { "{size}",  "1.0" } }, String(data.PM_AE_UG_1_0),  "µg/m3");
                self.publish({ { "{size}",  "2.5" } }, String(data.PM_AE_UG_2_5),  "µg/m3");
                self.publish({ { "{size}", "10.0" } }, String(data.PM_AE_UG_10_0), "µg/m3");
            }
        };
        snuffels.push_back(s);
    }

    {
        static Adafruit_BME280 bme;  // repeated .begin()s seems to be fine
        static int i2c_address = 0x76;

        struct SnuffelSensor rh = {
            enabled: false,  // enabled by default via WiFiSettings
            id: "BME280_RH",
            description: "relative humidity sensor",
            topic_suffix: "humidity",
            init: []() { bme.begin(i2c_address); },
            prepare: NULL,
            fetch: [](SnuffelSensor& self) {
                self.publish(String(bme.readHumidity()), "%");
            }
        };
        snuffels.push_back(rh);

        struct SnuffelSensor bp = {
            enabled: false,  // enabled by default via WiFiSettings
            id: "BME280_BP",
            description: "barometric pressure sensor",
            topic_suffix: "pressure",
            init: []() { bme.begin(i2c_address); },
            prepare: NULL,
            fetch: [](SnuffelSensor& self) {
                self.publish(String(bme.readPressure() / 100.0F), "hPa");
            }
        };
        snuffels.push_back(bp);
    }
}

void check_button() {
    if (digitalRead(buttonpin)) return;
    delay(50);
    if (digitalRead(buttonpin)) return;
    WiFiSettings.portal();
}

void setup() {
    Serial.begin(115200);
    SPIFFS.begin(true);
    Wire.begin(i2c_sda, i2c_scl);
    pinMode(buttonpin, INPUT);

    led.Begin();
    led.ClearTo(RgbwColor(0,0,255,0));
    led.Show();

    setup_sensors();

    String server = WiFiSettings.string("mqtt_server", 64, "test.mosquitto.org", "MQTT broker");
    int port      = WiFiSettings.integer("mqtt_port", 0, 65535, 1883, "MQTT broker TCP port");
    topic_prefix  = WiFiSettings.string("snuffelaar_mqtt_prefix", "snuffelaar/", "MQTT topic prefix (ending with '/' strongly advised)");
    add_units     = WiFiSettings.checkbox("snuffelaar_add_units", true, "Add units of measurement to MQTT messages");

    for (auto& s : snuffels) {
        String label = "Enable " + s.id + " " + s.description;
        s.enabled = WiFiSettings.checkbox(s.id + "_enabled", true, label);
        s.topic_suffix = WiFiSettings.string(s.id + "_topic", 1, 128, s.topic_suffix, s.id + " MQTT topic suffix");
    }

    WiFiSettings.onWaitLoop = []() {
        check_button();
        led.ClearTo(RgbwColor(0, 0, abs(sin(.001 * millis())*255), 0));
        led.Show();
        return 50;
    };
    WiFiSettings.onPortalWaitLoop = []() {
        led.ClearTo(RgbwColor(abs(cos(.001 * millis())*255), abs(.3*sin(.001 * millis())*255), 0, 0));
        led.Show();
    };
    if (!WiFiSettings.connect(false)) ESP.restart();

    led.ClearTo(RgbwColor(0,0,0,50));
    led.Show();

    for (auto& s : snuffels) if (s.enabled && s.init) s.init();

    static WiFiClient wificlient;
    mqtt.begin(server.c_str(), port, wificlient);
}

void loop() {
    unsigned long start = millis();

    while (!mqtt.connected()) {
        if (!mqtt.connect("")) delay(500);
    }

    for (auto& s : snuffels) if (s.enabled && s.prepare) s.prepare();
    for (auto& s : snuffels) if (s.enabled && s.fetch)   s.fetch(s);

    while (millis() < start + interval) check_button();
}


