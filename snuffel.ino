#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <WiFiSettings.h>
#include <MQTT.h>
#include <list>
#include <PMS.h>
#include <MHZ19.h>

using namespace std;

#define Sprintf(f, ...) ({ char* s; asprintf(&s, f, __VA_ARGS__); String r = s; free(s); r; })

const int interval = 5000;

const int buttonpin  = 15;

MQTTClient mqtt;
String topic_prefix;

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
    FN      request;
    LM      fetch;
    SnuffelSensor(
        String i, String n, String t, FN s, FN r, LM f
    ) : id(i), description(n), topic_suffix(t), init(s), request(r), fetch(f) {}

    void publish(list<pair<const char*, String>> mapping, String value, String unit) {
        String topic = topic_prefix + topic_suffix;
        for (auto i : mapping) {
            topic.replace(i.first, i.second);
        }
        if (true) {
            value += " " + unit;
        }
        retain(topic, value);
    }
    void publish(String value, String unit) {
        publish({ }, value, unit);
    }
};

list<SnuffelSensor> snuffels;

void setup_sensors() {
    {
        static OneWire ds(4);
        static DallasTemperature sensors(&ds);

        snuffels.push_back(SnuffelSensor {
            "DS18B20",
            "temperature sensor(s)",
            "temperature/{index}",
            []() { // init
                sensors.begin();
                sensors.setWaitForConversion(false);
            },
            []() { // request
                sensors.requestTemperatures();
            },
            [](SnuffelSensor& self) { // fetch
                unsigned int timeout = millis() + 500;
                while (millis() < timeout && !sensors.isConversionComplete()) { }

                for (int i = 0; i < 100 /* arbitrary maximum */; i++) {
                    float C = sensors.getTempCByIndex(i);
                    if (C == DEVICE_DISCONNECTED_C) break;
                    self.publish({ { "{index}", String(i) } }, String(C), "°C");
                }
            }
        });
    }

    {
        static HardwareSerial hwserial(2);
        static MHZ19 mhz;

        snuffels.push_back(SnuffelSensor {
            "MH-Z19",
            "CO2 sensor",
            "co2",
            []() { // init
                hwserial.begin(9600, SERIAL_8N1, 22, 21);
                mhz.begin(hwserial);
                mhz.autoCalibration();
            },
            []() { // request
                // empty
            },
            [](SnuffelSensor& self) { // fetch
                int CO2 = mhz.getCO2();
                self.publish(String(CO2), "PPM");
            }
        });
    }

    {
        static HardwareSerial hwserial(1);
        static PMS pms(hwserial);
        static PMS::DATA data;

        snuffels.push_back(SnuffelSensor {
            "PMS7003",
            "dust sensor",
            "dust/PM{size}",
            []() { // init
                hwserial.begin(9600, SERIAL_8N1, 25, 32);
                pms.passiveMode();
            },
            []() { // request
                pms.requestRead();
            },
            [](SnuffelSensor& self) { // fetch
                if (! pms.readUntil(data)) return;
                self.publish({ { "{size}",  "1.0" } }, String(data.PM_AE_UG_1_0),  "µg/m3");
                self.publish({ { "{size}",  "2.5" } }, String(data.PM_AE_UG_2_5),  "µg/m3");
                self.publish({ { "{size}", "10.0" } }, String(data.PM_AE_UG_10_0), "µg/m3");
            }
        });
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
    pinMode(buttonpin, INPUT);

    setup_sensors();

    String server = WiFiSettings.string("mqtt_server", 64, "test.mosquitto.org", "MQTT broker");
    int port      = WiFiSettings.integer("mqtt_port", 0, 65535, 1883, "MQTT broker TCP port");
    topic_prefix  = WiFiSettings.string("mqtt_prefix", "snuffelaar/", "MQTT topic prefix (ending with '/' strongly advised)");

    for (auto& s : snuffels) {
        String label = "Enable " + s.id + " " + s.description;
        s.enabled = WiFiSettings.checkbox(s.id + "_enabled", true, label);
        s.topic_suffix = WiFiSettings.string(s.id + "_topic", 1, 128, s.topic_suffix, s.id + " MQTT topic suffix");
    }

    WiFiSettings.onWaitLoop = []() {
        check_button();
        return 50;
    };
    if (!WiFiSettings.connect(false)) ESP.restart();

    for (auto& s : snuffels) if (s.enabled) s.init();

    static WiFiClient wificlient;
    mqtt.begin(server.c_str(), port, wificlient);


    // 0x76 bme begin
}

void loop() {
    unsigned long start = millis();

    while (!mqtt.connected()) {
        if (!mqtt.connect("")) delay(500);
    }

    for (auto& s : snuffels) if (s.enabled) s.request();
    for (auto& s : snuffels) if (s.enabled) s.fetch(s);

    while (millis() < start + interval) check_button();
}


