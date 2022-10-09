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
#include <SparkFunHTU21D.h>
#include <s8_uart.h>
#include <AHTxx.h>
#include <NeoPixelBus.h>
#include <ArduinoOTA.h>
#include <math.h>
#include <list>
using namespace std;

unsigned long interval;
const int buttonpin  = 15;
const int i2c_sda = 23;
const int i2c_scl = 13;
const int ledpin = 26;
int brightness;
bool ota_enabled;

NeoPixelBus<NeoGrbwFeature, Neo800KbpsMethod> led(1, ledpin);
MQTTClient mqtt;
HardwareSerial hwserial1(1);
HardwareSerial hwserial2(2);
String topic_prefix;
bool add_units;
int max_failures;

void retain(String topic, String message) {
    Serial.printf("%s %s\n", topic.c_str(), message.c_str());
    mqtt.publish(topic, message, true, 0);
}

void set_led(int r, int g, int b, int w = 0) {
    led.ClearTo(RgbwColor(r, g, b, w));
    led.Show();
}

void ledstatus_connecting() { 
    set_led(0, 0, abs(sin(.001 * millis())*brightness));
}

void ledstatus_portal() {
    float b = brightness ? brightness : 80;
    set_led(abs(sin(.001 * millis()) * 2*b), abs(cos(.001 * millis()) * b), 0);
}

void ledstatus_alarm() {
    float b = brightness ? 2 * brightness : 160;
    set_led(b, 0, 0);
}

void ledstatus_idle() {
    set_led(0, 0, 0, brightness/4);
}

#define FN function<void()>
#define LM function<void(SnuffelSensor&)>
struct SnuffelSensor {
    bool    enabled;        // default value for checkbox
    String  id;             // used in filenames and portal
    String  description;    // used in portal (appended to id)
    String  topic_suffix;   // default value for input field
    FN      settings;
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
            enabled: true,
            id: "DS18B20",
            description: "temperature sensor(s)",
            topic_suffix: "temperature/{index}",
            settings: NULL,
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
                    if (C == 85) break;  // sensor returns 85.0 on errors
                    self.publish({ { "{index}", String(i) } }, String(C), "°C");
                }
            },
        };
        snuffels.push_back(s);
    }

    {
        static MHZ19 mhz;
        static int rx = 22, tx = 21;
        static int alarm_level;

        struct SnuffelSensor s = {
            enabled: true,
            id: "MH-Z19",
            description: "CO2 sensor",
            topic_suffix: "co2",
            settings: []() {
                alarm_level = WiFiSettings.integer("MH-Z19_alarm", 0, 5000, 800, "CO2 warning level [PPM]");
            },
            init: []() {
                hwserial1.begin(9600, SERIAL_8N1, rx, tx);
                mhz.begin(hwserial1);
                mhz.setFilter(true, true); // Filter out erroneous readings (set to 0)
                mhz.autoCalibration();
            },
            prepare: NULL,
            fetch: [](SnuffelSensor& self) {
                int CO2 = mhz.getCO2();
                if (!CO2) return;

                self.publish(String(CO2), "PPM");
                if (alarm_level) {
                    if (CO2 >= alarm_level) ledstatus_alarm();
                    else ledstatus_idle();
                }
            }
        };
        snuffels.push_back(s);
    }

    {
        static PMS pms(hwserial2);
        static PMS::DATA data;
        static int rx = 25, tx = 32;

        struct SnuffelSensor s = {
            enabled: true,
            id: "PMS7003",
            description: "dust sensor",
            topic_suffix: "dust/PM{size}",
            settings: NULL,
            init: []() {
                hwserial2.begin(9600, SERIAL_8N1, rx, tx);
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
            enabled: true,
            id: "BME280_RH",
            description: "relative humidity sensor",
            topic_suffix: "humidity",
            settings: NULL,
            init: []() { bme.begin(i2c_address); },
            prepare: NULL,
            fetch: [](SnuffelSensor& self) {
                self.publish(String(bme.readHumidity()), "%");
            }
        };
        snuffels.push_back(rh);

        struct SnuffelSensor bp = {
            enabled: true,
            id: "BME280_BP",
            description: "barometric pressure sensor",
            topic_suffix: "pressure",
            settings: NULL,
            init: []() { bme.begin(i2c_address); },
            prepare: NULL,
            fetch: [](SnuffelSensor& self) {
                self.publish(String(bme.readPressure() / 100.0F), "hPa");
            }
        };
        snuffels.push_back(bp);
    }

    {
        static int alarm_level;
        static int rx = 22, tx = 21;

        struct SnuffelSensor s = {
            enabled: false,
            id: "T6613",
            description: "CO2 sensor",
            topic_suffix: "co2",
            settings: []() {
                alarm_level = WiFiSettings.integer("T6613_alarm", 0, 5000, 800, "CO2 warning level [PPM]");
                WiFiSettings.warning("Not present on standard Snuffelaar.");
                WiFiSettings.warning("Mutually exclusive with MH-Z19 and S8.");
            },
            init: []() {
                hwserial1.begin(19200, SERIAL_8N1, rx, tx);
                hwserial1.setTimeout(100);
            },
            prepare: []() {
                while (hwserial1.available()) hwserial1.read();  // flush
                hwserial1.write("\xff\xfe\x02\x02\x03");
            },
            fetch: [](SnuffelSensor& self) {
                char buf[5];
                hwserial1.readBytes(buf, 5);
                if (buf[0] != 0xFF || buf[1] != 0xFA || buf[2] != 2) return;

                int CO2 = 256 * buf[3] + buf[4];
                if (!CO2) return;
                self.publish(String(CO2), "PPM");

                if (alarm_level) {
                    if (CO2 >= alarm_level) ledstatus_alarm();
                    else ledstatus_idle();
                }
            }
        };
        snuffels.push_back(s);
    }

    {
        static S8_UART *sensor_S8;
        static S8_sensor S8;
        static int rx = 22, tx = 21;
        static int alarm_level;

        struct SnuffelSensor s = {
            enabled: false,
            id: "S8",
            description: "CO2 sensor",
            topic_suffix: "co2",
            settings: []() {
                alarm_level = WiFiSettings.integer("S8_alarm", 0, 5000, 800, "CO2 warning level [PPM]");
                WiFiSettings.warning("Not present on standard Snuffelaar.");
                WiFiSettings.warning("Mutually exclusive with MH-Z19 and T6613.");
            },
            init: []() {
                hwserial1.begin(S8_BAUDRATE, SERIAL_8N1, rx, tx);
                sensor_S8 = new S8_UART(hwserial1);
            },
            prepare: NULL,
            fetch: [](SnuffelSensor& self) {
                S8.co2 = sensor_S8->get_co2();
                if (!S8.co2) return;

                self.publish(String(S8.co2), "PPM");
                if (alarm_level) {
                    if (S8.co2 >= alarm_level) ledstatus_alarm();
                    else ledstatus_idle();
                }
            }
        };
        snuffels.push_back(s);
    }

    {
        static HTU21D HTU21D_sensor;

        struct SnuffelSensor s = {
            enabled: false,
            id: "HTU21D_temp",
            description: "temperature sensor",
            topic_suffix: "temperature",
            settings: []() {
                WiFiSettings.warning("Not present on standard Snuffelaar.");
            },
            init: []() { HTU21D_sensor.begin(); },
            prepare: NULL,
            fetch: [](SnuffelSensor& self) {
                self.publish(String(HTU21D_sensor.readTemperature()), "°C");
            }
        };
        snuffels.push_back(s);

        struct SnuffelSensor rh = {
            enabled: false,
            id: "HTU21D_RH",
            description: "relative humidity sensor",
            topic_suffix: "humidity",
            settings: []() {
                WiFiSettings.warning("Not present on standard Snuffelaar.");
            },
            init: []() { HTU21D_sensor.begin(); },
            prepare: NULL,
            fetch: [](SnuffelSensor& self) {
                self.publish(String(HTU21D_sensor.readHumidity()), "%");
            }
        };
        snuffels.push_back(rh);
    }

    {
        static AHTxx aht20(AHTXX_ADDRESS_X38, AHT2x_SENSOR);

        struct SnuffelSensor s = {
            enabled: false,
            id: "AHT2x_temp",
            description: "temperature sensor",
            topic_suffix: "temperature",
            settings: []() {
                WiFiSettings.warning("Not present on standard Snuffelaar.");
            },
            init: []() { aht20.begin(); },
            prepare: NULL,
            fetch: [](SnuffelSensor& self) {
                float ahtValue = aht20.readTemperature();
                if (ahtValue != AHTXX_ERROR) {
                    self.publish(String(ahtValue), "°C");
                }
            }
        };
        snuffels.push_back(s);

        struct SnuffelSensor rh = {
            enabled: false,
            id: "AHT2x_RH",
            description: "relative humidity sensor",
            topic_suffix: "humidity",
            settings: []() {
                WiFiSettings.warning("Not present on standard Snuffelaar.");
            },
            init: []() { aht20.begin(); },
            prepare: NULL,
            fetch: [](SnuffelSensor& self) {
                float ahtValue = aht20.readHumidity();
                if (ahtValue != AHTXX_ERROR) {
                    self.publish(String(ahtValue), "%");
                }
            }
        };
        snuffels.push_back(rh);
    }

}

void setup_ota() {
    ArduinoOTA.setHostname(WiFiSettings.hostname.c_str());
    ArduinoOTA.setPassword(WiFiSettings.password.c_str());
    ArduinoOTA.onStart(   []()              { set_led(  0,  0, 100); });
    ArduinoOTA.onEnd(     []()              { set_led(  0, 50,   0); });
    ArduinoOTA.onError(   [](ota_error_t e) { set_led(100,  0,   0); });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
        static bool x; x = !x; set_led(0, 0, x*50);
    });
    ArduinoOTA.begin();

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
    set_led(0, 0, 0);

    setup_sensors();

    WiFiSettings.hostname = "snuffelaar-";
    ota_enabled   = WiFiSettings.checkbox("snuffelaar_ota", false, "Enable remote programming through ArduinoOTA. (Uses portal password!)");
    brightness    = WiFiSettings.integer("snuffelaar_brightness", 0, 255, 80, "LED brightness");
    WiFiSettings.info("Set the brightness to 0 to disable the LED during normal operation; the default brightness will be used for the red warning signal.");


    WiFiSettings.heading("MQTT");
    String server = WiFiSettings.string("mqtt_server", 64, "test.mosquitto.org", "Broker");
    int port      = WiFiSettings.integer("mqtt_port", 0, 65535, 1883, "Broker TCP port");
    max_failures  = WiFiSettings.integer("snuffelaar_max_failures", 0, 1000, 10, "Maximum amount of failed subsequent connection attempts before restart");
    topic_prefix  = WiFiSettings.string("snuffelaar_mqtt_prefix", "snuffelaar/", "Topic prefix (ending with '/' strongly advised)");
    interval      = 1000UL * WiFiSettings.integer("snuffelaar_interval", 1, 3600, 5, "Publish interval [s]");
    add_units     = WiFiSettings.checkbox("snuffelaar_add_units", true, "Add units of measurement to messages");

    for (auto& s : snuffels) {
        String heading = s.id;
        int underscore = heading.indexOf("_");
        if (underscore > 0) heading.remove(underscore);
        heading += " " + s.description;

        WiFiSettings.heading(heading);

        s.enabled = WiFiSettings.checkbox(s.id + "_enabled", s.enabled, "Enable");
        s.topic_suffix = WiFiSettings.string(s.id + "_topic", 1, 128, s.topic_suffix, "MQTT topic suffix");
        if (s.settings) s.settings();
    }

    if (ota_enabled) WiFiSettings.onPortal = setup_ota;

    WiFiSettings.onWaitLoop = []() {
        check_button();
        ledstatus_connecting();
        return 50;
    };
    WiFiSettings.onPortalWaitLoop = []() {
        ledstatus_portal();
        if (ota_enabled) ArduinoOTA.handle();
    };

    if (!WiFiSettings.connect(false)) ESP.restart();
    ledstatus_idle();

    for (auto& s : snuffels) if (s.enabled && s.init) s.init();

    static WiFiClient wificlient;
    mqtt.begin(server.c_str(), port, wificlient);

    if (ota_enabled) setup_ota();
}

void connect_mqtt() {
    if (mqtt.connected()) return;  // already/still connected

    static int failures = 0;
    if (mqtt.connect("")) {
        failures = 0;
    } else {
        failures++;
        if (failures >= max_failures) ESP.restart();
    }
}

void loop() {
    unsigned long start = millis();

    connect_mqtt();

    mqtt.loop();
    if (ota_enabled) ArduinoOTA.handle();

    for (auto& s : snuffels) if (s.enabled && s.prepare) s.prepare();
    delay(500);
    for (auto& s : snuffels) if (s.enabled && s.fetch)   s.fetch(s);

    while (millis() < start + interval) check_button();
}
