# Snuffelding

Firmware for [Snuffelaar](https://revspace.nl/Snuffelaar) to push the sensor
values to MQTT.

Intended to be used with PlatformIO, which will automatically grab the
necessary libraries. Probably works with the Arduino IDE, but you'd have to
install numerous libraries manually.

## Magic incantation

Personally I don't bother with the PlatformIO GUI at all. It's easier to just
use it from the command line:

```
pio run -t upload && pio device monitor
```

## Configuration

Snuffelding uses [ESP-WiFiSettings](https://github.com/Juerd/ESP-WiFiSettings)
for lots of customizable settings, including WiFi network credentials and MQTT
topics. The first time your Snuffelaar uses ESP-WiFiSettings, it will become an
access point with a configuration portal. After that, you can get the portal
back by pressing the portal button. Just connect to `esp-123abc` and go to
http://192.168.4.1/. Hint: use a smartphone or tablet.

## OTA

ArduinoOTA is disabled by default. It can be enabled using the portal. Don't
forget to edit `platformio.ini` with your hostname and password.

To upload:

```
pio run -t upload -e ota
```
