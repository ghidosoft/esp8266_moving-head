#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Servo.h>

#define PAN_PIN D1
#define TILT_PIN D2
#define R_PIN D5
#define G_PIN D6
#define B_PIN D7

#define STATUS_PIN D4

#define WIFI_BLINK_DELAY 200

#define statusOn() digitalWrite(STATUS_PIN, LOW)
#define statusOff() digitalWrite(STATUS_PIN, HIGH)
#define statusBlink(ms) do { statusOn(); delay(ms); statusOff(); delay(ms); } while (0)

Servo panServo;
Servo tiltServo;

float r = 0.f, g = 0.f, b = 0.f;
float brightness = 0.f;
float targetPan = 0.f, targetTilt = 0.f;
float panSpeed = 180.f, tiltSpeed = 180.f; // per sec

float currentPan = targetPan, currentTilt = targetTilt;

unsigned long lastMicros;

void waitWiFi()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.print("Waiting for wifi...");
        while (WiFi.status() != WL_CONNECTED)
        {
            Serial.print(".");
            statusBlink(WIFI_BLINK_DELAY);
        }
        Serial.println(" done.");
        Serial.print("SSID:\t");
        Serial.println(WiFi.SSID());
        Serial.print("IP:\t");
        Serial.println(WiFi.localIP());
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting up...");

    analogWriteRange(255); // should be the default
    pinMode(R_PIN, OUTPUT);
    pinMode(G_PIN, OUTPUT);
    pinMode(B_PIN, OUTPUT);
    pinMode(STATUS_PIN, OUTPUT);
    panServo.attach(PAN_PIN);
    tiltServo.attach(TILT_PIN);
    WiFi.begin();
    WiFi.mode(WIFI_STA);

    waitWiFi();

    Serial.println("Setup completed.");

    lastMicros = micros();
}

void update(unsigned long us, float deltaTime)
{
    // TODO: use speeds
    currentPan = targetPan;
    currentTilt = targetTilt;
}

void loop()
{
    const auto now = micros();
    const auto elapsed = now - lastMicros;
    const auto deltaTime = static_cast<float>(elapsed) / 1e6f;
    lastMicros = now;

    update(elapsed, deltaTime);

    panServo.write(static_cast<int>(currentPan));
    tiltServo.write(static_cast<int>(currentTilt));
    analogWrite(R_PIN, static_cast<int>(r * brightness));
    analogWrite(G_PIN, static_cast<int>(g * brightness));
    analogWrite(B_PIN, static_cast<int>(b * brightness));
}
