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

Servo panServo;
Servo tiltServo;

int r = 0, g = 0, b = 0;
float brightness = 0.f;
int pan = 90, tilt = 90;

void waitWiFi()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        while (WiFi.status() != WL_CONNECTED)
        {
            statusOn();
            delay(WIFI_BLINK_DELAY);
            statusOff();
            delay(WIFI_BLINK_DELAY);
        }
    }
}

void setup()
{
    pinMode(R_PIN, OUTPUT);
    pinMode(G_PIN, OUTPUT);
    pinMode(B_PIN, OUTPUT);
    pinMode(STATUS_PIN, OUTPUT);
    panServo.attach(PAN_PIN);
    tiltServo.attach(TILT_PIN);
    WiFi.begin();
    WiFi.mode(WIFI_STA);

    waitWiFi();
}

void loop()
{
    panServo.write(pan);
    tiltServo.write(tilt);
    analogWrite(R_PIN, static_cast<int>(static_cast<float>(r) * brightness));
    analogWrite(G_PIN, static_cast<int>(static_cast<float>(g) * brightness));
    analogWrite(B_PIN, static_cast<int>(static_cast<float>(b) * brightness));
}
