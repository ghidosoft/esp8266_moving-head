/* This file is part of Lobster Mini Moving Head, Copyright (C) 2023 Andrea Ghidini.
 *
 * Lobster Mini Moving Head is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Lobster Mini Moving Head is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Lobster Mini Moving Head. If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Servo.h>

#include <ArtnetWiFi.h>

#define PAN_PIN D1
#define TILT_PIN D2
#define R_PIN D5
#define G_PIN D6
#define B_PIN D7

#define STATUS_PIN D4

#define WIFI_BLINK_DELAY 200

#define ARTNET_UNIVERSE 0
#define ARTNET_NET 0
#define ARTNET_SUBNET 0

#define DMX_CHANNEL_BASE 0
#define DMX_CHANNEL_R 0
#define DMX_CHANNEL_G 1
#define DMX_CHANNEL_B 2
#define DMX_CHANNEL_BRIGHTNESS 3
#define DMX_CHANNEL_STROBE 4
#define DMX_CHANNEL_PAN 5
#define DMX_CHANNEL_TILT 6
#define DMX_CHANNEL_SPEED 7

#define STROBE_MIN 0.2f
#define STROBE_MAX 50.f

#define MAX_SPEED 180.f

#define statusOn() digitalWrite(STATUS_PIN, LOW)
#define statusOff() digitalWrite(STATUS_PIN, HIGH)
#define statusBlink(ms) do { statusOn(); delay(ms); statusOff(); delay(ms); } while (0)

Servo panServo;
Servo tiltServo;

float r = 0.f, g = 0.f, b = 0.f;
float brightness = 0.f;
uint8_t strobe = 0;
float targetPan = 0.f, targetTilt = 0.f;
float panSpeed = 0.f, tiltSpeed = 0.f; // per sec

float currentPan = targetPan, currentTilt = targetTilt;
int strobeOn = 0;

unsigned long lastMicros;

unsigned long currentStrobePeriod;

ArtnetWiFiReceiver artnet;

void artnetCallback(const uint8_t* data, const uint16_t size)
{
    if (size > DMX_CHANNEL_BASE + DMX_CHANNEL_R)
        r = static_cast<float>(data[DMX_CHANNEL_BASE + DMX_CHANNEL_R]);
    if (size > DMX_CHANNEL_BASE + DMX_CHANNEL_G)
        g = static_cast<float>(data[DMX_CHANNEL_BASE + DMX_CHANNEL_G]);
    if (size > DMX_CHANNEL_BASE + DMX_CHANNEL_B)
        b = static_cast<float>(data[DMX_CHANNEL_BASE + DMX_CHANNEL_B]);
    if (size > DMX_CHANNEL_BASE + DMX_CHANNEL_BRIGHTNESS)
        brightness = static_cast<float>(data[DMX_CHANNEL_BASE + DMX_CHANNEL_BRIGHTNESS]) / 255.f;
    if (size > DMX_CHANNEL_BASE + DMX_CHANNEL_STROBE)
    {
        strobe = data[DMX_CHANNEL_BASE + DMX_CHANNEL_STROBE];
        const auto x = pow(2.f, (strobe - 10.f) / 235.f) * .5f - .5f;
        currentStrobePeriod = static_cast<unsigned long>(1000.f / (x * (STROBE_MAX - STROBE_MIN) + STROBE_MIN));
    }
    if (size > DMX_CHANNEL_BASE + DMX_CHANNEL_PAN)
        targetPan = static_cast<float>(data[DMX_CHANNEL_BASE + DMX_CHANNEL_PAN]) * 180.f / 255.f;
    if (size > DMX_CHANNEL_BASE + DMX_CHANNEL_TILT)
        targetTilt = static_cast<float>(data[DMX_CHANNEL_BASE + DMX_CHANNEL_TILT]) * 180.f / 255.f;
    if (size > DMX_CHANNEL_BASE + DMX_CHANNEL_SPEED)
        panSpeed = tiltSpeed = static_cast<float>(data[DMX_CHANNEL_BASE + DMX_CHANNEL_SPEED]) * MAX_SPEED / 255.f;
}

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
    panServo.write(static_cast<int>(currentPan));
    tiltServo.write(static_cast<int>(currentTilt));
    WiFi.begin();
    WiFi.mode(WIFI_STA);

    waitWiFi();

    artnet.begin(ARTNET_NET, ARTNET_SUBNET);
    artnet.subscribe(ARTNET_UNIVERSE, artnetCallback);
    artnet.shortname("LS Moving Head");
    artnet.longname("Lobster Mini Moving Head");
    artnet.nodereport("");

    Serial.println("Setup completed.");

    lastMicros = micros();
}

void update(unsigned long deltaUs, float deltaTime)
{
    // Serial.printf("%.1f\t%.1f\t%.1f\t%.3f\n", targetPan - currentPan, targetPan, currentPan, panSpeed * deltaTime);

    if (std::abs(targetPan - currentPan) >= panSpeed * deltaTime)
        currentPan += std::copysign(panSpeed * deltaTime, targetPan - currentPan);
    else
        currentPan = targetPan;

    if (std::abs(targetTilt - currentTilt) >= tiltSpeed * deltaTime)
        currentTilt += std::copysign(tiltSpeed * deltaTime, targetTilt - currentTilt);
    else
        currentTilt = targetTilt;

    if (strobe < 10)
        strobeOn = 0;
    else if (strobe > 245)
        strobeOn = 1;
    else
    {
        const auto ms = millis();
        strobeOn = (ms / currentStrobePeriod) & 1;
    }
}

void loop()
{
    const auto now = micros();
    const auto elapsed = now - lastMicros;
    const auto deltaTime = static_cast<float>(elapsed) / 1e6f;
    lastMicros = now;

    artnet.parse();

    update(elapsed, deltaTime);

    panServo.write(static_cast<int>(currentPan));
    tiltServo.write(static_cast<int>(currentTilt));
    analogWrite(R_PIN, static_cast<int>(r * brightness) * strobeOn);
    analogWrite(G_PIN, static_cast<int>(g * brightness) * strobeOn);
    analogWrite(B_PIN, static_cast<int>(b * brightness) * strobeOn);
}
