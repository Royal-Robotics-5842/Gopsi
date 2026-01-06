#include <IBusBM.h>
#include <ESP32Servo.h>
#include <NeoPixelBus.h>
// Audio/SD Libraries
//test change
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "AudioOutputI2S.h"
#include "AudioFileSourceSD.h"
#include "AudioGeneratorWAV.h"
//edit
// --- UPDATED PINS ---
#define SD_CS          5
#define LED_PIN        4   // Moved from 27
#define servoPinA     13   // Moved from 18
#define servoPinB     22  
#define servoPinC     21  
#define pinMainValve  14   // Moved from 26
#define pinNC         32
#define pinPressureUp 33
#define pinPressureDown 27 // Moved from 25
#define PRESSURE_PIN  34
#define IBus_RX       16
#define IBus_TX       17

#define BARREL_LEDS 94
#define TEMPBASE 400

// --- AUDIO GLOBALS ---
AudioGeneratorWAV *wav;
AudioFileSourceSD *file;
AudioOutputI2S *out;

// --- EXISTING GLOBALS ---
unsigned long lastShootTime = 0;
unsigned long lastBlinkTime = 0;
const int IDLE_DELAY = 2000;
const int blinkInterval = 500;
bool orangeOn = false;
int rainbowOffset = 0;

NeoPixelBus<NeoGrbwFeature, NeoEsp32Rmt0Ws2812xMethod> barrelStrip(BARREL_LEDS, LED_PIN);


uint16_t speed = 0;
uint16_t temp = TEMPBASE + 200;

IBusBM IBus;
Servo ServoA, ServoB, ServoC;

bool shootLatch = false;
unsigned long dumpStart = 0;
bool dumpingToZero = false;
const unsigned long dumpDuration = 10000;

const float RAW_ZERO = 350.0;
const float RAW_FULL = 1725.0;
const float PSI_FULL = 100.0;  
float psi = 0.0, targetPsi = 0.0;

int c1, c1sav, c2, c2sav, c3, c3sav, c4, c4sav, c5, c5sav;
int c6, c6sav, c7, c7sav, c8, c8sav, c9, c9sav, c10, c10sav;

// Prototypes
void read_stick(int index, int channel);
void output_drivetrain();
void psiBar();
int getSavedValue(int channel);
void putValues(int channel, int val, int savval);

void setup() {
  Serial.begin(115200);

  // 1. Initialize SD Card
  if(!SD.begin(SD_CS)) {
    Serial.println("SD Mount Failed");
  }

  // 2. Initialize Audio (Internal DAC)
  out = new AudioOutputI2S(0, 1);
  out->begin();
  wav = new AudioGeneratorWAV();

  // 3. IBus
  IBus.begin(Serial2, 1, IBus_RX, IBus_TX);
  IBus.addSensor(IBUSS_RPM);
  IBus.addSensor(IBUSS_TEMP);

  // 4. Servos
  ServoA.attach(servoPinA);
  ServoB.attach(servoPinB);
  ServoC.attach(servoPinC);

  // 5. Pins
  pinMode(pinMainValve, OUTPUT);
  pinMode(pinNC, OUTPUT);
  pinMode(pinPressureUp, OUTPUT);
  pinMode(pinPressureDown, OUTPUT);
  digitalWrite(pinNC, HIGH);
  digitalWrite(pinPressureUp, HIGH);
  digitalWrite(pinPressureDown, HIGH);
  digitalWrite(pinMainValve, HIGH);

  analogSetPinAttenuation(PRESSURE_PIN, ADC_11db);
  barrelStrip.Begin();
    playAudio("/applepay.wav");
}
// ---------------- Main Loop ----------------
void loop() {
  unsigned long now = millis();
  static unsigned long lastIdleUpdate = 0;
  static int idleOffset = 0;
  const unsigned long IDLE_INTERVAL = 200;

  // CRITICAL: Feed the audio buffer
  if (wav->isRunning()) {
    if (!wav->loop()) wav->stop();
  }

  for (int i = 0; i < 10; i++) read_stick(i, i + 1);

  int raw = analogRead(PRESSURE_PIN);
  psi = (raw - RAW_ZERO) * (PSI_FULL / (RAW_FULL - RAW_ZERO));
  if (psi < 0) psi = 0;

  // Servo & Valve Logic (Pins updated via #define)
  if (c7 > 1700) {
    output_drivetrain();
    ServoC.writeMicroseconds(c2);
  } else {
    ServoA.writeMicroseconds(1500);
    ServoB.writeMicroseconds(1500);
    ServoC.writeMicroseconds(1500);
    // Blink Logic...
    if (millis() - lastBlinkTime >= blinkInterval) {
        lastBlinkTime = millis();
        orangeOn = !orangeOn;
        for (int i = 0; i < BARREL_LEDS; i++) {
            barrelStrip.SetPixelColor(i, orangeOn ? RgbwColor(255, 50, 0, 0) : RgbwColor(0, 0, 0, 0));
        }
        barrelStrip.Show();
    }
  }

static int lastSound = -1;

// FORCE OFF if c8 is low
if (c8 > 1700) {
    if (wav->isRunning()) {
        wav->stop();
        delete file;
        file = nullptr;
    }
    lastSound = -1;
} else {
    // Map 1000–2000 → 1–10 (2000 included)
    int soundNum = constrain((c5 / 100) - 9, 1, 10);

    if (soundNum != lastSound) {
        lastSound = soundNum;
        Serial.println(soundNum);

        if (wav->isRunning()) {
            wav->stop();
            delete file;
            file = nullptr;
        }

        char filename[16];
        sprintf(filename, "/%d.wav", soundNum);
        file = new AudioFileSourceSD(filename);
        wav->begin(file, out);
    }
}



targetPsi = (c6 - 1000) / 10.0;
Serial.printf("Raw: %d, Target PSI: %.1f, Current PSI: %.1f, empty=%d\n", raw, targetPsi, psi, millis() - dumpStart);

// Valve logic
if (c7 > 1700) {
    if (c10 < 1200) { // not shooting
        // Dump to zero if target PSI is 0
        if (targetPsi == 0.0 && psi != 0.0) {
            if (!dumpingToZero) {
                dumpingToZero = true;
                dumpStart = millis();
            }
        }
        if (millis() - dumpStart < dumpDuration) {
            digitalWrite(pinPressureUp, LOW);
            digitalWrite(pinPressureDown, HIGH);
            digitalWrite(pinMainValve, HIGH);
            shootLatch = true;
        } else {
            dumpingToZero = false;
            digitalWrite(pinPressureUp, HIGH);
            digitalWrite(pinPressureDown, HIGH);
            digitalWrite(pinMainValve, HIGH);
            shootLatch = true;
        }
        if (psi < targetPsi - 4.0) {
            digitalWrite(pinPressureUp, HIGH);
            digitalWrite(pinPressureDown, LOW);
            digitalWrite(pinMainValve, HIGH);
            shootLatch = true;
            psiBar();
        } else if (psi >= targetPsi + 4.0) {
            digitalWrite(pinPressureUp, LOW);
            digitalWrite(pinPressureDown, HIGH);
            digitalWrite(pinMainValve, HIGH);
            shootLatch = true;
            dumpingToZero = false;
            psiBar();
        } else {
            digitalWrite(pinPressureUp, HIGH);
            digitalWrite(pinPressureDown, HIGH);
            digitalWrite(pinMainValve, HIGH);
            shootLatch = true;
            psiBar();
        }
    } else { // shooting
        if (shootLatch) {   // fire only once per press
            shootLatch = false;
            const int JOLT_LEN = 10;
            for (int head = 0; head < BARREL_LEDS; head += 7) {
                barrelStrip.ClearTo(RgbwColor(0,0,0,0));
                for (int i = 0; i < JOLT_LEN; i++) {
                    int p = head - i;
                    if (p >= 0 && p < BARREL_LEDS) {
                        uint8_t hue = (p * 256 / BARREL_LEDS) & 0xFF;
                        RgbColor c = HslColor(hue / 255.0f, 1.0f, 0.5f);
                        barrelStrip.SetPixelColor(p, RgbwColor(c.R, c.G, c.B, 0));
                    }
                }
                barrelStrip.Show();
                delay(7);
                barrelStrip.ClearTo(RgbwColor(0,0,0,0));
                barrelStrip.Show();
                lastShootTime = millis();
            }
            playAudio("/fahh.wav");
        } else {
            // Idle LED effect
            if (now - lastShootTime > IDLE_DELAY) {
                if (now - lastIdleUpdate > IDLE_INTERVAL) {
                    barrelStrip.ClearTo(RgbwColor(0,0,0,0));
                    for (int i = 0; i < BARREL_LEDS; i++) {
                        if ((i + idleOffset) % 3 == 0) {
                            barrelStrip.SetPixelColor(i, RgbwColor(255,0,0,0)); // Red dot
                        }
                    }
                    barrelStrip.Show();
                    idleOffset = (idleOffset + 1) % 3; // move pattern
                    lastIdleUpdate = now;
                }
                digitalWrite(pinPressureUp, HIGH);
                digitalWrite(pinPressureDown, HIGH);
                digitalWrite(pinMainValve, LOW);
            }
        }
        digitalWrite(pinPressureUp, HIGH);
        digitalWrite(pinPressureDown, HIGH);
        digitalWrite(pinMainValve, LOW);

    }
}

  delay(10); // Reduced delay to keep audio loop smooth
}

  

// ---------------- Helper Functions ----------------

void playAudio(const char* filename) {
    if (wav->isRunning()) {
        wav->stop();
        delete file;
        file = nullptr;
    }
    file = new AudioFileSourceSD(filename);
    wav->begin(file, out);
}

void psiBar() {
    // If both PSI and target PSI are zero, show a slowly moving rainbow
    if (psi == 0.0 && targetPsi == 0.0) {
        for (int i = 0; i < BARREL_LEDS; i++) {
            uint8_t hue = (uint8_t)((i * 256 / BARREL_LEDS + rainbowOffset) ) & 0xFF;
            RgbColor c = HslColor(hue / 255.0f, 1.0f, 0.5f);
            barrelStrip.SetPixelColor(i, RgbwColor(c.R, c.G, c.B, 0));
        }
        barrelStrip.Show();

        // Increment slowly for smooth movement
        rainbowOffset += 1.0;      // smaller = slower, larger = faster
        if (rainbowOffset >= 256) rainbowOffset = 0;
        return;
    }

    // LED bar for PSI
    int psiLeds    = constrain((int)(psi / 100.0 * BARREL_LEDS), 0, BARREL_LEDS);
    int targetLeds = constrain((int)(targetPsi / 100.0 * BARREL_LEDS), 0, BARREL_LEDS);

    barrelStrip.ClearTo(RgbwColor(0,0,0,0));

    // Green = actual PSI
    for (int i = 0; i < psiLeds; i++) {
        barrelStrip.SetPixelColor(i, RgbwColor(0,255,0,0));
    }

    // Yellow = target PSI
    if (targetLeds > psiLeds) {
        for (int i = psiLeds; i < targetLeds; i++) {
            barrelStrip.SetPixelColor(i, RgbwColor(0,0,0,200));
        }
    } else if (targetLeds < psiLeds) {
        if (targetLeds >= 0 && targetLeds < BARREL_LEDS) {
            barrelStrip.SetPixelColor(targetLeds, RgbwColor(0,0,0,200));
        }
    }

    barrelStrip.Show();
}






void output_drivetrain() {
  int Temp = c3;
  float LRMod = (c4 - 1500) * -1;
  int A = int(Temp + (LRMod * 0.75));
  int B = int(Temp - (LRMod * 0.75));
  ServoA.writeMicroseconds((-1 * (A - 1500)) + 1500);
  ServoB.writeMicroseconds(B);
}

void read_stick(int index, int channel) {
  int val = IBus.readChannel(index);
  int savval = getSavedValue(channel);
  if (savval != val) savval = val;
  putValues(channel, val, savval);
}

void shootRainbow() {
  for (int offset = 0; offset < BARREL_LEDS; offset++) {
    for (int i = 0; i < BARREL_LEDS; i++) {
      uint8_t hue = (i * 256 / BARREL_LEDS + offset * 10) & 0xFF;
      RgbColor c = HslColor(hue / 255.0f, 1.0f, 0.5f);
      barrelStrip.SetPixelColor(i, RgbwColor(c.R, c.G, c.B, 0));
    }
    barrelStrip.Show();
    delay(5);
  }
}

int getSavedValue(int channel) {
  switch (channel) {
    case 1: return c1sav;
    case 2: return c2sav;
    case 3: return c3sav;
    case 4: return c4sav;
    case 5: return c5sav;
    case 6: return c6sav;
    case 7: return c7sav;
    case 8: return c8sav;
    case 9: return c9sav;
    case 10: return c10sav;
    default: return 0;
  }
}

void putValues(int channel, int val, int savval) {
  switch (channel) {
    case 1: c1 = val; c1sav = savval; break;
    case 2: c2 = val; c2sav = savval; break;
    case 3: c3 = val; c3sav = savval; break;
    case 4: c4 = val; c4sav = savval; break;
    case 5: c5 = val; c5sav = savval; break;
    case 6: c6 = val; c6sav = savval; break;
    case 7: c7 = val; c7sav = savval; break;
    case 8: c8 = val; c8sav = savval; break;
    case 9: c9 = val; c9sav = savval; break;
    case 10: c10 = val; c10sav = savval; break;
  }
}
