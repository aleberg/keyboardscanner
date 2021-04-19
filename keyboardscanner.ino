/*
  Moura's Keyboard Scanner, copyright (C) 2017 Daniel Moura <oxe@oxesoft.com>
  Modified and expanded by Alessandro Guttenberg guttenba@gmail.com, April 2021

  This code is originally hosted at https://github.com/oxesoft/keyboardscanner

  This is a modified version for a 25 key Fatar keybed, which also adds Aftertouch (Channel Pressure), Mod Wheel and Pitch Wheel controls.
  The MIDI output is passed to a MIDI to CV converter, and used to control analog synthesizers.
 
  Tested & Working:
  Note On
  Note Off
  Velocity (passed with Note On and Note Off events, updated only for Note On, and routed to CC71 Volume)
  Aftertouch (Channel Pressure)
  Mod Wheel
  Pitch Wheel

  TODO:
  - add +/- octave switch

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
*/

#include <DIO2.h>

#define KEYS_NUMBER 25

#define KEY_OFF               0
#define KEY_START             1
#define KEY_ON                2
#define KEY_RELEASED          3


#define MIN_TIME_MS   3
#define MAX_TIME_MS   50
#define MAX_TIME_MS_N (MAX_TIME_MS - MIN_TIME_MS)

#define FSR_PIN A0 // Force Sensitive Resistor (ribbon), with pull-down resistor
#define PITCH_WHEEL_PIN A1
#define MOD_WHEEL_PIN A2
#define OCTAVE_UP_PIN //digital high low
#define OCTAVE_DOWN_PIN //digital high low

#define PIN_B1  52
#define PIN_B2  50
#define PIN_B3  48
#define PIN_B4  46
#define PIN_B5  44
#define PIN_B6  42
#define PIN_B7  40
#define PIN_B8  38

#define PIN_C3  49
#define PIN_C4  47
#define PIN_C5  45
#define PIN_C6  43
#define PIN_C7  41
#define PIN_C8  39
#define PIN_C9  37
#define PIN_C10 35

byte input_pins[] = {
  PIN_B5, //C2
  PIN_B5,
  PIN_B6,
  PIN_B6,
  PIN_B7,
  PIN_B7,
  PIN_B8,
  PIN_B8,
  PIN_B1,
  PIN_B1,
  PIN_B2,
  PIN_B2,
  PIN_B3,
  PIN_B3,
  PIN_B4,
  PIN_B4,
  PIN_B5,
  PIN_B5,
  PIN_B6,
  PIN_B6,
  PIN_B7,
  PIN_B7,
  PIN_B8,
  PIN_B8,
  PIN_B1, //C3
  PIN_B1,
  PIN_B2,
  PIN_B2,
  PIN_B3,
  PIN_B3,
  PIN_B4,
  PIN_B4,
  PIN_B5,
  PIN_B5,
  PIN_B6,
  PIN_B6,
  PIN_B7,
  PIN_B7,
  PIN_B8,
  PIN_B8,
  PIN_B1,
  PIN_B1,
  PIN_B2,
  PIN_B2,
  PIN_B3,
  PIN_B3,
  PIN_B4,
  PIN_B4,
  PIN_B5, //C4
  PIN_B5
};

byte output_pins[] = {
  PIN_C4, //C2
  PIN_C3,
  PIN_C4,
  PIN_C3,
  PIN_C4,
  PIN_C3,
  PIN_C4,
  PIN_C3,
  PIN_C6,
  PIN_C5,
  PIN_C6,
  PIN_C5,
  PIN_C6,
  PIN_C5,
  PIN_C6,
  PIN_C5,
  PIN_C6,
  PIN_C5,
  PIN_C6,
  PIN_C5,
  PIN_C6,
  PIN_C5,
  PIN_C6,
  PIN_C5,
  PIN_C8, //C3
  PIN_C7,
  PIN_C8,
  PIN_C7,
  PIN_C8,
  PIN_C7,
  PIN_C8,
  PIN_C7,
  PIN_C8,
  PIN_C7,
  PIN_C8,
  PIN_C7,
  PIN_C8,
  PIN_C7,
  PIN_C8,
  PIN_C7,
  PIN_C10,
  PIN_C9,
  PIN_C10,
  PIN_C9,
  PIN_C10,
  PIN_C9,
  PIN_C10,
  PIN_C9,
  PIN_C10, //C4
  PIN_C9
};

//uncomment the next line to inspect the number of scans per seconds
//#define DEBUG_SCANS_PER_SECOND

/*
  426 cyles per second (2,35ms per cycle) using standard digitalWrite/digitalRead
  896 cyles per second (1,11ms per cycle) using DIO2 digitalWrite2/digitalRead2
*/

//uncoment the next line to get text midi message at output
#define DEBUG_MIDI_MESSAGE

byte          keys_state[KEYS_NUMBER];
unsigned long keys_time[KEYS_NUMBER];
boolean       signals[KEYS_NUMBER * 2];
byte          fsrReading;      // the analog reading from the FSR resistor divider
byte          pitchWheelReading;
byte          modWheelReading;
byte          pitchWheelNewReading;
byte          modWheelNewReading;

const int numReadings = 10;
int readingsPitch[numReadings];
int readingsMod[numReadings];
int readIndex = 0;
int totalPitch = 0;
int totalMod = 0;
int averagePitch = 0;
int averageMod = 0;
int pitchMidPoint = 64;
int pitchDeadZone = 4;

void setup() {

#ifdef DEBUG_MIDI_MESSAGE
  Serial.begin(115200);
#else
  Serial.begin(31250);
#endif

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  int i;
  for (i = 0; i < KEYS_NUMBER; i++)
  {
    keys_state[i] = KEY_OFF;
    keys_time[i] = 0;
  }
  for (byte pin = 0; pin < sizeof(output_pins); pin++)
  {
    pinMode(output_pins[pin], OUTPUT);
  }
  for (byte pin = 0; pin < sizeof(input_pins); pin++)
  {
    pinMode(input_pins[pin], INPUT_PULLUP);
  }

  pinMode(FSR_PIN, INPUT);
  fsrReading = 0;

  pinMode(PITCH_WHEEL_PIN, INPUT);
  pitchWheelReading = 64;

  pinMode(MOD_WHEEL_PIN, INPUT);
  modWheelReading = 0;

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readingsPitch[thisReading] = 0;
    readingsMod[thisReading] = 0;
  }
}

void send_midi_note_event(int command, int key_index, unsigned long time)
{
  unsigned long t = time;

  if (t > MAX_TIME_MS)
    t = MAX_TIME_MS;
  if (t < MIN_TIME_MS)
    t = MIN_TIME_MS;
  t -= MIN_TIME_MS;
  unsigned long velocity = 127 - (t * 127 / MAX_TIME_MS_N);
  int vel = (((velocity * velocity) >> 7) * velocity) >> 7;
  int key = 48 + key_index; //octave switch in place of hardcoded number
#ifdef DEBUG_MIDI_MESSAGE
  char out[32];
  sprintf(out, "%02X %d %03d %d", command, key, vel, time);
  Serial.println(out);
#else
  Serial.write(command);
  Serial.write(key);
  Serial.write(vel);
  // only update velocity (volume) for Note On event
  if (command == 144)
  {
    send_midi_cc_event(176, 7, vel);
  }
#endif
}

void send_midi_pressure_event(int command, int pressure)
{
#ifdef DEBUG_MIDI_MESSAGE
  char out[32];
  sprintf(out, "%02X %d", command, pressure);
  Serial.println(out);
#else
  Serial.write(command);
  Serial.write(pressure);
#endif
}

void send_midi_cc_event(int status_byte, int param1, int param2)
{
#ifdef DEBUG_MIDI_MESSAGE
  char out[32];
  sprintf(out, "%02X %d %d", status_byte, param1, param2);
  Serial.println(out);
#else
  Serial.write(status_byte);
  Serial.write(param1);
  Serial.write(param2);
#endif
}


void loop() {
  
#ifdef DEBUG_SCANS_PER_SECOND
  static unsigned long cycles = 0;
  static unsigned long start = 0;
  static unsigned long current = 0;
  cycles++;
  current = millis();
  if (current - start >= 1000)
  {
    Serial.println(cycles);
    cycles = 0;
    start = current;
  }
#endif

  totalPitch = totalPitch - readingsPitch[readIndex];
  readingsPitch[readIndex] = map(analogRead(PITCH_WHEEL_PIN), 0, 1023, 0, 127);

  totalMod = totalMod - readingsMod[readIndex];
  readingsMod[readIndex] = map(analogRead(MOD_WHEEL_PIN), 0, 1023, 0, 127);

  if ((readingsPitch[readIndex] >= pitchMidPoint - pitchDeadZone) && (readingsPitch[readIndex]<= pitchMidPoint + pitchDeadZone))
  {
    readingsPitch[readIndex] = pitchMidPoint;
  }
  
  totalPitch = totalPitch + readingsPitch[readIndex];
  totalMod = totalMod + readingsMod[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= numReadings) 
  {
    readIndex = 0;
  }
  
  averagePitch = totalPitch / numReadings;
  averageMod = totalMod / numReadings;

   pitchWheelNewReading =  averagePitch;
  if (pitchWheelNewReading != pitchWheelReading)
  {
    send_midi_cc_event(224, 0, pitchWheelNewReading);
    pitchWheelReading = pitchWheelNewReading;
  }

  modWheelNewReading =  averageMod;
  if (modWheelNewReading != modWheelReading)
  {
    send_midi_cc_event(176, 1, modWheelNewReading);
    modWheelReading = modWheelNewReading;
  }

  fsrReading = map(analogRead(FSR_PIN), 0, 850, 0, 127);

  boolean *s = signals;
  for (byte i = 0; i < KEYS_NUMBER * 2; i++)
  {
    byte output_pin = output_pins[i];
    byte input_pin = input_pins[i];
    digitalWrite2(output_pin, LOW);
    *(s++) = !digitalRead2(input_pin);
    digitalWrite2(output_pin, HIGH);
  }

    byte          *state  = keys_state;
    unsigned long *ktime  = keys_time;
    boolean       *signal = signals;
    for (byte key = 0; key < KEYS_NUMBER; key++)
    {
        for (byte state_index = 0; state_index < 2; state_index++)
        {
            switch (*state)
            {
            case KEY_OFF:
                if (state_index == 0 && *signal)
                {
                    *state = KEY_START;
                    *ktime = millis();
                }
                break;
            case KEY_START:
            
                if (state_index == 0 && !*signal)
                {
                    *state = KEY_OFF;
                    break;
                }
                
                if (state_index == 1 && *signal)
                {
                    *state = KEY_ON;
                    send_midi_note_event(144, key, millis() - *ktime);
                }
                break;
            case KEY_ON:
                if (fsrReading > 0)
                {
                send_midi_pressure_event(208, fsrReading);
                }
                if (state_index == 1 && !*signal)
                {
                    send_midi_pressure_event(208, 0);
                    *state = KEY_RELEASED;
                    *ktime = millis();
                }
                break;
            case KEY_RELEASED:
                if (state_index == 0 && !*signal)
                {
                    *state = KEY_OFF;
                    send_midi_note_event(128, key, millis() - *ktime);
                }
                break;
            }
            signal++;
        }
        state++;
        ktime++;
    }
}
