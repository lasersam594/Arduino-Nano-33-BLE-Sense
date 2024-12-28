/*
Nano 33 BLE Sense RGB Cycle Colors V1.  (Same for Rev1 and Rev2.)

Copyright® Samuel M. Goldwasser, 1994-2025, all rights reserved.

This is sort of the fancy equivalent of Blink. ;-)  Cycles through a rainbow of colors in the RGB leds and toggles LED_BUILTIN,
as well as LED_PWR (which you may not even realize can be controlled).

If your first time using a Nano 33 BLE Sense, install the necessary board in the Arduino IDE:

1. Go to Tools > Board > Boards Manager or click the Boards icon, type the keyword "ble" in the search box, install "Arduino
   Mbed OS Nano Boards".
2. Go to Tools > Board, and select: Arduino MBed OS Nano Boards > Arduino Nano 33 BLE.
3. Go to Tools > Port, and select the correct port.

The sketch should then compile without errors (though there may be warnings that can be ignored).
*/

// Color palette for audio in RGB_LEDs
#define BLACK 0, 0, 0
#define GRAY 7, 7, 7
#define MAGENTA 25, 0, 25
#define BLUE 0, 0, 75
#define CYAN 0, 50, 50
#define GREEN 0, 192, 0
#define YELLOW 128, 92, 0
#define ORANGE 200, 40, 0
#define RED 255, 0, 0
#define WHITE 255, 255, 255

bool state;

void setup() {
  // initialize LEDs.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  // Turn LED_USER off and set the RGB LEDs at low brightness before cycling colors
  digitalWrite(LED_BUILTIN,1);
  digitalWrite(LED_PWR,1);
  RGB_LED_Color(GRAY);
  delay(1000); // Pause before cycling
}

// Cycle through a rainbow (more or less) of colors in the RGB_LEDs and toggle LED_USER
void loop() {
  digitalWrite(LED_BUILTIN, !(digitalRead(LED_BUILTIN)));
  if (state == LOW) digitalWrite(LED_PWR, !(digitalRead(LED_PWR)));
  state =!state;
  RGB_LED_Cycle_Colors();
}

void RGB_LED_Color(int r, int g, int b) {
  analogWrite(LEDR, 255 - r);
  analogWrite(LEDG, 255 - g);
  analogWrite(LEDB, 255 - b);
}

void RGB_LED_Cycle_Colors() {
  RGB_LED_Color(BLACK); delay(50);
  RGB_LED_Color(GRAY); delay(50);
  RGB_LED_Color(MAGENTA); delay(50);
  RGB_LED_Color(BLUE); delay(50);
  RGB_LED_Color(CYAN); delay(50);
  RGB_LED_Color(GREEN); delay(50);
  RGB_LED_Color(YELLOW); delay(50);
  RGB_LED_Color(ORANGE); delay(50);
  RGB_LED_Color(RED); delay(50);
  RGB_LED_Color(ORANGE); delay(50);
  RGB_LED_Color(YELLOW); delay(50);
  RGB_LED_Color(GREEN); delay(50);
  RGB_LED_Color(CYAN); delay(50);
  RGB_LED_Color(BLUE); delay(50);
  RGB_LED_Color(MAGENTA); delay(50);
  RGB_LED_Color(GRAY); delay(50);
}