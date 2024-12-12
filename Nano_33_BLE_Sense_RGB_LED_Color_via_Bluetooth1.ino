/*
This is a slightly fluffed up version of the tutorial sketch "Controlling RGB LED Through
Bluetooth®" at https://docs.arduino.cc/tutorials/nano-33-ble-sense/bluetooth/ .

Tested using the LightBlue iPhone App.  Compile the sketch and the RGB LEDs will cycle through a
rainbow of colors just to show off.  Then, using LightBlue, touch "Connect" when Arduino comes up.
The LED_BUILTIN (yellow) the should come and the LED_PWR (green) should go off indicating that the
Nano has connected and the RGB LEDs will go to black.  Select the ">" next to "Properties: Read,
Write" under Device Information.  To set a specific color, touch "Write new value" and enter a HEX
number from 1 to B.  Back up to the Peripheral selection screen to disconnect.  Yes, this gets
rather boring rather quickly, but it should be painless. ;-)

Modified from original by Samuel M. Goldwasser, no copyright® by me, do with it as you see fit. ;-)
*/

#include <ArduinoBLE.h>

BLEService ledService("180A"); // BLE LED Service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("2A57", BLERead | BLEWrite);

// Color palette for RGB_LEDs.
#define BLACK 0,0,0
#define GRAY 7,7,7
#define MAGENTA 15,0,30
#define BLUE 0,0,75
#define CYAN 0,50,50
#define GREEN 0,192,0
#define YELLOW 128,92,0
#define ORANGE 200,40,0
#define RED 255,0,0
#define WHITE 255,255,255

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // set LED's pin to output mode
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy failed!");
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Nano 33 BLE Sense");
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(ledService);

  // set the initial value for the characteristic:
  switchCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();
  Serial.println("BLE LED Peripheral");
}

void loop() {
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);            // turn on the LED_BUILTIN to indicate the connection
    digitalWrite(LED_PWR, LOW);                 // Also turn off PWR_LED

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic, use the value to control the RGB LEDs:
      if (switchCharacteristic.written()) {
        switch (switchCharacteristic.value()) {   // any value other than 0
          case 1: Serial.println("LEDs OFF"); RGB_LED_Color(BLACK); break;
          case 2: Serial.println("LEDs Gray"); RGB_LED_Color(GRAY); break;
          case 3: Serial.println("LEDs Magenta"); RGB_LED_Color(MAGENTA); break;
          case 4: Serial.println("LEDs Blue"); RGB_LED_Color(BLUE); break;
          case 5: Serial.println("LEDs Cyan"); RGB_LED_Color(CYAN); break;
          case 6: Serial.println("LEDs Green"); RGB_LED_Color(GREEN); break;
          case 7: Serial.println("LEDs Yellow"); RGB_LED_Color(YELLOW); break;          
          case 8: Serial.println("LEDs Orange"); RGB_LED_Color(ORANGE); break;
          case 9: Serial.println("LEDs Red"); RGB_LED_Color(RED); break;          
          case 10: Serial.println("LEDs White"); RGB_LED_Color(WHITE); break;
          case 11: RGB_LED_Cycle_Colors();
          break;
        }
      }
    }

    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, LOW);         // when the central disconnects, swap the LEDs
    digitalWrite(LED_PWR, HIGH);
  }

  // Cycle RGB LED colors while idle
  RGB_LED_Cycle_Colors();
}

void RGB_LED_Color(int r, int g, int b) {
  analogWrite(LEDR,255-r);
  analogWrite(LEDG,255-g);
  analogWrite(LEDB,255-b);
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
