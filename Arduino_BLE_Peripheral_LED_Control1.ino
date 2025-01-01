/*
  Arduino BLE Peripheral LED Control 1

  This example scans for Bluetooth® Low Energy peripherals until one with the advertised service
  "19B10000-E8F2-537E-4F6C-D104768A1214" UUID is found. Once discovered and connected, it turns on
  its BUILTIN LED and sends values to the peripheral to control its RGB LEDs either in a fixed
  or random sequence.
 
  Tested with Arduino Nano 33 BLE Sense boards but the Central sketch should work with any board 
  that is BLE-compatible.  The companion Peripheral sketch requires a BLE-compatible board with
  the RGB LEDs or could be modified as required.  Minor changes may be needed for boards like
  the Seeed Studio XIAO BLE nRF52840 for the USER LED since it is not on a normal digital pin.

  These sketches are modified from the original included ArduinoBLE examples LEDControl and LED
  by Samuel M. Goldwasser, no copyright® by me, do with them as you see fit. ;-)
*/

#define serial 0 // Send data to serial port if 1

#define LED_USER LED_BUILTIN // This should work for most boards.  But for the Seeed Studio XIAO
                             //  BLE nRF52840 boards, change the pin definition to: #define LED_USER 17;
                             //  Change the pin mode to: nrf_gpio_cfg_output(LED_USER);
                             //  To turn on, replace digitalWrite with: nrf_gpio_pin_write(LED_USER,LOW);
                             //  To turn off, replace digitalWrite with: nrf_gpio_pin_write(LED_USER,HIGH);

// Color palette for RGB_LEDs
#define BLACK 0,0,0
#define RED 255,0,0
#define GREEN 0,255,0
#define BLUE 0,0,255
#define CYAN 0,127,127
#define MAGENTA 127,0,127
#define YELLOW 127,127,0
#define WHITE 255,255,255

#include <ArduinoBLE.h>

BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Bluetooth® Low Energy LED Service

// Bluetooth® Low Energy LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

void setup() {

  // Set up LEDs
  pinMode(LED_USER, OUTPUT); // BUILTIN LED used for connect status
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  digitalWrite(LED_USER, 0); // Startup indactive LED state
  RGB_LED_Color(BLACK);

  if (serial == 1) {
    Serial.begin(9600);
    while (!Serial);
  }

  // begin initialization
  if (!BLE.begin()) {
    if (serial == 1) Serial.println("Starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("LED");
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();

  if (serial == 1) Serial.println("BLE LED Peripheral");
}

void loop() {
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    if (serial == 1) {
      Serial.print("Connected to central: ");    
      Serial.println(central.address()); // Send the central's MAC address:
    }
    digitalWrite(LED_USER, 1); // Connection active
    RGB_LED_Color(BLACK);         // Start with BLACK

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic, use the value for the color of the RGB LEDs:
      if (switchCharacteristic.written()) {
        switch (switchCharacteristic.value()) {   // any value other than 0
          case 0: if (serial == 1) Serial.println("LEDs OFF"); RGB_LED_Color(BLACK); break;
          case 1: if (serial == 1) Serial.println("LEDs Magenta"); RGB_LED_Color(MAGENTA); break;
          case 2: if (serial == 1) Serial.println("LEDs Blue"); RGB_LED_Color(BLUE); break;
          case 3: if (serial == 1) Serial.println("LEDs Cyan"); RGB_LED_Color(CYAN); break;
          case 4: if (serial == 1) Serial.println("LEDs Green"); RGB_LED_Color(GREEN); break;
          case 5: if (serial == 1) Serial.println("LEDs Yellow"); RGB_LED_Color(YELLOW); break;          
          case 6: if (serial == 1) Serial.println("LEDs Red"); RGB_LED_Color(RED); break;          
          case 7: if (serial == 1) Serial.println("LEDs White"); RGB_LED_Color(WHITE); break;
          break;
        }
      }
    }       

    // when the central disconnects, print it out:
      if (serial == 1) {
        Serial.print(F("Disconnected from central: "));
        Serial.println(central.address());
      }
      digitalWrite(LED_USER, 0); // Connection innactive
      RGB_LED_Color(BLACK);         // End with BLACK
  }
}

void RGB_LED_Color(int r, int g, int b) {
  analogWrite(LEDR,255-r);
  analogWrite(LEDG,255-g);
  analogWrite(LEDB,255-b);
}
