/*
  Arduino BLE Central LED Control 2

  This example scans for Bluetooth® Low Energy peripherals until one with the advertised service
  "19B10000-E8F2-537E-4F6C-D104768A1214" UUID is found. Once discovered and connected, Central
  sends values to Peripheral to control its RGB LEDs either in a fixed or random sequence.
 
  Tested with Arduino Nano 33 BLE Sense and Seeed Studio XIAO nRF52840 Sense boards, but the Central
  sketch should work with any board that is BLE-compatible.  The companion Peripheral sketch requires
  a BLE-compatible board with RGB LEDs, or could be modified with external LEDs on pins that support
  analogWrite, or as required.
 
  Uncomment the #define nRF52840 line for use with that board, which only affects how the USER LED is
  is accessed since it is not on a normal digital pin.

  These sketches are modified from the original included ArduinoBLE examples LEDControl and LED
  by Samuel M. Goldwasser, no copyright® by me, do with them as you see fit. ;-)
*/

#define serial 1 // Send to serial port if set to 1

// #define nRF52840 // If defined, use nRF52840 CHARGE LED (PIO P0.17) as USER LED.  Comment out otherwise.

#ifndef nRF52840
  #define LED_USER LED_BUILTIN 
#endif

#ifdef nRF52840
  #define LED_USER 17 
#endif

#include <ArduinoBLE.h>

// variables for BLE characteristic
uint8_t RGB_LED_Color = 0; // 8 bits for color value

char buffer[40];

void setup() {

  // Set up LED used for connect status

  #ifndef nRF52840
    pinMode(LED_USER, OUTPUT);
  #endif

  #ifdef nRF52840
    nrf_gpio_cfg_output(LED_USER);
  #endif

  // Startup inactive LED state

  #ifndef nRF52840
    digitalWrite(LED_USER, LOW); 
  #endif

  #ifdef nRF52840
    nrf_gpio_pin_write(LED_USER, HIGH);
  #endif

  if (serial == 1) {
    Serial.begin(9600);
    while (!Serial);
  }

  // initialize the Bluetooth® Low Energy hardware
  BLE.begin();

  if (serial == 1) {
    Serial.println();
    Serial.println("Bluetooth® Low Energy Central - LED control");
  }

  // start scanning for peripherals
  BLE.scanForUuid("19B10000-E8F2-537E-4F6C-D104768A1214");
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
  // discovered a peripheral, print out address, local name, and advertised service
  if (serial == 1) {
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();
  }

    if (peripheral.localName() != "LED") return;

    // stop scanning
    BLE.stopScan();
    controlLed(peripheral);

    // peripheral disconnected, start scanning again
    BLE.scanForUuid("19B10000-E8F2-537E-4F6C-D104768A1214");
  }
}

void controlLed(BLEDevice peripheral) {
  // connect to the peripheral
  if (serial == 1) Serial.println("Connecting ...");

  if (peripheral.connect()) {
    if (serial == 1) Serial.println("Connected");

    #ifndef nRF52840
      digitalWrite(LED_USER, HIGH); 
    #endif

    #ifdef nRF52840
      nrf_gpio_pin_write(LED_USER,LOW);
    #endif
  }
  else {
    if (serial == 1) Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
   if (serial ==1) Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    if (serial == 1) Serial.println("Attributes discovered");
    Serial.println();
    Serial.println("Running");
    Serial.println();
  }
    else {
    if (serial == 1) Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the LED characteristic
  BLECharacteristic ledCharacteristic = peripheral.characteristic("19b10001-e8f2-537e-4f6c-d104768a1214");

  if (!ledCharacteristic) {
    if (serial == 1) Serial.println("Peripheral does not have LED characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (!ledCharacteristic.canWrite()) {
    if (serial == 1) Serial.println("Peripheral does not have a writable LED characteristic!");
    peripheral.disconnect();
    return;
  }

  while (peripheral.connected()) {
    // While the peripheral is connected, send color values.
    ledCharacteristic.writeValue(RGB_LED_Color);
    if (serial == 1) {
      Serial.print("LED Color: ");
      sprintf(buffer, "%d", RGB_LED_Color);
      Serial.println(buffer);
    }
    RGB_LED_Color = rand() & 7;                    // Use random colors
//    RGB_LED_Color ++;                              // Use cyclic colors
    if (RGB_LED_Color > 7) RGB_LED_Color = 0;
    if ((RGB_LED_Color & 1) == 1) {                // On when color value is odd number. ;-)

    #ifndef nRF52840
      digitalWrite(LED_USER, HIGH);
      }  
      else digitalWrite(LED_USER, LOW);
    #endif

    #ifdef nRF52840
      nrf_gpio_pin_write(LED_USER, LOW);
      }  
      else nrf_gpio_pin_write(LED_USER, HIGH);
    #endif

    delay(150);
  }

  if (serial == 1) Serial.println("Peripheral disconnected");

  #ifndef nRF52840
    digitalWrite(LED_USER, LOW); 
  #endif

  #ifdef nRF52840
    nrf_gpio_pin_write(LED_USER, HIGH);
  #endif
}
