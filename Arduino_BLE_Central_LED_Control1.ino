/*
  Arduino BLE Central LED Control 1

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

#define serial 1 // Send comments to serial port only if set to 1

#define LED_USER LED_BUILTIN // Set based on specific board type

#include <ArduinoBLE.h>

// variables for BLE characteristic
uint8_t RGB_LED_Color = 0; // 8 bits for color value

char buffer[40];

void setup() {

  pinMode(LED_USER, OUTPUT); //  LED used for connect status

  digitalWrite(LED_USER, 0); // Startup indactive LED state

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
    digitalWrite(LED_USER, 1); // Set LED to active state
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
      RGB_LED_Color = rand() & 7; // Use random colors
//    RGB_LED_Color++;              // Use cyclic colors
    if (RGB_LED_Color > 7) RGB_LED_Color = 0;
    delay(200);
  }

  if (serial == 1) Serial.println("Peripheral disconnected");
    digitalWrite(LED_USER, 0); // Set LED to inactive state
}
