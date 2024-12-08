#include <ArduinoBLE.h>

BLEService environmentalService("181A");  // Bluetooth® Low Energy Environment Service

 BLEIntCharacteristic tempCharacteristic("2A6E", BLERead | BLENotify); // Bluetooth® Low Energy Temperature Level Characteristic

long previousMillis = 0;  // last time the battery level was checked, in ms

void setup() {
  Serial.begin(9600);    // initialize serial communication
  while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("TempMonitor");
  BLE.setAdvertisedService(environmentalService);
  // Add the characteristic to the service
  environmentalService.addCharacteristic(tempCharacteristic);
  // Add service BLE.addService(environmentalService);
  // Set initial value for temperature
  int initialTemp = 20 * 100;
  // 20°C, multiplied by 100 as per spec
  //According to the Bluetooth specifications, the Temperature Characteristic (0x2A6E) uses a 16-bit signed integer format representing temperature in units of 0.01 degrees Celsius. So:
  //A value of 2250 would represent 22.50°C; A value of -500 would represent -5.00°C; A value of 100 would represent 1.00°C
  tempCharacteristic.writeValue(initialTemp);

  // start advertising
  BLE.advertise();
  Serial.println("Bluetooth® device active, waiting for connections...");
}

int TempLevel, oldTempLevel, newValue, i;

void loop() {
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // while the central is connected,
    while (central.connected()) {
      long currentMillis = millis();
      // if 200ms have passed, check the Temperature level:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        updateTempLevel();
        TempLevel++;     // Fake TempLevel as counter.
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void updateTempLevel() {
  if (TempLevel != oldTempLevel) {        // if the Temperature level has changed
    Serial.print("Temperature is now: "); // print it
    Serial.println(TempLevel);  
  tempCharacteristic.writeValue(TempLevel);
  oldTempLevel = TempLevel;               // save the level for next comparison
  }
}
