// Nano BLE 33 Sense Gyro Data Send to BLE intended for iOS (iPhone or iPad) but probably works for other compatible BLE devices.
// This sketch sends the Pitch, Yaw, and Row values from the on-board gyroscope to BLE along with a sequence number.  Currently, the
// only way these are readible are using a BLE utility like LightBlue. To be displayed correctly in LightBlue, the format options
// must be set to something other than "None" and to "Signed Integer".  Four values are then displayed in sequence: Roll, Pitch, Yaw,
// and SN.
//
// Copyright (c) Sam Goldwasser and Jan Beck, 1994-2024
//

#define Rev1

#define data1 1                  // Send data
#define verbose1 1

// Board-specific corrections for possible Gyro offsets
#ifdef Rev1
#define GR_COR   6.5
#define GP_COR   0
#define GY_COR   2.5
#endif

#ifdef Rev2
#define GR_COR   0
#define GP_COR   0
#define GY_COR   0
#endif

#ifdef Rev1
#include <Arduino_LSM9DS1.h>       // Accelerometer, magnetometer and gyroscope
#endif

#ifdef Rev2
#include <Arduino_BMI270_BMM150.h> // Accelerometer, magnetometer and gyroscope
#endif

#include <ArduinoBLE.h>

char buffer[40];
float gr, gp, gy;
int led, ledr, ledp, ledy;
int SN = 0;
int previousMillis, currentMillis;

BLEService environmentalService("181A");  // Bluetooth® Low Energy Environment Service
BLEIntCharacteristic tempCharacteristic("2A6E", BLERead | BLENotify); // Bluetooth® Low Energy Temperature Level Characteristic

void setup() {
  Serial.begin(9600);    // initialize serial communication
  while (!Serial);

 // Set the LEDs pins as outputs
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  if (!IMU.begin()) {
    if (data1 == 1) Serial.println("Failed to initialize IMU!");
    while (1);
  }  

  // Turn the LED_BUILTIN and LED_PWR on and set the RGB LEDs at low brightness
  // analogWrite(LED_BUILTIN, 255);
  // digitalWrite(LED_PWR, 1);
  analogWrite(LEDR, 255);
  analogWrite(LEDG, 255);
  analogWrite(LEDB, 255);

  BLE.setLocalName("Gyro Monitor");
  BLE.setAdvertisedService(environmentalService);
  environmentalService.addCharacteristic(tempCharacteristic); // Add the characteristic to the service
  BLE.addService(environmentalService);
// According to the Bluetooth specifications, the Temperature Characteristic (0x2A6E) uses a 16-bit signed integer format representing temperature
// in units of 0.01 degrees Celsius. So: A value of 2250 would represent 22.50°C; A value of -500 would represent -5.00°C; A value of 100 would represent 1.00°C
  tempCharacteristic.writeValue(SN);

  // start advertising
  BLE.advertise();
  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {
  
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED_BUILTIN and turn of LED_PWR to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_PWR, LOW);

    // while the central is connected,
    while (central.connected()) {
      long currentMillis = millis();
      // if 200ms have passed, check:
      if (currentMillis - previousMillis >= 100) {
        previousMillis = currentMillis;
        updateValues();
       
 // Gyroscope

        while (!IMU.gyroscopeAvailable()) {}
          IMU.readGyroscope(gr, gp, gy);
          if (data1 == 1) {
          if (verbose1 == 1) Serial.print("Gyro (Degs/s) R: ");
          sprintf(buffer, "%8.2f", gr - GR_COR );
          Serial.print(buffer);
          if (verbose1 == 1) Serial.print( "  P: ");
          sprintf(buffer, "%8.2f", gp - GP_COR );
          Serial.print(buffer);
          if (verbose1 == 1) Serial.print( "  Y: ");
          sprintf(buffer, "%8.2f"  , gy - GY_COR);
          Serial.print(buffer);
          if (verbose1 == 1) Serial.print("  |  "); 
          Serial.print("SN: "); // print Serial Number
          sprintf(buffer, "%4d", SN );
          Serial.println(buffer);
        }

          ledr = abs(gr - GR_COR)/2;
          ledp = abs(gp - GP_COR)/2;
          ledy = abs(gy - GY_COR)/2;
         RGB_LED_Color(ledr, ledp, ledy);
      }
    }

  // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_PWR, HIGH);
    Serial.println();
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void updateValues() {
    tempCharacteristic.writeValue(SN);
    tempCharacteristic.writeValue(gr - GR_COR);
    tempCharacteristic.writeValue(gp - GP_COR);
    tempCharacteristic.writeValue(gy - GY_COR);
    SN++;
}

void RGB_LED_Color(int r, int g, int b) {
  analogWrite(LEDR,255-r);
  analogWrite(LEDG,255-g);
  analogWrite(LEDB,255-b);
}
