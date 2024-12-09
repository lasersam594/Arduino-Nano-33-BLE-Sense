// Nano BLE 33 Sense Gyro Data Send to BLE intended for iOS (iPhone or iPad) but probably works for other compatible BLE devices.
// This sketch sends the Pitch, Yaw, and Row values from the on-board gyroscope to BLE along with a sequence number.  The Gyro values
// are the integer part of the floating point Gyro data; the SN is itself. ;-)  Currently, the only way these are readible are by using
// a BLE utility like LightBlue. To be displayed correctly in LightBlue, the format options must be set to something other than "None"
// and to "Signed Integer".  Four values are then displayed in sequence: Roll, Pitch, Yaw, and SN.
//
// Copyright (c) Sam Goldwasser and Jan Beck, 1994-2024
//

#define Rev1                     // Set to appropriate board Rev

#define data1 1                  // Send data
#define verbose1 1               // Include labels

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

  BLE.setLocalName("Gyro Monitor");
  BLE.setAdvertisedService(environmentalService);
  environmentalService.addCharacteristic(tempCharacteristic); // Add the characteristic to the service
  BLE.addService(environmentalService);

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
    Serial.println("");
    // turn on the LED_BUILTIN and turn off LED_PWR to indicate the connection
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_PWR, LOW);

    // while central is connected, Update every 100 ms
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

          ledr = abs(gr - GR_COR)/2; // Format data for RGB LEDs
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
    Serial.println("");
  }

 // Cycle RGB LED colors while idle - it's got to have something to do. ;-)
  RGB_LED_Cycle_Colors();
}

void updateValues() { // Send BLE values
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
