/*
  Nano 33 BLE Sense Bluetooth Send Test 1.

  Copyright® Samuel M. Goldwasser, 1994-2025, all rights reserved.  Permission is granted for public use or modification as
  long as the Copyright notice is included.

  This sketch sends Gyro data via Bluetooth intended for iOS (iPhone or iPad) but probably works for other BLE devices and is
  confirmed to be at least recognized by Windows.

  The Roll, Pitch, and Yaw, values from the on-board gyroscope are sent via BLE along with a sequence number.  The Gyro values
  are the integer part of the floating point Gyro data; the SN is itself. ;-)  Currently, the only way these are readible are by using
  a BLE utility like LightBlue.  To be displayed correctly in LightBlue, the format options must be set to something other than "None"
  and to "Signed Integer".  Four values are then displayed in sequence: Roll, Pitch, Yaw, and SN.

  When not connected, the BUILTIN LED is off, the PWR LED is on, and the RGB LEDs cycle through a rainbow of colors at a rate of around
  once a second just for something to do. ;-)  When it connects, the BUILTIN LED goes off, the PWR LED goes on, and the RGB LEDs go dark
  unless there is Gyro activity.  Then their brightness is proportional to the absolute amplitude of Roll, Pitch, and Yaw, respectively.
*/

#define Rev2               // Set to appropriate board Rev

#define data1 0            // Send data
#define verbose1 0         // Include labels
#define GyroAutoCal 1      // Perform automatic Gyro offset compensation at startup: Board must be stationary when RGB LED is blinking.

// Gyro offset parameters and variables
#define CalValues 50       // Number of Gyro samples to average for calibration

float RollOffsetSum = 0;   // Temporary variables for Gyro AutoCal averaging
float PitchOffsetSum = 0;
float YawOffsetSum = 0;

float GR_COR = 0;          // Gyro offset correction values
float GP_COR = 0;
float GY_COR = 0;

int CalCount = CalValues;
int GyroAutoCalFlag = 0;
float pgr, pgp, pgy;

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
#define MAGENTA 25,0,25
#define BLUE 0,0,75
#define CYAN 0,50,50
#define GREEN 0,192,0
#define YELLOW 128,92,0
#define ORANGE 200,40,0
#define RED 255,0,0
#define WHITE 255,255,255

void setup() {

// Board-specific corrections for possible Gyro offsets

  #ifdef Rev2
    GR_COR = 6.5;
    GP_COR = 0;
    GY_COR = 2.5;
  #endif

  #ifdef Rev2
    GR_COR = 0;
    GP_COR = 0;
    GY_COR = 0;
  #endif

    if (data1 == 1) {
      Serial.begin(9600);    // initialize serial communication
      while (!Serial);
    }

  // Set the LEDs pins as outputs
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  // Turn the LED_BUILTIN and LED_PWR on and set the RGB LEDs at low brightness
  digitalWrite(LED_BUILTIN, 0);
  digitalWrite(LED_PWR, 1);
  analogWrite(LEDR, 200);
  analogWrite(LEDG, 200);
  analogWrite(LEDB, 200);

  // begin initialization
  if (!BLE.begin()) {
    if (data1 == 1) Serial.println("starting BLE failed!");
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
  if (data1 == 1) Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {
  
  // Gyro AutoCal
  while ((GyroAutoCal == 1) && (CalCount != 0)) {
    delay(25); // Provide time to see GyroAutoCal in progress. ;-)
    while (!IMU.gyroscopeAvailable());
    pgr = gr; pgp = gp; pgy = gy;
    IMU.readGyroscope(gr, gp, gy);

    if (CalCount == CalValues) {
      CalCount--;          // Skip corrupted first value
      GyroAutoCalFlag = 1; // Disable RGB_LED output while GyroAutoCal in progress
    }
    else if (CalCount > 1) { // Test for too much motion
      if (((fabs(gr - pgr) > 2)) || ((fabs(gp - pgp) > 2)) || ((fabs(gy - pgy) > 2))) { // Start over if too much gyro activity
        RollOffsetSum = 0;
        PitchOffsetSum = 0;
        YawOffsetSum = 0;
        CalCount = CalValues;
      } 
      else { // Update sums
        RollOffsetSum += gr;
        PitchOffsetSum += gp;
        YawOffsetSum += gy;
        if ((CalCount & 3) == 2) RGB_LED_Color(GRAY); // Heartbeat while AutoCal in progress
        else RGB_LED_Color(BLACK);
        CalCount--;
      }
    }
    else if (CalCount == 1) { // Compute average offsets
      GR_COR = RollOffsetSum / CalValues;
      GP_COR = PitchOffsetSum / CalValues;
      GY_COR = YawOffsetSum / CalValues;
      CalCount = 0;
      GyroAutoCalFlag = 0;
      RGB_LED_Color(BLACK);
    }
  }

  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    if (data1 == 1) {
      Serial.print("Connected to central: ");
      Serial.println(central.address()); // print the central's BT address:
      Serial.println("");
    }

    // turn on the LED_BUILTIN and turn off LED_PWR to indicate the connection
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_PWR, LOW);

    // while central is connected, Update every 100 ms
    while (central.connected()) {
      long currentMillis = millis();
      // if 100 ms have passed, check:
      if (currentMillis - previousMillis >= 100) {
        previousMillis = currentMillis;
        updateValues();
       
        // Gyroscope

        while (!IMU.gyroscopeAvailable());
        pgr = gr; pgp = gp; pgy = gy;
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
          if (verbose1 == 1) Serial.print("SN: "); // print Serial Number
          sprintf(buffer, "%4d", SN );
          Serial.println(buffer);
        }

      ledr = abs(gr - GR_COR)/2; // Format data for RGB LEDs
      ledp = abs(gp - GP_COR)/2;
      ledy = abs(gy - GY_COR)/2;
      if (GyroAutoCalFlag == 0) RGB_LED_Color(ledr, ledp, ledy); // Only update Gyro activity in RGB LEDs if Gyro AutoCal is not active
      }
    }
  
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_PWR, HIGH);
    if (data1 == 1) {
      Serial.println();
      Serial.print("Disconnected from central: ");
      Serial.println(central.address());
      Serial.println("");
    }
  }
  // Cycle RGB LED colors while idle - it's got to have something to do. ;-)
  if (GyroAutoCalFlag == 0) RGB_LED_Cycle_Colors(); // But only if Gyro Autocal is not in progress.
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
