/*
  Bluetooth Send Gyro Data 9

  Copyright® Samuel M. Goldwasser, 1994-2025, all rights reserved.  Permission is granted for public
  use or modification as long as the Copyright notice is included.

  This sketch scans for Bluetooth® Low Energy peripherals with "inertial plus service".  It then
  sends values for roll, pitch, and yaw of the gyroscope, and a sequence number to Peripheral via
  Bluetooth.  These are also sent to the serial ports on both boards if available.  The signed
  amplitude of the Gyro measurements are also displayed as intensity of the RGB LEDs on both boards
  coded as roll (+Red/-Cyan), Pitch (+Green/-Magenta), and Yaw (+Blue/-Yellow).  The USER LED
  will blink at 1/8 the sample rate if there is no gyro activity.  The PWR LED, if present and 
  controllable, will go off.  No, the thing hasn't crashed. ;-)

  It also sends a 12 bit random number called "Level" and if there is no gyro activity, that is
  displayed in the RGB LEDs on both boards as red, green, blue, and intensity in the format IBGR,
  each 3 bits, which will appear, uh, quite random.  Level was supposed to be peak sound intensity
  but starting the PDM microphone does cause the entire thing to crash.  Go figure. ;-)  Level could
  be assigned to something useful in the future.  Or the issue could be resolved.

  Tested with Arduino Nano 33 BLE Sense and Seeed Studio XIAO nRF52840 Sense boards (though not all
  combinations).  The Central sketch should work with any board that is BLE-compatible with minor
  changes depending on the specific IMU.  The companion Peripheral sketch requires a BLE-compatible
  board with RGB LEDs, or could be modified with external LEDs on pins that support analogWrite, or
  as required.  Please contact me with questions.
 
  Uncomment and select the #define Rev for the Nano 33 BLE Sense; Uncomment the #define nRF52840 line
  for use with that board.  This affects both selection of the appropriate IMU and, how the USER LED
  is accessed since it is not on a normal digital pin on the nRF52840.
*/

// user settings
#define Rev2          // Set to appropriate board Rev for Nano BLE; comment out if using a nRF52840 board.
// #define nRF52840      // Comment out if using a Nano or other board

#define data1 1        // Send data to serial port
#define verbose1 1     // Include labels
#define GyroAutoCal 1  // Perform automatic Gyro offset compensation at startup: Board must be stationary when RGB LED is blinking.

// Color palette for RGB_LEDs
#define BLACK 0, 0, 0
#define GRAY 7, 7, 7
#define MAGENTA 255, 0, 255
#define BLUE 0, 0, 255
#define CYAN 0, 255, 125
#define GREEN 0, 255, 0
#define YELLOW 255, 255, 0
#define ORANGE 255, 50, 0
#define RED 255, 0, 0
#define WHITE 255, 255, 255

#define scale 0.5

#define timeoutvalue 10


// gyro offset parameters and variables

#define CalValues 50      // Number of Gyro samples to average for calibration

float RollOffsetSum = 0;  // Temporary variables for Gyro AutoCal averaging
float PitchOffsetSum = 0;
float YawOffsetSum = 0;

float GR_COR = 0;         // Gyro offset correction values
float GP_COR = 0;
float GY_COR = 0;

int CalCount = CalValues; // Variables for GyroAutoCal
int GyroAutoCalFlag = 0;
float pgr, pgp, pgy;

// other variables
char buffer[40];
float gr, gp, gy, grcor, gpcor, gycor, bright;
int32_t grint, gpint, gyint;
int32_t SN = 0;
int SNPrint;
int led, ledr, ledg, ledb;
int sendData = data1;
uint16_t level = 0;
long longlevel;
int holdoff = timeoutvalue;

// board-specific libraries and settings

#ifdef nRF52840
#define LED_USER 17
#include <LSM6DS3.h>           // IMU
LSM6DS3 myIMU(I2C_MODE, 0x6A); //I2C device address 0x6A
#endif

#ifdef Rev1
#define LED_USER LED_BUILTIN
#include <Arduino_LSM9DS1.h>   // Accelerometer, magnetometer and gyroscope
#endif

#ifdef Rev2
#define LED_USER LED_BUILTIN
#include <Arduino_BMI270_BMM150.h>  // Accelerometer, magnetometer and gyroscope
#endif

#include <ArduinoBLE.h>
#include <stdlib.h>
#include <time.h>

// Bluetooth® Low Energy inertial plus service (Custom UUIDs)

#define BLE_UUID_INERTIAL_PLUS_SERVICE          "DA3F7226-D807-40E6-A24C-E9F16EDFCD3B"

#define BLE_UUID_GYRO_ROLL                      "DA3F7227-D807-40E6-A24C-E9F16EDFCD30"
#define BLE_UUID_GYRO_PITCH                     "DA3F7227-D807-40E6-A24C-E9F16EDFCD31"
#define BLE_UUID_GYRO_YAW                       "DA3F7227-D807-40E6-A24C-E9F16EDFCD32"
#define BLE_UUID_INPUT_LEVEL                    "DA3F7227-D807-40E6-A24C-E9F16EDFCD33"
#define BLE_UUID_SEQUENCE_NUMBER                "DA3F7227-D807-40E6-A24C-E9F16EDFCD34"

BLEService inertial_plus(BLE_UUID_INERTIAL_PLUS_SERVICE);

void setup() {

  srandom(time(NULL)); // Seed the random number generator

// Board-specific corrections for possible Gyro offsets

#ifdef Rev1
  GR_COR = 6.5;
  GP_COR = 0;
  GY_COR = 2.5;
#endif

#ifdef Rev2
  GR_COR = 0;
  GP_COR = 0;
  GY_COR = 0;
#endif

  // Set the LEDs pins as outputs and turn on LED_USER and set the RGB LEDs at low brightness

#if defined (Rev1) || defined (Rev2)
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR, HIGH);
#endif

#ifndef nRF52840
  pinMode(LED_USER, OUTPUT);
  digitalWrite(LED_USER, LOW);
#endif

#ifdef nRF52840
  nrf_gpio_cfg_output(LED_USER);
  nrf_gpio_pin_write(LED_USER, HIGH);
#endif

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  RGB_LED_Color(GRAY, 1.0);

  // enable serial port if data1 is 1 and port available.
  if (data1 == 1) {
    Serial.begin(9600);
    if (Serial == 0) sendData = 1;
  }

  // initialize the Bluetooth® Low Energy hardware
  if (!BLE.begin()) {
    if (sendData == 1) Serial.println("Starting Bluetooth® Low Energy module failed!");
    while (1);
  }
 
  // set advertised local name and service UUID:
  BLE.setLocalName("Gyro Monitor Sender");
  BLE.setAdvertisedService(inertial_plus);

  // start advertising
  BLE.advertise();
  if (sendData == 1) Serial.println("Bluetooth® device active, waiting for connections...");

  if (sendData == 1) {
    Serial.println();
    Serial.println("Bluetooth® Low Energy Central - Gyro Data Send");
  }

  // Configure the IMU

#ifndef nRF52840
  if (!IMU.begin()) {
    if (sendData == 1) Serial.println("Failed to initialize IMU!");
    while (1);
  }
#endif

#ifdef nRF52840
  if (myIMU.begin() != 0) {
    Serial.println("Failed to initialize IMU!");
  }
#endif

  // Gyro AutoCal
  if (GyroAutoCal == 0) delay(1000);
  if (GyroAutoCal == 1) Do_GyroAutoCal(25);  // Argument is the delay in ms inside GyroAutoCal loop
}

void loop() {
  // start scanning for peripherals
  BLE.scanForUuid(BLE_UUID_INERTIAL_PLUS_SERVICE);

  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  while (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    if (sendData == 1) {
      Serial.print("Found ");
      Serial.print(peripheral.address());
      Serial.print(" '");
      Serial.print(peripheral.localName());
      Serial.print("' ");
      Serial.print(peripheral.advertisedServiceUuid());
      Serial.println();
    }

    if (peripheral.localName() != "Gyro Monitor Receiver") return;

    // stop scanning, OK to proceed
    BLE.stopScan();
    SendGyroData(peripheral);
    return;
  }
}

void SendGyroData(BLEDevice peripheral) {
  // connect to the peripheral
  if (sendData == 1) Serial.println("Connecting ...");

  if (peripheral.connect()) {
    if (sendData == 1) Serial.println("Connected");

    // Connection active
#if defined (Rev1) || defined (Rev2)
    digitalWrite(LED_PWR, LOW);
#endif

#ifndef nRF52840  
    digitalWrite(LED_USER, HIGH);
#endif

#ifdef nRF52840
    nrf_gpio_pin_write(LED_USER, LOW);
#endif
  }
    else {
    if (sendData == 1) Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  if (sendData == 1) Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    if (sendData == 1) Serial.println("Attributes discovered");
    Serial.println();
    Serial.println("Running");
    Serial.println();
  }
  else {
    if (sendData == 1) Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the characteristics
  BLECharacteristic Gyro_Roll = peripheral.characteristic(BLE_UUID_GYRO_ROLL);
  BLECharacteristic Gyro_Pitch = peripheral.characteristic(BLE_UUID_GYRO_PITCH);
  BLECharacteristic Gyro_Yaw = peripheral.characteristic(BLE_UUID_GYRO_YAW);
  BLECharacteristic Input_Level = peripheral.characteristic(BLE_UUID_INPUT_LEVEL); 
  BLECharacteristic Sequence_Number = peripheral.characteristic(BLE_UUID_SEQUENCE_NUMBER);

  if (!Gyro_Roll) {
    if (sendData == 1) Serial.println("Peripheral does not have Roll Characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (sendData == 1) Serial.println("Peripheral has Roll Characteristic!");
  if (!Gyro_Roll.canWrite()) {
    if (sendData == 1) Serial.println("Peripheral does not have a writable Roll characteristic!");
    peripheral.disconnect();
    return;
  }

  if (!Gyro_Pitch) {
    if (sendData == 1) Serial.println("Peripheral does not have Pitch Characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (sendData == 1) Serial.println("Peripheral has Pitch Characteristic!");
  if (!Gyro_Pitch.canWrite()) {
    if (sendData == 1) Serial.println("Peripheral does not have a writable Pitch characteristic!");
    peripheral.disconnect();
    return;
  }

  if (!Gyro_Yaw) {
    if (sendData == 1) Serial.println("Peripheral does not have Yaw Characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (sendData == 1) Serial.println("Peripheral has Yaw Characteristic!");
  if (!Gyro_Yaw.canWrite()) {
    if (sendData == 1) Serial.println("Peripheral does not have a writable Yaw characteristic!");
    peripheral.disconnect();
    return;
  }

  if (!Input_Level) {
    if (sendData == 1) Serial.println("Peripheral does not have Input Level Characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (sendData == 1) Serial.println("Peripheral has Input_Level Characteristic!");
  if (!Input_Level.canWrite()) {
    if (sendData == 1) Serial.println("Peripheral does not have a writable Input_Level characteristic!");
    peripheral.disconnect();
    return;
  }

  if (!Sequence_Number) {
    if (sendData == 1) Serial.println("Peripheral does not have Sequence_Number Characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (sendData == 1) Serial.println("Peripheral has Sequence_Number Characteristic!");
  if (!Sequence_Number.canWrite()) {
    if (sendData == 1) Serial.println("Peripheral does not have a writable SN characteristic!");
    peripheral.disconnect();
    return;
  }

  if (sendData == 1) {
    Serial.print("Connected to Gyro Monitor Receiver: ");
    Serial.println(peripheral.address());  // Send the peripheral's MAC address
    if (sendData == 1) Serial.println();
  }

  while (peripheral.connected()) {

    // While the peripheral is connected, send Gyro values

#ifndef nRF52840
    while (!IMU.gyroscopeAvailable());
    IMU.readGyroscope(gr, gp, gy);
#endif

#ifdef nRF52840 
    gr = myIMU.readFloatGyroX();
    gp = myIMU.readFloatGyroY();
    gy = myIMU.readFloatGyroZ();
#endif

    grcor = (gr - GR_COR);
    gpcor = (gp - GP_COR);
    gycor = (gy - GY_COR);

    // level = rand() % 4096; // Fake sound level ;-)

    longlevel = random();
    level = longlevel % 4096;

    ledr =   (level & 0b000000000111) << 5;
    ledg =   (level & 0b000000111000) << 2;
    ledb =   (level & 0b000111000000) >> 1;
    bright = (((level & 0b111000000000) >> 9) + 1);
    bright /= 8;

    SNPrint = SN;

    if (sendData == 1) {
      if (verbose1 == 1) Serial.print("Gyro (Degs/s) Roll: ");
      sprintf(buffer, "%8.2f", grcor);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print("  Pitch: ");
      sprintf(buffer, "%8.2f", gpcor);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print("  Yaw: ");
      sprintf(buffer, "%8.2f", gycor);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print(" | Level: ");
      sprintf(buffer, "%4d", level);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print(" R: ");
      sprintf(buffer, "%1d", ledr >> 5);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print(" G: ");
      sprintf(buffer, "%1d", ledg >> 5);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print(" B: ");
      sprintf(buffer, "%1d", ledb >> 5);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print(" I: ");
      sprintf(buffer, "%4.2f", bright);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print(" | SN: ");
      sprintf(buffer, "%6d", SNPrint);
      Serial.println(buffer);
    }

    if ((fabs(grcor) > 1) || (fabs(gpcor) > 1) || (fabs(gycor) > 1)) holdoff = timeoutvalue;  // Display input level in RGB LEDs only if no Gyro activity
    if (holdoff > 0) {
      RGB_Gyro_Colors(grcor, gpcor, gycor, scale); 
      holdoff --;
      if (holdoff <= 0) holdoff = 0;
    }
    else {
      RGB_LED_Color (ledr, ledg, ledb, bright);
      if ((SN & 7) == 7) {

#ifndef nRF52840        
              digitalWrite(LED_USER, HIGH);
              delay(10);
              digitalWrite(LED_USER, LOW);
#endif

#ifdef nRF52840    
              digitalWrite(LED_USER, LOW);
              delay(10);
              digitalWrite(LED_USER, HIGH);
#endif
      }
    }
    enKludge(); // Kludge to get fractional resolution without using floating point. ;-)

    Gyro_Roll.writeValue(grint);
    Gyro_Pitch.writeValue(gpint);
    Gyro_Yaw.writeValue(gyint);
    Input_Level.writeValue(level);
    Sequence_Number.writeValue(SN);

    SN++;

  delay(1);
  }

  if (sendData == 1) Serial.println("Peripheral disconnected");

  // Connection innactive

#if defined (Rev1) || defined (Rev2)
  digitalWrite(LED_PWR, HIGH);
#endif

#ifndef nRF52840
  digitalWrite(LED_USER, LOW);
#endif

#ifdef nRF52840
  nrf_gpio_pin_write(LED_USER, HIGH);
#endif

  RGB_LED_Color(BLACK, 0);
}

void Do_GyroAutoCal(int Delay) {
  RGB_LED_Color(BLACK, 0);
  while (CalCount > 0) {
    pgr = gr;
    pgp = gp;
    pgy = gy;

#ifndef nRF52840
    while (!IMU.gyroscopeAvailable());
    IMU.readGyroscope(gr, gp, gy);
#endif

#ifdef nRF52840
    gr = myIMU.readFloatGyroX();
    gp = myIMU.readFloatGyroY();
    gy = myIMU.readFloatGyroZ();
#endif

    if (CalCount == CalValues) {
      CalCount--;  // Skip corrupted first value
    }
    else if (CalCount > 1) {
      delay(Delay);
      if (((fabs(gr - pgr) > 8)) || ((fabs(gp - pgp) > 8)) || ((fabs(gr - pgr) > 8))) {  // Start over if too much gyro activity
        CalCount = CalValues;
        RollOffsetSum = 0;
        PitchOffsetSum = 0;
        YawOffsetSum = 0;
      }
      else {
        RollOffsetSum += gr;     // Update sums
        PitchOffsetSum += gp;
        YawOffsetSum += gy;
        if ((CalCount & 3) == 2) RGB_LED_Color(GRAY, 1.0);  // Activity indicator while AutoCal in progress
        else RGB_LED_Color(BLACK, 0);
        CalCount--;
      }
    } else if (CalCount == 1) {  // Compute average offsets
      GR_COR = RollOffsetSum / CalValues;
      GP_COR = PitchOffsetSum / CalValues;
      GY_COR = YawOffsetSum / CalValues;
      CalCount = 0;
      RGB_LED_Color(BLACK, 0);
    }
  }
}

void RGB_LED_Color(int r, int g, int b, float intensity) {
  analogWrite(LEDR, (255 - (r * intensity)));
  analogWrite(LEDG, (255 - (g * intensity)));
  analogWrite(LEDB, (255 - (b * intensity)));
}

void RGB_Axis_Colors(int Pos_R, int Pos_G, int Pos_B, int Neg_R, int Neg_G, int Neg_B, float axis) {
  if (axis > 0) {
    ledr += ((axis * Pos_R) / 255);
    ledg += ((axis * Pos_G) / 255);
    ledb += ((axis * Pos_B) / 255);
  }
  else {
    ledr -= ((axis * Neg_R) / 255);
    ledg -= ((axis * Neg_G) / 255);
    ledb -= ((axis * Neg_B) / 255);
  }
}

void RGB_Gyro_Colors(int roll, int pitch, int yaw, float atten) {
  ledr = 0;
  ledg = 0;
  ledb = 0;
  if ((fabs(roll) > 1) || (fabs(pitch) > 1) || (fabs(yaw) > 1)) {  // Update if above threahold
    RGB_Axis_Colors(RED, CYAN, roll);
    RGB_Axis_Colors(GREEN, MAGENTA, pitch);
    RGB_Axis_Colors(BLUE, YELLOW, yaw);
    RGB_LED_Color(ledr, ledg, ledb, atten);
  } 
  else RGB_LED_Color(BLACK, 0);
}

void enKludge() {
  grint = grcor * 1000; 
  gpint = gpcor * 1000;
  gyint = gycor * 1000;
}

void Display_Color_Level (uint16_t sum) {
/*
  if (samplesRead) {  // wait for samples to be read 
    for (i = 0; i < samplesRead; i++)
      if (fabs(sampleBuffer[i]) > sum) sum = fabs(sampleBuffer[i]);  // Peak detect

    // Display the peak sound value in RGB_LED
    if (((fabs(grcor) < 1) && (fabs(gpcor) < 1) && (fabs(gycor)) < 1)) {  // Only if no Gyro activity and GyroAutoCAl not in progress
*/

   
    if (sum >= 900) RGB_LED_Color(WHITE, 1);
      else if (sum >= 800) RGB_LED_Color(RED, 1);
      else if (sum >= 700) RGB_LED_Color(ORANGE, 1);
      else if (sum >= 600) RGB_LED_Color(YELLOW, 1);
      else if (sum >= 500) RGB_LED_Color(GREEN, 1);
      else if (sum >= 400) RGB_LED_Color(CYAN, 1);
      else if (sum >= 300) RGB_LED_Color(BLUE, 1);
      else if (sum >= 200) RGB_LED_Color(MAGENTA, 1);
      else if (sum >= 100) RGB_LED_Color(GRAY, 1);
      else RGB_LED_Color(BLACK, 0);

/*
    if (sum >= 1000) RGB_LED_Color(WHITE, 1.0);
      else if (sum >= 600) RGB_LED_Color(RED, 1.0);
      else if (sum >= 400) RGB_LED_Color(ORANGE, 0.8);
      else if (sum >= 325) RGB_LED_Color(YELLOW, 0.5);
      else if (sum >= 250) RGB_LED_Color(GREEN, 0.75);
      else if (sum >= 175) RGB_LED_Color(CYAN, 0.2);
      else if (sum >= 100) RGB_LED_Color(BLUE, 0.3);
      else if (sum >= 50) RGB_LED_Color(MAGENTA, 0.2);
      else if (sum >= 25) RGB_LED_Color(GRAY, 1.0);
      else if (sum >= 0) RGB_LED_Color(BLACK, 0.0);
 //   if (sum >= 25) timeout = timeoutvalue * 2;
*/
}