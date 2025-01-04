/*
  Bluetooth Send Gyro Data 5

  Copyright® Samuel M. Goldwasser, 1994-2025, all rights reserved.  Permission is granted for public
  use or modification as long as the Copyright notice is included.

  This example scans for Bluetooth® Low Energy peripherals until one with the advertised service
  using generated "DA3F7226-D807-40E6-A24C-E9F16EDFCD3B" UUID is found. Once discovered and connected,
  Central sends values for roll, pitch, and yaw of the gyroscope, and a sequence number.  The signed
  amplitude of the Gyro measurements are also displayed as intensity of the RGB LEDs on both boards
  coded as roll (+Red/-Cyan), Pitch (+Green/-Magenta), and Yaw (+Blue/-Yellow).
 
  Tested with Arduino Nano 33 BLE Sense and Seeed Studio XIAO nRF52840 Sense boards, but the Central
  sketch should work with any board that is BLE-compatible with minor changes depending on the
  specific IMU.  The companion Peripheral sketch requires a BLE-compatible board with RGB LEDs, or
  could be modified with external LEDs on pins that support analogWrite, or as required.
 
  Uncomment and select the #define Rev for the Nano 33 BLE Sense; uncomment the #define nRF52840 line
  for use with that board.  This affects both selection of the appropriate IMU and, how the USER LED
  is is accessed since it is not on a normal digital pin on the nRF52840.
*/

#define Rev2        // Set to appropriate board Rev for Nano BLE; comment out if using a nRF52840 board.
// #define nRF52840   // Comment out if using a Nano or other board

// Gyro offset parameters and variables
#define CalValues 50  // Number of Gyro samples to average for calibration

#define data1 1        // Send data to serial port
#define verbose1 1     // Include labels
#define GyroAutoCal 1  // Perform automatic Gyro offset compensation at startup: Board must be stationary when RGB LED is blinking.

// Color palette for RGB_LEDs.
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

int16_t grint = 0;
int16_t gpint = 0;
int16_t gyint = 0;

// #define nRF52840 // If defined, use nRF52840 CHARGE LED (PIO P0.17) as USER LED.  Comment out otherwise.

float RollOffsetSum = 0;  // Temporary variables for Gyro AutoCal averaging
float PitchOffsetSum = 0;
float YawOffsetSum = 0;

float GR_COR = 0;  // Gyro offset correction values
float GP_COR = 0;
float GY_COR = 0;

int CalCount = CalValues;
int GyroAutoCalFlag = 0;
float pgr, pgp, pgy;

#ifndef nRF52840
#define LED_USER LED_BUILTIN
#endif

#ifdef nRF52840
#define LED_USER 17
#endif

#ifdef Rev1
#include <Arduino_LSM9DS1.h>  // Accelerometer, magnetometer and gyroscope
#endif

#ifdef Rev2
#include <Arduino_BMI270_BMM150.h>  // Accelerometer, magnetometer and gyroscope
#endif

#include <ArduinoBLE.h>

// variables for BLE characteristic

char buffer[40];
float gr, gp, gy, grcor, gpcor, gycor;
int16_t led, ledr, ledg, ledb;
int16_t SN = 0;

// Bluetooth® Low Energy inertial service (Custom UUID)
BLEService inertial("DA3F7226-D807-40E6-A24C-E9F16EDFCD3B");

// Bluetooth® Low Energy Characteristic - custom 128-bit UUID, read and writable by central
BLEIntCharacteristic Gyro_Roll("DA3F7227-D807-40E6-A24C-E9F16EDFCD31", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic Gyro_Pitch("DA3F7227-D807-40E6-A24C-E9F16EDFCD32", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic Gyro_Yaw("DA3F7227-D807-40E6-A24C-E9F16EDFCD33", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic Sequence_Number("DA3F7227-D807-40E6-A24C-E9F16EDFCD34", BLERead | BLEWrite | BLENotify);

void setup() {

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

  // Initialize serial port
  if (data1 == 1) {
    Serial.begin(9600);
    while (!Serial);
  }

  // initialize the Bluetooth® Low Energy hardware
  BLE.begin();

  if (data1 == 1) {
    Serial.println();
    Serial.println("Bluetooth® Low Energy Central - Gyro Data Send");
  }

  // start scanning for peripherals
  BLE.scanForUuid("DA3F7226-D807-40E6-A24C-E9F16EDFCD3B");

  // Configure the IMU

#ifndef nRF52840
  if (!IMU.begin()) {
    if (data1 == 1) Serial.println("Failed to initialize IMU!");
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

  // start advertising
  BLE.advertise();
  if (data1 == 1) Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  while (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    if (data1 == 1) {
      Serial.print("Found ");
      Serial.print(peripheral.address());
      Serial.print(" '");
      Serial.print(peripheral.localName());
      Serial.print("' ");
      Serial.print(peripheral.advertisedServiceUuid());
      Serial.println();
    }

    if (peripheral.localName() != "Gyro Monitor") return;

    // stop scanning
    BLE.stopScan();
    SendGyroData(peripheral);
  }
}

void SendGyroData(BLEDevice peripheral) {
  // connect to the peripheral
  if (data1 == 1) Serial.println("Connecting ...");

  if (peripheral.connect()) {
    if (data1 == 1) Serial.println("Connected");

#ifndef nRF52840
    digitalWrite(LED_USER, HIGH);
#endif

#ifdef nRF52840
    nrf_gpio_pin_write(LED_USER, LOW);
#endif
  }
    else {
    if (data1 == 1) Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  if (data1 == 1) Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    if (data1 == 1) Serial.println("Attributes discovered");
    Serial.println();
    Serial.println("Running");
    Serial.println();
  }
  else {
    if (data1 == 1) Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the characteristics
  BLECharacteristic Gyro_Roll = peripheral.characteristic("DA3F7227-D807-40E6-A24C-E9F16EDFCD31");
  BLECharacteristic Gyro_Pitch = peripheral.characteristic("DA3F7227-D807-40E6-A24C-E9F16EDFCD32");
  BLECharacteristic Gyro_Yaw = peripheral.characteristic("DA3F7227-D807-40E6-A24C-E9F16EDFCD33"); 
  BLECharacteristic Sequence_Number = peripheral.characteristic("DA3F7227-D807-40E6-A24C-E9F16EDFCD34");

  if (!Gyro_Roll) {
    if (data1 == 1) Serial.println("Peripheral does not have Roll Characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (data1 == 1) Serial.println("Peripheral has Roll Characteristic!");
  if (!Gyro_Roll.canWrite()) {
    if (data1 == 1) Serial.println("Peripheral does not have a writable Roll characteristic!");
    peripheral.disconnect();
    return;
  }

  if (!Gyro_Pitch) {
    if (data1 == 1) Serial.println("Peripheral does not have Pitch Characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (data1 == 1) Serial.println("Peripheral has Pitch Characteristic!");
  if (!Gyro_Pitch.canWrite()) {
    if (data1 == 1) Serial.println("Peripheral does not have a writable Pitch characteristic!");
    peripheral.disconnect();
    return;
  }

  if (!Gyro_Yaw) {
    if (data1 == 1) Serial.println("Peripheral does not have Yaw Characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (data1 == 1) Serial.println("Peripheral has Yaw Characteristic!");
  if (!Gyro_Yaw.canWrite()) {
    if (data1 == 1) Serial.println("Peripheral does not have a writable Yaw characteristic!");
    peripheral.disconnect();
    return;
  }

  if (!Sequence_Number) {
    if (data1 == 1) Serial.println("Peripheral does not have SN Characteristic!");
    peripheral.disconnect();
    return;
  }
  else if (data1 == 1) Serial.println("Peripheral has SN Characteristic!");
  if (!Sequence_Number.canWrite()) {
    if (data1 == 1) Serial.println("Peripheral does not have a writable SN characteristic!");
    peripheral.disconnect();
    return;
  }

  if (data1 == 1) Serial.println();

  while (peripheral.connected()) {
    // While the peripheral is connected, send Gyro values.

    // Gyroscope

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

    if (data1 == 1) {
      if (verbose1 == 1) Serial.print("Gyro (Degs/s) Roll: ");
      sprintf(buffer, "%8.2f", grcor);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print("  Pitch: ");
      sprintf(buffer, "%8.2f", gpcor);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print("  Yaw: ");
      sprintf(buffer, "%8.2f", gycor);
      Serial.print(buffer);
      if (verbose1 == 1) Serial.print(" | SN: ");  // print Sequence Number
      sprintf(buffer, "%4d", SN);
      Serial.println(buffer);
    }

    RGB_Gyro_Colors(grcor, gpcor, gycor, scale);

    SN++;

    enKludge (); // Kludge to get signed numbers through Bluetooth ;-)

    Gyro_Roll.writeValue(grint);
    Gyro_Pitch.writeValue(gpint);
    Gyro_Yaw.writeValue(gyint);
    Sequence_Number.writeValue(SN);

#ifdef nRF52840
    if ((SN & 1) == 1) nrf_gpio_pin_write(LED_USER, LOW);
    else nrf_gpio_pin_write(LED_USER, HIGH);
#endif

#ifndef nRF52840
    if ((SN & 1) == 1) digitalWrite(LED_USER, HIGH);
    else digitalWrite(LED_USER, LOW);
#endif 

  delay(1);
  }

if (data1 == 1) Serial.println("Peripheral disconnected");

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
        RollOffsetSum += gr;  // Update sums
        PitchOffsetSum += gp;
        YawOffsetSum += gy;
        if ((CalCount & 3) == 2) RGB_LED_Color(GRAY, 1.0);  // Heartbeat while AutoCal in progress
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
  grint = (grcor * 16);
  if (grint > 32767) grint = 32767;
  if (grint < -32767) grint = -32767;
  grint += 32768;

  gpint = (gpcor * 16);
  if (gpint > 32767) gpint = 32767;
  if (gpint < -32767) gpint = -32767;
  gpint += 32768;

  gyint = (gycor * 16);
  if (gyint > 32767) gyint = 32767;
  if (gyint < -32767) gyint = -32767; 
  gyint += 32768;
}