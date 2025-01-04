/*
   Bluetooth Receive Gyro Data 8

  Copyright® Samuel M. Goldwasser, 1994-2025, all rights reserved.  Permission is granted for public
  use or modification as long as the Copyright notice is included.

  This example scans for Bluetooth® Low Energy peripherals until one with the advertised service
  using generated "DA3F7226-D807-40E6-A24C-E9F16EDFCD3B" UUID is found. Once discovered and connected,
  Central sends values for roll, pitch, and yaw of the gyroscope, and a sequence number.  The signed
  amplitude of the Gyro measurements are also displayed as intensity of the RGB LEDs on both boards
  coded as roll (+Red/-Cyan), Pitch (+Green/-Magenta), and Yaw (+Blue/-Yellow).  The USER LED will
  blink at 1/2 the sample rate.
 
  Tested with Arduino Nano 33 BLE Sense and Seeed Studio XIAO nRF52840 Sense boards, but the Central
  sketch should work with any board that is BLE-compatible with minor changes depending on the
  specific IMU.  The companion Peripheral sketch requires a BLE-compatible board with RGB LEDs, or
  could be modified with external LEDs on pins that support analogWrite, or as required.
 
  Uncomment and select the #define Rev for the Nano 33 BLE Sense; Uncomment the #define nRF52840 line
  for use with that board.  This affects both selection of the appropriate IMU and, how the USER LED
  is is accessed since it is not on a normal digital pin on the nRF52840.
*/

#define data1 1      // Send to serial port if 1.  Set to 0 for a remote peripheral NOT on USB.
#define verbose1 1   // Include labels

#define nRF52840     // Use nRF52840 CHARGE LED (PIO P0.17) as USER LED.  Comment out otherwise.

#ifndef nRF52840
#define LED_USER LED_BUILTIN
#endif

#ifdef nRF52840
#define LED_USER 17
#endif

// Color palette for RGB_LEDs19B
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

#include <ArduinoBLE.h>

// Bluetooth® Low Energy inertial service (Custom UUID)
BLEService inertial("DA3F7226-D807-40E6-A24C-E9F16EDFCD3B");

BLEIntCharacteristic Gyro_Roll("DA3F7227-D807-40E6-A24C-E9F16EDFCD31", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic Gyro_Pitch("DA3F7227-D807-40E6-A24C-E9F16EDFCD32", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic Gyro_Yaw("DA3F7227-D807-40E6-A24C-E9F16EDFCD33", BLERead | BLEWrite | BLENotify);
BLEIntCharacteristic Sequence_Number("DA3F7227-D807-40E6-A24C-E9F16EDFCD34", BLERead | BLEWrite | BLENotify);

int SN = 0;
int SNprev = 0;
char buffer[40];
float grcor, gpcor, gycor;
int grint, gpint, gyint;
int ledr, ledg, ledb;

void setup() {

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

  if (data1 == 1) {
    Serial.begin(9600);
    while (!Serial);
  }

  // begin initialization
  if (!BLE.begin()) {
    if (data1 == 1) Serial.println("Starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Gyro Monitor");

  BLE.setAdvertisedService(inertial);

  // add the characteristic to the service
  inertial.addCharacteristic(Gyro_Roll);
  inertial.addCharacteristic(Gyro_Pitch);
  inertial.addCharacteristic(Gyro_Yaw);
  inertial.addCharacteristic(Sequence_Number);

  // add service
  BLE.addService(inertial);

  // set the initial value for the characteristic:

   Gyro_Roll.writeValue(0);
   Gyro_Pitch.writeValue(0);
   Gyro_Yaw.writeValue(0);
   Sequence_Number.writeValue(0);

  // start advertising
  BLE.advertise();

  if (data1 == 1) {
    Serial.println();
    Serial.println("BLE Gyro Monitor Peripheral");
  }
}

void loop() {
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    if (data1 == 1) {
      Serial.print("Connected to central: ");
      Serial.println(central.address());  // Send the central's MAC address:
      if (data1 == 1) Serial.println();
    }

// Connection active
#ifndef nRF52840
    digitalWrite(LED_USER, HIGH);
#endif

#ifdef nRF52840
    nrf_gpio_pin_write(LED_USER, LOW);
#endif

    RGB_LED_Color(BLACK, 0);  // Start with BLACK

    // while the central is still connected to peripheral:
    while (central.connected()) {
      SNprev = SN;
      if (Sequence_Number.written()) SN = Sequence_Number.value();
      if (SN != SNprev) {
        if (Gyro_Roll.written()) grint = Gyro_Roll.value(); // Kludge to get signed numbers through Bluetooth
        if (Gyro_Pitch.written()) gpint = Gyro_Pitch.value();
        if (Gyro_Yaw.written()) gyint = Gyro_Yaw.value();
       
        deKludge(); // Kludge to get signed numbers through Bluetooth ;-)

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

#ifdef nRF52840
            if ((SN & 1) == 1) nrf_gpio_pin_write(LED_USER, LOW);
            else nrf_gpio_pin_write(LED_USER, HIGH);
#endif

#ifndef nRF52840
            if ((SN & 1) == 1) digitalWrite(LED_USER, HIGH);
            else digitalWrite(LED_USER, LOW);
#endif 
        }
      RGB_Gyro_Colors(grcor, gpcor, gycor, scale);  
      }
    }

    // When the central disconnects, print it out:
    if (data1 == 1) {
      Serial.println();
      Serial.print(F("Disconnected from central: "));
      Serial.println(central.address());
    }

// Connection innactive
#ifndef nRF52840
    digitalWrite(LED_USER, LOW);
#endif

#ifdef nRF52840
    nrf_gpio_pin_write(LED_USER, HIGH);
#endif

  RGB_LED_Color(BLACK, 0);  // End with BLACK
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

void deKludge() {
  grcor = grint - 32768;
  grcor /= 16;

  gpcor = gpint - 32768;
  gpcor /= 16;

  gycor = gyint - 32768;
  gycor /= 16;
}
