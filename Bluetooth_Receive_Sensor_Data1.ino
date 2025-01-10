/*
  Bluetooth Receive Sensor Data 1
 
  Copyright® Samuel M. Goldwasser, 1994-2025, all rights reserved.  Permission is granted for public use or modification as
  long as the Copyright notice is included.

  This is a pair sketches to exercise selected Nano BLE 33 Sense Rev1 or Rev2 sensors using the on-board LEDs and serial
  port, and to send selected sets of values to another BLE enabled board.  Tested with the Nano 33 BLE Sense Rev1 and Rev2
  for the sender and the Seed Studio XIAO nRF53840 Sense for the receiver,  But many others boards should work with trivial
  modifications.  Famous last words! ;-)

  The required libraries are all either built into the Arduino IDE or Arduino Cloud Editor, or readily found via a Web
  search.  Note that the primary difference between the Rev1 and Rev2 sketches are the libraries for the IMU and T/H.

  Accelerometer (Gs) X, Y, Z; Gyroscope (Degs/s) Roll, Pitch, Yaw; Magnetic Field (Gauss) X, Y, Z; Temperature (DegC),
  Pressure (mm/Hg), Humidity (%), Proximity (Prox), RGB Light Detect (R, G, B), and peak audio level (Mic) are optionally
  acquired and may be sent via the serial port as data-only, or with labels, and via Bluetooth to another board.

  The way it is set up by default, all the data listed above is acquired and displayed, but only Gyro (roll, pitch, yaw),
  Proximity, and peak audio level are sent via Bluetooth.  Strightforward modifications will be required of both the sending
  and receiving sketches to add some or all of the others.

  In addition to the data, the on-board BUILTIN (or USER) LED, PWR LED (if available), and RGB LEDs provide visual output:

  1. Gyroscope: Displays the values for roll, pitch, and yaw as the brightness of each if the RGB leds as roll (+red/-cyan),
     pitch (+green/-magenta), and yaw (+blue/-yellow).  Optional gyro calibration to compensate for board-specific roll, pitch,
     and yaw offsets.  If GyroAutoCall is enabled, the board must remain stationary at startup while the RGB LEDs are blinking.
     The default duration is ~12 blinks, under 1 second.  This may only be needed for Rev1 boards.
  2. Proximity: Displays the distance as the brightness of the BUILTIN_LED (bright is closest).
  3. Static Tilt (accelerometer Z value): Turns on the PWR_LED (if available) if more than approximately 45 degrees.
  4. Microphone: Displays the peak intensity of the audio on a color scale using the RGB LEDs ONLY when the Gyro is not active.
  5. Heartbeat: The BUILTIN_LED flashes at an approximately 1 Hz rate if there is no display activity.
*/

// user settings
#define nRF52840     // Use nRF52840 CHARGE LED (PIO P0.17) as USER LED.  Comment out otherwise.

#define data1 1      // Send to serial port if 1.  Set to 0 for a remote peripheral NOT on USB.
#define verbose1 1   // Include labels

#define Gyroscope            // Send Roll, Pitch, Yaw over BLE
#define Microphone           // Send peak audio as level over BLE
// #define Random_Number        // Send random number as level over BLE

#ifndef nRF52840
#define LED_USER LED_BUILTIN
#endif

#ifdef nRF52840
#define LED_USER 17
#endif

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

char buffer[40];
float grcor, gpcor, gycor, bright;
int32_t grint, gpint, gyint;
uint32_t SN = 0;
int SNPrint;
int SNprev = 0;
int ledr, ledg, ledb;
int sendData = data1;
uint16_t level;
int holdoff = timeoutvalue;
uint Activity_Flags = 0;
uint Send_Flags = 0;

// Activity and send selection flags.  Must shift up by 20 to stuff in SN.
#define Accelerometer_Flag       0x1
#define Gyroscope_Flag           0x2
#define Magnetic_Field_Flag      0x4
#define Environment_Flag         0x8
#define Proximity_Flag          0x10
#define Light_Detector_Flag     0x20
#define Microphone_Flag         0x40
#define Input_Level_Flag        0x80
#define Random_Number_Flag     0x100
#define Diagnostics_Flag       0x200
#define Heartbeat_Flag         0x800

// Bluetooth® Low Energy inertial plus service (Custom UUIDs)

#define BLE_UUID_INERTIAL_PLUS_SERVICE          "DA3F7226-D807-40E6-A24C-E9F16EDFCD3B"

#ifdef Accelerometer
#define BLE_UUID_ACCEL_X                        "DA3F7227-D807-40E6-A24C-E9F16EDFCD30"
#define BLE_UUID_ACCEL_Y                        "DA3F7227-D807-40E6-A24C-E9F16EDFCD31"
#define BLE_UUID_ACCEL_Z                        "DA3F7227-D807-40E6-A24C-E9F16EDFCD32"
#endif

#ifdef Gyroscope
#define BLE_UUID_GYRO_ROLL                      "DA3F7227-D807-40E6-A24C-E9F16EDFCD33"
#define BLE_UUID_GYRO_PITCH                     "DA3F7227-D807-40E6-A24C-E9F16EDFCD34"
#define BLE_UUID_GYRO_YAW                       "DA3F7227-D807-40E6-A24C-E9F16EDFCD35"
#endif

#ifdef Magnetic_Field
#define BLE_UUID_FIELD_X                        "DA3F7227-D807-40E6-A24C-E9F16EDFCD36"
#define BLE_UUID_FIELD_Y                        "DA3F7227-D807-40E6-A24C-E9F16EDFCD37"
#define BLE_UUID_FIELD_Z                        "DA3F7227-D807-40E6-A24C-E9F16EDFCD38"
#endif

#ifdef Environment
#define BLE_UUID_EVIRN_T                        "DA3F7227-D807-40E6-A24C-E9F16EDFCD39"
#define BLE_UUID_EVIRN_P                        "DA3F7227-D807-40E6-A24C-E9F16EDFCD3A"
#define BLE_UUID_EVIRN_H                        "DA3F7227-D807-40E6-A24C-E9F16EDFCD3B"
#endif

#ifdef Proximity
#define BLE_UUID_PROXIMITY                      "DA3F7227-D807-40E6-A24C-E9F16EDFCD3C"
#endif

#ifdef Light_Intensity
#define BLE_UUID_LIGHT_R                        "DA3F7227-D807-40E6-A24C-E9F16EDFCD3D"
#define BLE_UUID_LIGHT_G                        "DA3F7227-D807-40E6-A24C-E9F16EDFCD3E"
#define BLE_UUID_LIGHT_B                        "DA3F7227-D807-40E6-A24C-E9F16EDFCD3F"
#endif

#ifdef Microphone
#define BLE_UUID_MICROPHONE                     "DA3F7227-D807-40E6-A24C-E9F16EDFCD40"
#endif

#ifdef Input_Level
#define BLE_UUID_INPUT_LEVEL                    "DA3F7227-D807-40E6-A24C-E9F16EDFCD41"
#endif

#ifdef Random
#define BLE_UUID_RANDOM_NUMBER                  "DA3F7227-D807-40E6-A24C-E9F16EDFCD43"
#endif

#ifdef Diagnostics
#define BLE_UUID_DIAGNOSTICS                    "DA3F7227-D807-40E6-A24C-E9F16EDFCD34"
#endif

#ifdef Spare_1
#define BLE_UUID_SPARE1                         "DA3F7227-D807-40E6-A24C-E9F16EDFCD45"
#endif

#define BLE_UUID_SEQUENCE_NUMBER                "DA3F7227-D807-40E6-A24C-E9F16EDFCD4F"

#include <ArduinoBLE.h>

BLEService inertial_plus(BLE_UUID_INERTIAL_PLUS_SERVICE);

#ifdef Gyroscope
BLEIntCharacteristic Gyro_Roll(BLE_UUID_GYRO_ROLL, BLEWrite );
BLEIntCharacteristic Gyro_Pitch(BLE_UUID_GYRO_PITCH, BLEWrite );
BLEIntCharacteristic Gyro_Yaw(BLE_UUID_GYRO_YAW, BLEWrite );
#endif

#ifdef Microphone
BLEIntCharacteristic Audio_Level(BLE_UUID_MICROPHONE, BLEWrite );
#endif

BLEIntCharacteristic Sequence_Number(BLE_UUID_SEQUENCE_NUMBER, BLEWrite );

void setup() {

 // Set the LEDs pins as outputs.  Turn on LED_USER and set the RGB LEDs at low brightness.

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

  // begin initialization

  // enable serial port if data1 is 1 and port available.
  if (data1 == 1) {
    Serial.begin(9600);
    if (Serial == 0) sendData = 1;
  }

  // start BLE
  if (!BLE.begin()) {
    if (sendData == 1) Serial.println("Starting Bluetooth® Low Energy module failed!");
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("Gyro Monitor Receiver");
  BLE.setAdvertisedService(inertial_plus);

  // add the characteristics to the service
#ifdef Gyroscope
  inertial_plus.addCharacteristic(Gyro_Roll);
  inertial_plus.addCharacteristic(Gyro_Pitch);
  inertial_plus.addCharacteristic(Gyro_Yaw);
  Send_Flags |= Gyroscope_Flag;
#endif

#ifdef Microphone
  inertial_plus.addCharacteristic(Audio_Level);
  Send_Flags |= Microphone_Flag;
#endif

  inertial_plus.addCharacteristic(Sequence_Number);

  // add service
  BLE.addService(inertial_plus);

  // start advertising
  BLE.advertise();
  if (sendData == 1) Serial.println("Bluetooth® device active, waiting for connections...");

  if (sendData == 1) {
    Serial.println();
    Serial.println("BLE Gyro Monitor Receiver");
  }
}

void loop() {
  // listen for Bluetooth® Low Energy peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    if (sendData == 1) {
      Serial.println();
      Serial.print("Connected to central: ");
      Serial.println(central.address());  // Send the central's MAC address:
      if (sendData == 1) Serial.println();
    }

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

    RGB_LED_Color(BLACK, 0);  // Start with BLACK

    // while the central is still connected to peripheral:
    while (central.connected()) {
      SNprev = SN;
      if (Sequence_Number.written()) SN = (Sequence_Number.value());
      if (SN != SNprev) {
        Send_Flags = (SN >> 20);
        SN &= 0x7FFFF;        
        grint = Gyro_Roll.value();
        gpint = Gyro_Pitch.value();
        gyint = Gyro_Yaw.value();
        deKludge(); // Undo the kludge to get fractional resolution without using floating point. ;-)

        level = Audio_Level.value();

        if ((fabs(grcor) > 1) || (fabs(gpcor) > 1) || (fabs(gycor) > 1)) holdoff = timeoutvalue;  // Display input level in RGB LEDs only if no Gyro activity
          if (holdoff > 0) {
            RGB_Gyro_Colors(grcor, gpcor, gycor, scale); 
            holdoff --;
            if (holdoff <= 0) holdoff = 0;
          }
          else {

     
#ifdef Microphone
            Display_Color_Level (level);
#endif

#ifdef random1
            ledr =   (level & 0b000000000111) << 5;
            ledg =   (level & 0b000000111000) << 2;
            ledb =   (level & 0b000111000000) >> 1;
            bright = (((level & 0b111000000000) >> 9) + 1);
            bright /= 8;
            RGB_LED_Color (ledr, ledg, ledb, bright);
#endif

        SNPrint = SN;

        if (sendData == 1) {
          if (verbose1 == 1) Serial.print("SN: ");
          sprintf(buffer, "%6d", SNPrint);
          Serial.print(buffer);
        }
#ifdef Gyroscope
        if (sendData == 1) {
            if (verbose1 == 1) Serial.print(" | Gyro (Degs/s) Roll: ");
          sprintf(buffer, "%8.2f", grcor);
          Serial.print(buffer);
          if (verbose1 == 1) Serial.print("  Pitch: ");
          sprintf(buffer, "%8.2f", gpcor);
          Serial.print(buffer);
          if (verbose1 == 1) Serial.print("  Yaw: ");
          sprintf(buffer, "%8.2f", gycor);
          Serial.print(buffer);
        }
#endif

#ifdef Microphone
         if (sendData == 1) {
          if (verbose1 == 1) Serial.print(" | Mic: ");
          sprintf(buffer, "%4d", level);
          Serial.println(buffer);
        }
#endif

            if ((Send_Flags & Heartbeat_Flag) != 0) {
#ifndef nRF52840        
              digitalWrite(LED_USER, HIGH);
              delay(10);
              digitalWrite(LED_USER, LOW);
#endif

#ifdef nRF52840    
              nrf_gpio_pin_write(LED_USER, LOW);
              delay(10);
              nrf_gpio_pin_write(LED_USER, HIGH);
#endif

            }
          }
      }
  }
    // When the central disconnects, print it out:
    if (sendData == 1) {
      Serial.println();
      Serial.print(F("Disconnected from central: "));
      Serial.println(central.address());
      Serial.println();
    }

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

  RGB_LED_Color(BLACK, 0);  // End with BLACK
  }
}

void RGB_LED_Color(int r, int g, int b, float intensity) {
  ledr = r;
  analogWrite(LEDR, (255 - (r * intensity)));
  ledg = g;
  analogWrite(LEDG, (255 - (g * intensity)));
  ledb = b;
  analogWrite(LEDB, (255 - (b * intensity)));
  bright = intensity;
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
  grcor = grint;
  grcor /= 1000; 
  gpcor = gpint;
  gpcor /= 1000;
  gycor = gyint;
  gycor /= 1000;
}

void Display_Color_Level (uint16_t sum) {

    // Display the peak sound value in RGB_LED
    if (((fabs(grcor) < 1) && (fabs(gpcor) < 1) && (fabs(gycor)) < 1)) {  // Only if no Gyro activity and GyroAutoCAl not in progress

    if (sum >= 1000) RGB_LED_Color(WHITE, 1.0);
      else if (sum >= 600) RGB_LED_Color(RED, 1.0);
      else if (sum >= 400) RGB_LED_Color(ORANGE, 0.8);
      else if (sum >= 325) RGB_LED_Color(YELLOW, 0.5);
      else if (sum >= 250) RGB_LED_Color(GREEN, 0.75);
      else if (sum >= 175) RGB_LED_Color(CYAN, 0.2);
      else if (sum >= 100) RGB_LED_Color(BLUE, 0.3);
      else if (sum >= 50) RGB_LED_Color(MAGENTA, 0.2);
      else if (sum >= 25) RGB_LED_Color(GRAY, 1.0);
      else RGB_LED_Color(BLACK, 0.0);

//    if (sum >= 25) timeout = timeoutvalue * 2;
}  
}