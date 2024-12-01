/*
  Arduino Nano 33 BLE Sense Rev1: Simple Accelerometer, Gyroscope, Magnetic Field, Environmental, Proximity, and Light Sensor Test

  Copyright 1994-2024 Sam Goldwasser

  This example reads the values from the sensors and optionally sends them to the Serial Monitor or Serial Plotter.
  It also shows Roll, Pitch, and Yaw, as well as proximity in the intensity of the BUILTIN LED, and turn PWR_LED on
  if tilt (az) is more than ~45 degrees.

  V9: 29-Nov-2024.  Added test to conditionally send data
*/

#define verbose1 1                 // Display labels if 1, data-only if 0
#define data1 1                    // Sends data if 1, LEDs-only on Nano if 0

#include <Arduino_LSM9DS1.h>       // Accelerometer, magnetometer and gyroscope
#include <Arduino_HTS221.h>        // Temperature and humidity
#include <Arduino_LPS22HB.h>       // Pressure
#include <Arduino_APDS9960.h>      // RGB light and proximity

int proximity = 0;

void setup() {
  
   // set the LEDs pins as outputs
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PWR, OUTPUT);
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  // turn all the LEDs off
  analogWrite(LED_BUILTIN, 255);
  digitalWrite(LED_PWR, 1);
  analogWrite(LEDR, 200);
  analogWrite(LEDG, 200);
  analogWrite(LEDB, 200);
  
  if (data1 == 1) {
     Serial.begin(9600);
     while (!Serial);
     Serial.println("Started.");
     }

  if (!IMU.begin()) {
    if (data1 == 1) Serial.println("Failed to initialize IMU!");
    while (1);
    }  
  if (!HTS.begin()) {
    if (data1 == 1) Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);  
    }

  if (!BARO.begin()) {
    if (data1 == 1) Serial.println("Failed to initialize pressure sensor!");
    while (1);
    }
 
  if (!APDS.begin()) {
    if (data1 == 1) Serial.println("Error initializing APDS9960 sensor!");
    }
  
  if ((verbose1 == 1) && (data1 == 1)) {
    Serial.println("Acceleration in Gs.");
    Serial.println("Angle in degrees/second. LED threshold 25.");  
    Serial.println("Magnetic field in Gauss.");
    Serial.println("Temperature in degrees Centigrade.");
    Serial.println("Pressure in mm/Hg.");
    Serial.println("Humidity in rel %.");
    Serial.println();
  }
}

void loop() {
  float ax, ay, az, gr, gp, gy, mx, my, mz;
  char buffer[40];
  int led = 0;

// Accelerometer

  while (!IMU.accelerationAvailable()) {}
    {
      IMU.readAcceleration(ax, ay, az);
 
      if (data1 == 1) {
        if (verbose1 == 1) Serial.print("Axl (Gs) X: ");
        sprintf(buffer, "%5.2f", ax );
        Serial.print(buffer);
        if (verbose1 == 1) Serial.print(" Y: ");
        sprintf(buffer, "%5.2f", ay );
        Serial.print(buffer);
        if (verbose1 == 1) Serial.print(" Z: ");
        sprintf(buffer, "%5.2f"  , az );
        Serial.print(buffer);
        if (verbose1 == 1) Serial.print(" | ");
      }

     led = (az * 255);
     if (led < 180) digitalWrite(LED_PWR, HIGH); // Turn on LED_PWR if tilt more than ~45 degrees
      else digitalWrite(LED_PWR, LOW);
     
     // analogWrite(LED_PWR, led); // Using analogWrite hangs here, even with a cosntant???
    }

// Gyroscope

#define GR_COR   6.5   // These are board-specific offset nulls
#define GP_COR   0
#define GY_COR   2.5

  while (!IMU.gyroscopeAvailable()) {}
    {
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
        if (verbose1 == 1) Serial.print(" | ");
      }

      led = gr - GR_COR;
      analogWrite(LEDR,255-abs(led/8));
    
      led = gp - GP_COR;
      analogWrite(LEDG,255-abs(led/8));

      led = gy - GY_COR;
      analogWrite(LEDB,255-abs(led/8));
    }

// Magnetic field

  while (!IMU.magneticFieldAvailable()) {}
    {
      IMU.readMagneticField(mx, my, mz);
      if (data1 == 1) {

      if (verbose1 == 1) Serial.print("Field (Gauss) X: ");
        sprintf(buffer, "%5.2f", mx/100 );
        Serial.print(buffer);
        if (verbose1 == 1) Serial.print( "  Y: ");
        sprintf(buffer, "%5.2f", my/100 );
        Serial.print(buffer);
        if (verbose1 == 1) Serial.print( "  Z: ");
        sprintf(buffer, "%5.2f", mz/100 );
        Serial.print(buffer);
        if (verbose1 == 1) Serial.print(" |");
      }
    }

// Temperature, humidity, and pressure

  float temperature = HTS.readTemperature();
  float humidity    = HTS.readHumidity();
  float pressure    = BARO.readPressure();

  if (data1 == 1) { 
    if (verbose1 == 1) Serial.print(" T:");
    sprintf(buffer, " %5.2f", temperature );
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print(" DegC; P: "); 
    sprintf(buffer, "%6.2f", pressure*7.50062 );
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print(" mm/Hg; H: ");
    sprintf(buffer, "%5.2f", humidity );
    Serial.print(buffer);
    if (verbose1 == 1) Serial.print("% | Prox:");  
  }

// Proximity

  if (APDS.proximityAvailable()) proximity = APDS.readProximity();
  analogWrite(LED_BUILTIN, 230-proximity); 

  if (data1 == 1) {
    sprintf(buffer, " %3d", proximity );
    Serial.print(buffer);
  }

// RGB light detect

  int r, g, b;

  while (!APDS.colorAvailable()) delay(5);
  APDS.readColor(r, g, b);
/*
  if (r > 16) analogWrite(LEDR, 255-(r/16)); // Removed due to useless behavior. ;-)
  if (g > 16) analogWrite(LEDG, 255-(g/16)); 
  if (b > 16) analogWrite(LEDB, 255-(b/16)); 
*/
   if (data1 == 1) {
     if (verbose1 == 1) Serial.print(" | Light R: ");
        sprintf(buffer, "%3d", r/16 );
        Serial.print(buffer);
        if (verbose1 == 1) Serial.print(" G: ");
        sprintf(buffer, "%3d", g/16 );
        Serial.print(buffer);
        if (verbose1 == 1) Serial.print(" B: ");
        sprintf(buffer, "%3d", b/16 );
        Serial.print(buffer);
        Serial.println("");
   }
  delay(25); 
}