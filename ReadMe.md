These are basic sketches for the Nano 33 BLE Sense (https://store-usa.arduino.cc/collections/boards-modules/products/nano-33-ble-sense-rev2).

These apply to both the Rev1 and Rev2 boards.

I put these here because while it's almost certain that sketches like these have been out
there for years, a Web search has real trouble finding them, especially with AI. ;-)

* Nano_33_BLE_Sense_RGB_LED_Cycle_Colors1.  Fancy version of Blink. ;-)
* Nano_33_BLE_Sense_Sensor_Test21: Exercise most of the sensors with data and visual displays.
* Nano_33_BLE_Sense_Sensor_Test12: Exercise most of the sensors with data and visual displays.
  This version runs much slower on the Rev2 board for unknown reasons.
* Nano_33_BLE_Sense_Rev1-2_IMU-TPH_Speed_Test3: Effort to diagnose apparent higher performance of
  Rev1 compared to Rev2 especially for Magnetometer.  The loop reading the magnetic field in X,Y,Z
  runs ~twice as fast for Rev1 compared to Rev2.  In addition, IMU intialization is virtually
  instantaneous for Rev1 but takes a second or more for Rev2.  Anyone with thoughts or a solution
  to this, please contact me.  And feel free to tell me I'm a moron. ;-)
* Nano_33_BLE_Sense_RGB_LED_Color_via_Bluetooth1: Display colors in RGB_LEDs via Bluetooth.
* Nano_33_BLE_Sense_Bluetooth_Send_Test1: Send Gyro data via Bluetooth and also display magnitude in
  RGB_LEDs.

Written/modified by Samuel M. Goldwasser.  Copyright® (if any) 1994-2025 and additional details in
the sketch headers.
