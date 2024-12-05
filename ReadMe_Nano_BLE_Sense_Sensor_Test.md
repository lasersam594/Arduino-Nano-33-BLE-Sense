This a simple utility to test most of the Nano BLE 33 Sense Ver1 or Ver2 sensors using the on-board LEDs and serial port. The required Nano BLE 33 libraries are all either built into the Arduino IDE or Arduino Cloud Editor, or readily found via a Web search.  Note that the primary difference between the Ver1 and Ver2 sketches are the libraries for the IMU and T/H.

Accelerometer (Gs) X, Y, Z; Gyroscope (Degs/s) Roll, Pitch, Yaw; Magnetic Field (Gauss) X, Y, Z; Temperature (DegC), Pressure (mm/Hg), Humidity (%), Proximity (Prox), RGB Light Detect (R, G, B) values, and peak Mic values are all optionally sent via the serial port, as data-only, or with labels.

In addition the on-board BUILTIN_LED, PWR_LED, and RGB_LED provide visual output:

1. Gyroscope: Displays the absolute value for Roll, Pitch, and Yaw as the brightness of the RGB leds.
2. Proximity: Displays the distance as the brightness of the BUILTIN_LED (bright is closest).
3. Static Tilt (accelerometer Z value): Turns on the PWR_LED if more than approximately 45 degrees.
4. Microphone: Displays the peak intensity of the audio on a color scale using the RGB leds.
5. Heartbeat: The BUILTIN_LED flashes at an approximately 1 Hz rate if there is no display activity.

To select the Nano BLE 33 Sense board type (Ver1 or Ver2), edit the #define at the beginning of the sketch.

Suggestions for (modest!) improvements welcome.

Thanks,

--- sam
