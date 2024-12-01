These are simple utilities to test most of the Nano BLE 33 Sense Ver1 and Ver2 sensors using the on-board LEDs and serial port. The required Nano BLE 33 libraries are all either built into the Arduino IDE or Arduino Cloud Editor, or readily found via a Web search.

Accelerometer (Gs) X, Y, Z; Gyroscope (Degs/s) Roll, Pitch, Yaw; Magnetic Field (Gauss) X, Y, Z; Temperature (DegC), Pressure (mm/Hg), Humidity (%), Proximity (Prox), and RGB Light Detect (R, G, B) values are all optionally sent via the serial port, as data-only, or with labels.

In addition:

1. Gyroscope: Displays the absolute value for Roll, Pitch, and Yaw as the brightness of the red, green, and blue RGB LEDs.
2. Proximity: Displays the distance as the brightness of the BUILTIN_LED (bright is closest).
3. Static tilt (accelerometer Z value): Turns on the PWR_LED if more than approximately 45 degrees.
4. The PWR_LED also flashes at an approximately 1 Hz rate if there is no Gyroscope or Proximity activity.

Suggestions for (modest!) improvements welcome.

Thanks,

--- sam
