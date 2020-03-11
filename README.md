# LED Flow
LEDs that respond to motion &amp; flow. Warning: can be addictive.

Supported LED types: Adafruit NeoPixels (WS2812b) & Dotstars (APA102c)

The motion controlled setting uses a GY-521 MPU-6050 3 Axis Accelerometer Gyroscope Module with 6 DOF to sense angular veolcity and orientation in space. The LEDs change speed, color, and pattern in real time based on sensor readings.

I also hope to upload a schematic soon for anyone who would like to build this themselves.

Prototyped on an Arduino Due with the final compiled code and wired components on a Teensy 3.2.

More recently I have also built a finished product with a Teensy Prop Shield. Significantly more expensive compared to the MPU-6050 however fits perfectly on top of the Teensy, making for an easy insert into the tight poly carbonate 1'' diameter tube housing. 
