# ESP-32 MPU9250 VR-Glove
This Arduino Library is meant to enable the ESP-32 MCU communicate with the 9 DoF MPU9250 sensor and also provide the math and functions for the C Design Lab VR glove. 

Repository Contents
-------------------

* **/examples** &mdash; Example sketch for the library (.ino). Run this from the Arduino IDE.
* **/src** &mdash; Source files for the library (.cpp, .h).
* **keywords.txt** &mdash; Keywords from this library that will be highlighted in the Arduino IDE.
* **library.properties** &mdash; General library properties for the Arduino package manager.

Example Briefs
--------------

* MPU9250_glove &mdash; Basic operation of IMU in glove
* MPU9250_glove_separated_filter &mdash; Initial attempt to make the filter into a useful library (backburner)
* ESPclient_RightHand_v#.# &mdash; Code used in right hand glove
* ESPserver_LeftHand_v#.# &mdash; Code used in left hand glove
* ESPtest &mdash; Sends test commands to Unity to elimenate need for full set up for testing
* ESPtouch &mdash; Test code for multiple touch register, also has a visualizer
* WiFi_receiverv2 &mdash; Program used for debugging, can be used to debug right hand communication
* WiFi_transmitterv2 &mdash; Program used for debugging

Documentation
--------------

* **[Register map](https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf)** &mdash; Register map containing the rest of the product documentation.
* **[Hookup Guide](https://learn.sparkfun.com/tutorials/MPU-9250-hookup-guide)** &mdash; Basic hookup guide for the MPU-9250 Breakout.

Code is an adaptation of SparkFun's lightly modified version of Kris Winer's [code](https://github.com/kriswiner/MPU-9250) which was licensed as Beerware.

- Trey Schisler <treyschisler@gmail.com>
