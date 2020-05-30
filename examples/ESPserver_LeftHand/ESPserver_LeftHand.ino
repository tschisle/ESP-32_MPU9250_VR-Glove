//Left hand glove control - Server mode
//touch input and magnetic sensing
//touch input always available and magnetic sensing only available after calibration received from right hand.

/* GOALS:
   - Make every Serial call compiled based off a defined symbol
   ~ Replace touch multiple tolerances with an autocalibrated value
*/

// Just test touch pin - Touch0 is T0 which is on GPIO 4
/* Press, hold, slide - trans number
   1  - press T7 - B - Right
   2  - press T6 - A - Bottom
   3  - press T8 - Y - Top
   4  - press T9 - X - Left
   5  - swipe from A to B
   6  - swipe from B to A
   7  - swipe from X to Y
   8  - swipe from Y to X
   9  - hold B
   10 - hold A
   11 - hold Y
   12 - hold X
*/
#include <WiFi.h>
#include <WiFiUdp.h>
//_____________________________________________________________________________________

class Touchpad
{
    unsigned long tpmat[3][2][2]; //layer, row, column
    /*  touch |top, right|   press |top, right|    time    |top, right|
        flags |left, bott|,  order |left, bott|,   pressed |left, bott|
        Y - Top; B - Right; A - Bottom; X - Left
    */
    const unsigned long inter_button_slide_time = 250; //number of milliseconds between button presses to distinguish a slide from quick button presses
    const unsigned long button_hold_time = 500; //number of milliseconds between distinguishing a press from a hold
    int slide_timer = 0; //counts down time between presses
    int hold_timer = 0; //counts down time holding a press
    const int set_point_h = 45; //value based from trial and error
    const int set_point_l = 10; //value based from trial and error
    int T9_init = 0;  //4 - Y - Top
    int T8_init = 0;  //3 - A - Bottom
    int T7_init = 0;  //1 - B - Right
    int T6_init = 0;  //2 - X - Left
    bool touched = false;//gets set when a touch is detected
    int num_touches = 0; //gets the number of touches within the gesture period
    unsigned long temp_touch_time_ref = 0;//serves as the reference time for the gesture recognition.
    bool slide_detected = false;
    bool hold_detected = false;
    bool press_detected = false;
    bool hold_mask[2][2] = {{false, false}, {false, false}}; //prevents presses from being reported after a hold
    int press_value; //allows the tpmat to be fully cleared
  public:
    initialize() {
      int i, j, k;
      delay(300); //will likely fix the initial reading issue
      T9_init = touchRead(T9); //this first reading is wrong - taken again at the end of the readings
      T8_init = touchRead(T8); //3 - A - Bottom
      T7_init = touchRead(T7); //1 - B - Right
      T6_init = touchRead(T6); //2 - X - Left
      T9_init = touchRead(T9); //4 - Y - Top
      for (i = 0; i < 3; i++) {
        for (j = 0; j < 2; j++) {
          for (k = 0; k < 2; k++) {
            tpmat[i][j][k] = 0;
          }
        }
      }
      num_touches = 0;
      temp_touch_time_ref = 0;
    }

    int update()
    {
      int i, j;
      slide_detected = false;
      hold_detected = false;
      press_detected = false;
      matrixUpdate();  //also performs touch reads
      /*NOTES
         Since there isn't a command for pressing multiple buttons at once the code assumes this won't happen.
         This can be added in the slide detection.

         Additionally the detection code is written to allow gesture changes with continuous contact to the touchpad

         Possible issue: The hold mask may not be completely robust as it's used now.    
      */
      //Slide Detection -- Reports the last slid-over button
      if (num_touches > 1) {
        if (temp_touch_time_ref != 0) {
          press_value = 0;
          for (j = 0; j < 2; j++) {
            for (k = 0; k < 2; k++) {
              if ((num_touches == tpmat[1][j][k]) && (tpmat[1][(j + 1) % 2][(k + 1) % 2] > 0)) {
                //Organized to remove else's
                slide_detected = true;
                press_value = 2; //2 - X - Left
                if (j == 0) {
                  press_value = 1; //1 - B - Right
                }
                if (j == k) {
                  press_value = 4; //4 - Y - Top
                  if (j == 0) {
                    press_value = 3; //3 - A - Bottom
                  }
                }
              }
            }
          }
        }
      }
      //Hold Detection -- Reports the longest held button
      if (hold_timer == 0) {
        press_value = 0;
        for (j = 0; j < 2; j++) {
          for (k = 0; k < 2; k++) {
            if (tpmat[2][j][k] > (button_hold_time - 1)) {
              //Organized to remove else's
              hold_detected = true;
              hold_mask[j][k] = true;
              press_value = 2; //2 - X - Left
              if (j == 0) {
                press_value = 1; //1 - B - Right
              }
              if (j == k) {
                press_value = 4; //4 - Y - Top
                if (j == 0) {
                  press_value = 3; //3 - A - Bottom
                }
              }
            }
          }
        }
      }
      //Press Detection  -- Reports the first pressed button
      if (!touched && (slide_timer == 0)) {
        press_value = 0;
        for (j = 0; j < 2; j++) {
          for (k = 0; k < 2; k++) {
            if ((tpmat[1][j][k] == 1) && (!hold_mask[j][k])) {
              //Organized to remove else's
              if (!hold_mask[j][k]) {
                hold_mask[j][k] = false;
              } else {
                press_detected = true;
                press_value = 2; //2 - X - Left
                if (j == 0) {
                  press_value = 1; //1 - B - Right
                }
                if (j == k) {
                  press_value = 4; //4 - Y - Top
                  if (j == 0) {
                    press_value = 3; //3 - A - Bottom
                  }
                }
              }
            }
          }
        }
      }
      //End Gesture
      if (slide_detected || hold_detected || press_detected) {
        num_touches = 0;
        temp_touch_time_ref = 0;
        for (j = 0; j < 2; j++) {
          for (k = 0; k < 2; k++) {
            tpmat[1][j][k] = 0;
            tpmat[2][j][k] = 0;
          }
        }
        if (slide_detected) {
          return (press_value + 4);
        } else if (hold_detected) {
          return (press_value + 8);
        } else if (hold_detected) {
          return (press_value);
        } else {
          return (0);
        }
      }
    }

  private:
    void matrixUpdate(void)
    {
      tpmat[0][0][0] = T8_init - touchRead(T8); //3 - A - Bottom
      tpmat[0][0][1] = T7_init - touchRead(T7); //1 - B - Right
      tpmat[0][1][0] = T6_init - touchRead(T6); //2 - X - Left
      tpmat[0][1][1] = T9_init - touchRead(T9); //4 - Y - Top
      int j, k;
      touched = false;
      if (slide_timer > 0) {
        slide_timer--;
      }
      for (j = 0; j < 2; j++) {
        for (k = 0; k < 2; k++) {
          if (tpmat[0][j][k] > ((set_point_h + set_point_l) / 2)) {
            tpmat[0][j][k] = 1; //boolean TRUE
            touched = true;
            num_touches++;
            slide_hold_timer = (int) inter_button_slide_time;
            if (hold_timer > 0) {
              hold_timer--;
            }
            tpmat[1][j][k] = num_touches; //Touch Order
            if (temp_touch_time_ref == 0) {
              temp_touch_time_ref = millis();
            } else {
              tpmat[2][j][k] = millis() - temp_touch_time_ref; //time held relative to the start of the gesture
            }
          } else {
            tpmat[0][j][k] = 0; //boolean FALSE
          }
        }
      }
      if (!touched) {
        hold_timer = (int) button_hold_time;
      }
    }
};

//WiFi configuration
WiFiUDP Udp; // Creation of wifi Udp instance
Touchpad touch; //Creates touchpad object
char packetBuffer[255];
unsigned int localPort = 9999;
const char *ssid =  "NETGEAR36-5G";// "NETGEAR36-5G";
const char *password = "pastelstreet317";
IPAddress ipCliente (192, 168, 4, 10);   // right hand glove IP address to send messages directly
//IPAddress ipCliente3(192, 168, 4, 12);   // right hand glove IP address to send messages directly
IPAddress ipCliente3(192, 168, 1, 8);  //UNITY client ip

//_____________________________________________________________________________________
//Touch configuration
int command = 0;
int aux_command = 0;


//_____________________________________________________________________________________

void setup() {
  //wifi setup
  Serial.begin(115200);
  WiFi.softAP(ssid, password);  // ESP-32 as access point
  Udp.begin(localPort);
  //_____________________________________________________________________________________
  //touch sensing setup
  Serial.println("ESP32 Touch Test");
  touch.initialize();
  delay(500); // delay to check the values in the serial monitor
}



//______________________________________________________________________________________________________________
void loop() {
  Serial.println(WiFi.localIP());
  //-----------------------------------------------------------------
  //Getting packages for calibration
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len - 1] = 0;
    Serial.print("Recibido(IP/Size/From RightHand): ");
    Serial.print(Udp.remoteIP()); Serial.print(" / ");
    Serial.print(packetSize); Serial.print(" / ");
    Serial.println(packetBuffer);//has the command number (1 to 4) for calibration
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort()); //sends confirmation message
    //Udp.printf("received: ");
    Udp.printf("RECIBIDOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO"); // sending command to the esp32-1
    //Udp.printf(packetBuffer);
    Udp.printf("\r\n");
    Udp.endPacket();
  }
  //-----------------------------------------------------------------
  command = touch.update();
  Serial.print("command  ");
  Serial.println(command);
  sendReadings(command); //sends a messace every cycle to the client 192, 168, 4, 10
  sendReadingsUnity(command); //sends a messace every cycle to the client 192, 168, 4, 12
  //-----------------------------------------------------------------
}//end loop


void sendReadings(int testID) { //sends message to the right hand
  //Send Package to desired Ip
  Udp.beginPacket(ipCliente, 9999); //send package to the desired IP address
  Udp.printf("for right hand: ");
  char buf[20];   // buffer to hold the string to append
  //unsigned long testID = 2015;
  sprintf(buf, "%lu", (long unsigned int) testID);  // appending the testID to create a char
  Udp.printf(buf);  // send the char
  //sending words
  Udp.printf("\r\n");   // End segment
  Udp.endPacket();
  Serial.print("testID  ");
  Serial.println(testID);
}

//----------------------------------------------------------------
void sendReadingsUnity(int testID) { //sends message to UNITY
  //Send Package to desired Ip
  Udp.beginPacket(ipCliente3, 9999); //send package to the desired IP address
  Udp.printf("for UNITY : ");
  char buf[20];   // buffer to hold the string to append
  sprintf(buf, "%lu", (long unsigned int) testID);  // appending the testID to create a char
  //Udp.printf(buf);  // send the char
  Udp.write(testID);
  //sending words
  Udp.printf("\r\n");   // End segment
  Udp.endPacket();
  Serial.print("testID  ");
  Serial.println(testID);
}
