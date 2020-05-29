//Left hand glove control - Server mode
//touch input and magnetic sensing
//touch input always available and magnetic sensing only available after calibration received from right hand.

/* GOALS:
 * - Replace delay calls with a time dependent call 
 * - Make every Serial call compiled based off a defined symbol
 * - Replace touch on/off elseif's with a more robust system  
 * - Replace touch multiple tolerances with an autocalibrated value 
 * - Replace/implement gesture recognization 
 */

// Just test touch pin - Touch0 is T0 which is on GPIO 4
/* Press, hold, slide - trans number
   1  - press T7
   2  - press T6
   3  - press T8
   4  - press T9
   5  - swipe from A to B
   6  - swipe from B to A
   7  - swipe from B to Y
   8  - swipe from Y to B
   9  - hold A
   10 - hold B
   11 - hold X
   12 - hold Y
*/
#include <WiFi.h>
#include <WiFiUdp.h>
//_____________________________________________________________________________________

//WiFi configuration
WiFiUDP Udp; // Creation of wifi Udp instance
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
const int set_point_h = 45; //value based from trial and error
const int set_point_l = 10; //value based from trial and error
int T9_init = 0;
int T8_init = 0;
int T7_init = 0;
int T6_init = 0;


//_____________________________________________________________________________________

void setup() {
  //wifi setup
  Serial.begin(115200);
  WiFi.softAP(ssid, password);  // ESP-32 as access point
  Udp.begin(localPort);
  //_____________________________________________________________________________________
  //touch sensing setup
  Serial.println("ESP32 Touch Test");
  //initial reading of the sensors
  Serial.print("Initial Values  ");
  T9_init = touchRead(T9); //this first reading is wrong - taken again at the end of the readings
  T8_init = touchRead(T8);
  Serial.print("T8 init ");
  Serial.println(T8_init);
  T7_init = touchRead(T7);
  Serial.print("T7 init ");
  Serial.println(T7_init);
  T6_init = touchRead(T6);
  Serial.print("T6 init ");
  Serial.println(T6_init);
  T9_init = touchRead(T9);
  Serial.print("T9 init  ");
  Serial.println(T9_init);
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
  /* NOTES:
   * Since the tolerance of the touchread conditions for the setting and clearing of the command variable are not the same for both, there exists a state between 
   * where the variable "command" isn't set or cleared.  This creates a schmitt trigger like performance. This may not be ideal since the tolerances are based on
   * arbitrary values and actual performance may cause the reading to not reach the tolerance.  Additionally switching to a single tolerance value and utilizing   
   * an actual switch case structure will simplify code.
   * The current setup and a switch case structure create a hierarchy which ranks setting and clearing from reading T7 at the top to reading from T9 last.  
   * This current setup doesn't handle multiple presses and is likely to cause issues, since you can easily accidently press a higher ranked pin/pad which alters
   * the command variable.  
   */
  //Switch Case based on sensor values, but with if else statements
  if ((T7_init - touchRead(T7) ) > set_point_h) {
    command = 1; //command[0] = 1; //this is for the wifi part
  } else if ((T6_init - touchRead(T6)) > set_point_h) {
    command = 2;//command[0] = 2;
  } else if ((T8_init - touchRead(T8)) > set_point_h) {
    command = 3; //command[0] = 3;
  } else if ((T9_init - touchRead(T9)) > set_point_h) {
    command = 4; //command[0] = 4;
  }
  Serial.print("command  ");
  Serial.println(command);
  sendReadings(command); //sends a messace every cycle to the client 192, 168, 4, 10
  sendReadingsUnity(command); //sends a messace every cycle to the client 192, 168, 4, 12
  delay(100);// needs to be removed
  //to make zero afer each touch
  if ((T7_init - touchRead(T7) ) < set_point_l) {  
    command = 0; //command[0] = 1; //this is for the wifi part
  } else if ((T6_init - touchRead(T6)) < set_point_l) {
    command = 0;//command[0] = 2;
  } else if ((T8_init - touchRead(T8)) < set_point_l) {
    command = 0; //command[0] = 3;
  } else if ((T9_init - touchRead(T9)) < set_point_l) {
    command = 0; //command[0] = 4;
  }
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
