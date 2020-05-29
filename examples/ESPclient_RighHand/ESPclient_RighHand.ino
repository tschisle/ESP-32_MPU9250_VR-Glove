//Left hand code - touch interactions - magnetic sensing and communiaction with Head mounted Device
/* GOALS:
   - Make every Serial call compiled based off a defined symbol
   - Add the sensor code
*/

#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP Udp;  // Creation of wifi Udp instance
char packetBuffer[255];
unsigned int localPort = 9999;
const char *ssid = "NETGEAR36-5G"; //seems like personal information
const char *password = "pastelstreet317"; //seems like personal information
IPAddress ipServidor (192, 168, 4, 1);   // Declaration of default IP for server
/*
    The ip address of the client has to be different to the server
    other wise it will conflict because the client tries to connect
    to it self.
*/
IPAddress ipCliente (192, 168, 4, 10);   // Different IP than server
//IPAddress ipCliente3(192, 168, 1, 8);  //UNITY client ip
IPAddress Subnet(255, 255, 255, 0);
const int set_point = 20; //example value based on several online tutorials
int command = 0; //variable that contains the step
char counter[255];
int T7_init = 0;
int touch_reading;

//delay timers -
unsigned long touch_timer;
int touch_timer_length = 125; //may cause issues with mismatched data types
int readings_timer_offset = 5; //may cause issues with mismatched data types
bool readings_flag = false; //this is done to avoid any case were the loop takes longer than a millisecond to complete

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  WiFi.mode(WIFI_STA); // ESP-32 as client
  WiFi.config(ipCliente, ipServidor, Subnet);
  Udp.begin(localPort);
  // capacitive touch
  Serial.println("ESP32 Touch Test");
  //initial reading of the sensors
  Serial.print("Initial Values  ");
  T7_init = touchRead(T7);
  Serial.print("T7 init ");
  Serial.println(T7_init);
  delay(600);
  //Timer Initializations
  touch_timer = millis();
}

void loop() {
  if (touch_timer <= (millis() - touch_timer_length)) { //checks the touch pin every 120 milliseconds
    touch_reading = touchRead(T7);
    Serial.print("T7  ");
    Serial.println(touch_reading);
    if ((T7_init - touch_reading ) > set_point) {
      switch (command) {
        case 4:
          command = 0;
          break;
        case 3:
          sendReadings(4);
          command = 4;
          Serial.println(counter);
          break;
        case 2:
          sendReadings(3);
          command = 3;
          Serial.println(counter);
          break;
        case 1:
          sendReadings(2);
          command = 2;
          Serial.println(counter);
          break;
        default:
          sendReadings(1);
          command = 1;
          Serial.println(counter);
      }
    }
    touch_timer = millis();
    readings_flag = false;
  }
  //---------------------------------------------------------
  if ((touch_timer <= (millis() - readings_timer_offset)) && (!readings_flag)) {
    getReadings();
    readings_flag = true;
  }
}//end of the Loop

//----------------------------------------------------------------------------------------------------------------

void sendReadings(int testID ) {
  Udp.beginPacket(ipServidor, 9999);  //Initiate transmission of data
  char buf[20];   // buffer to hold the string to append
  sprintf(buf, "%lu", (long unsigned int) testID);  // appending the millis to create a char
  Udp.printf(buf);  // print the char
  //sending words
  Udp.printf("\r\n");   // End segment
  Udp.endPacket();  // Close communication
  Serial.print("enviando: ");   // Serial monitor for user
  Serial.println(buf);
}

void getReadings() {
  int packetSize = Udp.parsePacket();   // Size of packet to receive
  if (packetSize) {       // If we received a package
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len - 1] = 0;
    Serial.print("RECIBIDO(IP/Port/Size/Datos11111): ");
    Serial.print(Udp.remoteIP()); Serial.print(" / ");
    Serial.print(Udp.remotePort()); Serial.print(" / ");
    Serial.print(packetSize); Serial.print(" / ");
    Serial.println(packetBuffer);
  }
  Serial.println("");
}
