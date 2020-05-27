//Left hand code - touch interactions - magnetic sensing and communiaction with Head mounted Device
/* GOALS:
 * - Replace delay calls with a time dependent call 
 * - Make every Serial call compiled based off a defined symbol
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
int testID = 0;
char command = 0;
int aux_command = 0;
const int set_point = 20; //example value based on several online tutorials
int contador = 0; //variable to contain the steps to send the commands
char counter[255];
int T7_init = 0;

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
}

void loop() {
  Serial.print("T7  ");
  Serial.println(touchRead(T7));
  if ((T7_init - touchRead(T7) ) > set_point) {
    command = aux_command;
    aux_command = aux_command + 1;
  }
  if ((aux_command - command) > 0 && contador == 0) {
    testID = 1;
    sendReadings(testID);
    contador = 1;
    aux_command = 0;
    Serial.println(counter);
  }
  if ((aux_command - command) > 0 && contador == 1) {
    testID = 2;
    sendReadings(testID);
    contador = 2;
    aux_command = 0;
    Serial.println(counter);
  }
  if ((aux_command - command) > 0 && contador == 2) {
    testID = 3;
    sendReadings(testID);
    contador = 3;
    aux_command = 0;
    Serial.println(counter);
  }
  if ((aux_command - command) > 0 && contador == 3) {
    testID = 4;
    sendReadings(testID);
    contador = 4;
    aux_command = 0;
    Serial.println(counter);
  }
  //---------------------------------------------------------
  getReadings();
  delay(120); //needs to be removed
  //-------------------------------------------------------------
  //to make zero afer each touch
  if (contador == 4) {
    testID = 0;
    contador = 0;
    aux_command = 0;
  }
}//end of the Loop

//----------------------------------------------------------------------------------------------------------------

void sendReadings(int testID_f ) {
  Udp.beginPacket(ipServidor, 9999);  //Initiate transmission of data
  char buf[20];   // buffer to hold the string to append
  sprintf(buf, "%lu", (long unsigned int) testID_f);  // appending the millis to create a char
  Udp.printf(buf);  // print the char
  //sending words
  Udp.printf("\r\n");   // End segment
  Udp.endPacket();  // Close communication
  Serial.print("enviando: ");   // Serial monitor for user
  Serial.println(buf);
  delay(5); // needs to be removed
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
  delay(5); // needs to be removed
}
