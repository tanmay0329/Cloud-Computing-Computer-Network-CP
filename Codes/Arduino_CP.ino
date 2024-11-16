#include <Wire.h>
#include <WebSockets2_Generic.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h> 
#include <ArduinoJson.h>
#define DEBUG_WEBSOCKETS_PORT Serial
#define WEBSOCKETS_LOGLEVEL 3
const char* ssid = "VIVO V25 PRO";
const char* password = "12341234";
const char* websockets_server_host = "192.168.175.109";
#define WEBSOCKETS_PORT 8080
const uint16_t websockets_server_port = WEBSOCKETS_PORT;
#define LED 2
using namespace websockets2_generic;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
// DynamicJsonDocument doc(1024);
StaticJsonDocument<1024> doc;

WebsocketsClient client;
uint32_t previous_time = 0;
uint32_t current_time = 0;
int yaw = 0;
int prev_val = 0;
int counter = 0;
int z = 0;
long timer = 0;
// char str[6];
char Zstr[6];
char Ystr[6];
String str;
String str2;
void onEventsCallback(WebsocketsEvent event, String data) {
  (void)data;

  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("Connnection Opened");
    digitalWrite(LED, HIGH);
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("Connnection Closed");
  } else if (event == WebsocketsEvent::GotPing) {
    Serial.println("Got a Ping!");
  } else if (event == WebsocketsEvent::GotPong) {
    Serial.println("Got a Pong!");
  }
}
void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(38400);
  Serial1.begin(38400, SERIAL_8N1, 18, 19);
  Serial2.begin(38400);
  Serial.println("Orientation Sensor Test");
  Serial.println("");

  /* Initialise the sensor */
  Serial.print("HELLo");
  if (!bno.begin()) {
    while(1);
  }
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    // WiFi.begin(ssid, password);
    Serial.print(".");
    delay(1000);
  }

  if (WiFi.status() != WL_CONNECTED) {  // Here we are checking if the client esp32 is connected to wifi or not
    Serial.println("No Wifi!");
    ESP.restart();
  }

  Serial.print("Connected to Wifi, Connecting to WebSockets Server @");
  Serial.println(websockets_server_host);
  client.onMessage([&](WebsocketsMessage message) {
    Serial.print("Got Message: ");
    Serial.println(message.data());
  });
  client.onEvent(onEventsCallback);

  bool connected = client.connect(websockets_server_host, websockets_server_port, "/BNO");
  delay(1000);
}

void loop() {
  
  // sensors_event_t orientationData;
  // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // sensors_event_t* event = &orientationData;
  
  // // if ((millis() - timer) > 20)
  // if (1) 
  // {
  //   yaw = event->orientation.x;
  //   if (yaw - prev_val <= -300) counter++;
  //   else if (yaw - prev_val >= 300) counter--;
  //   z = (yaw % 360) + 360 * counter;
  //   z = -z;
  //   int Z_val = (int)(z);
  //   int Y_val = Z_val+10;
  //   sprintf(Ystr, "%05d", Y_val);
    
  //   sprintf(Zstr, "%05d", Z_val);
  //   doc["Z"] = (String)Zstr;
  //    doc["Y"] =(String)Ystr  ;
    
  //   serializeJson(doc, str);
  //   // str = str + "*";
  //   // client.send("HII");
  //   client.send(str);
  //    Serial.println(str);
  //   timer = millis();
  //   prev_val = yaw;
  // }

  //   str.clear();
  //   Serial2.print(str);
  // Serial2.flush();
  // delay(50);

  doc["Z"] = String("10");
  doc["Y"] = String("15");
  serializeJson(doc, str);
  client.send(str);
  delay(20);

}
