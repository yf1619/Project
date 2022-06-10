#include <WiFi.h>
#include <PubSubClient.h>
 
// 设置wifi接入信息(请根据您的WiFi信息进行修改)
const char* ssid = "YIWEI_Laptop";
const char* password = "1234567890";
const char* mqttServer = "test.mosquitto.org";//This is the address of the server
// 如以上MQTT服务器无法正常连接，请前往以下页面寻找解决方案
// http://www.taichi-maker.com/public-mqtt-broker/
 
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
 
void setup() {
  Serial.begin(115200);
 
  //设置ESP8266工作模式为无线终端模式
  WiFi.mode(WIFI_STA);
  
  // 连接WiFi
  connectWifi();
  
  // 设置MQTT服务器和端口号
  mqttClient.setServer(mqttServer, 1883);
 
  // 连接MQTT服务器
  connectMQTTServer();
}
 
void loop() { //recommend not using delap in the loop due to the inturbance of using loop() to keep alive, 
  if (mqttClient.connected()) { // 如果开发板成功连接服务器  //check whether is connected, but not implement connection action//  
    mqttClient.loop();          // 保持客户端心跳 //  recommend not using delay in the loop
  } else {                  // 如果开发板未能成功连接服务器
    connectMQTTServer();    // 则尝试连接服务器
  }
}
 
void connectMQTTServer(){
  // 根据ESP8266的MAC地址生成客户端ID（避免与其它ESP8266的客户端ID重名）
  String clientId = "esp32-" + WiFi.macAddress();
 
  // 连接MQTT服务器
  if (mqttClient.connect(clientId.c_str())) { // implement connection,// if connect successfully, it will return true.
    Serial.println("MQTT Server Connected.");
    Serial.println("Server Address: ");
    Serial.println(mqttServer);
    Serial.println("ClientId:");
    Serial.println(clientId);
  } else {
    Serial.print("MQTT Server Connect Failed. Client State:");
    Serial.println(mqttClient.state());
    delay(3000);
  }   
}
 
// ESP8266连接wifi
void connectWifi(){
 
  WiFi.begin(ssid, password);
 
  //等待WiFi连接,成功连接后输出成功信息
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Connected!");  
  Serial.println(""); 
}