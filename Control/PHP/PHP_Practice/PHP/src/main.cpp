#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino_JSON.h>

const char* ssid = "YIWEI_Laptop";
const char* password = "1234567890";

//Your IP address or domain name with URL path
const char* serverName = "http://146.169.212.155/get-temp.php?action=outputs_state";

char* DriveMode = "S";

String outputsState;

String httpGETRequest(const char* serverName) {
  WiFiClient client;
  HTTPClient http;
    
  // Your IP address with path or Domain name with URL path 
  http.begin(client, serverName);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "{}"; 
  
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) { 
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Check WiFi connection status
   
  if(WiFi.status()== WL_CONNECTED ){ 
    outputsState = httpGETRequest(serverName);
    //Serial.println(outputsState);
    JSONVar myObject = JSON.parse(outputsState);
    if (JSON.typeof(myObject) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }
      // Serial.print("JSON object = ");
      // Serial.println(myObject);
    
      // myObject.keys() can be used to get an array of all the keys in the object
      JSONVar keys = myObject.keys();
    //Below can be changed to design the instruction//
      for (int i = 0; i < keys.length(); i++) {
        JSONVar value = myObject[keys[i]];
        //Serial.println(i);
        if (i== (keys.length()-1) )
        {
        Serial.print("temp_id: ");
        Serial.print(keys[i]);
        Serial.print("temp_value: ");
        Serial.println(value);
        Serial.print("i is :");
        Serial.println(i);
        Serial.println(atoi(value));//Convert the JASON value into number
        //Below using the pointer to change the mode variable
        if (atoi(value) == 10)//Moving Forward
        {
          char* DriveMode = "F";
          Serial.print("Moving Forward");
        }else if (atoi(value) == 20)//Moving backward
        {
          char* DriveMode = "B";
        }else if (atoi(value) == 30)//Moving Left
        {
          char* DriveMode = "L";
        }else if (atoi(value) == 40)//Moving Right
        {
          char* DriveMode = "R";
        }else if (atoi(value) == 50)//Moving Back
        {
          char* DriveMode = "S";
        }
        }
      }
    }    
    else {
      Serial.println("WiFi Disconnected");
    }
    delay(10000);
}

