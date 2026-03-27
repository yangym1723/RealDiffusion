#include <Adafruit_MCP4728.h>
#include <Wire.h>
#include <ArduinoJson.h>

#define CV 0.819
const int BUFFER_SIZE = 128;
char buffer[BUFFER_SIZE];
Adafruit_MCP4728 dac;
int value1, value2, value3, value4;

bool serial_output = true;
void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!dac.begin()) {
    debug("init Failed!");
  }
  value1 = 0;
  value2 = 0;
  value3 = 0;
  value4 = 0;
}

void loop() {
  int i = 0;
  StaticJsonDocument<64> doc;
  if (Serial.available() > 0) {
    String s = Serial.readStringUntil('\n');
    DeserializationError error = deserializeJson(doc, s);
    if (error) {
      debug("deserializeJson failed");
      return;
    }
    int cmd = doc["cmd"];
    if (cmd == 1) {
      doc["type"] = "param";
      doc["d1"] = value1;
      doc["d2"] = value2;
      doc["d3"] = value3;
      doc["d4"] = value4;
      serializeJson(doc, Serial);

    } else if (cmd == 2) {
      int d1 = doc["d1"];
      int d2 = doc["d2"];
      int d3 = doc["d3"];
      int d4 = doc["d4"];

      value1 = d1;
      value2 = d2;
      value3 = d3;
      value4 = d4;
      
      if (d1 != 9999) {
          d1 = constrain(int(CV * d1), 0, 4095);
          dac.setChannelValue(MCP4728_CHANNEL_A, d1);
      }
      if (d2 != 9999) {
          d2 = constrain(int(CV * d2), 0, 4095);
          dac.setChannelValue(MCP4728_CHANNEL_B, d2);
      }      
      if (d3 != 9999) {
          d3 = constrain(int(CV * d3), 0, 4095);
          dac.setChannelValue(MCP4728_CHANNEL_C, d3);
      }
      if (d4 != 9999) {
          d4 = constrain(int(CV * d4), 0, 4095);
          dac.setChannelValue(MCP4728_CHANNEL_D, d4);
      }      

      // dac.fastWrite(d1,d2,d3,d4);
      //dac.fastWrite(int(CV * d1), int(CV * d2), int(CV * d3), int(CV * d4));
      debug("done");


    } else {
      Serial.println("Error parsing JSON data");
      return;
    }
  }
}

void debug(int value) {
  if (!serial_output) {
    return;
  }
  StaticJsonDocument<50> param;
  param["type"] = "debug";
  param["value"] = value;
  serializeJson(param, Serial);
}
void debug(String msg) {
  if (!serial_output) {
    return;
  }
  StaticJsonDocument<50> param;
  param["type"] = "debug";
  param["msg"] = msg;
  serializeJson(param, Serial);
}

void debug(String msg, int value) {
  if (!serial_output) {
    return;
  }
  StaticJsonDocument<50> param;
  param["type"] = "debug";
  param["msg"] = msg;
  param["value"] = value;
  serializeJson(param, Serial);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  float value = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  value = constrain(value, out_min, out_max);
  return value;
}