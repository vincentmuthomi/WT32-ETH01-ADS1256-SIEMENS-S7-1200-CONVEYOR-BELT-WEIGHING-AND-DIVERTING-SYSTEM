#include "LoadCellState.h"


// LC1 callibration factor = -334
// Static IP configuration
IPAddress local_IP(192, 168, 0, 88);   
IPAddress gateway(192, 168, 0, 1);     
IPAddress subnet(255, 255, 255, 0);

WiFiServer server(23);
const int REG_FLOAT = 0;
const char* ssid ="Cypher";
const char* password = "11223344";
float value = 0.01;
float value2 = 1000.0;
int weightDroppedOnInterval = 5000;
int lastWeightDrop = 0;
void readModbusRegisters(){
  int value;
  mb.task();
    //if (mb.readHreg(reg, &value, 1)) {
        // Serial.print("Register 0 = ");
        // Serial.println(value);
    // } else {
        // Serial.println("Read failed");
    // }

}
void writeFloatToHreg(int reg, float val, float val2) {
  union {
    float f;
    uint16_t w[2];
  } data;

  union {
    float f;
    uint16_t w[2];
  } data2;

  data.f = val;
  data2.f = val2;

  // mb.Hreg(reg + 0, data.w[1]);     // High word
  // Serial.print("HIGH W1: ");
  // Serial.print(data.w[0]);
  // Serial.print(" - HIGH W2 ");
  // Serial.print(data2.w[0]);
  // Serial.print(" -||- ");

 // mb.Hreg(reg + 1, data.w[0]); // Low word
  // Serial.print("LOW W1: ");
  // Serial.print(data.w[1]);
  // Serial.print(" - LOW W2: ");

  //mb.Hreg(reg + 2, data2.w[1]);
  //mb.Hreg(reg + 3, data2.w[0]);
  // Serial.print(data2.w[1]);
  // Serial.print(" -||- ");
  // Serial.print("ACTUAL VAL1: ");
  // Serial.print(val);
  // Serial.print(" - VAL2: ");
  // Serial.print(val2);
  // Serial.println(" ");
  mb.task();
}
int gyration_val = 0;
float getGyroValue(float primal, float tolerance) {
    float out = primal + tolerance * sin(gyration_val * PI / 180.0);
    gyration_val++;
    if (gyration_val >= 360)
        gyration_val = 0;

    return out;
}
float getTol(float valTTol,float Threshold1, float Threshold2){
  if (valTTol < Threshold1){
    float tol = min(Threshold1-valTTol, valTTol);
    return tol;
  }
  if (valTTol >= Threshold1 && valTTol <= Threshold2){
    float tol = min(valTTol - Threshold1, Threshold2 - valTTol);
    return tol;
  } 
  else{
  return 0;}
}
void writeTestWeightToHreg(int reg){
  
if ((millis() - lastWeightDrop >= weightDroppedOnInterval)) {
      weightDropped = 0;
      lastWeightDrop = millis();
      transmittedWeight = 0;
    }  
    //float tol = getTol(transmittedWeight, 50, 100);
    //float toTransmit = getGyroValue(transmittedWeight,tol/2);
    //Serial.print("transmitted Weight: ");
    //Serial.println(toTransmit);  
    //mb.Hreg(reg+0, toTransmit);
    mb.Hreg(reg+0, transmittedWeight);
    mb.Hreg(reg+2, weightDropped);
    mb.task();
    
}

void writeWeightToHreg(int reg, float weight1, float weight2){
    
  int roundedWeight = (ceil(weight1) + ceil(weight2))/2;

    mb.Hreg(reg+0, roundedWeight);
    mb.Hreg(reg+2, weightDropped);
  
  mb.task();
}

void handleRoot(){
   String msg = "Hello from WT32-ETH01!\n";
   msg += "ETH IP: " + ETH.localIP().toString() + "\n";
   msg += "WiFi IP: " + WiFi.localIP().toString();
   
  server.send(200, "text/plain", msg);
}
void WT32_ETHSetup() {
  Serial.begin(115200);
  
  ETH.begin();

  WiFi.begin(ssid, password);
  if (!ETH.config(local_IP, gateway, subnet)) {
    Serial.println("Static IP configuration failed!");
  }

  // Wait for link
  Serial.println("Waiting for ethernet or wifi link!");
  while (!ETH.linkUp() && WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }


  Serial.print("ETH Static IP: ");
  Serial.println(ETH.localIP());
  Serial.print("Wifi Ip: ");
  Serial.println(WiFi.localIP());
  delay(5000);
  // Start Modbus TCP
  
  //server.on("/", handleRoot);
  server.begin();

  Serial.print("Waiting for ethernet link");
  while (!ETH.linkUp() ){
    server.handleClient();
    delay(100);
    
  }
  mb.server();

  mb.addHreg(REG_FLOAT, 0);
  mb.addHreg(REG_FLOAT + 1, 0);
  mb.addHreg(REG_FLOAT + 2, 0);
  mb.addHreg(REG_FLOAT + 3, 0);  
}
