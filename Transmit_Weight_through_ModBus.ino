#include "LoadCellState.h"
#include "WebServer.h"
extern const int NUM_LOAD_CELLS;
extern LoadCellState cells[];
extern long readADSValueForMux(uint8_t mux, bool &ok);
extern long dampOutlierRaw(LoadCellState &cell, long newRaw);
extern long rollingAverage(LoadCellState &cell, long newVal);
extern void handleAutoTare(LoadCellState &cell, long smoothRaw);
extern unsigned long last_print_ms;
extern const unsigned long PRINT_INTERVAL_MS;
extern void writeFloatToHreg(int reg, float val, float val2);
extern void readModbusRegisters();
extern void writeWeightToHreg(int reg, float weight1, float weight2);
void writeTestWeightToHreg(int reg);
extern void serviceSerialCommands();
extern void getCellReadings(float *weights);
extern const int REG_FLOAT;
extern WebServer server;
extern void addReading(float lc1, float lc2);

void setup() {
  WT32_ETHSetup();
  LoadCellStateSetup();
}

void loop() {
  server.handleClient();
  for (int i = 0; i < NUM_LOAD_CELLS; i++) {
		bool ok = false;
		long raw = readADSValueForMux(cells[i].mux, ok);
		if (!ok) {
      Serial.println("failed to get ADS value for some reason");
			continue;
		}
		long dampedRaw = dampOutlierRaw(cells[i], raw);
		long smoothRaw = rollingAverage(cells[i], dampedRaw);
		cells[i].last_raw = smoothRaw;
		handleAutoTare(cells[i], smoothRaw);
	}
	if (millis() - last_print_ms >= PRINT_INTERVAL_MS) {
		last_print_ms = millis();
    float weights[2];
		getCellReadings(weights);
    int LC1_int_weight = weights[0];
    int LC2_int_weight = weights[1];
    addReading(LC1_int_weight, LC2_int_weight);
    //writeFloatToHreg(REG_FLOAT, weights[0], weights[1]);
    readModbusRegisters();
    
    if (opMode == 0){
    writeWeightToHreg(REG_FLOAT, weights[0], weights[1]);
    
	}
  }
  if (opMode == 1){
    writeTestWeightToHreg(REG_FLOAT);
    
  }
	serviceSerialCommands();  
}
