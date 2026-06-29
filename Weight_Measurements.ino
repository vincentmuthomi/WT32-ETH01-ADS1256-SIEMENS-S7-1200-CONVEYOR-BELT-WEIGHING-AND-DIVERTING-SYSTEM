#include <SPI.h>
#include <Arduino.h>
#include "LoadCellState.h"
// --- PIN MAPPING FOR WT32-ETH01 ---
#define ADS_SCLK 14
#define ADS_DIN 15   // MOSI
#define ADS_DOUT 2   // MISO
#define ADS_CS 4
#define ADS_DRDY 36

// --- ADS1256 REGISTER DEFINITIONS ---
#define REG_STATUS 0x00
#define REG_MUX 0x01
#define REG_ADCON 0x02
#define REG_DRATE 0x03

// --- ADS1256 COMMANDS ---
#define CMD_WAKEUP 0x00
#define CMD_RDATA 0x01
#define CMD_SELFCAL 0xF0
#define CMD_SYNC 0xFC
#define CMD_RESET 0xFE

// PARAMS
const int NUM_LOAD_CELLS = 2;
const float OUTLIER_THRESHOLD_G = 3.0f;
const unsigned long STARTUP_SETTLE_MS = 2000;
const int STARTUP_TARE_SAMPLES = 30;
const unsigned long PRINT_INTERVAL_MS = 800;
const float DEFAULT_CALIBRATION_FACTOR_LC1 = -300.14f;
const float DEFAULT_CALIBRATION_FACTOR_LC2 = 300.14f;
const uint32_t ADS_SPI_HZ = 1900000;
const uint8_t ADS_DRATE_VALUE = 0xA1;  
const uint32_t ADS_PER_CHANNEL_TIMEOUT_MS = 8;
const uint8_t ADS_DISCARD_AFTER_MUX = 3;
const uint16_t ADS_REG_WRITE_DELAY_US = 10;
const uint32_t SERIAL_READ_TIMEOUT_MS = 3;
void writeTestWeightToHreg(int reg);
LoadCellState cells[NUM_LOAD_CELLS] = {
	{"LC1", 0x01, DEFAULT_CALIBRATION_FACTOR_LC1, 0, false, 1000.0f, 1.05f, {0}, 0, 0, 0, 0, 0, 0, {0}, 0, 0},
	{"LC2", 0x23, DEFAULT_CALIBRATION_FACTOR_LC2, 0, false, 1000.0f, 1.05f, {0}, 0, 0, 0, 0, 0, 0, {0}, 0, 0}
};

unsigned long last_print_ms = 0;

void initADS1256();
void writeReg(uint8_t reg, uint8_t val);
bool waitDRDY(uint32_t timeoutMs);
bool startConversionAndWait(uint32_t timeoutMs);
long readADSData24();
long readADSValueForMux(uint8_t mux, bool &ok);
long dampOutlierRaw(LoadCellState &cell, long newRaw);
long rollingAverage(LoadCellState &cell, long newVal);
void handleAutoTare(LoadCellState &cell, long smoothRaw);
float computeWeight(const LoadCellState &cell, long smoothRaw);
void printCellReadings();
void serviceSerialCommands();
void printCommandHelp();
int parseCellIndex(char token);
void applyManualTare(int cellIndex);
void showCalibration(int cellIndex);

void LoadCellStateSetup() {
	Serial.begin(115200);
	Serial.setTimeout(SERIAL_READ_TIMEOUT_MS);

	pinMode(ADS_CS, OUTPUT);
	pinMode(ADS_DRDY, INPUT);

	delay(100);

	// Initialize SPI for WT32-ETH01
	SPI.begin(ADS_SCLK, ADS_DOUT, ADS_DIN, ADS_CS);
	SPI.beginTransaction(SPISettings(ADS_SPI_HZ, MSBFIRST, SPI_MODE1));

	initADS1256();

	Serial.println("ADS1256 Ready (2 load cells).");
	Serial.println("Keep both scales unloaded for auto-tare...");
	printCommandHelp();
}

//void loop() {

//}

// --- ADS1256 CONFIGURATION ---
void initADS1256() {
	digitalWrite(ADS_CS, LOW);
	SPI.transfer(CMD_RESET);
	digitalWrite(ADS_CS, HIGH);
	delay(5);
	waitDRDY(500);

	// ACAL off so MUX switching between channels does not trigger auto-cal every time.
	writeReg(REG_STATUS, 0x00);

	// Start on load cell 1 input pair by default.
	writeReg(REG_MUX, cells[0].mux);

	// PGA gain 64 for load cells.
	writeReg(REG_ADCON, 0x06);

	// Increase sample rate to reduce latency when scanning two channels.
	writeReg(REG_DRATE, ADS_DRATE_VALUE);

	// Run one full self-calibration at startup.
	digitalWrite(ADS_CS, LOW);
	SPI.transfer(CMD_SELFCAL);
	digitalWrite(ADS_CS, HIGH);

	waitDRDY(1000);
}

void writeReg(uint8_t reg, uint8_t val) {
	digitalWrite(ADS_CS, LOW);
	SPI.transfer(0x50 | reg);  // WREG opcode
	SPI.transfer(0x00);        // Write 1 register
	SPI.transfer(val);
	digitalWrite(ADS_CS, HIGH);
	delayMicroseconds(ADS_REG_WRITE_DELAY_US);
}

bool waitDRDY(uint32_t timeoutMs) {
	uint32_t startMs = millis();
	while (digitalRead(ADS_DRDY) != LOW) {
		if ((millis() - startMs) > timeoutMs) {
			return false;
		}
	}
	return true;
}

bool startConversionAndWait(uint32_t timeoutMs) {
	digitalWrite(ADS_CS, LOW);
	SPI.transfer(CMD_SYNC);
	SPI.transfer(CMD_WAKEUP);
	digitalWrite(ADS_CS, HIGH);

	return waitDRDY(timeoutMs);
}

long readADSData24() {
	digitalWrite(ADS_CS, LOW);
	SPI.transfer(CMD_RDATA);
	delayMicroseconds(10);

	uint8_t h = SPI.transfer(0xFF);
	uint8_t m = SPI.transfer(0xFF);
	uint8_t l = SPI.transfer(0xFF);
	digitalWrite(ADS_CS, HIGH);

	long val = ((long)h << 16) | ((long)m << 8) | (long)l;
	if (val & 0x800000) {
		val |= 0xFF000000;  // Sign extension for 24-bit value
	}

	return val;
}

long readADSValueForMux(uint8_t mux, bool &ok) {
	writeReg(REG_MUX, mux);

	long val = 0;
	// First conversion after MUX change can contain residue from prior channel.
	for (uint8_t i = 0; i <= ADS_DISCARD_AFTER_MUX; i++) {
		if (!startConversionAndWait(ADS_PER_CHANNEL_TIMEOUT_MS)) {
			ok = false;
			return 0;
		}

		val = readADSData24();
	}

	ok = true;
	return val;
}

long dampOutlierRaw(LoadCellState &cell, long newRaw) {
	if (cell.outlier_count == 0) {
		cell.outlier_samples[0] = newRaw;
		cell.outlier_count = 1;
		cell.outlier_idx = 1;
		return newRaw;
	}

	long sum = 0;
	for (int i = 0; i < cell.outlier_count; i++) {
		sum += cell.outlier_samples[i];
	}
	long baseline = sum / cell.outlier_count;

	float calib = cell.calibration_factor;
	if (calib < 0.0f) {
		calib = -calib;
	}
	if (calib < 1.0f) {
    if (cell.name == "LC1") calib = DEFAULT_CALIBRATION_FACTOR_LC1;
    if (cell.name == "LC2") calib = DEFAULT_CALIBRATION_FACTOR_LC2;
	}

	long thresholdCounts = (long)(calib * OUTLIER_THRESHOLD_G);
	if (thresholdCounts < 1) {
		thresholdCounts = 1;
	}

	long delta = newRaw - baseline;
	long damped = newRaw;
	if (delta > thresholdCounts) {
		damped = baseline + thresholdCounts;
	} else if (delta < -thresholdCounts) {
		damped = baseline - thresholdCounts;
	}

	cell.outlier_samples[cell.outlier_idx] = damped;
	cell.outlier_idx = (cell.outlier_idx + 1) % OUTLIER_WINDOW_SAMPLES;
	if (cell.outlier_count < OUTLIER_WINDOW_SAMPLES) {
		cell.outlier_count++;
	}

	return damped;
}

long rollingAverage(LoadCellState &cell, long newVal) {
	if (cell.sample_count < AVG_SAMPLES) {
		cell.sample_count++;
	} else {
		cell.sample_sum -= cell.samples[cell.sample_idx];
	}

	cell.samples[cell.sample_idx] = newVal;
	cell.sample_sum += newVal;
	cell.sample_idx = (cell.sample_idx + 1) % AVG_SAMPLES;

	return cell.sample_sum / cell.sample_count;
}

void handleAutoTare(LoadCellState &cell, long smoothRaw) {
	if (cell.is_tared || millis() <= STARTUP_SETTLE_MS) {
		return;
	}

	cell.tare_collect_sum += smoothRaw;
	cell.tare_collect_count++;

	if (cell.tare_collect_count >= STARTUP_TARE_SAMPLES) {
		cell.tare_offset = cell.tare_collect_sum / STARTUP_TARE_SAMPLES;
		cell.is_tared = true;

		Serial.print(cell.name);
		Serial.print(" auto-tare complete. Offset=");
		Serial.println(cell.tare_offset);
	}
}

/*float computeWeight(const LoadCellState &cell, long smoothRaw) {
	if (cell.calibration_factor == 0.0f) {
		return 0.0f;
	}

	float weight = ((float)(smoothRaw - cell.tare_offset)) / cell.calibration_factor;
	if (weight < 0.0f && weight > -5.0f) {
		weight = 0.0f;
	}

	return weight;
}*/

float computeWeight(LoadCellState &cell, long smoothRaw) {
	if (cell.calibration_factor == 0.0f) {
		return 0.0f;
	}

	float weight = ((float)(smoothRaw - cell.tare_offset)) / cell.calibration_factor;

	// Auto-tare whenever weight drops below zero
	if (weight < -1.0f) {
		cell.tare_offset = smoothRaw;
		weight = 0.0f;

		Serial.print(cell.name);
		Serial.println(" auto-tared (negative weight detected)");
	}

	return weight;
}

void getCellReadings(float *weights) {
  float total = 0;
	for (int i = 0; i < NUM_LOAD_CELLS; i++) {
		LoadCellState &cell = cells[i];

		// if (i > 0) {
		// 	Serial.print(" || ");
		// }

//		Serial.print(cell.name);

		if (!cell.is_tared) {
			Serial.print(" | taring...");
			continue;
		}

		float weight = computeWeight(cell, cell.last_raw);
    weights[i] = weight;
    total += weight;
		// Serial.print(" | Weight:");
		Serial.print(weight, 2);
		// Serial.print(" g");
    if(i==0) Serial.print(",");
		if (weight > cell.capacity_g * cell.overload_margin) {
			Serial.print(" | WARNING: OVERLOAD");
		}
	}
  Serial.print(",");
  Serial.print(total);

	Serial.println();
}





int parseCellIndex(char token) {
	if (token == '1') {
		return 0;
	}
	if (token == '2') {
		return 1;
	}
	return -1;
}

void applyManualTare(int cellIndex) {
	LoadCellState &cell = cells[cellIndex];
	cell.tare_offset = cell.last_raw;
	cell.is_tared = true;
	cell.tare_collect_count = 0;
	cell.tare_collect_sum = 0;

	Serial.print(cell.name);
	Serial.print(" manual tare set. Offset=");
	Serial.println(cell.tare_offset);
}

void showCalibration(int cellIndex) {
	Serial.print(cells[cellIndex].name);
	Serial.print(" calibration factor (counts/g): ");
	Serial.println(cells[cellIndex].calibration_factor, 6);
}

void printCommandHelp() {
	Serial.println("Commands:");
	Serial.println("  z1 / z2               -> manual tare load cell 1 or 2");
	Serial.println("  c1 <grams> / c2 <grams> -> calibrate load cell 1 or 2");
	Serial.println("  f, f1, f2             -> show calibration factor(s)");
	Serial.print("  Filter: avg=");
	Serial.print(AVG_SAMPLES);
	Serial.print(", outlier clamp=");
	Serial.print(OUTLIER_THRESHOLD_G, 1);
	Serial.print("g over last ");
	Serial.print(OUTLIER_WINDOW_SAMPLES);
	Serial.println(" samples");
	Serial.print("  ADS: 1000SPS, discard-after-mux=");
	Serial.println(ADS_DISCARD_AFTER_MUX);
	Serial.println("  h                     -> show this help");
}

void serviceSerialCommands() {
	if (!Serial.available()) {
		return;
	}

	String cmd = Serial.readStringUntil('\n');
	cmd.trim();

	if (cmd.length() == 0) {
		return;
	}

	char op = cmd.charAt(0);

	if (op == 'z' || op == 'Z') {
		if (cmd.length() < 2) {
			Serial.println("Usage: z1 or z2");
			return;
		}

		int cellIndex = parseCellIndex(cmd.charAt(1));
		if (cellIndex < 0) {
			Serial.println("Error: use z1 or z2");
			return;
		}

		applyManualTare(cellIndex);
		return;
	}
  if (op == 'm'){
    int cellIndex = parseCellIndex(cmd.charAt(1));
    if (cellIndex == 0) opMode = 0;
    if(cellIndex == 1) opMode = 1;
  }
  if (op == 's'){
    
    int cellIndex = parseCellIndex(cmd.charAt(1));
    transmittedWeight = 70;
    if (cellIndex == 0) {
      transmittedWeight = 78;
      weightDropped = 1;
      writeTestWeightToHreg(REG_FLOAT);
      Serial.println("-------------->");
      }
    if (cellIndex == 1) {
      transmittedWeight = 25;
      weightDropped = 1;
      Serial.println("<--------------");
      writeTestWeightToHreg(REG_FLOAT);
      }
    if (cellIndex == -1) {
      String gramsTransmit = cmd.substring(2);
      gramsTransmit.trim();
      float gramsTransmitted = gramsTransmit.toFloat();
      transmittedWeight = gramsTransmitted;
      if(gramsTransmitted != 0){
        Serial.print(gramsTransmitted);
        writeTestWeightToHreg(REG_FLOAT);
      Serial.println("<------<->------->");
      }
      else {
        Serial.println("<-------------->");
    
      }
      }
    return;
  }
	if (op == 'f' || op == 'F') {
		if (cmd.length() == 1) {
			showCalibration(0);
			showCalibration(1);
			return;
		}

		int cellIndex = parseCellIndex(cmd.charAt(1));
		if (cellIndex < 0) {
			Serial.println("Error: use f, f1 or f2");
			return;
		}

		showCalibration(cellIndex);
		return;
	}

	if (op == 'c' || op == 'C') {
		if (cmd.length() < 3) {
			Serial.println("Usage: c1 <grams> or c2 <grams>");
			return;
		}

		int cellIndex = parseCellIndex(cmd.charAt(1));
		if (cellIndex < 0) {
			Serial.println("Error: use c1 <grams> or c2 <grams>");
			return;
		}

		if (!cells[cellIndex].is_tared) {
			Serial.println("Error: tare first using z1 or z2");
			return;
		}

		String gramsToken = cmd.substring(2);
		gramsToken.trim();
		float known_g = gramsToken.toFloat();

		if (known_g <= 0.0f) {
			Serial.println("Error: use positive grams. Example: c1 500");
			return;
		}

		long delta = cells[cellIndex].last_raw - cells[cellIndex].tare_offset;
		if (delta == 0) {
			Serial.println("Error: no change detected. Place known mass first.");
			return;
		}

		cells[cellIndex].calibration_factor = (float)delta / known_g;
		Serial.print(cells[cellIndex].name);
		Serial.print(" new calibration factor (counts/g): ");
    
		Serial.println(cells[cellIndex].calibration_factor, 6);
    delay(3000);
		return;
	}

	if (op == 'h' || op == 'H' || op == '?') {
		printCommandHelp();
		return;
	}

	Serial.println("Unknown command. Use h for help.");
}