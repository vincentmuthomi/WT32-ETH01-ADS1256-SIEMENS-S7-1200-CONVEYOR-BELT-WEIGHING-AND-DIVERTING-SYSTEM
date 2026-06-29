#pragma once
#include <ETH.h>
#include <WiFi.h>
#include <ModbusIP_ESP8266.h>
#include <WebServer.h>
ModbusIP mb;

constexpr int AVG_SAMPLES = 15;
constexpr int OUTLIER_WINDOW_SAMPLES = 5;
const size_t MAX_SAMPLES_DEQUE = 10;
float transmittedWeight = 30;
float weightDropped = 0;
int opMode = 0;
struct LoadCellState {
    const char *name;
    uint8_t mux;
    float calibration_factor;
    long tare_offset;
    bool is_tared;
    float capacity_g;
    float overload_margin;

    long samples[AVG_SAMPLES];
    int sample_idx;
    int sample_count;
    long sample_sum;

    int tare_collect_count;
    long tare_collect_sum;
    long last_raw;

    long outlier_samples[OUTLIER_WINDOW_SAMPLES];
    int outlier_idx;
    int outlier_count;
};
struct CircularBuffer {
    float data[MAX_SAMPLES_DEQUE];
    size_t head = 0;   // next write position
    size_t count = 0;  // number of valid samples
};