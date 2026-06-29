#include "LoadCellState.h"


CircularBuffer loadCell1_buffer;
CircularBuffer loadCell2_buffer;

void pushSample(CircularBuffer& buf, float value)
{
    buf.data[buf.head] = value;

    buf.head = (buf.head + 1) % MAX_SAMPLES_DEQUE;

    if (buf.count < MAX_SAMPLES_DEQUE) {
        buf.count++;
    }
}

float getSample(const CircularBuffer& buf, size_t index)
{
    if (index >= buf.count) {
        return NAN;
    }

    size_t oldest =
        (buf.head + MAX_SAMPLES_DEQUE - buf.count) % MAX_SAMPLES_DEQUE;

    size_t pos =
        (oldest + index) % MAX_SAMPLES_DEQUE;

    return buf.data[pos];
}

void printBuffer(const CircularBuffer& buf, const char* label)
{
    Serial.printf("%s [%d samples]: ",
                  label,
                  (int)buf.count);

    for (size_t i = 0; i < buf.count; i++) {
        Serial.printf("%.3f ", getSample(buf, i));
    }

    Serial.println();
}

void addReading(float lc1, float lc2)
{
    pushSample(loadCell1_buffer, lc1);
    pushSample(loadCell2_buffer, lc2);

    //printBuffer(loadCell1_buffer, "LC1");
    //printBuffer(loadCell2_buffer, "LC2");
}