/* Includes ---------------------------------------------------------------- */
#include <WiFi.h>
#include <HardwareSerial.h>
#include <fan_v2_inferencing.h>

#include <Arduino.h>
#include "wifi_function.h"

/*----------------- INA3221 Section ----------------*/
#include "ina3221.h"
#define OUTPUT_CHANNEL 3

INA3221 ina3221;
/*-------------------------------------------------*/

/*----------------- ADXL345 Section ---------------*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified adxl345 = Adafruit_ADXL345_Unified(12345);
/*-------------------------------------------------*/

/* Constant defines -------------------------------------------------------- */
#define TX 17
#define RX 18

#define SDA_PIN 21
#define SCL_PIN 46

/** Number sensor axes used */
#define N_SENSORS 4

bool init_IMU(void);
bool init_ADC(void);
uint8_t poll_IMU(void);
uint8_t poll_ADC(void);

/* Private variables ------------------------------------------------------- */
typedef struct
{
    float value;
    int stt;
    char *label;
} fan_state_t;
fan_state_t state;

HardwareSerial Uart1(1);

static float data[N_SENSORS];
static const bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal


void setup()
{
    WF_Setup();
    String ipString = WiFi.localIP().toString() + "\n";
    const char *ipCharArray = ipString.c_str();
    ei_printf(ipCharArray);

    Uart1.begin(115200, SERIAL_8N1, RX, TX);
    Wire.setPins(SDA_PIN, SCL_PIN);
    Wire.begin();

    init_IMU();
    init_ADC();
}

void loop()
{
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME)
    {
        int64_t next_tick = (int64_t)micros() + ((int64_t)EI_CLASSIFIER_INTERVAL_MS * 1000);

        poll_ADC();
        poll_IMU();

        buffer[ix] = data[0];
        buffer[ix + 1] = data[1];
        buffer[ix + 2] = data[2];
        buffer[ix + 3] = data[3];

        int64_t wait_time = next_tick - (int64_t)micros();

        if (wait_time > 0)
        {
            delayMicroseconds(wait_time);
        }
    }

    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0)
    {
        return;
    }

    ei_impulse_result_t result = {0};

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK)
    {
        return;
    }

    state.value = 0.0;
    state.stt = -1;

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
    {
        if (state.value < result.classification[ix].value)
        {
            state.value = result.classification[ix].value;
            state.stt = ix;
        }
    }

    Serial.println(state.stt);
    Uart1.print(state.stt);
    delay(500);
}

#if !defined(EI_CLASSIFIER_SENSOR) || (EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_FUSION && EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER)
#error "Invalid model for current sensor"
#endif

bool init_IMU(void)
{
    /* ADXL345 */
    if (!adxl345.begin(ADXL345_DEFAULT_ADDRESS))
    {
        WebSerial.println("Could not find a valid ADXL345 sensor, check wiring!");
        while (1)
            ;
    }
    // Set the range to +/- 2g
    adxl345.setRange(ADXL345_RANGE_2_G);
    adxl345.setDataRate(ADXL345_DATARATE_100_HZ);
    return true;
}

bool init_ADC(void)
{
    ina3221.begin(SDA_PIN, SCL_PIN);
    return true;
}

uint8_t poll_IMU(void)
{
    sensors_event_t event;
    adxl345.getEvent(&event);

    data[1] = event.acceleration.x;
    data[2] = event.acceleration.y;
    data[3] = event.acceleration.z;

    return 0;
}

uint8_t poll_ADC(void)
{
    float shuntvoltage = 0;
    float busvoltage = 0;
    float current_mA = 0;
    float loadvoltage = 0;

    busvoltage = ina3221.getBusVoltage_V(OUTPUT_CHANNEL);
    shuntvoltage = ina3221.getShuntVoltage_mV(OUTPUT_CHANNEL);
    current_mA = ina3221.getCurrent_mA(OUTPUT_CHANNEL);
    loadvoltage = busvoltage + (shuntvoltage / 1000);

    data[0] = current_mA;

    return 0;
}
