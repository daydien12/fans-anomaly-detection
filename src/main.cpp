/* Includes ---------------------------------------------------------------- */
#include <WiFi.h>
#include <HardwareSerial.h>
#include <fan_v2_inferencing.h>

#include <Arduino.h>
#include <PubSubClient.h>
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

void init_MQTT(void);
void connect_to_broker();
void callback(char *topic, byte *payload, unsigned int length);
/* Private variables ------------------------------------------------------- */
typedef struct
{
    float value;
    int stt;
    char *label;
} fan_state_t;
fan_state_t state;

HardwareSerial Uart1(1);
WiFiClient espClient;
PubSubClient client(espClient);

static int iterator = 0;
static float data[N_SENSORS];
static const bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

const char *mqtt_broker = "Mqtt.mysignage.vn";
const int mqtt_port = 1883;
const char *topic = "mqtt";

/**
 * @brief      Arduino setup function
 */
void setup()
{
    /* Init serial */
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif

    Serial.begin(115200);
    WF_Setup();
    String ipString = WiFi.localIP().toString() + "\n";
    // Convert String to const char*
    const char *ipCharArray = ipString.c_str();
    // Print the IP address
    ei_printf(ipCharArray);
    Uart1.begin(115200, SERIAL_8N1, RX, TX);
    Wire.setPins(SDA_PIN, SCL_PIN);
    Wire.begin();

    init_IMU();
    init_ADC();
    init_MQTT();
}

void loop()
{
    // Allocate a buffer here for the values we'll read from the sensor
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME)
    {
        // Determine the next tick (and then sleep later)
        int64_t next_tick = (int64_t)micros() + ((int64_t)EI_CLASSIFIER_INTERVAL_MS * 1000);

        poll_ADC();
        poll_IMU();

        String webstr;
        webstr = String(data[0]) + "|" + String(data[1]) + "|" + String(data[2]) + "|" + String(data[3]) + "\n";
        //WebSerial.print(webstr);

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

    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0)
    {
        // ei_printf("ERR:(%d)\r\n", err);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = {0};

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK)
    {
        // ei_printf("ERR:(%d)\r\n", err);
        return;
    }

    /* Reset fan state */
    state.value = 0.0;
    state.stt = -1;

    String webstr; // Tạo một đối tượng String để lưu chuỗi đã định dạng
    webstr = "Predictions (DSP: " + String(result.timing.dsp) + " ms., Classification: " + String(result.timing.classification) + " ms., Anomaly: " + String(result.timing.anomaly) + " ms.):\r\n";
    // WebSerial.println(buffers); // In chuỗi đã định dạng

    //  print the predictions
    // ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.):\r\n", result.timing.dsp, result.timing.classification, result.timing.anomaly);
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
    {
        String webstr;
        webstr = String(result.classification[ix].label) + ": " + String(result.classification[ix].value) + "|";
        //WebSerial.print(webstr);

        // ei_printf("%s: %.5f\r\n", result.classification[ix].label, result.classification[ix].value);
        if (state.value < result.classification[ix].value)
        {
            state.value = result.classification[ix].value;
            state.stt = ix;
        }
    }
    //WebSerial.println("\n-\n");

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\r\n", result.anomaly);
#endif

    Serial.println(state.stt);
    Uart1.print(state.stt);

    client.loop();
    if (!client.connected())
    {
        connect_to_broker();
    }
    delay(1);
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

void init_MQTT(void)
{
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);

    while (!client.connected())
    {
        String client_id = "esp32-client-";
        client_id += String(WiFi.macAddress());
        WebSerial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
        if (client.connect(client_id.c_str()))
        {
            WebSerial.println("Public emqx mqtt broker connected");
        }
        else
        {
            WebSerial.print("failed with state ");
            WebSerial.print(client.state());
            delay(2000);
        }
    }
    client.subscribe(topic);
    // init and get the time
}

void callback(char *topic, byte *payload, unsigned int length)
{
    char name[25];
    int i;
    WebSerial.print("Message arrived in topic: ");
    WebSerial.println(topic);
    WebSerial.print("Message:");
    for (i = 0; i < length; i++)
    {
        WebSerial.print((char)payload[i]);
        name[i] = payload[i];
    }
    name[i] = 0;
    WebSerial.println();
    WebSerial.println("-----------------------");
}

void connect_to_broker()
{
    while (!client.connected())
    {
        WebSerial.print("Attempting MQTT connection...");
        String clientId = "ESP32";
        clientId += String(random(0xffff), HEX);
        if (client.connect(clientId.c_str()))
        {
            WebSerial.println("connected");
            client.subscribe(topic);
        }
        else
        {
            WebSerial.print("failed, rc=");
            WebSerial.print(client.state());
            WebSerial.println(" try again in 2 seconds");
            delay(2000);
        }
    }
}