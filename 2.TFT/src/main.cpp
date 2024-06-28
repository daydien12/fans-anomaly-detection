#include <system.h>
#include "wifi_function.h"
#include <PubSubClient.h>

int curr_state = -1;
int prev_state = -1;
extern int count;
extern TFT_eSPI tft;

enum state
{
    BLADE_FAULT = 0,
    BROKEN_BLADE = 1,
    MOUNT_FALL = 2,
    OFF = 3,
    ON = 4,
    TAP = 5
};

WiFiClient espClient;
PubSubClient client(espClient);

//const char *mqtt_broker = "Mqtt.mysignage.vn";
const char *mqtt_broker = "192.168.1.1";
const int mqtt_port = 1883;
const char *topic = "mqtt_tft";

void init_MQTT(void);
void connect_to_broker();
void callback(char *topic, byte *payload, unsigned int length);

void setup()
{
    Serial.begin(115200);
    setup_tft();
    WF_Setup();
    init_MQTT();
}

void loop()
{

    if (curr_state != prev_state)
    {
        switch (curr_state)
        {
        case OFF:
            off();
            break;
        case ON:
            on();
            break;
        case TAP:
            tapping();
            break;
        case MOUNT_FALL:
            mount_fault();
            break;
        case BLADE_FAULT:
            blade_fault();
            break;
        case BROKEN_BLADE:
            broken_blades();
            break;
        default:
            break;
        }
        prev_state = curr_state;
    }

    if (curr_state == BLADE_FAULT || curr_state == BROKEN_BLADE || curr_state == MOUNT_FALL)
    {
        if (count % 2 == 0)
        {
            tft.fillRect(0, 50, 240, 10, TFT_RED);
            tft.fillRect(0, 50, 10, 320, TFT_RED);
            tft.fillRect(230, 50, 10, 320, TFT_RED);
            tft.fillRect(0, 310, 240, 10, TFT_RED);
        }
        else
        {
            tft.fillRect(0, 50, 240, 10, TFT_WHITE);
            tft.fillRect(0, 50, 10, 320, TFT_WHITE);
            tft.fillRect(230, 50, 10, 320, TFT_WHITE);
            tft.fillRect(0, 310, 240, 10, TFT_WHITE);
        }
    }
    client.loop();
    if (!client.connected())
    {
        connect_to_broker();
    }
}

void init_MQTT(void)
{
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);

    while (!client.connected())
    {
        String client_id = "esp32-client-";
        if (client.connect(client_id.c_str()))
        {
            Serial.println("Public emqx mqtt broker connected");
        }
        else
        {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
    client.subscribe(topic);
    // init and get the time
}

void callback(char *topic, byte *payload, unsigned int length)
{
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';

    Serial.print("Message: ");
    Serial.println(message);

    // Kiểm tra và tách số từ chuỗi nhận được nếu đúng định dạng [5]
    if (message[0] == '[' && message[length - 1] == ']')
    {
        String msgStr = String(message);
        String numberStr = msgStr.substring(1, length - 1);
        int number = numberStr.toInt();
        Serial.print("Extracted number: ");
        Serial.println(number);
        curr_state = number;
    }
    else
    {
        Serial.println("Invalid message format");
    }
}

void connect_to_broker()
{
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        String clientId = "ESP32";
        clientId += String(random(0xffff), HEX);
        if (client.connect(clientId.c_str()))
        {
            Serial.println("connected");
            client.subscribe(topic);
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 2 seconds");
            delay(2000);
        }
    }
}

/*

// For NodeMCU - use pin numbers in the form PIN_Dx where Dx is the NodeMCU pin designation
#define TFT_MISO  19  // Automatically assigned with ESP8266 if not defined
#define TFT_MOSI  23  // Automatically assigned with ESP8266 if not defined
#define TFT_SCLK  18  // Automatically assigned with ESP8266 if not defined

#define TFT_CS    15  // Chip select control pin D8
#define TFT_DC    2  // Data Command control pin
#define TFT_RST   4  // Reset pin (could connect to NodeMCU RST, see next line)

//#define TFT_RST  -1     // Set TFT_RST to -1 if the display RESET is connected to NodeMCU RST or 3.3V


//#define TFT_BL PIN_D1  // LED back-light (only for ST7789 with backlight control pin)

#define TOUCH_CS 21     // Chip select pin (T_CS) of touch screen

*/