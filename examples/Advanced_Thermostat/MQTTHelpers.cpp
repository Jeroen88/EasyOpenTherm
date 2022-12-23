#include "MQTTHelpers.h"

#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#else 
#error Unsupported or unknown board
#endif

static const char * TAG = "EasyOpenTherm MQTTHelpers";

#if !defined(ESP32) && !defined(ESP_LOGE)
#define ESP_LOGE(...)
#define ESP_LOGI(...)
#define ESP_LOGV(...)
#endif


const size_t smallJsonDocSize = 256;


/*
 * chipID()
 * Store the ESP32 unique ID once. Return a pointer to the stored chip ID. The ID is the same as the MAC but in reversed byte order
 */
const char * chipID() {
  static char serialNumber[13] = "";

  if(*serialNumber == '\0') {
#if defined(ESP32)
    sprintf(serialNumber, "%012llx", ESP.getEfuseMac());
#elif defined(ESP8266)
    sprintf(serialNumber, "%012llx", ESP.getChipId());
#else
#error Unsupported or unknown board
#endif
    ESP_LOGI(TAG, "Serial number is %s", serialNumber);
  }

  return serialNumber;
}

#include <machine/types.h>
/*
 * shortID()
 * Store the least significant two bytes of the ESP32 unique ID once. Return a pointer to the stored short ID. The ID is the same as the last two MAC bytes, in the same order
 */
const char * shortID() {
  static char shortNumber[5] = "";

  if(*shortNumber == '\0') {
#if defined(ESP32)
    uint64_t mac = ESP.getEfuseMac();
#elif defined(ESP8266)
    uint64_t mac = ESP.getChipId();
#else
#error Unsupported or unknown board
#endif

    uint8_t * MAC = (uint8_t *) &mac;
    sprintf(shortNumber, "%02X%02X", MAC[4], MAC[5]);
    ESP_LOGI(TAG, "Short ID is %s", shortNumber);
  }

  return shortNumber;
}


/*
 * replaceAll()
 * Do an 'in place' replacement into 'destination' of all occurances of the 'search' by 'replace'
 */
bool replaceAll(char *                                  destination,
                const char *                            search,
                const char *                            replace) {
  size_t searchLength = strlen(search);
  if(strlen(replace) != searchLength) {
    ESP_LOGE(TAG, "Length of search string (%s) does not match replace string (%s)\n", search, replace);

    return false;
  }

  size_t length = strlen(destination);
  char * searchFrom = destination;
  bool replaced = false;
  for(;;) {
    char * found = strstr(searchFrom, search);
    if(found == nullptr) break;
    memcpy(found, replace, searchLength);
    replaced = true;

    searchFrom = found + searchLength;
    if(searchFrom >= destination + length) break;
  }
  
  return replaced;
}


/*
 * discoveryMessageSetIDs()
 * Replace all placeholders '112233445566' with the chip ID, all placeholders '11:22:33:44:55:66' with the MAC and all placeholder '####' with the short ID.
 */
void discoveryMessageSetIDs(char *                      discoveryMsgJson) {
  replaceAll(discoveryMsgJson, "112233445566", chipID());
  replaceAll(discoveryMsgJson, "11:22:33:44:55:66", WiFi.macAddress().c_str());
  replaceAll(discoveryMsgJson, "####", shortID());
}


const char * topicByReference(const char *              key,
                              const char *              discoveryMsgJson) {
  static char * topicBuffer = nullptr;
  static size_t topicBufferSize = 0;  // Includes the trailing `\0'

  StaticJsonDocument<smallJsonDocSize> discoveryJsonPartDoc;

  StaticJsonDocument<smallJsonDocSize> filter;
  filter["~"] = true;
  filter[key] = true;

  DeserializationError error = deserializeJson(discoveryJsonPartDoc, discoveryMsgJson, DeserializationOption::Filter(filter));
  if(error) {
    ESP_LOGE(TAG, "Deserialize error %s for JSON '%s'", error.f_str(), discoveryMsgJson);

    return nullptr;
  }

  const char * baseTopic = discoveryJsonPartDoc["~"];
  const char * topic = discoveryJsonPartDoc[key];
  if(topic == nullptr) {
    ESP_LOGE(TAG, "Key '%s' not found in JSON '%s'", key, discoveryMsgJson);

    return nullptr;
  }

  if(*topic == '~' && baseTopic == nullptr) {
    ESP_LOGE(TAG, "Base topic '~' not found in JSON '%s'", discoveryMsgJson);

    return nullptr;
  }

  size_t newSize = strlen(topic) + 1;
  if(*topic == '~') newSize += strlen(baseTopic);
  if(topicBufferSize < newSize) {
    free(topicBuffer);
    topicBuffer = (char *) malloc(newSize);
    if(topicBuffer == nullptr) {
      topicBufferSize = 0;
      ESP_LOGE(TAG, "topicByReference out of memory");

      return nullptr;
    }
    topicBufferSize = newSize;
  }

  if(*topic == '~') {
    sprintf(topicBuffer, "%s%s", baseTopic, topic + 1);
  } else {
    sprintf(topicBuffer, "%s", topic);
  }

  return topicBuffer;
}


bool validJson(const char *                             discoveryMsgJson) {
  StaticJsonDocument<0> emptyDoc;

  StaticJsonDocument<0> filter;

  DeserializationError error = deserializeJson(emptyDoc, discoveryMsgJson, DeserializationOption::Filter(filter));
  if(error) {
    ESP_LOGE(TAG, "Deserialize error %s for JSON '%s'", error.f_str(), discoveryMsgJson);
  }

  return error == DeserializationError::Ok;
}


bool connectMQTT(MqttClient &                           client,
                  const char *                          clientID,
                  const char *                          user, 
                  const char *                          password) {
  // Start new MQTT connection
  ESP_LOGI(TAG, "Connecting");
  MqttClient::ConnectResult connectResult;
  // Connect
  MQTTPacket_connectData options = MQTTPacket_connectData_initializer;
  options.MQTTVersion = 4;
  options.clientID.cstring = (char *)clientID;
  options.username.cstring = (char *)user;
  options.password.cstring = (char *)password;
  options.cleansession = true;
  options.keepAliveInterval = 15; // 15 seconds
  MqttClient::Error::type rc = client.connect(options, connectResult);
  if (rc != MqttClient::Error::SUCCESS) {
    ESP_LOGE(TAG, "Connection error: %i", rc);

    return false;
  }

  return true;
}


bool publish(MqttClient &                               client,
              const char *                              topic, 
              const char *                              payload, 
              bool                                      retained) {
  if(!client.isConnected()) return false;

  MqttClient::Message message;
  message.qos = MqttClient::QOS0;
  message.retained = retained;
  message.dup = true;
  message.payload = (void *)payload;
  message.payloadLen = strlen(payload);
  client.publish(topic, message);
Serial.printf("PUBLISH topic is '%s' message is '%s'\n", topic, message.payload);

  return true;
}


bool discoveryMsgToJsonDoc(JsonDocument &               discoveryMsgDoc,
                            const char *                discoveryMsgJson) {
  DeserializationError error = deserializeJson(discoveryMsgDoc, discoveryMsgJson);
  if(error) {
    ESP_LOGE(TAG, "Deserialize error %s for JSON '%s'", error.f_str(), discoveryMsgJson);

    return false;
  }

  return true;
}


/*
 * addEntity()
 * Publish the prepared and deserialized discovery message to topic 'homeassistant/[component]/[chip ID]/[object]/config', with component being one of the
 * Home Assistant MQTT components like 'climate', 'sensor' or 'binary_sensor' and object a unique ID to differentiate between e.g. two sensors in the same
 * device.
 */
bool addEntity(MqttClient &                             client,
                const char *                            component,
                const char *                            object,
                JsonDocument &                          discoveryMsgDoc) {
  if(!client.isConnected()) return false;

  size_t jsonSize = measureJson(discoveryMsgDoc);
  char discoveryMsgJson[jsonSize + 1];
  serializeJson(discoveryMsgDoc, discoveryMsgJson, sizeof discoveryMsgJson);

  ESP_LOGI(TAG, "Discovery topic is '%s'\n", (String("homeassistant/") + String(component) + String("/") + String(chipID()) + String("/") + String(object) + String("/config")).c_str());
  ESP_LOGI(TAG, "Discovery message is '%s'", discoveryMsgJson);

  MqttClient::Message message;
  message.qos = MqttClient::QOS0;
  message.retained = false;
  message.dup = false;
  message.payload = (void*) discoveryMsgJson;
  message.payloadLen = jsonSize;
  client.publish((String("homeassistant/") + String(component) + String("/") + String(chipID()) + String("/") + String(object) + String("/config")).c_str(), message);
Serial.printf("PUBLISH topic is '%s' message is '%s'\n", (String("homeassistant/") + String(component) + String("/") + String(chipID()) + String("/") + String(object) + String("/config")).c_str(), message.payload);

  return true;
}


// addEntity()
// Same as above but providing the discovery message as a const char * JSON 
bool addEntity(MqttClient &                             client,
                const char *                            component,
                const char *                            object,
                const char *                            discoveryMsgJson) {
  DynamicJsonDocument discoveryMsgDoc(fullJsonDocSize);

  if(!discoveryMsgToJsonDoc(discoveryMsgDoc, discoveryMsgJson)) return false;

  return addEntity(client, component, object, discoveryMsgDoc);
}

/*
 * subscribe()
 * Subscribe to the topic at at the given key in the discovery message. Before subscription the topic in the discovery
 * message is expanded to the full topic by prefixing the "~" path.
 */
bool subscribe(MqttClient &                             client,
                MqttClient::MessageHandlerCbk           cbk,
                const char *                            discoveryMessage,
                const char *                            key) {
  const char * topic = topicByReference(key, discoveryMessage);
  if(topic == nullptr) return false;

  MqttClient::Error::type rc = client.subscribe(topic, MqttClient::QOS0, cbk);
  if (rc != MqttClient::Error::SUCCESS) {
    ESP_LOGE(TAG, "Subscribe error: %i for topic '%s'", rc, topic);
    ESP_LOGE(TAG, "Drop connection");
    client.disconnect();

    return false;
  }
  ESP_LOGI(TAG, "Subscribed to '%s'", topic);
Serial.printf("Subscribed to '%s'\n", topic);

  return true;
}