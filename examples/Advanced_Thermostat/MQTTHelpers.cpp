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

//#include <machine/types.h>
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

// ESP32   Chip ID is cca4f003f784 => 0x84, 0xf7, 0x03, 0xf0, 0xa4, 0xcc, 0x00, 0x00
// ESP8266 Chip ID is 00c2005a3cc2 => 0xc2, 0x3c, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00
// So for the ESP32 MAC[4] and MAC[5] are the least significant bytes, and for the ESP8266 MAC[1] and MAC[0] 
//  Serial.print("Serial number bytes: "); for(size_t index = 0; index < 8; index++) Serial.printf("0x%02x, ", MAC[index]); Serial.println();

//    sprintf(shortNumber, "%02X%02X", MAC[4], MAC[5]);
#if defined(ESP32)
    sprintf(shortNumber, "%02X%02X", MAC[4], MAC[5]);
#else
    sprintf(shortNumber, "%02X%02X", MAC[1], MAC[0]);
#endif
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
Serial.printf("Deserialize error %s for JSON '%s'\n", error.f_str(), discoveryMsgJson);

    return nullptr;
  }

  const char * baseTopic = discoveryJsonPartDoc["~"];
  const char * topic = discoveryJsonPartDoc[key];
  if(topic == nullptr) {
    ESP_LOGE(TAG, "Key '%s' not found in JSON '%s'", key, discoveryMsgJson);
Serial.printf("Key '%s' not found in JSON '%s'\n", key, discoveryMsgJson);

    return nullptr;
  }

  if(*topic == '~' && baseTopic == nullptr) {
    ESP_LOGE(TAG, "Base topic '~' not found in JSON '%s'", discoveryMsgJson);
Serial.printf("Base topic '~' not found in JSON '%s'\n", discoveryMsgJson);

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
Serial.printf("topicByReference out of memory\n");

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


bool connectMQTT(PubSubClient &                         client,
                  const char *                          clientID,
                  const char *                          user, 
                  const char *                          password) {
  // Start new MQTT connection
  ESP_LOGI(TAG, "Connecting...");

  // Connect
  bool connectionResult = client.connect(clientID, user, password);

  if(!connectionResult) {
    ESP_LOGE(TAG, "Connection error: %d", client.state());

    return false;
  }

  return true;
}


bool publish(PubSubClient &                             client,
              const char *                              topic, 
              const char *                              payload,
              size_t                                    payloadLength,
              bool                                      retained) {
  if(payloadLength == 0) return true;

  if(!client.beginPublish(topic, payloadLength, retained)) return false;
  size_t bytesWritten = client.write((const unsigned char *) payload, payloadLength);

  return client.endPublish() && bytesWritten == payloadLength;
}


bool publish(PubSubClient &                             client,
              const char *                              topic, 
              const char *                              payload,
              bool                                      retained) {
  size_t payloadLength = strlen(payload);

  return publish(client, topic, payload, payloadLength, retained);
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
bool addEntity(PubSubClient &                           client,
                const char *                            component,
                const char *                            object,
                JsonDocument &                          discoveryMsgDoc) {
  if(!client.connected()) return false;

  size_t jsonSize = measureJson(discoveryMsgDoc);
  char discoveryMsgJson[jsonSize + 1];
  serializeJson(discoveryMsgDoc, discoveryMsgJson, sizeof discoveryMsgJson);

  size_t topicSize = sizeof("homeassistant/") - 1 + strlen(component) + sizeof("/") - 1 + strlen(chipID()) + sizeof("/") - 1 + strlen(object) + sizeof("/config") - 1 + 1;
  char topic[topicSize];
  sprintf(topic, "homeassistant/%s/%s/%s/config", component, chipID(), object);

  ESP_LOGI(TAG, "Discovery topic is '%s'\n", topic);
  ESP_LOGI(TAG, "Discovery message is '%s'", discoveryMsgJson);

  return publish(client, topic, discoveryMsgJson);
}


// addEntity()
// Same as above but providing the discovery message as a const char * JSON 
bool addEntity(PubSubClient &                           client,
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
bool subscribe(PubSubClient &                           client,
                const char *                            discoveryMessage,
                const char *                            key) {
  const char * topic = topicByReference(key, discoveryMessage);
  if(topic == nullptr) return false;

  ESP_LOGI(TAG, "Subscribe to '%s'", topic);
Serial.printf("Subscribe to '%s'\n", topic);

  return client.subscribe(topic);
}
