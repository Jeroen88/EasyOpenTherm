#pragma once


#include <Arduino.h>
#include <MqttClient.h>
#include <ArduinoJson.h>


const size_t fullJsonDocSize = 2 * 1024;

const char * chipID();

const char * shortID();


// ============== Object to supply system functions ============================
class System: public MqttClient::System {
public:

	unsigned long millis() const {
		return ::millis();
	}

	void yield(void) {
		::yield();
	}
};

void discoveryMessageSetIDs(char *                      discoveryMsgJson);

const char * topicByReference(const char *              key,
                              const char *              discoveryMsgJson);

bool validJson(const char *                             discoveryMsgJson);

bool connectMQTT(MqttClient &                           client,
                  const char *                          clientID,
                  const char *                          user, 
                  const char *                          password);

bool publish(MqttClient &                               client,
              const char *                              topic, 
              const char *                              payload, 
              bool                                      retained = false);

bool discoveryMsgToJsonDoc(JsonDocument &               discoveryMsgDoc,
                            const char *                discoveryMsgJson);

bool addEntity(MqttClient &                             client,
                const char *                            component,
                const char *                            object,
                JsonDocument &                          discoveryMsgDoc);

bool addEntity(MqttClient &                             client,
                const char *                            component,
                const char *                            object,
                const char *                            discoveryMsgJson);

bool subscribe(MqttClient &                             client,
                MqttClient::MessageHandlerCbk           cbk,
                const char *                            discoveryMessage,
                const char *                            key);