#pragma once


#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


const size_t fullJsonDocSize = 2 * 1024;

const char * chipID();

const char * shortID();

void discoveryMessageSetIDs(char *                      discoveryMsgJson);

const char * topicByReference(const char *              key,
                              const char *              discoveryMsgJson);

bool validJson(const char *                             discoveryMsgJson);

bool connectMQTT(PubSubClient &                         client,
                  const char *                          clientID,
                  const char *                          user, 
                  const char *                          password);

bool publish(PubSubClient &                             client,
              const char *                              topic, 
              const char *                              payload, 
              size_t                                    payloadLength,
              bool                                      retained = false);

bool publish(PubSubClient &                             client,
              const char *                              topic, 
              const char *                              payload, 
              bool                                      retained = false);

bool discoveryMsgToJsonDoc(JsonDocument &               discoveryMsgDoc,
                            const char *                discoveryMsgJson);

bool addEntity(PubSubClient &                           client,
                const char *                            component,
                const char *                            object,
                JsonDocument &                          discoveryMsgDoc);

bool addEntity(PubSubClient &                           client,
                const char *                            component,
                const char *                            object,
                const char *                            discoveryMsgJson);

bool subscribe(PubSubClient &                           client,
                const char *                            discoveryMessage,
                const char *                            key);