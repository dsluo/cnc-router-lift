#include <PsychicHttp.h>
#include "Actuator.h"

#ifndef WEBSERVER_H
#define WEBSERVER_H

class WebServer
{
  PsychicHttpServer server;
  PsychicWebSocketHandler websocketHandler;

  Actuator *actuator;
  uint16_t port;

  std::vector<int> subscribers;

  void setupRESTHandlers();
  void setupWebsocketHandler();

  esp_err_t handleFrame(PsychicWebSocketRequest *request, httpd_ws_frame *frame);

  void subscribe(int socket);
  void unsubscribe(int socket);

  void broadcastState();
  void compileState(JsonObject doc);

public:
  WebServer(Actuator *actuator, uint16_t port = 80);
  void begin();
};

#endif
