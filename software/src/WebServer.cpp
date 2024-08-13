#include "WebServer.h"
#include <optional>

WebServer::WebServer(Actuator *actuator, uint16_t port)
    : actuator(actuator), port(port)
{
  setupRESTHandlers();
  setupWebsocketHandler();
}

void WebServer::setupRESTHandlers()
{
}

void WebServer::setupWebsocketHandler()
{
  websocketHandler.onOpen(
      [](PsychicWebSocketClient *client)
      {
        Serial.printf(
            "[socket] connection #%u connected from %s\n",
            client->socket(),
            client->remoteIP().toString());
      });
  websocketHandler.onFrame(
      [this](PsychicWebSocketRequest *request, httpd_ws_frame *frame) -> esp_err_t
      {
        Serial.printf("[socket] #%d sent: %s\n", request->client()->socket(), (char *)frame->payload);

        return handleFrame(request, frame);
      });
  websocketHandler.onClose(
      [](PsychicWebSocketClient *client)
      {
        Serial.printf(
            "[socket] connection #%u closed from %s\n",
            client->socket(),
            client->remoteIP().toString());
      });
}

esp_err_t WebServer::handleFrame(PsychicWebSocketRequest *request, httpd_ws_frame *frame)
{
  JsonDocument requestDoc;
  deserializeJson(requestDoc, frame->payload);

  const char *action = requestDoc["action"];
  auto messageId =
      requestDoc.containsKey("messageId")
          ? std::optional<JsonDocument>{requestDoc["messageId"]}
          : std::nullopt;

  JsonDocument responseDoc;
  bool success = true;
  if (strcmp(action, "home") == 0)
    actuator->home();
  else if (strcmp(action, "moveTo") == 0)
  {
    int32_t position = requestDoc["position"];
    success = actuator->moveTo(position);
  }
  else if (strcmp(action, "move") == 0)
  {
    int32_t delta = requestDoc["delta"];
    success = actuator->move(delta);
  }
  else if (strcmp(action, "stop") == 0)
    actuator->stop();
  else if (strcmp(action, "forceStop") == 0)
    actuator->forceStop();
  else if (strcmp(action, "disable") == 0)
    actuator->disable();
  else if (strcmp(action, "subscribe") == 0)
    subscribe(request->client()->socket());
  else if (strcmp(action, "unsubscribe") == 0)
    unsubscribe(request->client()->socket());
  else if (strcmp(action, "sendState") == 0)
  {
    compileState(responseDoc["state"].to<JsonObject>());
  }
  else if (strcmp(action, "setProfileParameter") == 0)
  {
    success = false;
    if (requestDoc.containsKey("profile"))
    {
      auto profileDoc = requestDoc["profile"];
      if (profileDoc.containsKey("name"))
      {
        const char *profileName = profileDoc["name"];
        MotionProfile *profile;
        if (strcmp(profileName, "runningPositive") == 0)
          profile = actuator->runningPositiveProfile;
        else if (strcmp(profileName, "runningNegative") == 0)
          profile = actuator->runningNegativeProfile;
        else if (strcmp(profileName, "homing") == 0)
          profile = actuator->homingProfile;

        if (profile)
        {
          if (profileDoc.containsKey("acceleration"))
          {
            profile->acceleration = profileDoc["acceleration"];
            success = true;
          }
          if (profileDoc.containsKey("velocity"))
          {
            profile->velocity = profileDoc["velocity"];
            success = true;
          }
          if (profileDoc.containsKey("stallThreshold"))
          {
            profile->stallThreshold = profileDoc["stallThreshold"];
            success = true;
          }
          if (profileDoc.containsKey("velocityThreshold"))
          {
            profile->velocityThreshold = profileDoc["velocityThreshold"];
            success = true;
          }
        }
      }
    }
  }
  else
    success = false;

  responseDoc["status"] = success ? "ok" : "error";

  if (messageId.has_value())
  {
    responseDoc["messageId"] = messageId.value();
  }

  String output;
  serializeJson(responseDoc, output);

  request->reply(output.c_str());

  return ESP_OK;
}

void WebServer::subscribe(int socket)
{
  subscribers.push_back(socket);
}

void WebServer::unsubscribe(int socket)
{
  subscribers.erase(std::remove(subscribers.begin(), subscribers.end(), socket));
}

void WebServer::broadcastState()
{
  if (subscribers.empty())
  {
    vTaskDelay(1);
    return;
  }

  JsonDocument doc;
  compileState(doc.to<JsonObject>());
  String output;
  serializeJson(doc, output);
  for (int socket : subscribers)
  {
    PsychicWebSocketClient *client = websocketHandler.getClient(socket);
    if (client)
      client->sendMessage(output.c_str());
    else
      unsubscribe(socket);
  }
}

void WebServer::compileState(JsonObject doc)
{
  doc["driverVelocity"] = actuator->getDriverVelocity();
  doc["medianDriverVelocity"] = actuator->getMedianDriverVelocity();
  doc["driverStallValue"] = actuator->getDriverStallValue();
  doc["medianDriverStallValue"] = actuator->getMedianDriverStallValue();
  doc["state"] = STATE_NAMES[actuator->getState()];
  doc["isHomed"] = actuator->isHomed();
  doc["min"] = actuator->getMin();
  doc["max"] = actuator->getMax();
  doc["currentPosition"] = actuator->getCurrentPosition();
  doc["targetPosition"] = actuator->getTargetPosition();
  doc["velocity"] = actuator->getVelocity();
  doc["acceleration"] = actuator->getAcceleration();
  doc["homingDirection"] = DIRECTION_NAMES[actuator->homingDirection];
  JsonObject profilesObj = doc["profiles"].to<JsonObject>();

  auto activeProfile = actuator->getActiveProfile();
  profilesObj["active"] = activeProfile ? activeProfile->name : nullptr;

  for (auto profile : actuator->allProfiles)
  {
    JsonObject profileObj = profilesObj[profile->name].to<JsonObject>();
    profileObj["displayName"] = profile->displayName;
    profileObj["velocity"] = profile->velocity;
    profileObj["acceleration"] = profile->acceleration;
    profileObj["velocityThreshold"] = profile->velocityThreshold;
    profileObj["stallThreshold"] = profile->stallThreshold;
  }
}

void WebServer::begin()
{
  server.listen(port);
  server.on("/ws", &websocketHandler);
  xTaskCreate(
      [](void *instance)
      {
        while (true)
        {
          ((WebServer *)instance)->broadcastState();
        }
      },
      "BroadcastTask",
      1024 * 4,
      this,
      1,
      NULL);
}
