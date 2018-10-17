#include <AccelStepper.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

#define motorPin1 5
#define motorPin2 4 
#define motorPin3 14
#define motorPin4 12
#define HALFSTEP 8
#define LED 2
const char* ssid = "";
const char* password = "";

#define stepOpen 0
#define stepClosed -60000
#define maxSpeed 1000.0
#define maxAccel 1000.0

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
AccelStepper stepper1(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);
ESP8266WebServer server(80);

boolean motorsDisabled = false;
boolean hold = false;

String getStepperInfo() {
  String s = "Current Pos: " + String(stepper1.currentPosition());
  s += "\n";
  s += "Target Pos: " + String(stepper1.targetPosition());
  s += "\n";
  s += "Distance to Go: " + String(stepper1.distanceToGo());
  s += "\n";
  s += "Speed " + String(stepper1.speed());
  s += "\n";
  s += "Is running: " + String(stepper1.isRunning() == 1 ? true : false);
  return s;
}

void moveTo(long step, boolean relative = false) {
  motorsDisabled = false;
  if(relative) {
    stepper1.move(step);
  } else {
    stepper1.moveTo(step);
  }
}

void handleStepTo() {
  if(server.args() > 0 && server.hasArg("step")) {
    long step = server.arg("step").toInt();
    boolean relative = server.hasArg("rel");
    moveTo(step, relative);
    String response = "Set pos to " + String(stepper1.targetPosition());
    server.send(200, "text/plain", response);
    return;
  }
  server.send(400, "text/plain", "Needs step arg");
}

void handleOpen() {
  moveTo(stepOpen);
  server.send(200, "text/plain", "Blinds opening.");
}

void handleClose() {
  moveTo(stepClosed);
  server.send(200, "text/plain", "Blinds closing.");
}

void handleDisableSteppers() {
  stepper1.disableOutputs();
  Serial.println("Disable outputs");
  motorsDisabled = true;
  server.send(200, "text/plain", "Outputs disabled");
}

void handleEnableSteppers() {
  hold = true;
  stepper1.enableOutputs();
  Serial.println("Enable outputs");
  motorsDisabled = false;
  server.send(200, "text/plain", "Outputs enabled");
}

void handleStop() {
  stepper1.stop();
  server.send(200,  "text/plain", "Motors stopped");
}

void handleResetPos() {
  stepper1.setCurrentPosition(0);
  moveTo(stepper1.currentPosition(), false);
  server.send(200, "text/plain", "Stepper position reset. Current pos is now new 0.");
}

void handleRoot() {
  digitalWrite(LED, 1);
  server.send(200, "text/plain", "hello from esp8266!");
  digitalWrite(LED, 0);
}


void handleSetSpeed() {
  if(server.args() > 0 && server.hasArg("speed")) {
    long speed = server.arg("speed").toInt();
    if(speed <= (long) maxSpeed) {
      stepper1.setSpeed(speed);
      server.send(200, "text/plain", "Set speed to " + String(speed));
    } else {
      server.send(400, "text/plain", "Speed cannot be higher than maxSpeed: " + String(maxSpeed));
    }
  }
  server.send(400, "text/plain", "Needs speed arg"); 
}

void handleSetAcceleration() {
  if(server.args() > 0 && server.hasArg("accel")) {
    long accel = server.arg("accel").toInt();
    if(accel > 0 && accel <= (long) maxAccel) {
      stepper1.setAcceleration(accel);
      server.send(200, "text/plain", "Set acceleration to " + String(accel)); 
      return;
    } else {
      server.send(400, "text/plain", "Acceleration must be higher than 0.0, and lower than" + String(maxAccel));
      return;
    }
  }
  server.send(400, "text/plain", "Needs speed arg"); 
}

void setupOTA() {
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
}

void setup() {  
  pinMode(LED, OUTPUT);
  stepper1.setMaxSpeed(maxSpeed);
  stepper1.setAcceleration(100.0);
  stepper1.setSpeed(700);

  digitalWrite(LED, LOW);
  
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED, LOW);
    delay(250);
    Serial.print(".");
    digitalWrite(LED, HIGH);
    delay(250);
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  setupOTA();
  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);
  server.on("/step", handleStepTo);
  server.on("/open", handleOpen);
  server.on("/close", handleClose);
  server.on("/speed", handleSetSpeed);
  server.on("/disable", handleDisableSteppers);
  server.on("/enable", handleEnableSteppers);
  server.on("/stop", handleStop);
  server.on("/reset", handleResetPos);
  server.on("/accel", handleSetAcceleration);
  server.on("/info", []() {
    server.send(200, "text/plain", getStepperInfo());
  });
  //server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
}//--(end setup )---

void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  
 
  stepper1.run();

   //Turn off motors when stopping
  if (stepper1.distanceToGo() == 0 && hold == false && motorsDisabled == false) {
    stepper1.disableOutputs();
    Serial.println("Disable outputs");
    motorsDisabled = true;
  }
}
