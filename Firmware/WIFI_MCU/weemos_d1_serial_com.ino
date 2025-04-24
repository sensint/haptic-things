#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <SoftwareSerial.h>

// WiFi credentials
const char* ssid = "your wifi id"; // wifi credentials here (setup a hotspot)
const char* password = "your wifi password";

// Web server on port 80
ESP8266WebServer server(80);

// Software serial for communication with Daisy
SoftwareSerial daisySerial(D2, D3); // RX, TX (D2=GPIO4, D3=GPIO0)

// Current parameter values
int currentMode = 1;
int currentFreq = 150;
float currentAmp = 0.7;
int currentDelta = 20;

void setup() {
  Serial.begin(115200);  // Debug serial
  daisySerial.begin(9600); // Communication with Daisy (use lower baud rate for stability)
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
  
  // Define web server routes
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  
  // Process any incoming serial data from Daisy
  if (daisySerial.available()) {
    String data = daisySerial.readStringUntil('\n');
    Serial.println("From Daisy: " + data);
  }
}

// void loop() {
//   daisySerial.println("HELLO FROM D1;");
//   delay(1000);

//   if (daisySerial.available()) {
//     String data = daisySerial.readStringUntil('\n');
//     Serial.println("From Daisy: " + data);
//   }
// }

// Send HTML page
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: Arial; margin: 20px; max-width: 500px; }";
  html += ".slider { width: 100%; margin: 10px 0; }";
  html += ".control { margin-bottom: 20px; }";
  html += "button { padding: 10px; margin-right: 5px; cursor: pointer; }";
  html += ".active { background-color: #4CAF50; color: white; }";
  html += "</style>";
  html += "</head><body>";
  html += "<h1>Haptic Controller</h1>";
  
  // Mode buttons
  html += "<div class='control'>";
  html += "<h3>Mode:</h3>";
  html += "<button id='mode0' onclick='setMode(0)'>OFF</button>";
  html += "<button id='mode1' onclick='setMode(1)'>FSR</button>";
  html += "<button id='mode2' onclick='setMode(2)'>Orientation</button>";
  html += "<button id='mode3' onclick='setMode(3)'>Accelerometer</button>";
  html += "</div>";
  
  // Frequency slider
  html += "<div class='control'>";
  html += "<h3>Frequency: <span id='freqValue'>" + String(currentFreq) + "</span> Hz</h3>";
  html += "<input type='range' min='50' max='500' value='" + String(currentFreq) + "' class='slider' id='freqSlider'>";
  html += "</div>";
  
  // Amplitude slider
  html += "<div class='control'>";
  html += "<h3>Amplitude: <span id='ampValue'>" + String(currentAmp) + "</span></h3>";
  html += "<input type='range' min='0' max='0.7' step='0.01' value='" + String(currentAmp) + "' class='slider' id='ampSlider'>";
  html += "</div>";
  
  // Delta threshold slider
  html += "<div class='control'>";
  html += "<h3>Delta Threshold: <span id='deltaValue'>" + String(currentDelta) + "</span></h3>";
  html += "<input type='range' min='5' max='50' value='" + String(currentDelta) + "' class='slider' id='deltaSlider'>";
  html += "</div>";
  
  // Update button
  html += "<button onclick='updateAll()'>Update All</button>";
  
  // JavaScript
  html += "<script>";
  html += "function setMode(mode) {";
  html += "  document.querySelectorAll('button[id^=\"mode\"]').forEach(btn => btn.classList.remove('active'));";
  html += "  document.getElementById('mode'+mode).classList.add('active');";
  html += "  fetch('/set?mode='+mode);";
  html += "}";
  
  html += "document.getElementById('freqSlider').oninput = function() {";
  html += "  document.getElementById('freqValue').textContent = this.value;";
  html += "};";
  
  html += "document.getElementById('ampSlider').oninput = function() {";
  html += "  document.getElementById('ampValue').textContent = this.value;";
  html += "};";
  
  html += "document.getElementById('deltaSlider').oninput = function() {";
  html += "  document.getElementById('deltaValue').textContent = this.value;";
  html += "};";
  
  html += "function updateAll() {";
  html += "  const freq = document.getElementById('freqSlider').value;";
  html += "  const amp = document.getElementById('ampSlider').value;";
  html += "  const delta = document.getElementById('deltaSlider').value;";
  html += "  fetch('/set?freq='+freq+'&amp='+amp+'&delta='+delta);";
  html += "}";
  
  html += "// Initialize active mode button";
  html += "document.getElementById('mode" + String(currentMode) + "').classList.add('active');";
  
  html += "</script>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

// Handle parameter changes
void handleSet() {
  bool changed = false;
  
  // Check if mode was changed
  if (server.hasArg("mode")) {
    currentMode = server.arg("mode").toInt();
    changed = true;
  }
  
  // Check other parameters
  if (server.hasArg("freq")) {
    currentFreq = server.arg("freq").toInt();
    changed = true;
  }
  
  if (server.hasArg("amp")) {
    currentAmp = server.arg("amp").toFloat();
    changed = true;
  }
  
  if (server.hasArg("delta")) {
    currentDelta = server.arg("delta").toInt();
    changed = true;
  }
  
  // If any parameter changed, send update to Daisy
  if (changed) {
    String cmd = "SET," + String(currentMode) + "," + 
                String(currentFreq) + "," + 
                String(currentAmp) + "," + 
                String(currentDelta) + ";";
                
    daisySerial.println(cmd);
    daisySerial.flush();
    delay(50);

    Serial.println("Sent to Daisy: " + cmd);
  }
  
  server.send(200, "text/plain", "OK");
}