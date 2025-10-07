/*
 TechnoMCU Home Automation - No external libs
 - Per-relay timers (seconds)
 - Defense mode protected by password
 - Branding: Made by Techno MCU (Raktim Hazra) in web + OLED
 - Minimal SSD1306 driver using Wire (no Adafruit libraries)
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <EEPROM.h>
#include <Wire.h>

// -------- CONFIG ----------
const char* apSsid = "TechnoMCU_Home_Automation";
const char* apPass = "raktim@12345";

const char* defensePass = "raktim52"; // change if you want

// I2C OLED (fixed)
// SDA -> D2 (GPIO4), SCL -> D1 (GPIO5)
#define OLED_SDA_PIN 4
#define OLED_SCL_PIN 5

// Relay pins (avoid I2C pins)
const int relayPinsArr[4] = {14, 12, 13, 15}; // relay1=D5(14), relay2=D6(12), relay3=D7(13), relay4=D8(15)

const int EEPROM_SIZE = 128; // enough for states + timers

// EEPROM layout
// 0: init flag 0xA5
// 1: relay1 state (0/1)
// 2: relay2
// 3: relay3
// 4: defenseMode (0/1)
// 5: relay4
// 6..21: 4 x uint32 timer remaining seconds (relay0..3) => 16 bytes

// -------- GLOBALS ----------
ESP8266WebServer server(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;

bool relayState[4] = {false,false,false,false};
bool defenseMode = false;
unsigned long relayTimerEnd[4] = {0,0,0,0}; // millis when each relay should turn off (0 = none)

unsigned long lastMillis = 0;
unsigned long startMillis = 0;

IPAddress AP_IP;

#define OLED_ADDR 0x3C
uint8_t ssd_buf[1024]; // 128*64/8

// 5x7 font (same as before) - abbreviated here but included fully in code
const uint8_t font5x7_full[] = {
  // full font bytes (same as your previous font table). For brevity it's included here in full:
  0x00,0x00,0x00,0x00,0x00, // 32
  0x00,0x00,0x5F,0x00,0x00,0x00,0x07,0x00,0x07,0x00,0x14,0x7F,0x14,0x7F,0x14,
  0x24,0x2A,0x7F,0x2A,0x12,0x23,0x13,0x08,0x64,0x62,0x36,0x49,0x55,0x22,0x50,
  0x00,0x05,0x03,0x00,0x00,0x00,0x1C,0x22,0x41,0x00,0x00,0x41,0x22,0x1C,0x00,
  0x14,0x08,0x3E,0x08,0x14,0x08,0x08,0x3E,0x08,0x08,0x00,0x50,0x30,0x00,0x00,
  0x08,0x08,0x08,0x08,0x08,0x00,0x60,0x60,0x00,0x00,0x20,0x10,0x08,0x04,0x02,
  0x3E,0x51,0x49,0x45,0x3E,0x00,0x42,0x7F,0x40,0x00,0x42,0x61,0x51,0x49,0x46,
  0x21,0x41,0x45,0x4B,0x31,0x18,0x14,0x12,0x7F,0x10,0x27,0x45,0x45,0x45,0x39,
  0x3C,0x4A,0x49,0x49,0x30,0x01,0x71,0x09,0x05,0x03,0x36,0x49,0x49,0x49,0x36,
  0x06,0x49,0x49,0x29,0x1E,0x00,0x36,0x36,0x00,0x00,0x00,0x56,0x36,0x00,0x00,
  0x08,0x14,0x22,0x41,0x00,0x14,0x14,0x14,0x14,0x14,0x00,0x41,0x22,0x14,0x08,
  0x02,0x01,0x51,0x09,0x06,0x32,0x49,0x79,0x41,0x3E,0x7E,0x11,0x11,0x11,0x7E,
  0x7F,0x49,0x49,0x49,0x36,0x3E,0x41,0x41,0x41,0x22,0x7F,0x41,0x41,0x22,0x1C,
  0x7F,0x49,0x49,0x49,0x41,0x7F,0x09,0x09,0x09,0x01,0x3E,0x41,0x49,0x49,0x7A,
  0x7F,0x08,0x08,0x08,0x7F,0x00,0x41,0x7F,0x41,0x00,0x20,0x40,0x41,0x3F,0x01,
  0x7F,0x08,0x14,0x22,0x41,0x7F,0x40,0x40,0x40,0x40,0x7F,0x02,0x0C,0x02,0x7F,
  0x7F,0x04,0x08,0x10,0x7F,0x3E,0x41,0x41,0x41,0x3E,0x7F,0x09,0x09,0x09,0x06,
  0x3E,0x41,0x51,0x21,0x5E,0x7F,0x09,0x19,0x29,0x46,0x46,0x49,0x49,0x49,0x31,
  0x01,0x01,0x7F,0x01,0x01,0x3F,0x40,0x40,0x40,0x3F,0x1F,0x20,0x40,0x20,0x1F,
  0x3F,0x40,0x38,0x40,0x3F,0x63,0x14,0x08,0x14,0x63,0x07,0x08,0x70,0x08,0x07,
  0x61,0x51,0x49,0x45,0x43,0x00,0x7F,0x41,0x41,0x00,0x02,0x04,0x08,0x10,0x20,
  0x00,0x41,0x41,0x7F,0x00,0x04,0x02,0x01,0x02,0x04,0x40,0x40,0x40,0x40,0x40,
  0x00,0x01,0x02,0x04,0x00,0x20,0x54,0x54,0x54,0x78,0x7F,0x48,0x44,0x44,0x38,
  0x38,0x44,0x44,0x44,0x20,0x38,0x44,0x44,0x48,0x7F,0x38,0x54,0x54,0x54,0x18,
  0x08,0x7E,0x09,0x01,0x02,0x0C,0x52,0x52,0x52,0x3E,0x7F,0x08,0x04,0x04,0x78,
  0x00,0x44,0x7D,0x40,0x00,0x20,0x40,0x44,0x3D,0x00,0x7F,0x10,0x28,0x44,0x00,
  0x00,0x41,0x7F,0x40,0x00,0x7C,0x04,0x18,0x04,0x78,0x7C,0x08,0x04,0x04,0x78,
  0x38,0x44,0x44,0x44,0x38,0x7C,0x14,0x14,0x14,0x08,0x08,0x14,0x14,0x18,0x7C,
  0x7C,0x08,0x04,0x04,0x08,0x48,0x54,0x54,0x54,0x20,0x04,0x3F,0x44,0x40,0x20,
  0x3C,0x40,0x40,0x20,0x7C,0x1C,0x20,0x40,0x20,0x1C,0x3C,0x40,0x30,0x40,0x3C,
  0x44,0x28,0x10,0x28,0x44,0x0C,0x50,0x50,0x50,0x3C,0x44,0x64,0x54,0x4C,0x44,
  0x00,0x08,0x36,0x41,0x00,0x00,0x00,0x7F,0x00,0x00,0x00,0x41,0x36,0x08,0x00,
  0x02,0x01,0x02,0x04,0x02,0x7F,0x41,0x41,0x41,0x7F
};

// ---------- SSD1306 helpers ----------
void ssd_cmd(uint8_t c){
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(0x00);
  Wire.write(c);
  Wire.endTransmission();
}
void ssd_data(uint8_t c){
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(0x40);
  Wire.write(c);
  Wire.endTransmission();
}

void ssd_init(){
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  delay(50);
  ssd_cmd(0xAE);
  ssd_cmd(0xD5); ssd_cmd(0x80);
  ssd_cmd(0xA8); ssd_cmd(0x3F);
  ssd_cmd(0xD3); ssd_cmd(0x00);
  ssd_cmd(0x40);
  ssd_cmd(0x8D); ssd_cmd(0x14);
  ssd_cmd(0x20); ssd_cmd(0x00);
  ssd_cmd(0xA1);
  ssd_cmd(0xC8);
  ssd_cmd(0xDA); ssd_cmd(0x12);
  ssd_cmd(0x81); ssd_cmd(0xCF);
  ssd_cmd(0xD9); ssd_cmd(0xF1);
  ssd_cmd(0xDB); ssd_cmd(0x40);
  ssd_cmd(0xA4);
  ssd_cmd(0xA6);
  ssd_cmd(0x2E);
  ssd_cmd(0xAF);
  memset(ssd_buf,0,sizeof(ssd_buf));
}

void ssd_display(){
  for(uint8_t page=0; page<8; page++){
    ssd_cmd(0xB0 + page);
    ssd_cmd(0x00);
    ssd_cmd(0x10);
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(0x40);
    for(uint8_t col=0; col<128; col++){
      Wire.write(ssd_buf[page*128 + col]);
    }
    Wire.endTransmission();
  }
}

void ssd_clearBuffer(){ memset(ssd_buf,0,sizeof(ssd_buf)); }

void ssd_setPixel(int x,int y,bool on){
  if(x<0||x>=128||y<0||y>=64) return;
  int page = y >> 3;
  int bit = y & 7;
  if(on) ssd_buf[page*128 + x] |= (1<<bit);
  else ssd_buf[page*128 + x] &= ~(1<<bit);
}

void ssd_drawChar(int x,int y,char c){
  if(c<32||c>127) c='?';
  const uint8_t* ch = &font5x7_full[(c-32)*5];
  for(int col=0;col<5;col++){
    uint8_t line = ch[col];
    for(int row=0;row<8;row++){
      bool on = line & (1<<row);
      ssd_setPixel(x+col,y+row,on);
    }
  }
}

void ssd_drawText(int x,int y,const char* s){
  while(*s){
    ssd_drawChar(x,y,*s++);
    x += 6;
    if(x>122) break;
  }
}

// ---------- EEPROM helpers ----------
void saveToEEPROM(){
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.write(0,0xA5);
  EEPROM.write(1, relayState[0] ? 1:0);
  EEPROM.write(2, relayState[1] ? 1:0);
  EEPROM.write(3, relayState[2] ? 1:0);
  EEPROM.write(4, defenseMode ? 1:0);
  EEPROM.write(5, relayState[3] ? 1:0);
  // store 4 timers (remaining seconds as uint32)
  uint32_t addr = 6;
  for(int i=0;i<4;i++){
    uint32_t remain = 0;
    if(relayTimerEnd[i] && millis() < relayTimerEnd[i]) remain = (relayTimerEnd[i] - millis())/1000;
    EEPROM.write(addr + i*4 + 0, (remain>>24)&0xFF);
    EEPROM.write(addr + i*4 + 1, (remain>>16)&0xFF);
    EEPROM.write(addr + i*4 + 2, (remain>>8)&0xFF);
    EEPROM.write(addr + i*4 + 3, (remain)&0xFF);
  }
  EEPROM.commit();
}

void loadFromEEPROM(){
  EEPROM.begin(EEPROM_SIZE);
  if(EEPROM.read(0) == 0xA5){
    relayState[0] = EEPROM.read(1);
    relayState[1] = EEPROM.read(2);
    relayState[2] = EEPROM.read(3);
    defenseMode   = EEPROM.read(4);
    relayState[3] = EEPROM.read(5);
    uint32_t addr = 6;
    for(int i=0;i<4;i++){
      uint32_t val = ((uint32_t)EEPROM.read(addr + i*4 + 0) << 24) |
                     ((uint32_t)EEPROM.read(addr + i*4 + 1) << 16) |
                     ((uint32_t)EEPROM.read(addr + i*4 + 2) << 8) |
                     ((uint32_t)EEPROM.read(addr + i*4 + 3));
      if(val > 0) relayTimerEnd[i] = millis() + (val * 1000UL);
      else relayTimerEnd[i] = 0;
    }
  } else {
    // defaults
    relayState[0]=relayState[1]=relayState[2]=relayState[3]=false;
    defenseMode=false;
    for(int i=0;i<4;i++) relayTimerEnd[i]=0;
    EEPROM.write(0,0xA5);
    EEPROM.commit();
  }
}

// ---------- Relay outputs ----------
void applyRelayOutputs(){
  // active LOW relays: LOW = ON, HIGH = OFF
  for(int i=0;i<4;i++){
    digitalWrite(relayPinsArr[i], relayState[i] ? LOW : HIGH);
  }
}

// ---------- Web UI (modern improved) ----------
String dashboardPage(){
  String html = R"rawliteral(<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"><title>TechnoMCU Panel</title>
  <style>
  body{font-family:Segoe UI,Arial;background:linear-gradient(135deg,#0f2027,#203a43,#2c5364);color:#eee;margin:0}
  .hdr{padding:16px;text-align:center;font-weight:700;color:#00ffc6;background:rgba(0,0,0,0.06)}
  .sub{padding:6px;text-align:center;color:#dcdcdc;font-size:13px}
  .container{padding:12px;display:flex;flex-wrap:wrap;gap:12px;justify-content:center}
  .card{width:220px;background:rgba(255,255,255,0.03);padding:14px;border-radius:12px;box-shadow:0 6px 18px rgba(0,0,0,0.45)}
  .card b{display:block;margin-bottom:6px;font-size:16px}
  .small{color:#c9d1d9;font-size:13px}
  .btn{padding:12px;width:100%;border-radius:10px;border:none;font-weight:700;margin-top:8px;cursor:pointer}
  .on{background:#00c853;color:#071218}
  .off{background:#d32f2f;color:#fff}
  .danger{background:#a00000;color:#fff}
  input[type=number]{width:90px;padding:8px;border-radius:6px;border:none;margin-left:6px}
  .defbtn{display:block;margin:14px auto;padding:12px 22px;border-radius:12px;border:none;background:#ff6b6b;color:#fff;font-weight:800;box-shadow:0 6px 18px rgba(255,107,107,0.25);cursor:pointer}
  .footer{padding:10px;text-align:center;color:#bfc7cf;font-size:12px}
  .statusbox{width:94%;max-width:740px;margin:10px auto;color:#cdd6dd;text-align:left;padding:10px;background:rgba(255,255,255,0.02);border-radius:8px}
  </style>
  <script>
    function askDef(){ var p=prompt('Enter Defense Password:'); if(p!=null) location.href='/defense_toggle?pass='+encodeURIComponent(p); }
    function toggle(idx){ location.href='/toggle' + idx; }
    function setTimer(idx){ var s=document.getElementById('t' + idx).value; location.href='/settimer?relay='+idx+'&sec='+encodeURIComponent(s); }
    setInterval(()=>{ fetch('/status').then(r=>r.text()).then(t=>document.getElementById('statusbox').innerHTML=t); },1500);
  </script>
  </head><body>)rawliteral";

  if(defenseMode) html += "<div class='hdr' style='background:#3a0000'>DEFENSE MODE - TechnoMCU</div>";
  else html += "<div class='hdr'>TechnoMCU Smart Panel</div>";

  html += "<div class='sub'>Made by Techno MCU (Raktim Hazra)</div>";

  html += "<div class='container'>";

  // For each relay card
  for(int i=0;i<4;i++){
    String name = String("Relay ") + String(i+1);
    String small = (i==0) ? "Main appliance" : (i==1 ? "Secondary" : (i==2 ? "Aux" : "Defense-only"));
    html += "<div class='card'><b>" + name + "</b><div class='small'>" + small + "</div>";
    // toggle button
    String cls = relayState[i] ? "on" : "off";
    String label = relayState[i] ? "TURN OFF" : "TURN ON";
    // For relay 2/3 hide toggle when defense mode is active
    if( (i==1 || i==2) && defenseMode ){
      html += "<div style='margin-top:8px' class='small' style='color:#ffaaaa'>Disabled in Defense Mode</div>";
    } else if(i==3 && !defenseMode){
      html += "<div style='margin-top:8px' class='small' style='color:#ffaaaa'>Visible only in Defense Mode</div>";
    } else {
      html += "<div style='margin-top:8px'><button class='btn " + cls + "' onclick=\"toggle(" + String(i+1) + ")\">" + label + "</button></div>";
    }
    // timer input and show remaining
    unsigned long rem = 0;
    if(relayTimerEnd[i] && millis() < relayTimerEnd[i]) rem = (relayTimerEnd[i] - millis())/1000;
    html += "<div class='small' style='margin-top:8px'>Timer remaining: " + String(rem) + "s</div>";
    html += "<div style='margin-top:8px' class='small'>Set timer (seconds): <input id='t" + String(i) + "' type='number' min='1' placeholder='sec' /> <button class='btn' onclick='setTimer(" + String(i) + ")'>Set</button></div>";
    html += "</div>";
  }

  // Defense control
  html += "<div style='width:100%;text-align:center'><button class='defbtn' onclick='askDef()'>" + String(defenseMode? "DISABLE DEFENSE":"ENABLE DEFENSE") + "</button></div>";

  html += "<div id='statusbox' class='statusbox'></div>";
  html += "</div>"; // container

  html += "<div class='footer'>Made by Techno MCU (Raktim Hazra) | © 2025 Raktim Hazra</div>";
  html += "</body></html>";
  return html;
}

void handleRoot(){
  server.send(200,"text/html", dashboardPage());
}

void sendStatusBox(){
  String s;
  for(int i=0;i<4;i++){
    unsigned long rem = 0;
    if(relayTimerEnd[i] && millis() < relayTimerEnd[i]) rem = (relayTimerEnd[i] - millis())/1000;
    s += "<b>R" + String(i+1) + ":</b>" + String(relayState[i]?"ON":"OFF") + " (Timer: " + String(rem) + "s)<br>";
  }
  unsigned long uptime = (millis() - startMillis)/1000;
  s += "<b>Defense:</b>" + String(defenseMode?"ON":"OFF") + " | Uptime: " + String(uptime) + "s<br>";
  server.send(200,"text/html", s);
}

// Toggle endpoints: /toggle1 .. /toggle4
void handleToggleN(int idx){
  // idx: 0..3
  if(idx==1 || idx==2){ // relay2/3 (index 1,2) disabled when defenseMode true
    if(defenseMode){
      // ignore
    } else {
      relayState[idx] = !relayState[idx];
    }
  } else if(idx==3){ // relay4 only when defenseMode true
    if(defenseMode) relayState[idx] = !relayState[idx];
  } else {
    relayState[idx] = !relayState[idx];
    if(!relayState[idx]) relayTimerEnd[idx] = 0;
  }
  applyRelayOutputs();
  saveToEEPROM();
  server.sendHeader("Location","/"); server.send(303);
}

void toggle1(){ handleToggleN(0); }
void toggle2(){ handleToggleN(1); }
void toggle3(){ handleToggleN(2); }
void toggle4(){ handleToggleN(3); }

// settimer?relay=0..3&sec=SECONDS
void handleSetTimer(){
  String r = server.arg("relay");
  String s = server.arg("sec");
  if(r.length()>0 && s.length()>0){
    int idx = r.toInt();
    unsigned long sec = (unsigned long)s.toInt();
    if(idx>=0 && idx<4 && sec>0){
      relayState[idx] = true;
      relayTimerEnd[idx] = millis() + sec * 1000UL;
      applyRelayOutputs();
      saveToEEPROM();
    }
  }
  server.sendHeader("Location","/"); server.send(303);
}

void handleDefenseToggle(){
  String p = server.arg("pass");
  if(p.length()>0 && p == String(defensePass)){
    defenseMode = !defenseMode;
    if(defenseMode){
      // enforce relay2/3 off when enabling defense
      relayState[1] = false;
      relayState[2] = false;
      relayTimerEnd[1] = 0;
      relayTimerEnd[2] = 0;
    }
    applyRelayOutputs();
    saveToEEPROM();
    server.sendHeader("Location","/"); server.send(303);
  } else {
    server.send(200,"text/html","<h3>Defense password incorrect</h3><a href='/'>Back</a>");
  }
}

void handleNotFound(){
  server.sendHeader("Location", String("http://") + AP_IP.toString() + "/");
  server.send(302,"text/plain","");
}

// ---------- Setup & Loop ----------
void setup(){
  Serial.begin(115200);
  // initialize relays
  for(int i=0;i<4;i++){
    pinMode(relayPinsArr[i], OUTPUT);
    digitalWrite(relayPinsArr[i], HIGH); // off (active-low)
  }

  EEPROM.begin(EEPROM_SIZE);
  loadFromEEPROM();
  applyRelayOutputs();

  WiFi.softAP(apSsid, apPass);
  AP_IP = WiFi.softAPIP();
  Serial.print("AP IP: "); Serial.println(AP_IP);
  dnsServer.start(DNS_PORT, "*", AP_IP);

  // routes
  server.on("/", handleRoot);
  server.on("/status", sendStatusBox);
  server.on("/toggle1", toggle1);
  server.on("/toggle2", toggle2);
  server.on("/toggle3", toggle3);
  server.on("/toggle4", toggle4);
  server.on("/settimer", handleSetTimer);
  server.on("/defense_toggle", handleDefenseToggle);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");

  ssd_init();
  startMillis = millis();

  // startup splash
  ssd_clearBuffer();
  ssd_drawText(0,0,"TechnoMCU");
  ssd_drawText(0,12,"Made by Techno MCU");
  ssd_drawText(0,22,"Raktim Hazra");
  ssd_drawText(0,40,"Booting...");
  ssd_display();
  delay(900);
}

void updateDisplay(){
  ssd_clearBuffer();
  // Header
  if(defenseMode) ssd_drawText(0,0,"Defense Mode");
  else ssd_drawText(0,0,"TechnoMCU");
  // Branding small
  ssd_drawText(0,8,"Made by Techno MCU");
  ssd_drawText(0,16,"Raktim Hazra");

  // Relay lines with timers
  char buf[40];
  for(int i=0;i<4;i++){
    unsigned long rem = 0;
    if(relayTimerEnd[i] && millis() < relayTimerEnd[i]) rem = (relayTimerEnd[i] - millis())/1000;
    int y = 24 + i*8; // 24,32,40,48 (fits in 64 rows)
    snprintf(buf, sizeof(buf), "R%d:%s %lus", i+1, relayState[i] ? "ON" : "OFF", rem);
    ssd_drawText(0,y, buf);
  }

  unsigned long uptime = (millis() - startMillis)/1000;
  char upb[24];
  snprintf(upb, sizeof(upb), "Up:%lus", uptime);
  ssd_drawText(64,56, upb); // small uptime bottom-right

  // footer branding
  ssd_drawText(0,56,"Techno MCU");
  ssd_display();
}

void loop(){
  dnsServer.processNextRequest();
  server.handleClient();

  // check timers
  bool changed = false;
  for(int i=0;i<4;i++){
    if(relayTimerEnd[i] && millis() >= relayTimerEnd[i]){
      relayState[i] = false;
      relayTimerEnd[i] = 0;
      changed = true;
    }
  }
  if(changed){
    applyRelayOutputs();
    saveToEEPROM();
  }

  if(millis() - lastMillis > 500){
    lastMillis = millis();
    updateDisplay();
  }
}
