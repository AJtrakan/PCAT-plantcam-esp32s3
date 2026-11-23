#include <Arduino.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <SD_MMC.h>
#include <WebServer.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <esp_sleep.h>
#include <esp_random.h>
#include <img_converters.h>
#include <math.h>
#include <stdio.h>
#include "canopy_ai.h"

constexpr int LED_PIN = 2;
constexpr int LED_ON = HIGH;
constexpr int LED_OFF = LED_ON == HIGH ? LOW : HIGH;
constexpr int SD_CLK = 39;
constexpr int SD_CMD = 38;
constexpr int SD_D0 = 40;
constexpr uint32_t INTERVAL_MS = 1000;
constexpr uint32_t BLINK_MS = 100;
constexpr unsigned PHOTOS_PER_FOLDER = 1000;
constexpr int ROI_RADIUS = 96;
constexpr int GREEN_MIN = 45;
constexpr int GREEN_RED_MARGIN = 15;
constexpr int GREEN_BLUE_MARGIN = 10;

WebServer server(80);
char sessionPath[32];
char latestPhotoPath[72];
char accessPointName[24];
unsigned photoNumber = 0;
unsigned captureErrors = 0;
uint32_t nextCapture = 0;
uint32_t ledStarted = 0;
bool ledLit = false;
bool recording = false;
bool cardMounted = false;
bool cameraReady = false;
uint8_t photosRemaining = 0;
uint8_t validMeasurements = 0;
uint8_t *rgbBuffer = nullptr;
uint64_t batchGreenPixels = 0;
uint64_t batchRoiPixels = 0;
unsigned batchNumber = 0;
float canopyPercent = NAN;
float previousCanopyPercent = NAN;
float baselineCanopyPercent = NAN;
float changeFromPrevious = NAN;
float growthFromBaseline = NAN;
int plantCenterX = -1;
int plantCenterY = -1;
float plantConfidence = 0;
uint32_t batchCenterXSum = 0;
uint32_t batchCenterYSum = 0;
float batchConfidenceSum = 0;
String recorderStatus = "Starting";

// ---------- Deep sleep + dashboard ----------
constexpr char FW_VERSION[] = "2.1.0";
constexpr uint32_t SETUP_IDLE_MS = 10UL * 60 * 1000; // setup web idle -> sleep
constexpr uint32_t WIFI_TIMEOUT_MS = 20000;
constexpr uint32_t MQTT_TIMEOUT_MS = 10000;
constexpr uint32_t RTC_MAGIC = 0x504C4341; // "PLCA"

// Survives deep sleep (lost on power-off).
RTC_DATA_ATTR uint32_t rtcMagic = 0;
RTC_DATA_ATTR char rtcSessionPath[32];
RTC_DATA_ATTR unsigned rtcPhotoNumber = 0;
RTC_DATA_ATTR unsigned rtcBatchNumber = 0;
RTC_DATA_ATTR float rtcCanopyPercent = NAN;
RTC_DATA_ATTR uint32_t rtcCycles = 0;

struct Config {
  String ssid, pass, devId, mqttHost, mqttUser, mqttPass, mqttTopic;
  uint16_t mqttPort = 1883;
  uint16_t sleepMinutes = 60;
} cfg;

Preferences prefs;
WiFiClient netClient;
PubSubClient mqtt(netClient);
bool autoMode = false;      // true = woke from timer: capture, send, sleep
uint32_t lastWebActivity = 0;
bool sleepRequested = false;

const char WEB_PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="th"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>PlantCam</title>
<style>body{font:17px system-ui;max-width:850px;margin:auto;padding:16px;background:#eef5ef;color:#173d27}button,a{display:inline-block;margin:4px;padding:10px 15px;border:0;border-radius:8px;background:#176b38;color:white;text-decoration:none;font-size:16px}.card{background:white;border-radius:14px;padding:16px;margin:12px 0;box-shadow:0 2px 8px #0001}#preview,#latest{display:block;width:100%;border-radius:10px;background:#ddd}.preview-wrap{position:relative;max-width:640px;margin:auto}.roi{position:absolute;left:20%;top:10%;width:60%;height:80%;border:3px solid #00e676;border-radius:50%;box-sizing:border-box;pointer-events:none}.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px;margin-top:8px}.stat{background:#eff8f1;padding:10px;border-radius:10px}.gallery{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px}.gallery img{width:100%;border-radius:8px;display:block}.muted{color:#58705f;font-size:14px}.cfg{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:8px}.cfg label{font-size:14px}.cfg input{display:block;width:100%;box-sizing:border-box;padding:8px;font-size:16px;border:1px solid #bcd;border-radius:6px}</style></head>
<body><div class="card"><h1>PlantCam เก็บภาพผัก</h1><p id="message">กำลังอ่านสถานะ…</p>
<div class="stats"><div class="stat">สถานะ<br><b id="state">-</b></div><div class="stat">จำนวนภาพ<br><b id="count">0</b></div><div class="stat">พื้นที่ว่าง SD<br><b id="free">-</b></div><div class="stat">ข้อผิดพลาด<br><b id="errors">0</b></div></div>
<div class="stats"><div class="stat">พื้นที่พุ่มล่าสุด<br><b id="canopy">-</b></div><div class="stat">เทียบครั้งก่อน<br><b id="previous">-</b></div><div class="stat">โตจากค่าเริ่มต้น<br><b id="baseline">-</b></div><div class="stat">AI ตรวจพบ<br><b id="ai">-</b></div></div>
<p><form method="post" action="/api/capture" style="display:inline"><button type="submit">ถ่าย 3 รูป</button></form><a id="download" href="/latest.jpg" download>ดาวน์โหลดภาพล่าสุด</a></p>
<p><form method="post" action="/api/baseline" style="display:inline"><button type="submit">ตั้งค่าพุ่มล่าสุดเป็นค่าเริ่มต้น</button></form></p>
<p class="muted">กดหนึ่งครั้งเพื่อถ่าย 3 รูป ห่างกันรูปละ 1 วินาที • GPIO2 กะพริบเมื่อบันทึกสำเร็จ</p></div>
<div class="card"><h2>ตั้งค่าส่งผลไปแดชบอร์ด + โหมดหลับ</h2>
<form method="post" action="/api/config" class="cfg">
<label>Wi-Fi SSID<input name="ssid" id="c_ssid"></label><label>Wi-Fi รหัสผ่าน<input name="pass" type="password" placeholder="(เว้นว่าง = ใช้ค่าเดิม)"></label>
<label>รหัสเครื่อง (devId)<input name="devId" id="c_devId"></label><label>MQTT host<input name="mHost" id="c_mHost"></label>
<label>MQTT port<input name="mPort" id="c_mPort" type="number"></label><label>MQTT user<input name="mUser" id="c_mUser"></label>
<label>MQTT รหัสผ่าน<input name="mPass" type="password" placeholder="(เว้นว่าง = ใช้ค่าเดิม)"></label><label>MQTT topic prefix<input name="mTopic" id="c_mTopic"></label>
<label>ตื่นมาวัดทุก (นาที)<input name="sleepMin" id="c_sleepMin" type="number" min="1" max="1440"></label>
<button type="submit">บันทึกการตั้งค่า</button></form>
<form method="post" action="/api/sleep"><button type="submit" style="background:#8a4b00">เริ่มทำงานอัตโนมัติ (วัด → ส่ง → หลับ)</button></form>
<p class="muted">ผลส่งไปที่ topic <b id="c_topic">-</b> • ถ้าไม่มีการใช้งานหน้าเว็บ 10 นาที บอร์ดจะเริ่มโหมดอัตโนมัติเอง • กด RST เพื่อกลับมาหน้านี้</p></div>
<div class="card"><h2>ภาพตัวอย่างสด</h2><div class="preview-wrap"><img id="preview" src="/preview.jpg" alt="กำลังเปิดกล้อง"><div class="roi" id="roi" style="display:none"></div></div><p class="muted">หลังถ่าย วงกลมจะแสดงบริเวณที่ AI พบต้นและใช้วัดพุ่ม ภาพตัวอย่างไม่ถูกบันทึกลง SD</p></div>
<div class="card"><h2>ภาพล่าสุด</h2><img id="latest" alt="ยังไม่มีภาพ"></div>
<div class="card"><h2>12 ภาพล่าสุด</h2><div class="gallery" id="gallery"></div></div>
<script>
const stateText=document.getElementById('state'),countText=document.getElementById('count'),errorText=document.getElementById('errors'),freeText=document.getElementById('free'),messageText=document.getElementById('message'),canopyText=document.getElementById('canopy'),previousText=document.getElementById('previous'),baselineText=document.getElementById('baseline'),aiText=document.getElementById('ai'),roi=document.getElementById('roi'),previewImage=document.getElementById('preview'),latestImage=document.getElementById('latest'),downloadLink=document.getElementById('download'),galleryBox=document.getElementById('gallery');let lastCount=-1,isCapturing=false;
async function refresh(){try{const s=await fetch('/api/status',{cache:'no-store'}).then(r=>r.json());isCapturing=s.recording;stateText.textContent=s.recording?'กำลังถ่ายชุด ('+s.remaining+' รูป)':'พร้อมถ่าย';countText.textContent=s.photos;errorText.textContent=s.errors;freeText.textContent=s.sd_free_mb===null?'-':s.sd_free_mb+' MB';messageText.textContent=s.message;canopyText.textContent=s.canopy_percent===null?'-':s.canopy_percent.toFixed(2)+'%';previousText.textContent=s.change_previous_percent===null?'-':(s.change_previous_percent>=0?'+':'')+s.change_previous_percent.toFixed(2)+'%';baselineText.textContent=s.growth_baseline_percent===null?'-':(s.growth_baseline_percent>=0?'+':'')+s.growth_baseline_percent.toFixed(2)+'%';aiText.textContent=s.ai_state==='plant_found'?(s.ai_confidence*100).toFixed(1)+'%':s.ai_state;if(s.ai_x!==null){roi.style.display='block';roi.style.left=((s.ai_x-96)/320*100)+'%';roi.style.top=((s.ai_y-96)/240*100)+'%'}else roi.style.display='none';if(s.photos>0){latestImage.src='/latest.jpg?v='+s.photos;downloadLink.href='/latest.jpg?v='+s.photos}if(s.photos!==lastCount){lastCount=s.photos;const a=await fetch('/api/recent',{cache:'no-store'}).then(r=>r.json());galleryBox.replaceChildren(...a.map(x=>{const l=document.createElement('a');l.href=x.url;l.target='_blank';const i=document.createElement('img');i.src=x.url;i.loading='lazy';i.alt='ภาพ '+x.number;l.append(i);return l}))}}catch(e){messageText.textContent='เชื่อมต่อบอร์ดไม่ได้'}}function refreshPreview(){if(!isCapturing)previewImage.src='/preview.jpg?t='+Date.now()}refresh();setInterval(refresh,1000);setInterval(refreshPreview,800);
fetch('/api/config',{cache:'no-store'}).then(r=>r.json()).then(c=>{for(const k of ['ssid','devId','mHost','mPort','mUser','mTopic','sleepMin'])document.getElementById('c_'+k).value=c[k];document.getElementById('c_topic').textContent=c.mTopic+'/'+c.devId+'/plantcam'}).catch(()=>{});
</script></body></html>)HTML";

bool startCamera() {
  if (!psramFound()) {
    recorderStatus = "PSRAM unavailable";
    return false;
  }
  camera_config_t camera = {};
  camera.pin_d0 = 11;
  camera.pin_d1 = 9;
  camera.pin_d2 = 8;
  camera.pin_d3 = 10;
  camera.pin_d4 = 12;
  camera.pin_d5 = 18;
  camera.pin_d6 = 17;
  camera.pin_d7 = 16;
  camera.pin_xclk = 15;
  camera.pin_sccb_sda = 4;
  camera.pin_sccb_scl = 5;
  camera.pin_vsync = 6;
  camera.pin_href = 7;
  camera.pin_pclk = 13;
  camera.pin_pwdn = -1;
  camera.pin_reset = -1;
  camera.ledc_channel = LEDC_CHANNEL_0;
  camera.ledc_timer = LEDC_TIMER_0;
  camera.xclk_freq_hz = 20000000;
  camera.pixel_format = PIXFORMAT_JPEG;
  camera.frame_size = FRAMESIZE_QVGA;
  camera.jpeg_quality = 12;
  camera.fb_count = 1;
  camera.fb_location = CAMERA_FB_IN_PSRAM;
  camera.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  esp_err_t error = esp_camera_init(&camera);
  if (error != ESP_OK) {
    recorderStatus = "Camera init failed: " + String(esp_err_to_name(error));
    return false;
  }
  rgbBuffer = (uint8_t *)ps_malloc(320 * 240 * 3);
  if (!rgbBuffer) {
    recorderStatus = "Cannot allocate canopy measurement buffer";
    esp_camera_deinit();
    return false;
  }
  return true;
}

bool startCard() {
  if (!SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0) ||
      !SD_MMC.begin("/sdcard", true, false) || SD_MMC.cardType() == CARD_NONE) {
    recorderStatus = "microSD mount failed";
    return false;
  }
  cardMounted = true;
  if (!SD_MMC.exists("/lettuce") && !SD_MMC.mkdir("/lettuce")) {
    recorderStatus = "Cannot create /lettuce";
    return false;
  }
  // After a timer wake, keep writing into the same run folder.
  if (rtcMagic == RTC_MAGIC && rtcSessionPath[0] &&
      SD_MMC.exists(rtcSessionPath)) {
    strlcpy(sessionPath, rtcSessionPath, sizeof(sessionPath));
    photoNumber = rtcPhotoNumber;
    batchNumber = rtcBatchNumber;
    canopyPercent = rtcCanopyPercent;
    return true;
  }
  photoNumber = 0;
  batchNumber = 0;
  for (int attempt = 0; attempt < 8; ++attempt) {
    snprintf(sessionPath, sizeof(sessionPath), "/lettuce/run_%08lx",
             (unsigned long)esp_random());
    if (!SD_MMC.exists(sessionPath) && SD_MMC.mkdir(sessionPath)) {
      strlcpy(rtcSessionPath, sessionPath, sizeof(rtcSessionPath));
      rtcPhotoNumber = 0;
      rtcBatchNumber = 0;
      rtcCanopyPercent = NAN;
      rtcMagic = RTC_MAGIC;
      return true;
    }
  }
  recorderStatus = "Cannot create session folder";
  return false;
}

void photoPath(unsigned number, char *output, size_t size) {
  snprintf(output, size, "%s/%05u/IMG_%08u.jpg", sessionPath,
           (number - 1) / PHOTOS_PER_FOLDER, number);
}

bool savePhoto(const camera_fb_t &frame) {
  char folder[48];
  snprintf(folder, sizeof(folder), "%s/%05u", sessionPath,
           photoNumber / PHOTOS_PER_FOLDER);
  if (photoNumber % PHOTOS_PER_FOLDER == 0 && !SD_MMC.exists(folder) &&
      !SD_MMC.mkdir(folder))
    return false;
  char finalPath[72], tempPath[72], vfsPath[88];
  photoPath(photoNumber + 1, finalPath, sizeof(finalPath));
  snprintf(tempPath, sizeof(tempPath), "%s/%05u/IMG_%08u.tmp", sessionPath,
           photoNumber / PHOTOS_PER_FOLDER, photoNumber + 1);
  snprintf(vfsPath, sizeof(vfsPath), "/sdcard%s", tempPath);
  FILE *file = fopen(vfsPath, "wb");
  if (!file)
    return false;
  bool written = fwrite(frame.buf, 1, frame.len, file) == frame.len;
  bool closed = fclose(file) == 0;
  if (!written || !closed || !SD_MMC.rename(tempPath, finalPath)) {
    SD_MMC.remove(tempPath);
    return false;
  }
  ++photoNumber;
  strlcpy(latestPhotoPath, finalPath, sizeof(latestPhotoPath));
  return true;
}

bool measureCanopy(const camera_fb_t &frame, uint32_t &greenPixels,
                   uint32_t &roiPixels) {
  if (frame.width != 320 || frame.height != 240 ||
      frame.format != PIXFORMAT_JPEG ||
      !fmt2rgb888(frame.buf, frame.len, frame.format, rgbBuffer))
    return false;

  int detectedX = -1;
  int detectedY = -1;
  float detectedConfidence = 0;
  if (!findPlantWithAI(rgbBuffer, detectedX, detectedY, detectedConfidence))
    return false;

  plantCenterX = detectedX;
  plantCenterY = detectedY;
  plantConfidence = detectedConfidence;
  greenPixels = 0;
  roiPixels = 0;
  for (int y = 0; y < 240; ++y) {
    int dy = y - plantCenterY;
    for (int x = 0; x < 320; ++x) {
      int dx = x - plantCenterX;
      if (dx * dx + dy * dy > ROI_RADIUS * ROI_RADIUS)
        continue;
      size_t pixel = (y * 320 + x) * 3;
      int redOrBlue1 = rgbBuffer[pixel];
      int green = rgbBuffer[pixel + 1];
      int redOrBlue2 = rgbBuffer[pixel + 2];
      ++roiPixels;
      if (green >= GREEN_MIN && green - redOrBlue1 >= GREEN_RED_MARGIN &&
          green - redOrBlue2 >= GREEN_BLUE_MARGIN)
        ++greenPixels;
    }
  }
  return roiPixels > 0;
}

bool saveBaseline() {
  if (!cardMounted || !isfinite(canopyPercent))
    return false;
  File file = SD_MMC.open("/lettuce/baseline.txt", FILE_WRITE);
  if (!file)
    return false;
  file.println(canopyPercent, 4);
  file.close();
  baselineCanopyPercent = canopyPercent;
  growthFromBaseline = 0;
  return true;
}

void loadBaseline() {
  File file = SD_MMC.open("/lettuce/baseline.txt", FILE_READ);
  if (!file)
    return;
  float value = file.parseFloat();
  file.close();
  if (isfinite(value) && value > 0 && value <= 100)
    baselineCanopyPercent = value;
}

void appendCanopyMeasurement() {
  File file = SD_MMC.open("/lettuce/canopy.csv", FILE_APPEND);
  if (!file) {
    recorderStatus = "Canopy measured but CSV write failed";
    return;
  }
  if (file.size() == 0)
    file.println("session,batch,uptime_ms,first_photo,last_photo,valid_samples,"
                 "green_pixels,roi_pixels,canopy_percent,change_previous_percent,"
                 "growth_baseline_percent,ai_x,ai_y,ai_confidence");
  file.printf("%s,%u,%lu,%u,%u,%u,%llu,%llu,%.4f,", sessionPath, batchNumber,
              (unsigned long)millis(), photoNumber - 2, photoNumber,
              validMeasurements, batchGreenPixels, batchRoiPixels,
              canopyPercent);
  if (isfinite(changeFromPrevious))
    file.printf("%.4f,", changeFromPrevious);
  else
    file.print(',');
  if (isfinite(growthFromBaseline))
    file.printf("%.4f,", growthFromBaseline);
  else
    file.print(',');
  file.printf("%d,%d,%.4f\n", plantCenterX, plantCenterY, plantConfidence);
  file.close();
}

void finishMeasurementBatch() {
  ++batchNumber;
  if (validMeasurements == 0 || batchRoiPixels == 0) {
    recorderStatus = "Photos saved but canopy measurement failed";
    return;
  }
  previousCanopyPercent = canopyPercent;
  canopyPercent = 100.0f * batchGreenPixels / batchRoiPixels;
  plantCenterX = (int)lroundf((float)batchCenterXSum / validMeasurements);
  plantCenterY = (int)lroundf((float)batchCenterYSum / validMeasurements);
  plantConfidence = batchConfidenceSum / validMeasurements;
  changeFromPrevious =
      isfinite(previousCanopyPercent) && previousCanopyPercent > 0
          ? 100.0f * (canopyPercent - previousCanopyPercent) /
                previousCanopyPercent
          : NAN;
  if (!isfinite(baselineCanopyPercent))
    saveBaseline();
  growthFromBaseline =
      isfinite(baselineCanopyPercent) && baselineCanopyPercent > 0
          ? 100.0f * (canopyPercent - baselineCanopyPercent) /
                baselineCanopyPercent
          : NAN;
  appendCanopyMeasurement();
  rtcCanopyPercent = canopyPercent;
  recorderStatus = "AI found plant; 3 photos saved and canopy measured";
}

// ---------- Config (NVS) ----------
String macId() {
  char id[7];
  snprintf(id, sizeof(id), "%06X",
           (unsigned)((ESP.getEfuseMac() >> 24) & 0xFFFFFF));
  return String(id);
}

void loadConfig() {
  prefs.begin("plantcam", true);
  cfg.ssid = prefs.getString("ssid", "");
  cfg.pass = prefs.getString("pass", "");
  cfg.devId = prefs.getString("devId", "");
  cfg.mqttHost = prefs.getString("mHost", "mqtt.aj-tay.com");
  cfg.mqttPort = prefs.getUShort("mPort", 1883);
  cfg.mqttUser = prefs.getString("mUser", "");
  cfg.mqttPass = prefs.getString("mPass", "");
  cfg.mqttTopic = prefs.getString("mTopic", "PCAT_Kit");
  cfg.sleepMinutes = prefs.getUShort("sleepMin", 60);
  prefs.end();
  if (cfg.devId.isEmpty())
    cfg.devId = "CAM" + macId();
  if (cfg.mqttTopic.isEmpty())
    cfg.mqttTopic = "PCAT_Kit";
  if (cfg.sleepMinutes < 1)
    cfg.sleepMinutes = 1;
  if (cfg.sleepMinutes > 1440)
    cfg.sleepMinutes = 1440;
}

void saveConfig() {
  prefs.begin("plantcam", false);
  prefs.putString("ssid", cfg.ssid);
  prefs.putString("pass", cfg.pass);
  prefs.putString("devId", cfg.devId);
  prefs.putString("mHost", cfg.mqttHost);
  prefs.putUShort("mPort", cfg.mqttPort);
  prefs.putString("mUser", cfg.mqttUser);
  prefs.putString("mPass", cfg.mqttPass);
  prefs.putString("mTopic", cfg.mqttTopic);
  prefs.putUShort("sleepMin", cfg.sleepMinutes);
  prefs.end();
}

String jsonNum(float v, int digits) {
  return isfinite(v) ? String(v, digits) : String("null");
}

String resultJson(bool ok) {
  String j = "{\"dev\":\"" + cfg.devId + "\",\"fw\":\"" + FW_VERSION + "\"";
  j += ",\"ok\":" + String(ok ? "true" : "false");
  j += ",\"cycle\":" + String(rtcCycles);
  j += ",\"batch\":" + String(batchNumber);
  j += ",\"photos\":" + String(photoNumber);
  j += ",\"valid_samples\":" + String(validMeasurements);
  j += ",\"canopy_percent\":" + jsonNum(ok ? canopyPercent : NAN, 4);
  j += ",\"change_previous_percent\":" + jsonNum(ok ? changeFromPrevious : NAN, 4);
  j += ",\"growth_baseline_percent\":" + jsonNum(ok ? growthFromBaseline : NAN, 4);
  j += ",\"baseline_percent\":" + jsonNum(baselineCanopyPercent, 4);
  j += ",\"ai_state\":\"" + String(canopyAIState) + "\"";
  j += ",\"ai_x\":" + (plantCenterX >= 0 ? String(plantCenterX) : String("null"));
  j += ",\"ai_y\":" + (plantCenterY >= 0 ? String(plantCenterY) : String("null"));
  j += ",\"ai_confidence\":" + (plantCenterX >= 0 ? String(plantConfidence, 4) : String("null"));
  j += ",\"ai_elapsed_ms\":" + String(canopyAIElapsedMs);
  j += ",\"errors\":" + String(captureErrors);
  j += ",\"sd_free_mb\":";
  j += cardMounted ? String((SD_MMC.totalBytes() - SD_MMC.usedBytes()) / (1024 * 1024))
                   : String("null");
  j += ",\"rssi\":" + String(WiFi.RSSI());
  j += ",\"sleep_min\":" + String(cfg.sleepMinutes);
  j += ",\"message\":\"" + recorderStatus + "\"}";
  return j;
}

// Stream the newest JPEG from SD to <topic>/<devId>/plantcam/image.
bool publishLatestPhoto(const String &topic) {
  if (!cardMounted || !latestPhotoPath[0])
    return false;
  File file = SD_MMC.open(latestPhotoPath, FILE_READ);
  if (!file)
    return false;
  size_t size = file.size();
  if (size == 0 || size > 200000 ||
      !mqtt.beginPublish(topic.c_str(), size, false)) {
    file.close();
    return false;
  }
  uint8_t chunk[1024];
  size_t sentBytes = 0;
  while (sentBytes < size) {
    size_t n = file.read(chunk, sizeof(chunk));
    if (n == 0 || mqtt.write(chunk, n) != n)
      break;
    sentBytes += n;
  }
  file.close();
  bool ok = mqtt.endPublish() && sentBytes == size;
  Serial.printf("Dashboard: photo %s (%u bytes)\n", ok ? "sent" : "failed",
                (unsigned)size);
  return ok;
}

// Connect to the farm Wi-Fi and publish the result (retained) to
// <topic>/<devId>/plantcam. Returns true when the broker accepted it.
bool publishResult(bool ok) {
  if (cfg.ssid.isEmpty() || cfg.mqttHost.isEmpty()) {
    Serial.println("Dashboard: Wi-Fi/MQTT not configured, skip send");
    return false;
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.ssid.c_str(), cfg.pass.c_str());
  uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < WIFI_TIMEOUT_MS)
    delay(100);
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Dashboard: Wi-Fi connect failed");
    return false;
  }
  String payload = resultJson(ok);
  mqtt.setServer(cfg.mqttHost.c_str(), cfg.mqttPort);
  mqtt.setBufferSize(1024);
  String clientId = "plantcam-" + cfg.devId;
  String base = cfg.mqttTopic + "/" + cfg.devId;
  String statusTopic = base + "/status";
  bool sent = false;
  started = millis();
  while (!sent && millis() - started < MQTT_TIMEOUT_MS) {
    if (mqtt.connect(clientId.c_str(), cfg.mqttUser.c_str(),
                     cfg.mqttPass.c_str(), statusTopic.c_str(), 0, true,
                     "sleeping")) {
      publishLatestPhoto(base + "/plantcam/image");
      sent = mqtt.publish((base + "/plantcam").c_str(), payload.c_str(), true);
      mqtt.publish(statusTopic.c_str(), "sleeping", true);
      mqtt.loop();
      delay(200);
      mqtt.disconnect();
    } else {
      delay(500);
    }
  }
  Serial.printf("Dashboard: %s %s\n", sent ? "sent" : "MQTT failed",
                payload.c_str());
  return sent;
}

void goToSleep(uint64_t seconds) {
  Serial.printf("Deep sleep %llu s\n", (unsigned long long)seconds);
  Serial.flush();
  digitalWrite(LED_PIN, LED_OFF);
  if (cameraReady)
    esp_camera_deinit();
  if (cardMounted)
    SD_MMC.end();
  server.stop();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  rtcPhotoNumber = photoNumber;
  rtcBatchNumber = batchNumber;
  esp_sleep_enable_timer_wakeup(seconds * 1000000ULL);
  esp_deep_sleep_start();
}

// Timer wake: take 3 photos, measure, send to the dashboard, sleep again.
void runAutoCycle() {
  ++rtcCycles;
  bool ok = false;
  if (cameraReady && cardMounted) {
    startCaptureBatch();
    // Let auto-exposure settle after waking.
    for (int i = 0; i < 5; ++i) {
      camera_fb_t *warm = esp_camera_fb_get();
      if (warm)
        esp_camera_fb_return(warm);
    }
    uint8_t attempts = 0;
    while (recording && attempts++ < 6) {
      uint32_t t = millis();
      captureAndSave();
      digitalWrite(LED_PIN, LED_OFF);
      int32_t wait = (int32_t)INTERVAL_MS - (int32_t)(millis() - t);
      if (recording && wait > 0)
        delay(wait);
    }
    recording = false;
    ok = validMeasurements > 0 && isfinite(canopyPercent) &&
         recorderStatus.startsWith("AI found");
  }
  Serial.println(recorderStatus);
  publishResult(ok);
  goToSleep((uint64_t)cfg.sleepMinutes * 60);
}

void startCaptureBatch() {
  if (!cardMounted || recording)
    return;
  photosRemaining = 3;
  validMeasurements = 0;
  batchGreenPixels = 0;
  batchRoiPixels = 0;
  batchCenterXSum = 0;
  batchCenterYSum = 0;
  batchConfidenceSum = 0;
  plantCenterX = -1;
  plantCenterY = -1;
  plantConfidence = 0;
  recording = true;
  nextCapture = millis();
  recorderStatus = "Capturing 3 photos";
}

void servePhoto(const char *path, bool noCache) {
  if (!cardMounted || !path[0]) {
    server.send(404, "text/plain", "Photo unavailable");
    return;
  }
  File file = SD_MMC.open(path, FILE_READ);
  if (!file) {
    server.send(404, "text/plain", "Photo unavailable");
    return;
  }
  if (noCache)
    server.sendHeader("Cache-Control", "no-store");
  server.streamFile(file, "image/jpeg");
  file.close();
}

void startWebServer() {
  snprintf(accessPointName, sizeof(accessPointName), "PlantCam-%04lX",
           (unsigned long)(esp_random() & 0xffff));
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(accessPointName);
  server.on("/", HTTP_GET,
            [] { server.send_P(200, "text/html; charset=utf-8", WEB_PAGE); });
  server.on("/preview.jpg", HTTP_GET, [] {
    if (!cameraReady) {
      server.send(503, "text/plain", "Camera unavailable");
      return;
    }
    if (recording) {
      if (latestPhotoPath[0])
        servePhoto(latestPhotoPath, true);
      else
        server.send(409, "text/plain", "Capture in progress");
      return;
    }
    camera_fb_t *frame = esp_camera_fb_get();
    if (!frame) {
      ++captureErrors;
      server.send(503, "text/plain", "Preview capture failed");
      return;
    }
    server.sendHeader("Cache-Control", "no-store");
    server.setContentLength(frame->len);
    server.send(200, "image/jpeg", "");
    server.sendContent((const char *)frame->buf, frame->len);
    esp_camera_fb_return(frame);
  });
  server.on("/latest.jpg", HTTP_GET, [] { servePhoto(latestPhotoPath, true); });
  server.on("/photo", HTTP_GET, [] {
    unsigned n = server.hasArg("n") ? server.arg("n").toInt() : 0;
    if (n == 0 || n > photoNumber) {
      server.send(404, "text/plain", "Photo unavailable");
      return;
    }
    char p[72];
    photoPath(n, p, sizeof(p));
    servePhoto(p, false);
  });
  server.on("/api/status", HTTP_GET, [] {
    lastWebActivity = millis();
    String j = "{\"recording\":" + String(recording ? "true" : "false") +
               ",\"photos\":" + String(photoNumber) +
               ",\"remaining\":" + String(photosRemaining) +
               ",\"errors\":" + String(captureErrors) + ",\"sd_free_mb\":";
    if (cardMounted)
      j += String((SD_MMC.totalBytes() - SD_MMC.usedBytes()) / (1024 * 1024));
    else
      j += "null";
    j += ",\"canopy_percent\":";
    j += isfinite(canopyPercent) ? String(canopyPercent, 4) : "null";
    j += ",\"change_previous_percent\":";
    j += isfinite(changeFromPrevious) ? String(changeFromPrevious, 4) : "null";
    j += ",\"growth_baseline_percent\":";
    j += isfinite(growthFromBaseline) ? String(growthFromBaseline, 4) : "null";
    j += ",\"ai_state\":\"" + String(canopyAIState) + "\"";
    j += ",\"ai_x\":";
    j += plantCenterX >= 0 ? String(plantCenterX) : "null";
    j += ",\"ai_y\":";
    j += plantCenterY >= 0 ? String(plantCenterY) : "null";
    j += ",\"ai_confidence\":";
    j += plantCenterX >= 0 ? String(plantConfidence, 4) : "null";
    j += ",\"ai_elapsed_ms\":" + String(canopyAIElapsedMs);
    j += ",\"message\":\"" + recorderStatus + "\"}";
    server.send(200, "application/json", j);
  });
  server.on("/api/recent", HTTP_GET, [] {
    String j = "[";
    unsigned first = photoNumber > 12 ? photoNumber - 11 : 1;
    for (unsigned n = photoNumber; n >= first && photoNumber > 0; --n) {
      if (n != photoNumber)
        j += ',';
      j += "{\"number\":" + String(n) + ",\"url\":\"/photo?n=" + String(n) +
           "\"}";
      if (n == first)
        break;
    }
    j += ']';
    server.send(200, "application/json", j);
  });
  server.on("/api/capture", HTTP_POST, [] {
    startCaptureBatch();
    server.sendHeader("Location", "/");
    server.send(303, "text/plain", "");
  });
  server.on("/api/baseline", HTTP_POST, [] {
    bool saved = saveBaseline();
    recorderStatus = saved ? "Baseline saved" : "No valid canopy value for baseline";
    server.sendHeader("Location", "/");
    server.send(303, "text/plain", "");
  });
  server.on("/api/config", HTTP_GET, [] {
    String j = "{\"ssid\":\"" + cfg.ssid + "\",\"devId\":\"" + cfg.devId +
               "\",\"mHost\":\"" + cfg.mqttHost + "\",\"mPort\":" +
               String(cfg.mqttPort) + ",\"mUser\":\"" + cfg.mqttUser +
               "\",\"mTopic\":\"" + cfg.mqttTopic + "\",\"sleepMin\":" +
               String(cfg.sleepMinutes) + ",\"fw\":\"" + FW_VERSION + "\"}";
    server.send(200, "application/json", j);
  });
  server.on("/api/config", HTTP_POST, [] {
    lastWebActivity = millis();
    cfg.ssid = server.arg("ssid");
    if (server.arg("pass").length())
      cfg.pass = server.arg("pass");
    if (server.arg("devId").length())
      cfg.devId = server.arg("devId");
    cfg.mqttHost = server.arg("mHost");
    if (server.arg("mPort").toInt() > 0)
      cfg.mqttPort = server.arg("mPort").toInt();
    cfg.mqttUser = server.arg("mUser");
    if (server.arg("mPass").length())
      cfg.mqttPass = server.arg("mPass");
    if (server.arg("mTopic").length())
      cfg.mqttTopic = server.arg("mTopic");
    long minutes = server.arg("sleepMin").toInt();
    if (minutes >= 1 && minutes <= 1440)
      cfg.sleepMinutes = minutes;
    saveConfig();
    recorderStatus = "Settings saved";
    server.sendHeader("Location", "/");
    server.send(303, "text/plain", "");
  });
  server.on("/api/sleep", HTTP_POST, [] {
    server.send(200, "text/html; charset=utf-8",
                "<meta charset=utf-8><p>เริ่มโหมดอัตโนมัติ บอร์ดจะวัด ส่งผล แล้วหลับ "
                "กด RST เพื่อกลับมาหน้าตั้งค่า</p>");
    sleepRequested = true;
  });
  server.onNotFound([] { server.send(404, "text/plain", "Not found"); });
  server.begin();
  Serial.printf("Open Wi-Fi: %s (no password)\n", accessPointName);
  Serial.print("Open http://");
  Serial.println(WiFi.softAPIP());
}

void captureAndSave() {
  camera_fb_t *queued = esp_camera_fb_get();
  if (queued)
    esp_camera_fb_return(queued);
  camera_fb_t *frame = esp_camera_fb_get();
  if (!frame) {
    ++captureErrors;
    recorderStatus = "Capture failed; retrying";
    return;
  }
  bool saved = savePhoto(*frame);
  uint32_t greenPixels = 0;
  uint32_t roiPixels = 0;
  bool measured = saved && measureCanopy(*frame, greenPixels, roiPixels);
  if (saved) {
    Serial.printf("Canopy AI: %s (%lu ms)\n", canopyAIState,
                  (unsigned long)canopyAIElapsedMs);
  }
  if (measured) {
    ++validMeasurements;
    batchGreenPixels += greenPixels;
    batchRoiPixels += roiPixels;
    batchCenterXSum += plantCenterX;
    batchCenterYSum += plantCenterY;
    batchConfidenceSum += plantConfidence;
  }
  size_t bytes = frame->len;
  esp_camera_fb_return(frame);
  if (!saved) {
    ++captureErrors;
    recording = false;
    photosRemaining = 0;
    recorderStatus = "SD write failed; recording stopped";
    Serial.println(recorderStatus);
    return;
  }
  if (photosRemaining > 0)
    --photosRemaining;
  if (photosRemaining == 0) {
    recording = false;
    finishMeasurementBatch();
  } else {
    recorderStatus = "Capturing 3 photos";
  }
  digitalWrite(LED_PIN, LED_ON);
  ledLit = true;
  ledStarted = millis();
  Serial.printf("Saved IMG_%08u.jpg (%u bytes)\n", photoNumber,
                (unsigned)bytes);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);
  Serial.begin(115200);
  autoMode = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER;
  if (!autoMode) {
    delay(1200);
    rtcMagic = 0; // power-on / RST: start a new run folder
  }
  loadConfig();
  Serial.printf("PlantCam FW %s, dev %s, %s mode\n", FW_VERSION,
                cfg.devId.c_str(), autoMode ? "AUTO (timer wake)" : "SETUP");
  cameraReady = startCamera();
  bool cardReady = cameraReady && startCard();
  if (cardReady)
    loadBaseline();
  if (autoMode) {
    if (!cameraReady || !cardReady)
      Serial.println(recorderStatus);
    runAutoCycle(); // never returns
  }
  lastWebActivity = millis();
  startWebServer();
  if (!cameraReady || !cardReady) {
    Serial.println(recorderStatus);
    return;
  }
  recorderStatus = "Ready; press Capture 3 Photos";
  recording = false;
  Serial.printf("Ready at %s. Send c to capture 3 photos.\n", sessionPath);
}

void loop() {
  server.handleClient();
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'c')
      startCaptureBatch();
  }
  uint32_t now = millis();
  // Leave setup mode: wake right away in AUTO mode for the first measurement.
  if (!recording &&
      (sleepRequested || now - lastWebActivity >= SETUP_IDLE_MS)) {
    delay(300); // let the HTTP reply go out
    goToSleep(1);
  }
  if (ledLit && now - ledStarted >= BLINK_MS) {
    digitalWrite(LED_PIN, LED_OFF);
    ledLit = false;
  }
  if (!recording || static_cast<int32_t>(now - nextCapture) < 0) {
    delay(1);
    return;
  }
  nextCapture += INTERVAL_MS;
  captureAndSave();
  now = millis();
  if (static_cast<int32_t>(now - nextCapture) >= 0)
    nextCapture += ((now - nextCapture) / INTERVAL_MS + 1) * INTERVAL_MS;
}
