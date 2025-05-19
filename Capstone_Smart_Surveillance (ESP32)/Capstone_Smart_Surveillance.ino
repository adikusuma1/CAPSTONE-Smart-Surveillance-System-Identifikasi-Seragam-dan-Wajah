#include <CAPSTONE_Prediksi_Seragam_-_Non_Seragam_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include <WiFi.h>
#include <esp_task_wdt.h>

#define UPLOAD_TIMEOUT 60

#define CAMERA_MODEL_AI_THINKER 
#include "camera_pins.h"
#include "esp_camera.h"

#define BLYNK_TEMPLATE_ID "TMPL6rdWujWvm"
#define BLYNK_TEMPLATE_NAME "CAPSTONE LabCAM"
#define BLYNK_AUTH_TOKEN "D2gK6GgemEHDPrcZAIV0ROQl8KKz_xx_"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

const char* serverName = "http://54.80.42.231:5000/recognize-face";
#include <Base64.h>
#include <HTTPClient.h>

#define FLASH_PIN 4

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Seannamon";
char pass[] = "adiyaadi1";

WebServer server(80);

const char HEADER[] = "HTTP/1.1 200 OK\r\n" \
                      "Access-Control-Allow-Origin: *\r\n" \
                      "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n";
const char BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
const char CTNTTYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";
const int hdrLen = strlen(HEADER);
const int bdrLen = strlen(BOUNDARY);
const int cntLen = strlen(CTNTTYPE);

void setupWatchdog(uint32_t timeout_sec) {
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = timeout_sec * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // Memantau semua core
    .trigger_panic = true
  };
  esp_task_wdt_init(&twdt_config);
  esp_task_wdt_add(NULL); // Tambahkan task utama ke WDT
}

void handle_jpg_stream(void) {
  WiFiClient client = server.client();
  
  client.write(HEADER, hdrLen);
  client.write(BOUNDARY, bdrLen);

  while (true) {
    if (!client.connected()) break;
    
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      break;
    }
    
    client.write(CTNTTYPE, cntLen);
    client.printf("%d\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.write(BOUNDARY, bdrLen);
    
    esp_camera_fb_return(fb);
  }
}

const char JHEADER[] = "HTTP/1.1 200 OK\r\n" \
                       "Content-disposition: inline; filename=capture.jpg\r\n" \
                       "Content-type: image/jpeg\r\n\r\n";
const int jhdLen = strlen(JHEADER);

void handle_jpg(void) {
    WiFiClient client = server.client();

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb || !client.connected()) {
        if (fb) esp_camera_fb_return(fb);
        return;
    }

    client.write(JHEADER, jhdLen);
    client.write(fb->buf, fb->len);
    esp_camera_fb_return(fb);
}

void handleNotFound(){
  String message = "Server is running!\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  server.send(200, "text / plain", message);
}

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#else
#error "Camera model not selected"
#endif

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf = nullptr;
camera_fb_t *fb = nullptr;
int LED_GREEN = 12;
int BUZZER = 15;
int LED_RED = 13;

hw_timer_t *timer = NULL;
volatile bool watchdogTriggered = false;
unsigned long last_seragam_time = 0;

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;
bool detect_hardware_fault(camera_fb_t *fb, uint64_t* processing_time);

void setup() {
    Serial.begin(115200);
    
    pinMode(FLASH_PIN, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(LED_RED, OUTPUT);

    // Inisialisasi kamera hanya sekali
    if (!ei_camera_init()) {
        ei_printf("Failed to initialize Camera!\r\n");
        return;
    }
    ei_printf("Camera initialized\r\n");
    
    // Setup WiFi dan server
    IPAddress ip;
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    digitalWrite(LED_RED, 1);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Menghubungkan ke WiFi...");
    }
    ip = WiFi.localIP();
    digitalWrite(LED_RED, 0);
    Serial.println("Terhubung ke WiFi");
    
    Blynk.begin(auth, ssid, pass);

    Serial.println(ip);
    Serial.print("Stream Link: http://");
    Serial.print(ip);
    Serial.println("/mjpeg/1");
    
    server.on("/mjpeg/1", HTTP_GET, handle_jpg_stream);
    server.on("/jpg", HTTP_GET, handle_jpg);
    server.onNotFound(handleNotFound);
    server.begin();

    ei_sleep(500);
    setupWatchdog(UPLOAD_TIMEOUT);
}

void loop(){
    float upload_time_sec = 0.0f;    // For V8 (upload time)
    float prediction_time_sec = 0.0f; // For V7 (model prediction time)
    float server_time_sec = 0.0f; 
    esp_task_wdt_reset();
    
    while (WiFi.status() != WL_CONNECTED) {
        digitalWrite(LED_RED, 1);
        esp_task_wdt_reset();
        delay(1000);
        Serial.println("Menghubungkan kembali...");
    }
    digitalWrite(LED_RED, 0);
    
    int facerecog = -1;
    digitalWrite(LED_RED, 0);
    digitalWrite(LED_GREEN, 0);
    digitalWrite(BUZZER, 0);

    server.handleClient();
    Blynk.run();

    delay(500);
    if (ei_sleep(5) != EI_IMPULSE_OK) return;
    
    uint64_t start_capture = esp_timer_get_time();
    fb = esp_camera_fb_get();
    uint64_t end_capture = esp_timer_get_time();
    if (!fb) {
        Serial.println("Gagal mengambil gambar");
        return;
    }
    uint64_t capture_time = end_capture - start_capture;

    uint64_t alt_logic_time;
    bool fault_detected = detect_hardware_fault(fb, &alt_logic_time);

    if(fault_detected) {
        Blynk.logEvent("cam-error","ALERT: Kamera error (bit stuck/Shorted pin)");
        esp_camera_fb_return(fb);
        // Print total time including fault detection
        return;
    }
    //Serial.printf("Waktu capture kamera: %.2f ms\n", capture_time/1000.0);
    Serial.printf("Waktu Alternating Logic: %.2f ms\n", alt_logic_time/1000.0);
    Serial.printf("Total waktu: %.2f ms\n", (capture_time+alt_logic_time)/1000.0);

    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin(serverName);
        http.addHeader("Content-Type", "application/json");

        String imageBase64 = base64::encode(fb->buf, fb->len);
        String jsonPayload = "{\"image_base64\":\"" + imageBase64 + "\"}";

        
        unsigned long start_total_http = millis();
        unsigned long start_upload = millis();
        esp_task_wdt_reset();
        int httpResponseCode = http.POST(jsonPayload);
        esp_task_wdt_reset();
        unsigned long end_upload = millis();
        upload_time_sec = (end_upload - start_upload) / 1000.0;  // Convert to seconds as float
        Blynk.virtualWrite(V8, upload_time_sec);
        Serial.printf("Waktu upload gambar ESP32 -> Server (POST): %.3f s\n", upload_time_sec);
        
        if (httpResponseCode > 0) {
            String response = http.getString();
            facerecog = extractFacesInfo(response);
            //Serial.println("Response dari server: " + response);
        } else {
            Serial.print("Error pada HTTP: ");
            Serial.println(httpResponseCode);
        }
        unsigned long end_total_http = millis();
        //Serial.printf("Waktu total ESP32 -> Server (Predict) -> ESP32: %lu ms\n", end_total_http - start_total_http);
        server_time_sec = ((end_total_http - start_total_http)/1000.0);
        Blynk.virtualWrite(V6, server_time_sec);
        Serial.printf("Waktu Server (Predict) -> ESP32: %.3f s\n", server_time_sec);
        http.end();
    }

    //Serial.printf("\nINPUT : facerecog: %d", facerecog);
    //Serial.printf("\n======================\n", facerecog);

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);
    if (!snapshot_buf) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        esp_camera_fb_return(fb); // selalu kembalikan buffer
        return;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);
    if (!converted) {
        ei_printf("Conversion failed\n");
        free(snapshot_buf);
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    ei_impulse_result_t result = { 0 };
    unsigned long start_seragam = millis();
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    unsigned long end_seragam = millis();

    prediction_time_sec = (end_seragam - start_seragam) / 1000.0;  
    Blynk.virtualWrite(V7, prediction_time_sec);

    //Serial.printf("Waktu prediksi seragam ESP32: %.3f s\n", prediction_time_sec);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }
    //Serial.printf("Waktu prediksi seragam ESP32: %lu ms\n", end_seragam - start_seragam);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    float total_seragam = 0.0f;
    float total_non_seragam = 0.0f;
    int count_seragam = 0;
    int count_non_seragam = 0;

    int non_object = 1;
    int non_seragam = 0;
    int seragam = 0;

    float avg_seragam = 0;
    float avg_non_seragam = 0;

    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);

        if (bb.value >= 0.8 && strcmp(bb.label, "Seragam-Lab") == 0) {
            total_seragam += bb.value;
            count_seragam++;
        } else if (bb.value >= 0.8 && strcmp(bb.label, "Non Seragam-Lab") == 0) {
            total_non_seragam += bb.value;
            count_non_seragam++;
        }

        if (count_seragam > 0) {
            avg_seragam = total_seragam / count_seragam;
        } else { 
            avg_seragam = 0.0f;
        }

        if (count_non_seragam > 0) {
            avg_non_seragam = total_non_seragam / count_non_seragam;
        } else {
            avg_non_seragam = 0.0f;
        }

        if (count_seragam == 0 && count_non_seragam == 0) {
            non_object = 1;  
            Blynk.virtualWrite(V1,"NULL");
            Blynk.virtualWrite(V2,0);
        } else {
            if (avg_seragam > avg_non_seragam && avg_seragam >= 0.8) {
                seragam = 1;
                non_seragam = 0;
                non_object = 0;
                Blynk.virtualWrite(V1,"Seragam-Lab");
                Blynk.virtualWrite(V2,avg_seragam);
            } else if (avg_non_seragam > avg_seragam && avg_non_seragam >= 0.8) {
                seragam = 0;
                non_seragam = 1;
                non_object = 0;
                Blynk.virtualWrite(V1,"Non Seragam-Lab");
                Blynk.virtualWrite(V2,avg_non_seragam);
            } else {
                non_object = 1;  
                Blynk.virtualWrite(V1,"NULL");
                Blynk.virtualWrite(V2,0);
            }
        }
    }
    ei_printf("=== Statistik Deteksi ===\r\n");
    ei_printf("Seragam:     %d deteksi (rata-rata: %.3f)\r\n", count_seragam, avg_seragam);
    ei_printf("Non-Seragam: %d deteksi (rata-rata: %.3f)\r\n", count_non_seragam, avg_non_seragam);
    ei_printf("\r\n");

    ei_printf("=== Hasil Klasifikasi ===\r\n");
    ei_printf("Non-Object:  %d\r\n", non_object);
    ei_printf("Non-Seragam: %d\r\n", non_seragam);
    ei_printf("Seragam:     %d\r\n", seragam);
    ei_printf("=======================\r\n\r\n");

#endif
    free(snapshot_buf);
    if (facerecog == 1) {
        if (seragam == 1 || non_object == 1) {
            Serial.println("Wajah Terdeteksi & Seragam");
            digitalWrite(LED_GREEN, 1);
            digitalWrite(BUZZER, 0);
        } 
        else if (non_seragam == 1) {
            Serial.println("Non Seragam");
            Blynk.logEvent("notify_detection", "[False] Seragam tidak sesuai");
            digitalWrite(LED_GREEN, 0);
            digitalWrite(BUZZER, 1);
        }
    } 
    else if (facerecog == 2) {
        Blynk.virtualWrite(V3,"Unknown");
        Serial.println("Unknown Face");
        Blynk.logEvent("notify_detection", "[Unknown] Wajah Tidak Dikenali");
        digitalWrite(LED_GREEN, 0);
        digitalWrite(BUZZER, 1);
    } 
    else if (facerecog == -1) {
        Blynk.virtualWrite(V3,"NULL");
        Serial.println("NULL");
        digitalWrite(LED_GREEN, 0);
        digitalWrite(BUZZER, 0);
    }
    delay(100);
}

int extractFacesInfo(String response) {
    if (response.indexOf("\"num_faces\":0") != -1 || response.indexOf("\"faces\":[]") != -1) {
        return -1;
    }

    int nameStart = response.indexOf("\"name\":\"") + 8;
    if (nameStart < 8) return -1;

    int nameEnd = response.indexOf("\"", nameStart);
    String name = response.substring(nameStart, nameEnd);
    Blynk.virtualWrite(V3,name);
    if (name == "Unknown") {
        return 2;
    }

    int simStart = response.indexOf("\"similarity\":");
    if (simStart == -1) return -1;

    simStart = response.indexOf(":", simStart) + 1;
    int simEnd = simStart;

    while (simEnd < response.length()) {
        char c = response.charAt(simEnd);
        if (c == ',' || c == '}' || c == ' ') {
            break;
        }
        simEnd++;
    }

    String simStr = response.substring(simStart, simEnd);
    simStr.trim();

    if (simStr == "null") return -1;

    float similarity = simStr.toFloat();
    Serial.print("Debug - Similarity extracted: ");
    Serial.println(similarity, 4);
    
    if (similarity < 0.4) {
        float acc= 0.0f;
        acc = 1-similarity;
        Blynk.virtualWrite(V4,acc);
        return 1;
    }
    return 0; 
}

bool detect_hardware_fault(camera_fb_t *fb, uint64_t* processing_time) {
    uint64_t start = esp_timer_get_time();
    bool fault_detected = false;

    uint8_t* inverted_buf = (uint8_t*)malloc(fb->len);
    if (!inverted_buf) {
        //alocated memory fail
        *processing_time = 0;
        return false;
    }

    for (size_t i = 0; i < fb->len; i++) {
        inverted_buf[i] = ~(fb->buf[i]);
    }

    for (size_t i = 0; i < fb->len; i++) {
        if (fb->buf[i] == inverted_buf[i]) {
            fault_detected = true;
            break;
        }
    }

    *processing_time = esp_timer_get_time() - start;
    free(inverted_buf);

    return fault_detected;
}


bool ei_camera_init(void) {
    if (is_initialised) return true;

    #if defined(CAMERA_MODEL_ESP_EYE)
        pinMode(13, INPUT_PULLUP);
        pinMode(14, INPUT_PULLUP);
    #endif

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x\n", err);
      return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1);
      s->set_brightness(s, 1); 
      s->set_saturation(s, 0); 
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}

void ei_camera_deinit(void) {
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK){
        ei_printf("Camera deinit failed\n");
        return;
    }
    is_initialised = false;
    return;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;
    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }
    digitalWrite(FLASH_PIN, HIGH);   // Nyalakan flash
    camera_fb_t *fb = esp_camera_fb_get();
    digitalWrite(FLASH_PIN, LOW);

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }
    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

    esp_camera_fb_return(fb);
    
    if(!converted){
       ei_printf("Conversion failed\n");
       return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }
    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr){
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
