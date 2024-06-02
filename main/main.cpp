#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <string.h>
#include <iostream>
#include <map>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_crt_bundle.h"
#include "esp_ota_ops.h"
#include "cJSON.h"
#include "esp_log.h"

#include "main.hpp"
#include "util.hpp"

#define TAG "Application"
#define ROOT "/mnt"

#define BUFFSIZE 1024
static char ota_write_data[BUFFSIZE + 1] = { 0 };

Application app;

// メッセージ種別 (メッセージキュー用)
enum class AppMessage {
    UpdateDisplay,      // ディスプレイに現在状態表示
    WIFIConnection,     // Wi-Fi接続
    WIFIDisconnection,  // Wi-Fi切断
    OTA,                // ファームウェアアップデート
    Quit                // 終了
};

Application::Application() {
    m_LedState = false;
    m_xHandle = NULL;
    m_xQueue = NULL;
    m_xHandleOTA = NULL;
    m_isWiFi = false;
    m_30sec_off = false;
    m_isCheckOTA = false;
    m_isOTA = false;
}

// 初期化
void Application::init() {
    ESP_LOGI(TAG, "Init(S)");

    // LED初期化
    gpio_reset_pin((gpio_num_t)CONFIG_LED_PIN);
    gpio_set_direction((gpio_num_t)CONFIG_LED_PIN, GPIO_MODE_OUTPUT);
    led(1);

    // ボタン初期化
    gpio_reset_pin((gpio_num_t)CONFIG_BTN_PIN);
    gpio_set_intr_type((gpio_num_t)CONFIG_BTN_PIN, GPIO_INTR_NEGEDGE);
    gpio_set_direction((gpio_num_t)CONFIG_BTN_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_dis((gpio_num_t)CONFIG_BTN_PIN);
    gpio_pullup_en((gpio_num_t)CONFIG_BTN_PIN);
    gpio_install_isr_service(CONFIG_BTN_PIN);
    gpio_isr_handler_add((gpio_num_t)CONFIG_BTN_PIN, btn0HandlerFunc, (void*)this);

    // タスク作成
    xTaskCreate(Application::app_task, TAG, configMINIMAL_STACK_SIZE * 2, (void*)this, tskIDLE_PRIORITY, &m_xHandle);

    // メッセージキューの初期化
    m_xQueue = xQueueCreate(10, sizeof(AppMessage));

    // モーターコントローラー
    Motor::setGroupID(0);
    m_motor_a.init((gpio_num_t)CONFIG_AIN1_PIN, (gpio_num_t)CONFIG_AIN2_PIN);
    m_motor_b.init((gpio_num_t)CONFIG_BIN1_PIN, (gpio_num_t)CONFIG_BIN2_PIN);
    Motor::startTimer();

    // サーボコントローラー
    Servo::setGroupID(1);
    m_servo_a.init(CONFIG_A_PWM_PIN, 0);
    m_servo_b.init(CONFIG_B_PWM_PIN, 0);
    Servo::startTimer();

    // SDカード初期化
    m_sd_card.init(ROOT);
    m_sd_card.setMountCallback(mountFunc, this);

    // 保存データ初期化
    m_save_data.init(ROOT);

    // OLED(SSD1306)ディスプレイ初期化
    m_oled.init(dispInitCompFunc, this);

    // Wi-Fi初期化
    m_wifi.init(wifiConnectFunc, this);

    // Webサーバー初期化
    m_web.init();
    m_web.addHandler(HTTP_GET, "get_data", getData, this);
    m_web.addHandler(HTTP_POST, "set_data", setData, this);
    m_web.addHandler(HTTP_POST, "save", save, this);
    m_web.setWebSocketHandler(sebSocketFunc, this);

    ESP_LOGI(TAG, "Init(E)");
}

// タスク
void Application::app_task(void* arg) {
    Application* pThis = (Application*)arg;
    AppMessage msg;
    bool loop = true;
    while(loop) {
        // メッセージキュー読み取り
        if (pThis->m_xQueue != NULL && xQueueReceive(pThis->m_xQueue, (void*)&msg, portMAX_DELAY) == pdTRUE) {
            switch(msg) {
                case AppMessage::UpdateDisplay:     // ディスプレイに現在状態表示
                    pThis->updateDisplay();
                    break;
                case AppMessage::WIFIConnection:    // Wi-Fi接続
                    pThis->wifiConnection();
                    break;
                case AppMessage::WIFIDisconnection: // Wi-Fi切断
                    pThis->wifiDisconnection();
                    break;
                case AppMessage::OTA:               // ファームウェアアップデート
                    pThis->ota();
                    break;
                case AppMessage::Quit:              // 終了
                    loop = false;
                    break;
            }
        }
    }
    // 終了処理
    vTaskDelete(NULL);
}

// LED点灯制御
//  state   : 1=点灯, 0=消灯
void Application::led(int state) {
    m_LedState = state;
    gpio_set_level((gpio_num_t)CONFIG_LED_PIN, m_LedState);
}

// SDカードマウントコールバック
void Application::mountFunc(bool isMount, void* context) {
    ESP_LOGI(TAG, "SD Card mount : %d", isMount);
    Application* pThis = (Application*)context;
    AppMessage msg = AppMessage::UpdateDisplay;
    xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
    if (isMount && pThis->getConfig(ROOT)) {
        AppMessage msg = AppMessage::WIFIConnection;
        xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
    } else {
        AppMessage msg = AppMessage::WIFIDisconnection;
        xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
    }
    // 保存データ読み込み
    pThis->m_save_data.read();
    // サーボトリム
    // const char* trima = pThis->m_save_data.get("servo_trim_a");
    // pThis->m_servo_a.setAngle(std::stod(trima == NULL ? "0" : trima));
    // const char* trimb = pThis->m_save_data.get("servo_trim_b");
    // pThis->m_servo_b.setAngle(std::stod(trimb == NULL ? "0" : trimb));
}

// ファイル一覧コールバック
bool Application::fileFunc(bool isFile, const char* name, void* context) {
    ESP_LOGI(TAG, "%s : %s", isFile ? "File" : "Directory", name);
    return true;
}

// ディスプレイ初期化完了コールバック
void Application::dispInitCompFunc(void* context) {
    Application* pThis = (Application*)context;
    AppMessage msg = AppMessage::UpdateDisplay;
    xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
}

// Wi-Fi接続コールバック
void Application::wifiConnectFunc(bool isConnect, void* context) {
    Application* pThis = (Application*)context;
    if (isConnect) {
        pThis->m_isWiFi = true;
        pThis->m_30sec_off = true;
        pThis->m_isCheckOTA = false;
        pThis->m_isOTA = false;
        const char* ipAddress = pThis->m_wifi.getIPAddress();
        ESP_LOGI(TAG, "IP Address: %s", ipAddress);
        pThis->led(0);
        pThis->m_web.start(ipAddress, ROOT);   // Webサーバー開始

        // 30秒後に画面を消灯するためにタイマ設定
        pThis->timer30secStart();

        // OTA
        AppMessage msg = AppMessage::OTA;
        xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
        
        // pThis->m_sd_card.fileLists("/document/", fileFunc, pThis);
    } else {
        ESP_LOGW(TAG, "wi-fi disconnect");
        pThis->m_30sec_off = pThis->m_isWiFi = false;
    }
    AppMessage msg = AppMessage::UpdateDisplay;
    xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
}

// 30秒タイマ開始
void Application::timer30secStart() {
    TimerHandle_t xTimer = xTimerCreate(
        "30SecTimer",
        pdMS_TO_TICKS(30000),
        pdFALSE,
        this,
        timer30secFunc
    );
    if (xTimer != NULL)
        xTimerStart(xTimer, 0);
}

// 30秒タイマ
void Application::timer30secFunc(TimerHandle_t xTimer) {
    Application* pThis = (Application*)pvTimerGetTimerID(xTimer);
    ESP_LOGI(TAG, "30s timer handler");
    pThis->m_30sec_off = false;
    AppMessage msg = AppMessage::UpdateDisplay;
    xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
}

// 基盤上のボタン押下ハンドラ (GPIO2)
void IRAM_ATTR Application::btn0HandlerFunc(void* context) {
    Application* pThis = (Application*)context;
    if (gpio_get_level((gpio_num_t)CONFIG_BTN_PIN) == 0) {
        if (pThis->m_30sec_off == false) {
            pThis->m_30sec_off = true;
            pThis->timer30secStart();
            AppMessage msg = AppMessage::UpdateDisplay;
            xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
        }
    }
}

// ディスプレイに現在状態表示
void Application::updateDisplay() {
    if (!m_oled.isInitialize())
        return;
    if (m_isWiFi) {
        if (m_isCheckOTA) {
            m_oled.dispString("check update");
        } else if (m_isOTA) {
            m_oled.dispString("now update");
        } else if (m_30sec_off) {
            ESP_LOGI(TAG, "QRCode output");
            const char* ipAddress = m_wifi.getIPAddress();
            char url[256];
            sprintf(url, "http://%s/", ipAddress);
            m_oled.dispQRCode(url);
        } else {
            // 画面消灯
            m_oled.dispClear();
        }
    } else if (m_sd_card.isMount()) {
        m_oled.dispString("Wi-Fi Connecting");
        led(1);
    } else {
        m_oled.dispString("No File");
        led(0);
    }
}

// Wi-Fi接続
void Application::wifiConnection() {
    wifiDisconnection();
    if (m_configMap.find("ssid") == m_configMap.end() || m_configMap.find("pass") == m_configMap.end())
        return; // CONFIGファイルにssidまたはpassの設定がない
    const char* ssid = m_configMap["ssid"].c_str();
    const char* pass = m_configMap["pass"].c_str();
    m_wifi.connect(ssid, pass);
}

// Wi-Fi切断
void Application::wifiDisconnection() {
    m_web.stop();
    m_wifi.disconnect();
}

// WebAPI GET /API/get_data
// {
//   "ip_address": "192.168.0.1",
//   "servo_trim_a": 5,
//   "servo_trim_b": 5
// }
void Application::getData(httpd_req_t *req, void* context) {
    ESP_LOGI(TAG, "getData");
    Application* pThis = (Application*)context;
    char resp[100];
    const char* ip_address = pThis->m_wifi.getIPAddress();
    const char* servo_trim_a = pThis->m_save_data.get("servo_trim_a");
    const char* servo_trim_b = pThis->m_save_data.get("servo_trim_b");
    ip_address = ip_address == NULL ? "" : ip_address;
    servo_trim_a = servo_trim_a == NULL ? "0" : servo_trim_a;
    servo_trim_b = servo_trim_b == NULL ? "0" : servo_trim_b;
    sprintf(resp, "{\"ip_address\": \"%s\", \"servo_trim_a\": %s, \"servo_trim_b\": %s}", ip_address, servo_trim_a, servo_trim_b);
    ESP_LOGI(TAG, "%s", resp);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));
}

// WebAPI POST /API/set_data
// {
//   "servo_trim_a": 5,
//   "servo_trim_b": 5
// }
void Application::setData(httpd_req_t *req, void* context) {
    ESP_LOGI(TAG, "setData");
    Application* pThis = (Application*)context;
    int ret, remaining = req->content_len;
    char body[100];
    if (remaining >= sizeof(body)) {
        ESP_LOGE(TAG, "oversize request body");
        httpd_resp_send_500(req);
        return;
    }
    ret = httpd_req_recv(req, body, remaining);
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "timeout");
        httpd_resp_send_408(req);
        return;
    }
    body[ret] = '\0';
    ESP_LOGI(TAG, "body : %s", body);
    cJSON* json = cJSON_Parse(body);
    if (json == NULL) {
        ESP_LOGI(TAG, "json null");
        httpd_resp_send_500(req);
        return;
    }
    cJSON* servo_trim_a = cJSON_GetObjectItemCaseSensitive(json, "servo_trim_a");
    if (!cJSON_IsNumber(servo_trim_a)) {
        ESP_LOGI(TAG, "json format error");
        httpd_resp_send_500(req);
        return;
    }
    double trim_a = cJSON_GetNumberValue(servo_trim_a);
    pThis->m_save_data.set("servo_trim_a", std::to_string(trim_a).c_str());

    cJSON* servo_trim_b = cJSON_GetObjectItemCaseSensitive(json, "servo_trim_b");
    if (!cJSON_IsNumber(servo_trim_b)) {
        ESP_LOGI(TAG, "json format error");
        httpd_resp_send_500(req);
        return;
    }
    double trim_b = cJSON_GetNumberValue(servo_trim_b);
    pThis->m_save_data.set("servo_trim_b", std::to_string(trim_b).c_str());

    cJSON_Delete(json);
    httpd_resp_send(req, NULL, 0);
}

// WebAPI POST /API/save
void Application::save(httpd_req_t *req, void* context) {
    ESP_LOGI(TAG, "save");
    Application* pThis = (Application*)context;
    pThis->m_save_data.save();
    httpd_resp_send(req, NULL, 0);
}

// WebSocketコールバック
char* Application::sebSocketFunc(const char* data, void* context) {
    Application* pThis = (Application*)context;
    ESP_LOGI(TAG, "data = %s", data);
    int speed = 0;
    int steering = 0;
    int servoA = 0;
    int servoB = 0;
    std::string val = data;
    if (val != "ERROR") {
        std::vector<std::string> words = split(val, ',');
        if (words.size() == 4) {
            std::vector<std::string> arrSpeed = split(words[0], '=');
            std::vector<std::string> arrSteering = split(words[1], '=');
            std::vector<std::string> arrServoA = split(words[2], '=');
            std::vector<std::string> arrServoB = split(words[3], '=');
            if (arrSpeed.size() == 2 && arrSteering.size() == 2 && arrServoA.size() == 2 && arrServoB.size() == 2) {
                //ESP_LOGI(TAG, "speed=%s, steering=%s", arrSpeed[1].c_str(), arrSteering[1].c_str());
                try {
                    speed = std::stoi(arrSpeed[1]);
                    steering = std::stoi(arrSteering[1]);
                    servoA = std::stoi(arrServoA[1]);
                    servoB = std::stoi(arrServoB[1]);
                } catch (...) {
                    speed = 0;
                    steering = 0;
                    servoA = 0;
                    servoB = 0;
                }
            }
        }
        pThis->led(0);
    } else {
        pThis->led(1);
    }
    if (speed == 0 && steering == 0) {
        pThis->m_motor_a.setDirection(MotorDirection::Stop);
        pThis->m_motor_b.setDirection(MotorDirection::Stop);
        pThis->m_motor_a.setSpeed(0);
        pThis->m_motor_b.setSpeed(0);
    } else if (speed != 0) {
        MotorDirection md = speed > 0 ? MotorDirection::Foward : MotorDirection::Back;
        pThis->m_motor_a.setDirection(md);
        pThis->m_motor_b.setDirection(md);
        int speedabs = abs(speed);
        int steeringabs = abs(steering);
        if (steering == 0) {
            // 直進
            pThis->m_motor_a.setSpeed(speedabs);
            pThis->m_motor_b.setSpeed(speedabs);
        } else if (steering > 0) {
            // 右旋回
            int speedRight = (int)((double)speedabs * ((100.0 - steeringabs) / 100.0));
            pThis->m_motor_a.setSpeed(speedRight);
            pThis->m_motor_b.setSpeed(speedabs);
        } else {
            // 左旋回
            int speedLeft = (int)((double)speedabs * ((100.0 - steeringabs) / 100.0));
            pThis->m_motor_a.setSpeed(speedabs);
            pThis->m_motor_b.setSpeed(speedLeft);
        }
    } else {
        // 超信地旋回
        int steeringabs = abs(steering);
        if (steering > 0) {
            // 右旋回
            pThis->m_motor_a.setDirection(MotorDirection::Back);
            pThis->m_motor_b.setDirection(MotorDirection::Foward);
        } else {
            // 左旋回
            pThis->m_motor_a.setDirection(MotorDirection::Foward);
            pThis->m_motor_b.setDirection(MotorDirection::Back);
        }
        pThis->m_motor_a.setSpeed(steeringabs);
        pThis->m_motor_b.setSpeed(steeringabs);
    }
    pThis->m_servo_a.setAngle(servoA);
    pThis->m_servo_b.setAngle(servoB);
    return NULL;
}

void Application::ota() {
    if (m_xHandleOTA != NULL)
        return;
    xTaskCreate(Application::checkOTA, TAG, 8192, (void*)this, tskIDLE_PRIORITY, &m_xHandle);
}

// アップデートチェック
void Application::checkOTA(void* arg) {
    Application *pThis = (Application*)arg;
    const char* updateuri = pThis->m_configMap["updateuri"].c_str();
    ESP_LOGI(TAG, "update uri = %s", updateuri);

    pThis->m_isCheckOTA = true;
    pThis->m_isOTA = false;
    AppMessage msg = AppMessage::UpdateDisplay;
    xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
    
    esp_http_client_config_t config = {
        .url = updateuri,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .keep_alive_enable = true,
    };

    // バージョンチェック有OTA
    esp_err_t err;
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = NULL;
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialise HTTP connection");
        ESP_ERROR_CHECK(ESP_FAIL);
    }
    ESP_ERROR_CHECK(esp_http_client_open(client, 0));
    esp_http_client_fetch_headers(client);

    update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        ESP_LOGE(TAG, "update partition NULL");
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    int binary_file_length = 0;
    bool image_header_was_checked = false;
    while(1) {
        int data_read = esp_http_client_read(client, ota_write_data, BUFFSIZE);
        int http_status = esp_http_client_get_status_code(client);
        if (http_status == 302) {
            // リダイレクト
            esp_http_client_set_redirection(client);
            ESP_ERROR_CHECK(esp_http_client_open(client, 0));
            esp_http_client_fetch_headers(client);
            continue;
        }
        if (data_read < 0) {
            ESP_LOGE(TAG, "Error: SSL data read error");
            esp_http_client_close(client);
            esp_http_client_cleanup(client); 
        } else if (data_read > 0) {
            if (image_header_was_checked == false) {
                esp_app_desc_t new_app_info;
                if (data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t)) {
                    memcpy(&new_app_info, &ota_write_data[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                    ESP_LOGI(TAG, "New firmware version: %s", new_app_info.version);

                    esp_app_desc_t running_app_info;
                    if (esp_ota_get_partition_description(running, &running_app_info) == ESP_OK) {
                        ESP_LOGI(TAG, "Running firmware version: %s", running_app_info.version);
                    }

                    const esp_partition_t* last_invalid_app = esp_ota_get_last_invalid_partition();
                    esp_app_desc_t invalid_app_info;
                    if (esp_ota_get_partition_description(last_invalid_app, &invalid_app_info) == ESP_OK) {
                        ESP_LOGI(TAG, "Last invalid firmware version: %s", invalid_app_info.version);
                    }

                    // check current version with last invalid partition
                    if (last_invalid_app != NULL) {
                        if (memcmp(invalid_app_info.version, new_app_info.version, sizeof(new_app_info.version)) == 0) {
                            ESP_LOGW(TAG, "New version is the same as invalid version.");
                            ESP_LOGW(TAG, "Previously, there was an attempt to launch the firmware with %s version, but it failed.", invalid_app_info.version);
                            ESP_LOGW(TAG, "The firmware has been rolled back to the previous version.");
                            pThis->m_isCheckOTA = false;
                            xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
                            esp_http_client_close(client);
                            esp_http_client_cleanup(client);
                            vTaskDelete(NULL);
                        }
                    }

                    if (memcmp(new_app_info.version, running_app_info.version, sizeof(new_app_info.version)) == 0) {
                        ESP_LOGW(TAG, "Current running version is the same as a new. We will not continue the update.");
                        pThis->m_isCheckOTA = false;
                        xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
                        esp_http_client_close(client);
                        esp_http_client_cleanup(client);
                        vTaskDelete(NULL);
                    }

                    image_header_was_checked = true;

                    err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &update_handle);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
                        esp_http_client_close(client);
                        esp_http_client_cleanup(client);
                        esp_ota_abort(update_handle);
                        ESP_ERROR_CHECK(ESP_FAIL);
                    }
                    ESP_LOGI(TAG, "esp_ota_begin succeeded");                    
                } else {
                    ESP_LOGE(TAG, "received package is not fit len");
                    esp_http_client_close(client);
                    esp_http_client_cleanup(client);
                    ESP_ERROR_CHECK(ESP_FAIL);
                }
            }
            err = esp_ota_write( update_handle, (const void *)ota_write_data, data_read);
            if (err != ESP_OK) {
                esp_http_client_close(client);
                esp_http_client_cleanup(client);
                esp_ota_abort(update_handle);
                ESP_ERROR_CHECK(ESP_FAIL);
            }
            binary_file_length += data_read;
            ESP_LOGD(TAG, "Written image length %d", binary_file_length);
        } else if (data_read == 0) {
            if (errno == ECONNRESET || errno == ENOTCONN) {
                ESP_LOGE(TAG, "Connection closed, errno = %d", errno);
                break;
            }
            if (esp_http_client_is_complete_data_received(client) == true) {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }            
        }
    }

    pThis->m_isCheckOTA = false;
    pThis->m_isOTA = true;
    xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);

    ESP_LOGI(TAG, "Total Write binary data length: %d", binary_file_length);
    if (esp_http_client_is_complete_data_received(client) != true) {
        ESP_LOGE(TAG, "Error in receiving complete file");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        esp_ota_abort(update_handle);
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
            ESP_LOGE(TAG, "Image validation failed, image is corrupted");
        } else {
            ESP_LOGE(TAG, "esp_ota_end failed (%s)!", esp_err_to_name(err));
        }
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        ESP_ERROR_CHECK(ESP_FAIL);
    }
    ESP_LOGI(TAG, "Prepare to restart system!");
    esp_restart();
}

extern "C" void app_main(void)
{
    esp_err_t err = nvs_flash_init();     // Flash初期化  (お約束のようなもの)
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    app.init();
}
