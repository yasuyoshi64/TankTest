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
#include "cJSON.h"
#include "esp_log.h"

#include "main.hpp"
#include "util.hpp"

#define TAG "Application"
#define ROOT "/mnt"

Application app;

// メッセージ種別 (メッセージキュー用)
enum class AppMessage {
    UpdateDisplay,      // ディスプレイに現在状態表示
    WIFIConnection,     // Wi-Fi接続
    WIFIDisconnection,  // Wi-Fi切断
    Quit                // 終了
};

Application::Application() {
    m_LedState = false;
    m_xHandle = NULL;
    m_xQueue = NULL;
    m_isWiFi = false;
    m_30sec_off = false;
}

// 初期化
void Application::init() {
    ESP_LOGI(TAG, "Init(S)");

    // LED初期化
    gpio_reset_pin((gpio_num_t)CONFIG_LED_PIN);
    gpio_set_direction((gpio_num_t)CONFIG_LED_PIN, GPIO_MODE_OUTPUT);
    led(0);

    // ボタン初期化(GPIO0)
    gpio_reset_pin((gpio_num_t)0);
    gpio_set_intr_type((gpio_num_t)0, GPIO_INTR_NEGEDGE);
    gpio_set_direction((gpio_num_t)0, GPIO_MODE_INPUT);
    gpio_pulldown_dis((gpio_num_t)0);
    gpio_pullup_en((gpio_num_t)0);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)0, btn0HandlerFunc, (void*)this);

    // タスク作成
    xTaskCreate(Application::app_task, TAG, configMINIMAL_STACK_SIZE * 2, (void*)this, tskIDLE_PRIORITY, &m_xHandle);

    // メッセージキューの初期化
    m_xQueue = xQueueCreate(10, sizeof(AppMessage));

    // SDカード初期化
    m_sd_card.init(ROOT);
    m_sd_card.setMountCallback(mountFunc, this);

    // 保存データ初期化
    m_save_data.init(ROOT);

    // モーターコントローラー
    m_motor_a.init(0, (gpio_num_t)CONFIG_AIN1_PIN, (gpio_num_t)CONFIG_AIN2_PIN);
    m_motor_b.init(1, (gpio_num_t)CONFIG_BIN1_PIN, (gpio_num_t)CONFIG_BIN2_PIN);

    // サーボコントローラー
    // m_servo_a.init(CONFIG_A_PWM_PIN, 0);
    // m_servo_b.init(CONFIG_B_PWM_PIN, 0);

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
        pThis->m_30sec_off = pThis->m_isWiFi = true;
        const char* ipAddress = pThis->m_wifi.getIPAddress();
        ESP_LOGI(TAG, "IP Address: %s", ipAddress);
        pThis->led(0);
        pThis->m_web.start(ipAddress, ROOT);   // Webサーバー開始

        // 30秒後に画面を消灯するためにタイマ設定
        pThis->timer30secStart();

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

// 基盤上のボタン押下ハンドラ (GPIO0)
void IRAM_ATTR Application::btn0HandlerFunc(void* context) {
    Application* pThis = (Application*)context;
    if (pThis->m_30sec_off == false) {
        pThis->m_30sec_off = true;
        pThis->timer30secStart();
        AppMessage msg = AppMessage::UpdateDisplay;
        xQueueSend(pThis->m_xQueue, &msg, portMAX_DELAY);
    }
}

// ディスプレイに現在状態表示
void Application::updateDisplay() {
    if (!m_oled.isInitialize())
        return;
    if (m_isWiFi) {
        if (m_30sec_off) {
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
    //ESP_LOGI(TAG, "data = %s", data);
    int speed = 0;
    int steering = 0;
    std::string val = data;
    if (val != "ERROR") {
        std::vector<std::string> words = split(val, ',');
        if (words.size() == 2) {
            std::vector<std::string> arrSpeed = split(words[0], '=');
            std::vector<std::string> arrSteering = split(words[1], '=');
            if (arrSpeed.size() == 2 && arrSteering.size() == 2) {
                //ESP_LOGI(TAG, "speed=%s, steering=%s", arrSpeed[1].c_str(), arrSteering[1].c_str());
                try {
                    speed = std::stoi(arrSpeed[1]);
                    steering = std::stoi(arrSteering[1]);
                } catch (...) {
                    speed = 0;
                    steering = 0;
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

    // char rl = data[0];
    // std::string str = &data[1];
    // int speed = std::stoi(str);
    // MotorDirection md = MotorDirection::Stop;
    // if (speed == 0) {
    //     md = MotorDirection::Stop;
    // } else if (speed > 0) {
    //     md = MotorDirection::Foward;
    // } else if (speed < 0) {
    //     md = MotorDirection::Back;
    //     speed *= -1;
    // }
    // if (rl == 'R') {
    //     pThis->m_motor_a.setDirection(md);
    //     pThis->m_motor_a.setSpeed(speed);
    // } else {
    //     pThis->m_motor_b.setDirection(md);
    //     pThis->m_motor_b.setSpeed(speed);
    // }
    return NULL;
}

extern "C" void app_main(void)
{
    nvs_flash_init();     // Flash初期化  (お約束のようなもの)

    app.init();
}
