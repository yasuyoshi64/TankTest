#pragma once

#include <iostream>
#include <map>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "sd_card.hpp"
#include "oled_display.hpp"
#include "wifi.hpp"
#include "web.hpp"
#include "motor.hpp"
#include "servo.hpp"
#include "save_data.hpp"

class Application {
    public:
        Application();

    public:
        void init();
        void led(int state);

    private:
        // タスク
        static void app_task(void* arg);
        // コールバック        
        static void mountFunc(bool isMount, void* context);
        static bool fileFunc(bool isFile, const char* name, void* context);
        static void dispInitCompFunc(void* context);
        static void wifiConnectFunc(bool isConnect, void* context);
        void timer30secStart();
        static void timer30secFunc(TimerHandle_t xTimer);
        static void btn0HandlerFunc(void* context);
        // Webコールバック
        static void getData(httpd_req_t *req, void* context);
        static void setData(httpd_req_t *req, void* context);
        static void save(httpd_req_t *req, void* context);
        // WebSocketコールバック
        static char* sebSocketFunc(const char* data, void* context);
        //
        bool getConfig(const char* root);   // SDカード内の./configファイル読み込み。結果はconfigMapに格納。
        void updateDisplay();               // ディスプレイ更新
        void wifiConnection();              // Wi-Fi接続
        void wifiDisconnection();           // Wi-Fi切断

    private:
        int m_LedState;
        TaskHandle_t m_xHandle; // タスクハンドル
        QueueHandle_t m_xQueue; // メッセージキュー
        SDCard m_sd_card;   // SDカード
        OledDisplay m_oled; // OLED(SSD1306)ディスプレイ
        WiFi m_wifi;        // Wi-Fi
        WebServer m_web;    // Webサーバー
        Motor m_motor_a;    // モーター
        Motor m_motor_b;    // モーター
        Servo m_servo_a;    // サーボコントローラー
        Servo m_servo_b;    // サーボコントローラー
        SaveData m_save_data;   // データ保存
        std::map<std::string, std::string> m_configMap{};     // CONFIG
        bool m_isWiFi;
        bool m_30sec_off;
};