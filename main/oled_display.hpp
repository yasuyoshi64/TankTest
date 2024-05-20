/**
 * I2C OLED(有機EL) SSD1306 0.96インチ 128x64 ディスプレイ
 * 
*/
#pragma once

#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"

typedef void (*CallbackInitializationCompletionFunction)(void* context);

class OledDisplay {
    public:
        OledDisplay();
    
    public:
        void init(CallbackInitializationCompletionFunction func, void* context);

        bool isInitialize() { return m_panel_handle != NULL ? true : false; };

        void dispClear();   // 表示内容をクリア  
        void dispOn();      // ディスプレイON
        void dispOff();     // ディスプレイOFF
        void dispQRCode(const char* text);  // QRコード表示
        void dispString(const char* text);  // 文字列表示

    private:
        void clear();
        // タスク
        static void task(void* arg);
        // 
        void initDisplay();

    private:
        CallbackInitializationCompletionFunction m_funcInitilizationComplettion;
        void* m_funcInitilizationComplettionContext;
        lv_disp_t *m_hDisp;
        esp_lcd_panel_handle_t m_panel_handle;
        TaskHandle_t m_xHandle; // タスクハンドル
        QueueHandle_t m_xQueue; // メッセージキュー
        std::string m_text;     // QRコードますたはテキスト表示用
};
