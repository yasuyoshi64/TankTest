/**
 * I2Cに接続されているSSD1306(0.96インチ 128x64 OLEDディスプレイ)のドライブ
 * 
 * 
*/
#include <stdio.h>
#include <string.h>
#include <iostream>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "extra/libs/qrcode/lv_qrcode.h"
#include "oled_display.hpp"

#define TAG "OledDisplay"

#define I2C_HOST        0
#define I2C_HW_ADDR     0x3C
#define I2C_SDA_PIN     CONFIG_SDA_PIN
#define I2C_SCL_PIN     CONFIG_SCL_PIN
#define I2C_OLED_H      128
#define I2C_OLED_V      64

// メッセージ種別 (メッセージキュー用)
enum class OledDisplayMessage {
    Init,       // 画面初期処理
    Clear,      // 画面消去
    On,         // 画面On
    Off,        // 画面Off
    QRCode,     // QRコード表示
    String,     // 文字列表示
    Quit        // 終了
};

OledDisplay::OledDisplay() {
    clear();
}

void OledDisplay::clear() {
    m_funcInitilizationComplettion = NULL;
    m_hDisp = NULL;
    m_panel_handle = NULL;
    m_xHandle = NULL;
    m_xQueue = NULL;
    m_text = "";
}

void OledDisplay::init(CallbackInitializationCompletionFunction func, void* context) {
    ESP_LOGI(TAG, "Init(S)");

    m_funcInitilizationComplettion = func;
    m_funcInitilizationComplettionContext = context;

    // タスク作成
    xTaskCreate(OledDisplay::task, TAG, configMINIMAL_STACK_SIZE * 3, (void*)this, tskIDLE_PRIORITY, &m_xHandle);

    // メッセージキューの初期化
    m_xQueue = xQueueCreate(10, sizeof(OledDisplayMessage));

    // 初期化用メッセージポスト
    OledDisplayMessage msg = OledDisplayMessage::Init;
    xQueueSend(m_xQueue, &msg, portMAX_DELAY);

   ESP_LOGI(TAG, "Init(E)");
}

// タスク
void OledDisplay::task(void* arg) {
    OledDisplay* pThis = (OledDisplay*)arg;
    OledDisplayMessage msg;
    bool loop = true;
    while(loop) {
        // メッセージキュー読み取り
        if (pThis->m_xQueue != NULL && xQueueReceive(pThis->m_xQueue, (void*)&msg, portMAX_DELAY) == pdTRUE) {
            switch(msg) {
                case OledDisplayMessage::Init:      // 画面初期化処理
                    pThis->initDisplay();
                    break;
                case OledDisplayMessage::Clear:     // 画面消去
                    lv_obj_clean(lv_scr_act());
                    break;
                case OledDisplayMessage::On:        // 画面On
                    esp_lcd_panel_disp_on_off(pThis->m_panel_handle, true);     // LCDのON/OFF  ONにする
                    break;
                case OledDisplayMessage::Off:       // 画面Off
                    esp_lcd_panel_disp_on_off(pThis->m_panel_handle, false);    // LCDのON/OFF  OFFにする
                    break;
                case OledDisplayMessage::QRCode:    // QRコード表示
                    {
                        lv_obj_t* active_screen = lv_scr_act();
                        lv_obj_clean(active_screen);
                        lv_obj_t *qr = lv_qrcode_create(active_screen, 64, lv_color_hex3(0xFF), lv_color_hex3(0x00));
                        lv_qrcode_update(qr, pThis->m_text.c_str(), pThis->m_text.size());
                        lv_obj_center(qr);
                    }
                    break;
                case OledDisplayMessage::String:    // 文字列表示
                    {
                        lv_obj_t* active_screen = lv_scr_act();
                        lv_obj_clean(active_screen);
                        lv_obj_t *label = lv_label_create(active_screen);
                        lv_label_set_text(label, pThis->m_text.c_str());
                        lv_obj_center(label);
                    }
                    break;
                case OledDisplayMessage::Quit:      // 終了
                    loop = false;
                    break;
            }
        }
    }
    // 終了処理
    pThis->clear();
    vTaskDelete(NULL);
}

// 表示内容をクリア
void OledDisplay::dispClear() {
    ESP_LOGI(TAG, "dispClear");
    OledDisplayMessage msg = OledDisplayMessage::Clear;
    xQueueSend(m_xQueue, &msg, portMAX_DELAY);
}

// ディスプレイON
void OledDisplay::dispOn() {
    ESP_LOGI(TAG, "dispOn");
    OledDisplayMessage msg = OledDisplayMessage::On;
    xQueueSend(m_xQueue, &msg, portMAX_DELAY);
}

// ディスプレイOFF
void OledDisplay::dispOff() {
    ESP_LOGI(TAG, "dispOff");
    OledDisplayMessage msg = OledDisplayMessage::Off;
    xQueueSend(m_xQueue, &msg, portMAX_DELAY);
}

// QRコード表示
void OledDisplay::dispQRCode(const char* text) {
    ESP_LOGI(TAG, "dispQRCode");
    m_text = text;
    OledDisplayMessage msg = OledDisplayMessage::QRCode;
    xQueueSend(m_xQueue, &msg, portMAX_DELAY);
}

// 文字列表示
void OledDisplay::dispString(const char* text) {
    ESP_LOGI(TAG, "dispString");
    m_text = text;
    OledDisplayMessage msg = OledDisplayMessage::String;
    xQueueSend(m_xQueue, &msg, portMAX_DELAY);
}

void OledDisplay::initDisplay() {
    ESP_LOGI(TAG, "initDisplay(S)");

    // I2Cバス作成
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,              // 
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,    // ここでプルアップできるので別途プルアップする必要なかった。
        .scl_pullup_en = GPIO_PULLUP_ENABLE,    // ここでプルアップできるので別途プルアップする必要なかった。
        .master = {
            .clk_speed = (400 * 1000)   // ESP-IDFでは1MHz以下に抑えること、SSD1306は特に上限が決まっていないが3.8MHz以上だと表示されなくなるらしい。
                                        // 左記値(0.4MHz)はサンプルのものをそのまま使用しています。
        }
    };
    ESP_ERROR_CHECK(i2c_param_config((i2c_port_t)I2C_HOST, &i2c_conf));                     // I2Cバス構成
    ESP_ERROR_CHECK(i2c_driver_install((i2c_port_t)I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));    // I2Cドライバーインストール

    // I2C用LCDパネルIO初期化
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = I2C_HW_ADDR,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_HOST, &io_config, &io_handle));

    // SSD1306用のLCDパネル作成
    m_panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1,       // リセットピンは未使用なので-1
        .bits_per_pixel = 1         // モノクロのOLEDなので1
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &m_panel_handle));

    // ここまででI2C SSD1306の初期化は完了 
    // esp_lcd_panel_io_tx_param でコマンドとパラメータを送信すればLCDを操作できます。

    // 以下はesp_lcd_panel_io.hで定義されている16進数のコマンド関数化してあるものを呼び出せます。
    // 汎用なのでLCD固有の操作ができない場合があるので、その場合はesp_lcd_panel_io_tx_paramで頑張ります。

    // LCDパネル初期化
    ESP_ERROR_CHECK(esp_lcd_panel_reset(m_panel_handle));     // パネルリセット(初期化前に呼び出し)
    ESP_ERROR_CHECK(esp_lcd_panel_init(m_panel_handle));      // ハネル初期化
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(m_panel_handle, true));     // LCDのON/OFF  ONにする
    
    // 「LVGL」https://lvgl.io/
    // マイコン用のグラフィックライブラリです。bitmapを頑張って加工する必要がなくなります。
    // ESP-IDFのコンポーネントで追加できます。idf_component.ymlにhttps://components.espressif.com/で検索できるライブラリを追加して使います。
    // ドキュメントは https://docs.lvgl.io/8.3/index.html
    //

    // espressif__esp_lvgl_port (https://components.espressif.com/components/espressif/esp_lvgl_port)
    // 上記はLVGLをESP上で動作させるためのブリッジドライバ・・・のようなもの？
    // 初期化
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);
    // 画面追加
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = m_panel_handle,
        .buffer_size = I2C_OLED_H * I2C_OLED_V,
        .double_buffer = true,
        .hres = I2C_OLED_H,
        .vres = I2C_OLED_V,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    m_hDisp = lvgl_port_add_disp(&disp_cfg);

    /* LCDの回転を180°回転に設定 */
    lv_disp_set_rotation(m_hDisp, LV_DISP_ROT_180);     // 他に LV_DISP_ROT_NONE, LV_DISP_ROT_90, LV_DISP_ROT_180, LV_DISP_ROT_270 がある。

    // "Initilize"を表示
    lv_obj_t* active_screen = lv_scr_act();
    lv_obj_clean(active_screen);
    lv_obj_t *label = lv_label_create(active_screen);
    lv_label_set_text(label, "Initilize");
    lv_obj_center(label);

    // 初期化完了
    if (m_funcInitilizationComplettion != NULL)
        m_funcInitilizationComplettion(m_funcInitilizationComplettionContext);

    ESP_LOGI(TAG, "initDisplay(E)");
}
