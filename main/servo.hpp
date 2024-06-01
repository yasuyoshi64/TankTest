/**
 * サーボコントローラー
*/
#pragma once

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
//#include "driver/mcpwm.h"
//#include "soc/mcpwm_periph.h"
#include "driver/mcpwm_prelude.h"

class Servo {
    public:
        Servo();

    public:
        static void setGroupID(int groupID);
        static void startTimer();
        void clear();
        void init(uint8_t gpio, double angle);
        void setAngle(double angle);
        double getAngle() { return m_angle; }

    private:
        // タスク
        static void task(void* arg);
        //
        void angle();

    private:
        TaskHandle_t m_xHandle; // タスクハンドル
        QueueHandle_t m_xQueue; // メッセージキュー
        uint8_t m_gpio;         // サーボ制御用GPIO
        mcpwm_cmpr_handle_t m_comparator;
        mcpwm_gen_handle_t m_generator;
        double m_angle;            // 0～180°の角度
        char tag[100];
};
