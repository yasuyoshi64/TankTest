/**
 * モーターコントローラー
*/
#pragma once

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"

enum class MotorDirection {
    Foward,     // 前進
    Back,       // 後退
    Stop,       // 停止
    Brake       // ブレーキ
};

class Motor {
    public:
        Motor();

    public:
        void clear();
        void init(int groupID, gpio_num_t gpioINA, gpio_num_t gpioINB);
        void setDirection(MotorDirection md);
        MotorDirection getDirection() { return m_md; }
        void setSpeed(int speed);
        int getSpeed() { return m_speed; }

    private:
        // タスク
        static void task(void* arg);
        //
        void speed();

    private:
        TaskHandle_t m_xHandle; // タスクハンドル
        QueueHandle_t m_xQueue; // メッセージキュー
        int m_groupID;
        gpio_num_t m_gpioINA;   // 制御用GPIO
        gpio_num_t m_gpioINB;   // 制御用GPIO
        MotorDirection m_md;
        mcpwm_timer_handle_t m_timer;
        mcpwm_oper_handle_t m_oper;
        mcpwm_cmpr_handle_t m_comparator;
        mcpwm_gen_handle_t m_generator;
        int m_speed;            // 0～100%
};
