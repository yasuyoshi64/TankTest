/**
 * モーターコントローラー
*/
#pragma once

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
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
        static void setGroupID(int groupID);
        static void startTimer();
        void clear();
        void init(gpio_num_t gpioINA, gpio_num_t gpioINB);
        void setDirection(MotorDirection md);
        MotorDirection getDirection() { return m_md; }
        void setSpeed(int speed);
        int getSpeed() { return m_speed; }

    private:
        // タスク
        static void task(void* arg);
        //
        void speed();
        // コールバック
        static void timer100msFunc(TimerHandle_t xTimer);

    private:
        TaskHandle_t m_xHandle; // タスクハンドル
        QueueHandle_t m_xQueue; // メッセージキュー
        gpio_num_t m_gpioINA;   // 制御用GPIO
        gpio_num_t m_gpioINB;   // 制御用GPIO
        MotorDirection m_md;
        mcpwm_cmpr_handle_t m_comparator;
        mcpwm_gen_handle_t m_generator;
        int m_speedTarget;
        int m_speed;            // 0～100%
        char tag[100];
};
