/**
 * Webサーバー
*/
#pragma once

#include <vector>
#include <iostream>
#include <stack>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_http_server.h"

typedef void (*CallbackWebAPIFunction)(httpd_req_t *req, void* context);
typedef char* (*CallbackWebSocketFunction)(const char* data, void* context);

struct ST_API_CALLBACK_DATA {
    httpd_method_t method;
    std::string path;
    CallbackWebAPIFunction callback;
    void* context;
};

struct ST_WEBSOCKET_SESSION {
    bool isActive;
    httpd_req_t* req;
};

class WebServer {
    public:
        WebServer();
    
    public:
        void init();
        void start(const char* ipAddress, const char* root);
        void stop();

        // "/API"用コールバック
        int addHandler(httpd_method_t method, const char* path, CallbackWebAPIFunction callback, void* context);
        void removeHandler(int handle);
        // WebSocket用コールバック
        void setWebSocketHandler(CallbackWebSocketFunction callback, void* context);
        void sendWebSocket(const char* data);   // WebSocketの接続先にデータ送信

    private:
        void clear();
        // タスク
        static void task(void* arg);
        //
        void webInit();
        void webStart();
        void webStop();
        //
        void webSocketSend();
        // Webリクエストハンドラ
        static esp_err_t get_api(httpd_req_t *req);
        static esp_err_t get_api_get(httpd_req_t *req);
        static esp_err_t get_api_post(httpd_req_t *req);
        static esp_err_t get_api_put(httpd_req_t *req);
        static esp_err_t get_api_delete(httpd_req_t *req);
        static esp_err_t get_api_head(httpd_req_t *req);
        static esp_err_t get_api_options(httpd_req_t *req);
        static esp_err_t get_api_patch(httpd_req_t *req);
        static esp_err_t get_root(httpd_req_t *req);
        static esp_err_t websocket_callback(httpd_req_t *req);
        static bool custom_uri_matcher(const char* reference_uri, const char* uri_to_match, size_t match_upto);
        esp_err_t trigger_async_send(httpd_handle_t handle, httpd_req_t *req);
        static void ws_async_send(void *arg);
        //
        const char* getContentType(const char* uri);

    private:
        TaskHandle_t m_xHandle;     // タスクハンドル
        QueueHandle_t m_xQueue;     // メッセージキュー
        httpd_handle_t m_server;    // httpdサーバー
        std::string m_ipAddress;    // IPアドレス
        std::string m_root;         // ファイルシステムルートパス
        std::vector<ST_API_CALLBACK_DATA*> m_apiCallbacks;  // "/API"用コールバック
        std::vector<ST_WEBSOCKET_SESSION*> m_webSocketSessions; // WebSocket用のセッションリスト
        std::stack<char*> m_webSocketSendMessages;          // WebSocket送信データのスタック
        CallbackWebSocketFunction m_webSocketCallback;      // WebSocket用コールバック
        void* m_webSocketCallbackContext;
};
