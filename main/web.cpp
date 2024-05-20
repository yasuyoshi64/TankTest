#include <stdio.h>
#include <string.h>
#include <regex>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_http_server.h"
#include "esp_log.h"

#include "web.hpp"

#define TAG "Web"

#define STX 0x02
#define ETX 0x03

// メッセージ種別 (メッセージキュー用)
enum class WebMessage {
    Init,       // 初期化
    Start,      // 開始
    Stop,       // 停止
    WebSocketSend,       // WebSocket送信
    Quit        // 終了
};

WebServer::WebServer() {
    clear();
}

void WebServer::clear() {
    m_xHandle = NULL;
    m_xQueue = NULL;
    m_server = NULL;
    for(int i=0; i<m_apiCallbacks.size(); i++) {
        delete (ST_API_CALLBACK_DATA*)m_apiCallbacks[i];
    }
    m_apiCallbacks.clear();
    m_webSocketSessions.clear();
    while(!m_webSocketSendMessages.empty()) {
        delete[] m_webSocketSendMessages.top();
        m_webSocketSendMessages.pop();
    }
    m_webSocketCallback = NULL;
    m_webSocketCallbackContext = NULL;
}

void WebServer::init() {
    ESP_LOGI(TAG, "Init(S)");

    // タスク作成
    xTaskCreate(WebServer::task, TAG, configMINIMAL_STACK_SIZE * 2, (void*)this, tskIDLE_PRIORITY, &m_xHandle);

    // メッセージキューの初期化
    m_xQueue = xQueueCreate(10, sizeof(WebMessage));

    // 初期化用メッセージポスト
    WebMessage msg = WebMessage::Init;
    xQueueSend(m_xQueue, &msg, portMAX_DELAY);

    ESP_LOGI(TAG, "Init(E)");
}

void WebServer::start(const char* ipAddress, const char* root) {
    m_ipAddress = ipAddress;
    m_root = root;
    WebMessage msg = WebMessage::Start;
    xQueueSend(m_xQueue, &msg, portMAX_DELAY);
}

void WebServer::stop() {
    WebMessage msg = WebMessage::Stop;
    xQueueSend(m_xQueue, &msg, portMAX_DELAY);
}

// タスク
void WebServer::task(void* arg) {
    WebServer* pThis = (WebServer*)arg;
    WebMessage msg;
    bool loop = true;
    while(loop) {
        // メツセージキュー読み取り
        if (pThis->m_xQueue != NULL && xQueueReceive(pThis->m_xQueue, (void*)&msg, portMAX_DELAY) == pdTRUE) {
            switch(msg) {
                case WebMessage::Init:          // 初期化
                    pThis->webInit();
                    break;
                case WebMessage::Start:         // 開始
                    pThis->webStart();
                    break;
                case WebMessage::Stop:          // 停止
                    pThis->webStop();
                    break;
                case WebMessage::WebSocketSend: // WebSocket送信
                    pThis->webSocketSend();
                    break;
                case WebMessage::Quit:          // 終了
                    loop = false;
                    break;
            }
        }
    }
    // 終了処理
    pThis->stop();
    pThis->clear();
    vTaskDelete(NULL);
}

void WebServer::webInit() {
}

// GET "/API" ハンドラ
esp_err_t WebServer::get_api(httpd_req_t *req) {
    WebServer* pThis = (WebServer*)req->user_ctx;
    ESP_LOGI(TAG, "request uri : %s", req->uri);
    // コールバックを検索/実行
    bool isCall = false;
    std::string path = req->uri;
    std::regex pattern(R"(^.*/API/)");
    std::regex pattern2(R"(&.*)");
    path = std::regex_replace(path, pattern, "");
    path = std::regex_replace(path, pattern2, "");
    ESP_LOGI(TAG, "API request path=%s", path.c_str());
    for(int i=0; i<pThis->m_apiCallbacks.size(); i++) {
        ST_API_CALLBACK_DATA* v = pThis->m_apiCallbacks[i];
        if (v != NULL) {
            if (req->method == v->method && path == v->path) {
                ESP_LOGI(TAG, "API Call path=%s", v->path.c_str());
                v->callback(req, v->context);
                isCall = true;
                break;
            }
        }
    }
    if (isCall == false) {
        ESP_LOGI(TAG, "API NOT FOUND");
        httpd_resp_send_404(req);
    }
    return ESP_OK;
}

esp_err_t WebServer::get_api_get(httpd_req_t *req) {
    return get_api(req);
}

esp_err_t WebServer::get_api_post(httpd_req_t *req) {
    return get_api(req);
}

esp_err_t WebServer::get_api_put(httpd_req_t *req) {
    return get_api(req);
}

esp_err_t WebServer::get_api_delete(httpd_req_t *req) {
    return get_api(req);
}

esp_err_t WebServer::get_api_head(httpd_req_t *req) {
    return get_api(req);
}

esp_err_t WebServer::get_api_options(httpd_req_t *req) {
    return get_api(req);
}

esp_err_t WebServer::get_api_patch(httpd_req_t *req) {
    return get_api(req);
}

// GET "/*" ハンドラ
esp_err_t WebServer::get_root(httpd_req_t *req) {
    WebServer* pThis = (WebServer*)req->user_ctx;
    std::string contentType = pThis->getContentType(req->uri);
    const char* docRoot = "/document";
    char* path = (char*)heap_caps_malloc(pThis->m_root.size() + strlen(docRoot) + strlen(req->uri) + strlen("/index.html") + 1, MALLOC_CAP_8BIT);
    if (req->uri[strlen(req->uri)-1] == '/') {
        sprintf(path, "%s%s%sindex.html", pThis->m_root.c_str(), docRoot, req->uri);
        contentType = "text/html";
    } else {
        sprintf(path, "%s%s%s", pThis->m_root.c_str(), docRoot, req->uri);
    }
    ESP_LOGI(TAG, "request path : %s", path);
    
    FILE* fd = fopen(path, "rb");
    if (fd != NULL) {
        char* buffer = new char[1000];
        int ret;
        httpd_resp_set_type(req, contentType.c_str());
        while ((ret = fread(buffer, 1, 1000, fd)) > 0) {
            httpd_resp_send_chunk(req, buffer, ret);
        }
        httpd_resp_send_chunk(req, NULL, 0);
        delete[] buffer;
        fclose(fd);
    } else {
        ESP_LOGI(TAG, "NOT FOUND");
        httpd_resp_send_404(req);
    }
    heap_caps_free(path);
    return ESP_OK;
}

// GET "/ws" WebSocketハンドラ
esp_err_t WebServer::websocket_callback(httpd_req_t *req) {
    WebServer* pThis = (WebServer*)req->user_ctx;
    if (req->method == HTTP_GET) {
        // 始めて接続しにくるクライアントはココに一度来る
        // セッション格納
        ST_WEBSOCKET_SESSION* pSession = new ST_WEBSOCKET_SESSION {
            .isActive = true,
            .req = req
        };
        pThis->m_webSocketSessions.push_back(pSession);

        ESP_LOGI(TAG, "Handshake done, the new connection was opened");
        return ESP_OK;
    }
    // ESP_LOGI(TAG, "web socket callback");
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed to get frame len with %d", ret);
        return ret;
    }
    // ESP_LOGI(TAG, "frame len is %d", ws_pkt.len);
    if (ws_pkt.len) {
        // データ受信
        buf = (uint8_t*)calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to calloc memory for buf");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }
        // ESP_LOGI(TAG, "Got packet with message: %s", ws_pkt.payload);

        // 受信データが[STX][DATA][ETX][CHKSUM]となっていることを確認。化けたデータは読み捨てします。
        bool isOK = false;
        if (ws_pkt.len >= 3 && buf[0] == STX && buf[ws_pkt.len - 2] == ETX) {
            // チェックサムの確認
            char checksum = 0;
            for(int i=1; i<=ws_pkt.len-3; i++) {
                checksum ^= buf[i]; 
            }
            if (buf[ws_pkt.len - 1] == checksum) {
                buf[ws_pkt.len - 2] = '\0';
                isOK = true;
            } else {
                ESP_LOGI(TAG, "checksum error : %d, %d", (int)buf[ws_pkt.len - 1], (int)checksum);
            }
        } else {
            ESP_LOGI(TAG, "STX or ETX error : %d, %d", (int)buf[0], (int)buf[ws_pkt.len - 2]);
        }
        if (isOK == false) {
            if (pThis->m_webSocketCallback != NULL) {
                pThis->m_webSocketCallback("ERROR", pThis->m_webSocketCallbackContext);
            }
            free(buf);
            return ESP_OK;
        }
    }
    // ESP_LOGI(TAG, "Packet type: %d", ws_pkt.type);
    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT && strcmp((char*)&ws_pkt.payload[1], "Trigger async") == 0) {
        free(buf);
        return pThis->trigger_async_send(req->handle, req);
    }

    if (pThis->m_webSocketCallback != NULL) {
        const char* send = pThis->m_webSocketCallback((const char*)&buf[1], pThis->m_webSocketCallbackContext);
        if (send != NULL) {
            // 戻り値がNULL以外なら送信
            ws_pkt.type = HTTPD_WS_TYPE_TEXT;
            ws_pkt.payload = (uint8_t*)send;
            ws_pkt.len = strlen(send);
            ws_pkt.final = true;
            ret = httpd_ws_send_frame(req, &ws_pkt);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
            }
        }
    }
    // 以下はECHOバックするロジック
    // ret = httpd_ws_send_frame(req, &ws_pkt);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "httpd_ws_send_frame failed with %d", ret);
    // }

    free(buf);
    return ret;
}

struct async_resp_arg {
    httpd_handle_t hd;
    int fd;
};

esp_err_t WebServer::trigger_async_send(httpd_handle_t handle, httpd_req_t *req) {
    struct async_resp_arg* resp_arg = (async_resp_arg*)malloc(sizeof(struct async_resp_arg));
    if (resp_arg == NULL) {
        return ESP_ERR_NO_MEM;
    }
    resp_arg->hd = req->handle;
    resp_arg->fd = httpd_req_to_sockfd(req);
    esp_err_t ret = httpd_queue_work(handle, ws_async_send, resp_arg);
    if (ret != ESP_OK) {
        free(resp_arg);
    }
    return ret;
}

void WebServer::ws_async_send(void *arg) {
    static const char *data = "Async data";
    struct async_resp_arg *resp_arg = (async_resp_arg*)arg;
    httpd_handle_t hd = resp_arg->hd;
    int fd = resp_arg->fd;
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)data;
    ws_pkt.len = strlen(data);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(hd, fd, &ws_pkt);
    free(resp_arg);
}

// URIマッチャー
bool WebServer::custom_uri_matcher(const char* reference_uri, const char* uri_to_match, size_t match_upto) {
    if (reference_uri[strlen(reference_uri)-1] == '*') {
        return strncmp(reference_uri, uri_to_match, strlen(reference_uri) - 1) == 0;
    } else {
        return strcmp(reference_uri, uri_to_match) == 0;
    }
}

void WebServer::webStart() {
    webStop();
    // Webサーバー開始
    httpd_config_t conf = HTTPD_DEFAULT_CONFIG();
    conf.max_uri_handlers = 15;
    conf.uri_match_fn = custom_uri_matcher;
    if (httpd_start(&m_server, &conf) == ESP_OK) {
        // URLマップハンドラ登録 (API)
        httpd_uri_t api = {
            .uri      = "/API/*",
            .user_ctx = this
        };
        api.method = HTTP_GET;
        api.handler = get_api_get;      
        httpd_register_uri_handler(m_server, &api);
        api.method = HTTP_POST;
        api.handler = get_api_post;      
        httpd_register_uri_handler(m_server, &api);
        api.method = HTTP_PUT;
        api.handler = get_api_put;      
        httpd_register_uri_handler(m_server, &api);
        api.method = HTTP_DELETE;
        api.handler = get_api_delete;      
        httpd_register_uri_handler(m_server, &api);
        api.method = HTTP_HEAD;
        api.handler = get_api_head;      
        httpd_register_uri_handler(m_server, &api);
        api.method = HTTP_OPTIONS;
        api.handler = get_api_options;      
        httpd_register_uri_handler(m_server, &api);
        api.method = HTTP_PATCH;
        api.handler = get_api_patch;      
        httpd_register_uri_handler(m_server, &api);
        // WebScoketハンドラ登録
        httpd_uri_t ws = {
            .uri      = "/ws",
            .method   = HTTP_GET,
            .handler  = websocket_callback,
            .user_ctx = this,
            .is_websocket = true
        };
        httpd_register_uri_handler(m_server, &ws);
        // URLマップハンドラ登録 (ワイルドカード)
        httpd_uri_t root = {
            .uri      = "/*",
            .method   = HTTP_GET,
            .handler  = get_root,
            .user_ctx = this
        };
        httpd_register_uri_handler(m_server, &root);
    }
}

void WebServer::webStop() {
    if (m_server == NULL)
        return;
    // Webサーバー停止
    httpd_stop(m_server);
    m_server = NULL;
}

void WebServer::webSocketSend() {
    while(!m_webSocketSendMessages.empty()) {
        char* data = m_webSocketSendMessages.top();
        httpd_ws_frame_t ws_pkt = {
            .final = true,
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t*)data,
            .len = strlen(data),
        };
        for(int i=0; i<m_webSocketSessions.size(); i++) {
            ST_WEBSOCKET_SESSION* pSession = m_webSocketSessions[0];
            esp_err_t ret = httpd_ws_send_frame(pSession->req, &ws_pkt);
            if (ret != ESP_OK) {
                pSession->isActive = false;
            }
        }
        m_webSocketSessions.erase(std::remove_if(m_webSocketSessions.begin(), m_webSocketSessions.end(), [](ST_WEBSOCKET_SESSION* pSession) {
            bool isDelete = !pSession->isActive;
            delete pSession;
            return isDelete;
        }), m_webSocketSessions.end());
        delete[] data;
        m_webSocketSendMessages.pop();
    }
}

// "/API"のハンドラを登録
int WebServer::addHandler(httpd_method_t method, const char* path, CallbackWebAPIFunction callback, void* context) {
    ST_API_CALLBACK_DATA* pCallback = new ST_API_CALLBACK_DATA{
        method,
        path,
        callback,
        context
    };
    m_apiCallbacks.push_back(pCallback);
    return m_apiCallbacks.size() - 1;
}

// "/APIのハンドラを削除"
void WebServer::removeHandler(int handle) {
    delete (ST_API_CALLBACK_DATA*)m_apiCallbacks[handle];
    m_apiCallbacks[handle] = NULL;
}

// WebSocket用コールバック設定
void WebServer::setWebSocketHandler(CallbackWebSocketFunction callback, void* context) {
    m_webSocketCallback = callback;
    m_webSocketCallbackContext = context;
}

// WebSocketの接続先にデータ送信
void WebServer::sendWebSocket(const char* data) {
    char* buf = new char[strlen(data) + 1];
    strcpy(buf, data);
    m_webSocketSendMessages.push(buf);
    WebMessage msg = WebMessage::WebSocketSend;
    xQueueSend(m_xQueue, &msg, portMAX_DELAY);
}

// URIから拡張子のみを返します
const char* get_file_extension(const char* uri) {
    const char* dot = strrchr(uri, '.');
    if (!dot || dot == uri) return "";
    return dot + 1;
}

const char* WebServer::getContentType(const char* uri) {
    std::string ext = get_file_extension(uri);
    if (ext == "htm") {
        return "text/html";
    } else if (ext == "html") {
        return "text/html";
    } else if (ext == "css") {
        return "text/css";
    } else if (ext == "js") {
        return "application/javascript";
    } else if (ext == "png") {
        return "image/png";
    } else if (ext == "gif") {
        return "image/gif";
    } else if (ext == "jpg") {
        return "image/jpeg";
    } else if (ext == "ico") {
        return "image/x-icon";
    } else if (ext == "xml") {
        return "text/xml";
    } else if (ext == "pdf") {
        return "application/x-pdf";
    } else if (ext == ".zip") {
        return "application/x-zip";
    } else if (ext == "gz") {
        return "application/x-gzip";
    }
    return "text/plain";
}
