#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "driver/ledc.h"
#include "esp_mac.h"

//=====ssidとパスワードの設定=====
#define A "Pixel_6661"
#define a "1234567890"
#define B "aterm-79aeda-g"
#define b "83f9da3da2bd0"

// ===== WiFi設定 =====
#define WIFI_SSID      B
#define WIFI_PASS       b
#define WIFI_MAX_RETRY 5

// ===== モータードライバ用ピン設定 =====
// L298N Hブリッジモータードライバ用
#define MOTOR_LEFT_FWD    GPIO_NUM_12  // 左モーター前進
#define MOTOR_LEFT_BWD    GPIO_NUM_13  // 左モーター後退
#define MOTOR_RIGHT_FWD   GPIO_NUM_14  // 右モーター前進
#define MOTOR_RIGHT_BWD   GPIO_NUM_15  // 右モーター後退

// ===== PWM設定 =====
#define PWM_FREQ          1000              // 1kHz
#define PWM_RESOLUTION    LEDC_TIMER_8_BIT  // 8bit = 0-255
#define PWM_CH_LF         LEDC_CHANNEL_0
#define PWM_CH_LB         LEDC_CHANNEL_1
#define PWM_CH_RF         LEDC_CHANNEL_2
#define PWM_CH_RB         LEDC_CHANNEL_3

// ===== モーター速度制限 =====
#define MOTOR_SPEED_MIN   0
#define MOTOR_SPEED_MAX   255
#define MOTOR_SPEED_DEFAULT 200

// ===== タイムアウト設定 =====
#define MOTOR_TIMEOUT_MS  10000  // 5秒後に自動停止

static const char *TAG = "MOTOR_CTRL";
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static int retry_count = 0;
static TickType_t last_command_time = 0;

// ===== 速度制限関数 =====
static int constrain_speed(int speed)
{
    if (speed > MOTOR_SPEED_MAX) return MOTOR_SPEED_MAX;
    if (speed < -MOTOR_SPEED_MAX) return -MOTOR_SPEED_MAX;
    return speed;
}

// ===== WiFiイベントハンドラ =====
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi接続開始");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            retry_count++;
            ESP_LOGW(TAG, "WiFi再接続中... (%d/%d)", retry_count, WIFI_MAX_RETRY);
        } else {
            ESP_LOGE(TAG, "WiFi接続失敗");
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "IPアドレス取得: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ===== WiFi初期化 =====
esp_err_t wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();
    if (!wifi_event_group) {
        ESP_LOGE(TAG, "イベントグループ作成失敗");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    return ESP_OK;
}

// ===== モーター制御関数（改善版） =====
void motor_control(int left_speed, int right_speed)
{
    // 速度を-255〜255の範囲に制限
    left_speed = constrain_speed(left_speed);
    right_speed = constrain_speed(right_speed);
    
    // 左モーター制御
    if (left_speed > 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_LF, left_speed);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_LB, 0);
    } else if (left_speed < 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_LF, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_LB, -left_speed);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_LF, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_LB, 0);
    }
    
    // 右モーター制御
    if (right_speed > 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_RF, right_speed);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_RB, 0);
    } else if (right_speed < 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_RF, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_RB, -right_speed);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_RF, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CH_RB, 0);
    }
    
    // PWM更新を適用
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CH_LF);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CH_LB);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CH_RF);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CH_RB);
    
    // 最終コマンド時刻を更新
    last_command_time = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "モーター制御: L=%d, R=%d", left_speed, right_speed);
}

// ===== モーター停止 =====
void motor_stop(void)
{
    motor_control(0, 0);
    ESP_LOGI(TAG, "モーター停止");
}

// ===== HTML UI =====
static const char* index_html = 
"<!DOCTYPE html>"
"<html><head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
"<title>ESP32 モーター制御</title>"
"<style>"
"body{font-family:Arial;text-align:center;background:#1a1a1a;color:#fff;margin:0;padding:20px}"
"h1{color:#4CAF50}"
".controls{max-width:400px;margin:20px auto}"
".btn{background:#4CAF50;border:none;color:white;padding:20px;font-size:18px;"
"margin:5px;border-radius:10px;cursor:pointer;min-width:100px}"
".btn:active{background:#45a049}"
".stop-btn{background:#f44336}"
".speed-ctrl{margin:20px 0}"
"input[type=range]{width:80%;height:10px}"
".status{background:#333;padding:15px;border-radius:10px;margin:20px 0}"
"</style></head><body>"
"<h1>🚗 ESP32 モーター制御</h1>"
"<div class='status'><p>接続状態: <span id='status'>接続中...</span></p></div>"
"<div class='speed-ctrl'>"
"<label>速度: <span id='speedVal'>200</span></label><br>"
"<input type='range' id='speed' min='100' max='255' value='200' "
"oninput=\"document.getElementById('speedVal').textContent=this.value\">"
"</div>"
"<div class='controls'>"
"<button class='btn' onclick=\"send('forward')\">⬆️ 前進</button><br>"
"<button class='btn' onclick=\"send('left')\">⬅️ 左</button>"
"<button class='btn stop-btn' onclick=\"send('stop')\">⏹️ 停止</button>"
"<button class='btn' onclick=\"send('right')\">➡️ 右</button><br>"
"<button class='btn' onclick=\"send('backward')\">⬇️ 後退</button>"
"</div>"
"<script>"
"function send(cmd){"
"const speed=document.getElementById('speed').value;"
"fetch(`/control?cmd=${cmd}&speed=${speed}`)"
".then(r=>r.text())"
".then(d=>{document.getElementById('status').textContent='✅ '+d})"
".catch(e=>{document.getElementById('status').textContent='❌ エラー'});"
"}"
"setInterval(()=>{"
"fetch('/status').then(r=>r.text())"
".then(d=>{if(d!=='timeout')document.getElementById('status').textContent='✅ 接続中'})"
".catch(e=>{document.getElementById('status').textContent='❌ 切断'});"
"},2000);"
"</script></body></html>";

// ===== ルートハンドラ（HTML UI） =====
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// ===== 制御APIハンドラ（改善版） =====
static esp_err_t control_handler(httpd_req_t *req)
{
    char buf[128];
    char cmd[32] = {0};
    char speed_str[32] = {0};
    int speed = MOTOR_SPEED_DEFAULT;
    
    // クエリパラメータ取得
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "パラメータなし");
        return ESP_FAIL;
    }
    
    // コマンド取得
    if (httpd_query_key_value(buf, "cmd", cmd, sizeof(cmd)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "cmdパラメータ必須");
        return ESP_FAIL;
    }
    
    // 速度取得（オプション）
    if (httpd_query_key_value(buf, "speed", speed_str, sizeof(speed_str)) == ESP_OK) {
        speed = atoi(speed_str);
        if (speed < MOTOR_SPEED_MIN || speed > MOTOR_SPEED_MAX) {
            speed = MOTOR_SPEED_DEFAULT;
        }
    }
    
    // コマンド実行
    if (strcmp(cmd, "forward") == 0) {
        motor_control(speed, speed);
        httpd_resp_sendstr(req, "前進");
    } else if (strcmp(cmd, "backward") == 0) {
        motor_control(-speed, -speed);
        httpd_resp_sendstr(req, "後退");
    } else if (strcmp(cmd, "left") == 0) {
        motor_control(-speed/2, speed);
        httpd_resp_sendstr(req, "左旋回");
    } else if (strcmp(cmd, "right") == 0) {
        motor_control(speed, -speed/2);
        httpd_resp_sendstr(req, "右旋回");
    } else if (strcmp(cmd, "stop") == 0) {
        motor_stop();
        httpd_resp_sendstr(req, "停止");
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "不明なコマンド");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// ===== ステータスハンドラ =====
static esp_err_t status_handler(httpd_req_t *req)
{
    // タイムアウトチェック
    TickType_t current_time = xTaskGetTickCount();
    if ((current_time - last_command_time) > pdMS_TO_TICKS(MOTOR_TIMEOUT_MS)) {
        httpd_resp_sendstr(req, "timeout");
    } else {
        httpd_resp_sendstr(req, "ok");
    }
    return ESP_OK;
}

// ===== Webサーバー起動 =====
httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    if (httpd_start(&server, &config) == ESP_OK) {
        // ルートハンドラ（HTML UI）
        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = root_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root);
        
        // 制御API
        httpd_uri_t control = {
            .uri = "/control",
            .method = HTTP_GET,
            .handler = control_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &control);
        
        // ステータスAPI
        httpd_uri_t status = {
            .uri = "/status",
            .method = HTTP_GET,
            .handler = status_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &status);
        
        ESP_LOGI(TAG, "Webサーバー起動成功");
    } else {
        ESP_LOGE(TAG, "Webサーバー起動失敗");
    }
    return server;
}

// ===== モーターPWM初期化 =====
esp_err_t motor_init(void)
{
    // LEDCタイマー設定
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    // LEDCチャンネル設定
    ledc_channel_config_t channels[] = {
        {
            .channel = PWM_CH_LF,
            .gpio_num = MOTOR_LEFT_FWD,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .duty = 0,
            .hpoint = 0
        },
        {
            .channel = PWM_CH_LB,
            .gpio_num = MOTOR_LEFT_BWD,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .duty = 0,
            .hpoint = 0
        },
        {
            .channel = PWM_CH_RF,
            .gpio_num = MOTOR_RIGHT_FWD,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .duty = 0,
            .hpoint = 0
        },
        {
            .channel = PWM_CH_RB,
            .gpio_num = MOTOR_RIGHT_BWD,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .duty = 0,
            .hpoint = 0
        }
    };

    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&channels[i]));
    }

    ESP_LOGI(TAG, "モーターPWM初期化完了");
    return ESP_OK;
}

// ===== 安全監視タスク =====
void safety_monitor_task(void *pvParameters)
{
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        if ((current_time - last_command_time) > pdMS_TO_TICKS(MOTOR_TIMEOUT_MS)) {
            motor_stop();
            ESP_LOGW(TAG, "タイムアウト: モーター自動停止");
            last_command_time = current_time;  // 連続ログ防止
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ===== メイン関数 =====
void app_main(void)
{
    // NVSフラッシュ初期化
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP32モーター制御システム起動");

    // モーター初期化
    ESP_ERROR_CHECK(motor_init());
    
    // WiFi初期化
    ESP_ERROR_CHECK(wifi_init_sta());
    
    // WiFi接続待機
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, 
                        false, true, portMAX_DELAY);
    
    // Webサーバー起動
    start_webserver();
    
    // 安全監視タスク起動
    xTaskCreate(safety_monitor_task, "safety_monitor", 2048, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "===============================================");
    ESP_LOGI(TAG, "準備完了！ブラウザで http://<ESP32のIP>/ にアクセス");
    ESP_LOGI(TAG, "===============================================");
}