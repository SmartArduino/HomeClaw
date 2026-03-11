/**
 * @file main.cpp
 * @brief WireClaw - ESP32 AI Agent
 *
 * A reimplementation of PicoClaw's core agent loop for ESP32.
 * Phase 5: Polish - history persistence, LED heartbeat, watchdog.
 *
 * Type a message in the serial monitor, get an LLM response.
 * Use 115200 baud, send with newline.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <esp_task_wdt.h>
#if !defined(CONFIG_IDF_TARGET_ESP32)
#include "driver/temperature_sensor.h"
#endif
#include "llm_client.h"
#include "tools.h"
#include "devices.h"
#include "rules.h"
#include "setup_portal.h"
#include "version.h"
#include "web_config.h"
#include "nats_hal.h"
#include <nats_esp32.h>

/*============================================================================
 * Configuration
 *============================================================================*/

#define LED_BRIGHTNESS 20
#define SERIAL_BUF_SIZE 512
#define MAX_HISTORY 4 /* Keep last N user+assistant turns (pairs) */

/* Runtime config - loaded from LittleFS, falls back to secrets.h */
char cfg_wifi_ssid[64];
char cfg_wifi_pass[64];
char cfg_api_key[128];
char cfg_model[64];
char cfg_device_name[32];
char cfg_api_base_url[128];
char cfg_nats_host[64];
int cfg_nats_port = 4222;
char cfg_telegram_token[64];
char cfg_telegram_chat_id[16];
char cfg_qq_http_host[64];
int cfg_qq_http_port = 8080;
char cfg_qq_app_id[32];
char cfg_qq_app_secret[64];
char cfg_qq_access_token[128]; /* Runtime access token */
unsigned long cfg_qq_token_expires = 0;
int cfg_qq_cooldown = 3; /* seconds, 0 = disabled */
char cfg_system_prompt[4096];
char cfg_timezone[64];
int cfg_telegram_cooldown = 3; /* seconds, 0 = disabled */

/**
 * 用编译期回退值初始化内存中的配置。
 *
 * 该函数会在读取 LittleFS 之前执行。之后如果 `/config.json`
 * 中成功加载到对应字段，就会覆盖这里的默认值。
 */
static void configDefaults() {
    cfg_wifi_ssid[0] = '\0';
    cfg_wifi_pass[0] = '\0';
    cfg_api_key[0] = '\0';
    strncpy(cfg_model, "MiniMax-M2.5", sizeof(cfg_model));
    strncpy(cfg_device_name, "wireclaw", sizeof(cfg_device_name));
    cfg_api_base_url[0] = '\0';
    cfg_nats_host[0] = '\0';
    cfg_nats_port = 4222;
    cfg_telegram_token[0] = '\0';
    cfg_telegram_chat_id[0] = '\0';
    cfg_qq_http_host[0] = '\0';
    cfg_qq_http_port = 8080;
    cfg_qq_app_id[0] = '\0';
    cfg_qq_app_secret[0] = '\0';
    cfg_qq_access_token[0] = '\0';
    cfg_qq_token_expires = 0;
    cfg_qq_cooldown = 3;
    strncpy(cfg_timezone, "UTC0", sizeof(cfg_timezone));
    strncpy(cfg_system_prompt,
            "You are WireClaw, a helpful AI assistant running on an ESP32 microcontroller. "
            "Be concise. Keep responses under 200 words unless asked for detail.",
            sizeof(cfg_system_prompt));
}

/*============================================================================
 * LED Helpers (RGB on C6/S3/C3, on/off fallback on classic ESP32)
 *============================================================================*/

static uint8_t ledBrightness = LED_BRIGHTNESS;

/**
 * 在应用全局亮度限制后，将状态 LED 设置为指定 RGB 颜色。
 *
 * 在支持 RGB LED 的目标板上会输出完整颜色；只有单色 LED 的板子上
 * 会退化为简单的开关灯行为。
 */
void led(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t rawR = r;
    uint8_t rawG = g;
    uint8_t rawB = b;
    r = (uint8_t)((r * ledBrightness) / 255);
    g = (uint8_t)((g * ledBrightness) / 255);
    b = (uint8_t)((b * ledBrightness) / 255);
    if (g_debug) {
        Serial.printf("[LED] set raw=(%u,%u,%u) scaled=(%u,%u,%u) brightness=%u\n",
                      rawR, rawG, rawB, r, g, b, ledBrightness);
    }
#ifdef RGB_BUILTIN
    rgbLedWrite(RGB_BUILTIN, r, g, b);
#elif defined(LED_BUILTIN)
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, (r || g || b) ? HIGH : LOW);
#endif
}

/** 关闭状态 LED。 */
void ledOff() {
    led(0, 0, 0);
}

/** 将状态 LED 设为红色。 */
void ledRed() {
    led(255, 0, 0);
}

/** 将状态 LED 设为橙色。 */
void ledOrange() {
    led(255, 80, 0);
}

/** 将状态 LED 设为绿色。 */
void ledGreen() {
    led(0, 255, 0);
}

/** 将状态 LED 设为青色。 */
void ledCyan() {
    led(0, 255, 255);
}

/** 将状态 LED 设为蓝色。 */
void ledBlue() {
    led(0, 0, 255);
}

/** 将状态 LED 设为紫色。 */
void ledPurple() {
    led(128, 0, 255);
}

/* Deferred reboot: allows Telegram ACK cycle to complete before restart */
bool g_reboot_pending = false;
unsigned long g_reboot_at = 0;

/*============================================================================
 * LittleFS Config Loading
 *============================================================================*/

/**
 * 从 JSON 中提取 `key` 对应的非空字符串值到 `dst`。
 *
 * 这里故意实现成轻量且宽松的解析器，因为它只处理本项目受信任的
 * 配置和工具参数，而不是任意 JSON 输入。
 */
static bool jsonGetString(const char *json, const char *key,
                          char *dst, int dst_len) {
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(json, pattern);
    if (!p) return false;

    p += strlen(pattern);
    while (*p == ' ' || *p == ':') p++;
    if (*p != '"') return false;
    p++; /* skip opening quote */

    int w = 0;
    while (*p && *p != '"' && w < dst_len - 1) {
        if (*p == '\\' && *(p + 1)) {
            p++; /* skip backslash, take next char */
        }
        dst[w++] = *p++;
    }
    dst[w] = '\0';
    return w > 0;
}

/**
 * 与 `jsonGetString()` 类似，但会把空字符串也视为有效配置值。
 *
 * 这对使用 `""` 明确关闭某项功能的字段是必要的。
 */
static bool jsonGetStringAllowEmpty(const char *json, const char *key,
                                    char *dst, int dst_len) {
    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(json, pattern);
    if (!p) return false;

    p += strlen(pattern);
    while (*p == ' ' || *p == ':') p++;
    if (*p != '"') return false;
    p++; /* skip opening quote */

    int w = 0;
    while (*p && *p != '"' && w < dst_len - 1) {
        if (*p == '\\' && *(p + 1)) {
            p++;
        }
        dst[w++] = *p++;
    }
    dst[w] = '\0';
    return true;
}

/**
 * 将整个 LittleFS 文件读入调用方提供的缓冲区。
 *
 * 只要文件打开成功，结果一定会以空字符结尾。
 * 返回值是写入 `buf` 的字节数；如果文件无法打开则返回 `-1`。
 */
static int readFile(const char *path, char *buf, int buf_len) {
    File f = LittleFS.open(path, "r");
    if (!f) return -1;

    int len = f.readBytes(buf, buf_len - 1);
    buf[len] = '\0';
    f.close();
    return len;
}

/**
 * 从 LittleFS 加载运行时配置，并覆盖到默认值之上。
 *
 * 该函数会先调用 `configDefaults()` 重置内存中的配置，
 * 然后在存在时加载 `/config.json` 与 `/system_prompt.txt`
 * 中的持久化内容。
 */
static bool loadConfig() {
    configDefaults();

    if (!LittleFS.begin(false)) {
        Serial.printf("LittleFS: mount failed (no filesystem?)\n");
        Serial.printf("LittleFS: using compile-time defaults\n");
        return false;
    }

    Serial.printf("LittleFS: mounted OK\n");

    /* Load config.json */
    static char json_buf[1024];
    int len = readFile("/config.json", json_buf, sizeof(json_buf));
    if (len > 0) {
        Serial.printf("LittleFS: loaded config.json (%d bytes)\n", len);
        jsonGetStringAllowEmpty(json_buf, "wifi_ssid", cfg_wifi_ssid, sizeof(cfg_wifi_ssid));
        jsonGetStringAllowEmpty(json_buf, "wifi_pass", cfg_wifi_pass, sizeof(cfg_wifi_pass));
        jsonGetStringAllowEmpty(json_buf, "api_key", cfg_api_key, sizeof(cfg_api_key));
        jsonGetString(json_buf, "model", cfg_model, sizeof(cfg_model));
        jsonGetString(json_buf, "device_name", cfg_device_name, sizeof(cfg_device_name));
        jsonGetStringAllowEmpty(json_buf, "api_base_url", cfg_api_base_url, sizeof(cfg_api_base_url));
        jsonGetStringAllowEmpty(json_buf, "nats_host", cfg_nats_host, sizeof(cfg_nats_host));
        char port_buf[8];
        if (jsonGetString(json_buf, "nats_port", port_buf, sizeof(port_buf))) {
            cfg_nats_port = atoi(port_buf);
        }
        jsonGetStringAllowEmpty(json_buf, "telegram_token", cfg_telegram_token, sizeof(cfg_telegram_token));
        jsonGetStringAllowEmpty(json_buf, "telegram_chat_id", cfg_telegram_chat_id, sizeof(cfg_telegram_chat_id));
        char cd_buf[8];
        if (jsonGetString(json_buf, "telegram_cooldown", cd_buf, sizeof(cd_buf))) {
            cfg_telegram_cooldown = atoi(cd_buf);
        }
        jsonGetStringAllowEmpty(json_buf, "qq_http_host", cfg_qq_http_host, sizeof(cfg_qq_http_host));
        char qq_port_buf[8];
        if (jsonGetString(json_buf, "qq_http_port", qq_port_buf, sizeof(qq_port_buf))) {
            cfg_qq_http_port = atoi(qq_port_buf);
        }
        jsonGetStringAllowEmpty(json_buf, "qq_app_id", cfg_qq_app_id, sizeof(cfg_qq_app_id));
        jsonGetStringAllowEmpty(json_buf, "qq_app_secret", cfg_qq_app_secret, sizeof(cfg_qq_app_secret));
        char qq_cd_buf[8];
        if (jsonGetString(json_buf, "qq_cooldown", qq_cd_buf, sizeof(qq_cd_buf))) {
            cfg_qq_cooldown = atoi(qq_cd_buf);
        }
        jsonGetStringAllowEmpty(json_buf, "timezone", cfg_timezone, sizeof(cfg_timezone));
    } else {
        Serial.printf("LittleFS: no config.json, using defaults\n");
    }

    /* Load system prompt */
    len = readFile("/system_prompt.txt", cfg_system_prompt, sizeof(cfg_system_prompt));
    if (len > 0) {
        Serial.printf("LittleFS: loaded system_prompt.txt (%d bytes)\n", len);
    } else {
        Serial.printf("LittleFS: no system_prompt.txt, using default prompt\n");
    }

    return true;
}

/*============================================================================
 * Globalstrue
 *============================================================================*/

bool g_debug = false;
bool g_led_user = false; /* true when LED was set by a tool - don't overwrite with status */
#if !defined(CONFIG_IDF_TARGET_ESP32)
temperature_sensor_handle_t g_temp_sensor = NULL;
#endif

LlmClient llm;
char serialBuf[SERIAL_BUF_SIZE];
int serialPos = 0;

/* NATS client (optional - only used if nats_host is configured) */
NatsClient natsClient;
bool g_nats_enabled = false;
bool g_nats_connected = false;
static unsigned long natsLastReconnect = 0;
#define NATS_RECONNECT_DELAY_MS 30000
static char natsSubjectChat[64];
static char natsSubjectCmd[64];
char natsSubjectEvents[64];
static char natsSubjectToolExec[64];
static char natsSubjectCapabilities[64];
static char natsSubjectHal[64];
static const char natsSubjectDiscover[] = "_ion.discover";

/* Conversation history */
struct Turn {
    char user[256];
    char assistant[LLM_MAX_RESPONSE_LEN];
    bool used;
};

static Turn history[MAX_HISTORY];
static int historyCount = 0;

/*============================================================================
 * History Persistence (LittleFS)
 *============================================================================*/

#define HISTORY_FILE "/history.json"

/**
 * 对字符串做转义，使其可以安全写入 JSON 字符串字面量。
 *
 * 这里只实现了本固件实际需要的最小转义集合。
 */
static int jsonEscape(char *dst, int dst_len, const char *src) {
    int w = 0;
    for (int i = 0; src[i] && w < dst_len - 1; i++) {
        char c = src[i];
        if (c == '"' || c == '\\') {
            if (w + 2 >= dst_len) break;
            dst[w++] = '\\';
            dst[w++] = c;
        } else if (c == '\n') {
            if (w + 2 >= dst_len) break;
            dst[w++] = '\\';
            dst[w++] = 'n';
        } else if (c == '\r' || (uint8_t)c < 0x20) {
            /* skip control chars */
        } else {
            dst[w++] = c;
        }
    }
    dst[w] = '\0';
    return w;
}

/**
 * 将内存中的对话历史环形缓冲持久化到 LittleFS。
 *
 * 历史记录会以紧凑的 JSON 数组保存，便于下次开机时恢复，
 * 且不需要重新请求 LLM。
 */
static void historySave() {
    File f = LittleFS.open(HISTORY_FILE, "w");
    if (!f) return;

    static char escaped[LLM_MAX_RESPONSE_LEN + 512];

    f.print("[");
    for (int i = 0; i < historyCount; i++) {
        if (i > 0) f.print(",");
        f.print("{\"u\":\"");
        jsonEscape(escaped, sizeof(escaped), history[i].user);
        f.print(escaped);
        f.print("\",\"a\":\"");
        jsonEscape(escaped, sizeof(escaped), history[i].assistant);
        f.print(escaped);
        f.print("\"}");
    }
    f.print("]");
    f.close();

    if (g_debug) Serial.printf("History: saved %d turns\n", historyCount);
}

/**
 * 从 LittleFS 恢复先前保存的对话历史。
 *
 * 解析器故意保持轻量，只识别 `historySave()` 生成的那种固定 JSON 结构。
 */
static void historyLoad() {
    static char buf[8192];
    int len = readFile(HISTORY_FILE, buf, sizeof(buf));
    if (len <= 0) return;

    historyCount = 0;
    const char *p = buf;

    while (*p && historyCount < MAX_HISTORY) {
        const char *uStart = strstr(p, "\"u\":\"");
        if (!uStart) break;
        uStart += 5;

        int w = 0;
        const char *s = uStart;
        while (*s && *s != '"' && w < (int)sizeof(history[0].user) - 1) {
            if (*s == '\\' && *(s + 1)) {
                s++;
                if (*s == 'n')
                    history[historyCount].user[w++] = '\n';
                else
                    history[historyCount].user[w++] = *s;
            } else {
                history[historyCount].user[w++] = *s;
            }
            s++;
        }
        history[historyCount].user[w] = '\0';

        const char *aStart = strstr(s, "\"a\":\"");
        if (!aStart) break;
        aStart += 5;

        w = 0;
        s = aStart;
        while (*s && *s != '"' && w < (int)sizeof(history[0].assistant) - 1) {
            if (*s == '\\' && *(s + 1)) {
                s++;
                if (*s == 'n')
                    history[historyCount].assistant[w++] = '\n';
                else
                    history[historyCount].assistant[w++] = *s;
            } else {
                history[historyCount].assistant[w++] = *s;
            }
            s++;
        }
        history[historyCount].assistant[w] = '\0';

        history[historyCount].used = true;
        historyCount++;
        p = s;
    }

    if (historyCount > 0) {
        Serial.printf("History: loaded %d turns from %s\n", historyCount, HISTORY_FILE);
    }
}

/*============================================================================
 * Temperature Sensor
 *============================================================================*/

#if !defined(CONFIG_IDF_TARGET_ESP32)
/**
 * 在支持该功能的芯片上初始化内置温度传感器。
 *
 * 传统 ESP32 目标板不走这条路径，因为其 Arduino/IDF 支持方式不同。
 */
void initTempSensor() {
    temperature_sensor_config_t config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    esp_err_t err = temperature_sensor_install(&config, &g_temp_sensor);
    if (err != ESP_OK) {
        Serial.printf("Temp sensor install failed: %d\n", err);
        return;
    }
    err = temperature_sensor_enable(g_temp_sensor);
    if (err != ESP_OK) { Serial.printf("Temp sensor enable failed: %d\n", err); }
}
#endif

/*============================================================================
 * WiFi
 *============================================================================*/

/**
 * 让 STA 接口连接到配置中的 WiFi 网络。
 *
 * 该函数会在有限次数内阻塞重试，并更新状态 LED。
 * 只有在成功连网并拿到 DHCP 地址后才返回 `true`。
 */
bool connectWiFi() {
    Serial.printf("WiFi: Connecting to %s", cfg_wifi_ssid);
    ledOrange();

    WiFi.mode(WIFI_STA);
    WiFi.begin(cfg_wifi_ssid, cfg_wifi_pass);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (attempts % 2 == 0)
            ledOrange();
        else
            ledOff();
        if (++attempts > 30) {
            Serial.println(" FAILED!");
            ledRed();
            return false;
        }
    }

    Serial.printf(" OK!\n");
    Serial.printf("WiFi: IP = %s\n", WiFi.localIP().toString().c_str());
    ledGreen();
    return true;
}

/*============================================================================
 * Chat with LLM - Agentic Loop with Tool Calling
 *============================================================================*/

#define MAX_AGENT_ITERATIONS 5

/* Static storage for tool call results (persists across loop iterations) */
static char toolResultBufs[LLM_MAX_TOOL_CALLS][TOOL_RESULT_MAX_LEN];
char toolCallJsonBuf[4096]; /* copy of tool_calls_json for message building */
static char memoryBuf[512]; /* persistent AI memory from /memory.txt */

/**
 * 执行一次完整的 Agent 式 LLM 交互，包括多轮工具调用。
 *
 * 该函数会组装当前系统提示词、可选持久记忆、最近对话历史以及新的用户消息，
 * 然后在“模型响应”和“工具执行”之间循环，直到得到最终文本回复或发生错误。
 * 返回的指针在下一次调用 LLM 之前保持有效。
 */
const char *chatWithLLM(const char *userMessage) {
    /* Re-entrancy guard: remote_chat calls natsClient.process() which can
     * dispatch onNatsChat, leading to nested chatWithLLM. Block it. */
    static bool chatActive = false;
    if (chatActive) {
        Serial.printf("[Agent] Blocked re-entrant chatWithLLM call\n");
        return "[error: busy]";
    }
    chatActive = true;

    g_led_user = false; /* Reset - status LEDs allowed until a tool sets the LED */
    ledBlue();          /* Thinking... */

    /*
     * Message array for the full agentic conversation.
     * Layout: system + history pairs + user + [assistant+tool results]*iterations
     * Static to avoid blowing the 8KB loop task stack.
     */
    static LlmMessage messages[LLM_MAX_MESSAGES];
    int msgCount = 0;

    /* System prompt */
    messages[msgCount++] = llmMsg("system", cfg_system_prompt);

    /* Persistent AI memory */
    memoryBuf[0] = '\0';
    int memLen = readFile("/memory.txt", memoryBuf, sizeof(memoryBuf));
    if (memLen > 0) {
        messages[msgCount++] = llmMsg("system", memoryBuf);
    }

    /* History */
    int histStart = msgCount; /* index where history pairs begin */
    for (int i = 0; i < historyCount && msgCount < LLM_MAX_MESSAGES - 2; i++) {
        messages[msgCount++] = llmMsg("user", history[i].user);
        messages[msgCount++] = llmMsg("assistant", history[i].assistant);
    }
    int histEnd = msgCount; /* index after last history message */

    /* Current user message */
    messages[msgCount++] = llmMsg("user", userMessage);

    Serial.printf("\n--- Thinking... ---\n");
    unsigned long t0 = millis();

    const char *tools_json = toolsGetDefinitions();
    static LlmResult result;
    int totalPromptTokens = 0;
    int totalCompletionTokens = 0;
    const char *finalContent = nullptr;
    bool ok = false;

    for (int iter = 0; iter < MAX_AGENT_ITERATIONS; iter++) {
        ok = llm.chat(messages, msgCount, tools_json, &result);

        /* If request too large, drop oldest history pair and retry */
        while (!ok && strstr(llm.lastError(), "too large") && histStart + 2 <= histEnd) {
            Serial.printf("[Agent] Request too large, dropping oldest history\n");
            /* Remove 2 messages (user+assistant) at histStart */
            memmove(&messages[histStart], &messages[histStart + 2],
                    (msgCount - histStart - 2) * sizeof(LlmMessage));
            msgCount -= 2;
            histEnd -= 2;
            ok = llm.chat(messages, msgCount, tools_json, &result);
        }
        if (!ok) break;

        totalPromptTokens += result.prompt_tokens;
        totalCompletionTokens += result.completion_tokens;

        /* No tool calls - we're done */
        if (result.tool_call_count == 0) {
            finalContent = result.content;
            break;
        }

        /* Execute tool calls */
        Serial.printf("[Agent] %d tool call(s) in iteration %d:\n",
                      result.tool_call_count, iter + 1);

        /* Save tool_calls_json for message building */
        strncpy(toolCallJsonBuf, result.tool_calls_json, sizeof(toolCallJsonBuf) - 1);
        toolCallJsonBuf[sizeof(toolCallJsonBuf) - 1] = '\0';

        /* Add assistant message with tool calls */
        if (msgCount < LLM_MAX_MESSAGES) {
            messages[msgCount++] = llmToolCallMsg(
                result.content[0] ? result.content : nullptr,
                toolCallJsonBuf);
        }

        /* Execute each tool and add result messages */
        for (int t = 0; t < result.tool_call_count && msgCount < LLM_MAX_MESSAGES; t++) {
            LlmToolCall *tc = &result.tool_calls[t];

            Serial.printf("  -> %s(%s)\n", tc->name, tc->arguments);

            toolExecute(tc->name, tc->arguments,
                        toolResultBufs[t], TOOL_RESULT_MAX_LEN);

            Serial.printf("     = %s\n", toolResultBufs[t]);

            messages[msgCount++] = llmToolResult(tc->id, toolResultBufs[t]);
        }

        if (!g_led_user) ledPurple(); /* Show we're in a tool loop */
    }

    unsigned long elapsed = millis() - t0;

    if (ok && finalContent && finalContent[0]) {
        if (!g_led_user) ledGreen();

        Serial.printf("\n%s\n", finalContent);
        Serial.printf("--- (%lums, %d+%d tokens) ---\n\n",
                      elapsed, totalPromptTokens, totalCompletionTokens);

        /* Save to history (circular buffer) */
        int slot;
        if (historyCount >= MAX_HISTORY) {
            for (int i = 0; i < MAX_HISTORY - 1; i++)
                history[i] = history[i + 1];
            slot = MAX_HISTORY - 1;
        } else {
            slot = historyCount++;
        }
        strncpy(history[slot].user, userMessage, sizeof(history[slot].user) - 1);
        history[slot].user[sizeof(history[slot].user) - 1] = '\0';
        strncpy(history[slot].assistant, finalContent,
                sizeof(history[slot].assistant) - 1);
        history[slot].assistant[sizeof(history[slot].assistant) - 1] = '\0';
        history[slot].used = true;
        historySave();

        chatActive = false;
        return finalContent;

    } else if (ok) {
        /* Tools executed but no final text (LLM only used tools) */
        if (!g_led_user) ledGreen();
        Serial.printf("\n[Agent] Tools executed, no text response.\n");
        Serial.printf("--- (%lums, %d+%d tokens) ---\n\n",
                      elapsed, totalPromptTokens, totalCompletionTokens);
        chatActive = false;
        return "[Tools executed, no text response]";
    } else {
        ledRed();
        Serial.printf("\n[ERROR] LLM call failed: %s\n\n", llm.lastError());
        chatActive = false;
        return nullptr;
    }
}

/*============================================================================
 * NATS Callbacks
 *============================================================================*/

/**
 * 跟踪 NATS 客户端的高层状态变化，用于状态展示和日志输出。
 *
 * 该回调会在连接初始化时注册一次，并让全局连接状态与底层传输状态保持同步。
 */
static void onNatsEvent(nats_client_t *client, nats_event_t event,
                        void *userdata) {
    (void)client;
    (void)userdata;
    switch (event) {
    case NATS_EVENT_CONNECTED:
        Serial.printf("NATS: connected\n");
        g_nats_connected = true;
        break;
    case NATS_EVENT_DISCONNECTED:
        Serial.printf("NATS: disconnected\n");
        g_nats_connected = false;
        break;
    case NATS_EVENT_ERROR:
        Serial.printf("NATS: error: %s\n",
                      nats_err_str(nats_get_last_error(client)));
        break;
    default:
        break;
    }
}

static void tgYield(); /* forward declaration */

/**
 * 处理来自 NATS 的聊天请求，并返回 LLM 回复。
 *
 * 这条路径与本地聊天逻辑基本一致，只是外层包了一层 NATS request/reply，
 * 并可选地把最终回复重新发布到设备事件主题。
 */
static void onNatsChat(nats_client_t *client, const nats_msg_t *msg,
                       void *userdata) {
    (void)userdata;
    if (msg->data_len == 0) return;

    /* Copy payload (not null-terminated) */
    static char chatBuf[512];
    size_t len = msg->data_len < sizeof(chatBuf) - 1 ? msg->data_len : sizeof(chatBuf) - 1;
    memcpy(chatBuf, msg->data, len);
    chatBuf[len] = '\0';

    Serial.printf("\n[NATS] chat: %s\n", chatBuf);

    tgYield(); /* Free Telegram TLS so LLM can allocate */
    const char *response = chatWithLLM(chatBuf);

    /* Reply if caller expects a response */
    if (msg->reply_len > 0) {
        if (response) {
            nats_msg_respond_str(client, msg, response);
        } else {
            nats_msg_respond_str(client, msg, "[error]");
        }
    }

    /* Also publish to events */
    if (response && g_nats_connected) {
        natsClient.publish(natsSubjectEvents, response);
    }

    Serial.printf("> ");
}

/* Forward declaration (defined in Telegram section, needed by handleCommand) */
extern bool g_telegram_enabled;
extern bool g_qq_enabled;

/* Shared command response buffer (NATS + Telegram + Serial, single-threaded so safe) */
static char cmdResponseBuf[1024];

/**
 * 执行所有前端共用的斜杠维护命令。
 *
 * `cmd` 不应包含前导 `/`。格式化后的结果会写入 `buf`，
 * 返回值表示该命令是否被识别并成功处理。
 */
static bool handleCommand(const char *cmd, char *buf, int buf_len) {
    if (strcmp(cmd, "status") == 0) { // 返回设备整体状态：WiFi、Heap、History、Model、Debug、NATS、Telegram、QQ、Uptime
        snprintf(buf, buf_len,
                 "WiFi: %s (%s)\n"
                 "Heap: %u / %u\n"
                 "History: %d turns\n"
                 "Model: %s\n"
                 "Debug: %s\n"
                 "NATS: %s\n"
                 "Telegram: %s\n"
                 "QQ: %s\n"
                 "Uptime: %lus",
                 WiFi.status() == WL_CONNECTED ? "connected" : "disconnected",
                 WiFi.localIP().toString().c_str(),
                 ESP.getFreeHeap(), ESP.getHeapSize(),
                 historyCount, cfg_model,
                 g_debug ? "ON" : "OFF",
                 g_nats_enabled ? (g_nats_connected ? "connected" : "disconnected") : "disabled",
                 g_telegram_enabled ? "enabled" : "disabled",
                 g_qq_enabled ? "enabled" : "disabled",
                 millis() / 1000);
        return true;
    }
    if (strcmp(cmd, "clear") == 0) { // clear清空对话历史，并删除 history.json
        historyCount = 0;
        LittleFS.remove(HISTORY_FILE);
        snprintf(buf, buf_len, "History cleared");
        return true;
    }
    if (strcmp(cmd, "heap") == 0) { /// heap查看当前剩余堆内存
        snprintf(buf, buf_len, "Free heap: %u bytes", ESP.getFreeHeap());
        return true;
    }
    if (strcmp(cmd, "debug") == 0) { /// debug切换调试开关，开和关是同一个命令
        g_debug = !g_debug;
        snprintf(buf, buf_len, "Debug %s", g_debug ? "ON" : "OFF");
        return true;
    }
    if (strcmp(cmd, "devices") == 0) { // /devices 列出当前注册的设备/传感器
        int w = 0;
        Device *devs = deviceGetAll();
        for (int i = 0; i < MAX_DEVICES && w < buf_len - 80; i++) {
            if (!devs[i].used) continue;
            Device *d = &devs[i];
            if (w > 0) w += snprintf(buf + w, buf_len - w, "\n");
            if (d->kind == DEV_SENSOR_SERIAL_TEXT) {
                float val = deviceReadSensor(d);
                w += snprintf(buf + w, buf_len - w,
                              "%s [serial_text] %ubaud = %.1f %s",
                              d->name, (unsigned)d->baud, val, d->unit);
            } else if (d->kind == DEV_SENSOR_NATS_VALUE) {
                float val = deviceReadSensor(d);
                w += snprintf(buf + w, buf_len - w,
                              "%s [nats_value] %s = %.1f %s",
                              d->name, d->nats_subject, val, d->unit);
            } else if (deviceIsSensor(d->kind)) {
                float val = deviceReadSensor(d);
                w += snprintf(buf + w, buf_len - w,
                              "%s [%s] pin=%d = %.1f %s",
                              d->name, deviceKindName(d->kind), d->pin, val, d->unit);
            } else {
                w += snprintf(buf + w, buf_len - w,
                              "%s [%s] pin=%d%s",
                              d->name, deviceKindName(d->kind), d->pin,
                              d->inverted ? " (inverted)" : "");
            }
        }
        if (w == 0) snprintf(buf, buf_len, "No devices");
        return true;
    }
    if (strcmp(cmd, "rules") == 0) { /// rules列出当前自动化规则
        int w = 0;
        const Rule *rules = ruleGetAll();
        for (int i = 0; i < MAX_RULES && w < buf_len - 120; i++) {
            if (!rules[i].used) continue;
            const Rule *r = &rules[i];
            if (w > 0) w += snprintf(buf + w, buf_len - w, "\n");
            /* Header: id 'name' [ON] sensor cond threshold val=X state */
            uint32_t eval_ago = r->last_eval ? (millis() - r->last_eval) / 1000 : 0;
            w += snprintf(buf + w, buf_len - w,
                          "%s '%s' [%s] %s %s %d val=%.1f %s eval=%us every=%us",
                          r->id, r->name,
                          r->enabled ? "ON" : "OFF",
                          r->sensor_name[0] ? r->sensor_name :
                                              (r->condition == COND_CHAINED ? "" : "gpio"),
                          conditionOpName(r->condition),
                          (int)r->threshold,
                          r->last_reading,
                          r->fired ? "FIRED" : "idle",
                          (unsigned)eval_ago, (unsigned)(r->interval_ms / 1000));
            /* ON action */
            w += snprintf(buf + w, buf_len - w, "\n  on: %s",
                          actionTypeName(r->on_action));
            if (r->on_action == ACT_LED_SET) {
                int32_t v = r->on_value;
                w += snprintf(buf + w, buf_len - w,
                              "(%d,%d,%d)", (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF);
            } else if (r->on_action == ACT_TELEGRAM
                       || r->on_action == ACT_NATS_PUBLISH
                       || r->on_action == ACT_SERIAL_SEND) {
                w += snprintf(buf + w, buf_len - w, " \"%s\"", r->on_nats_pay);
            } else if (r->on_action == ACT_ACTUATOR) {
                w += snprintf(buf + w, buf_len - w, " %s", r->on_actuator);
            } else if (r->on_action == ACT_GPIO_WRITE) {
                w += snprintf(buf + w, buf_len - w,
                              " pin=%d val=%d", r->on_pin, (int)r->on_value);
            }
            /* OFF action */
            if (r->has_off_action) {
                w += snprintf(buf + w, buf_len - w, "\n  off: %s",
                              actionTypeName(r->off_action));
                if (r->off_action == ACT_LED_SET) {
                    int32_t v = r->off_value;
                    w += snprintf(buf + w, buf_len - w,
                                  "(%d,%d,%d)", (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF);
                } else if (r->off_action == ACT_TELEGRAM
                           || r->off_action == ACT_NATS_PUBLISH
                           || r->off_action == ACT_SERIAL_SEND) {
                    w += snprintf(buf + w, buf_len - w,
                                  " \"%s\"", r->off_nats_pay);
                } else if (r->off_action == ACT_ACTUATOR) {
                    w += snprintf(buf + w, buf_len - w,
                                  " %s", r->off_actuator);
                } else if (r->off_action == ACT_GPIO_WRITE) {
                    w += snprintf(buf + w, buf_len - w,
                                  " pin=%d val=%d", r->off_pin, (int)r->off_value);
                }
            }
            /* Chain links */
            if (r->chain_id[0])
                w += snprintf(buf + w, buf_len - w,
                              "\n  chain: ->%s (%us)",
                              r->chain_id, (unsigned)(r->chain_delay_ms / 1000));
            if (r->chain_off_id[0])
                w += snprintf(buf + w, buf_len - w,
                              "\n  chain-off: ->%s (%us)",
                              r->chain_off_id, (unsigned)(r->chain_off_delay_ms / 1000));
        }
        if (w == 0) snprintf(buf, buf_len, "No rules");
        return true;
    }
    if (strcmp(cmd, "memory") == 0) { /// memory读取 /memory.txt 的内容
        int len = readFile("/memory.txt", buf, buf_len);
        if (len <= 0) snprintf(buf, buf_len, "(no memory file)");
        return true;
    }
    if (strcmp(cmd, "time") == 0) { /// time查看当前本地时间和时区
        struct tm timeinfo;
        if (getLocalTime(&timeinfo, 0)) {
            snprintf(buf, buf_len,
                     "%04d-%02d-%02d %02d:%02d:%02d (TZ=%s)",
                     timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                     timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec,
                     cfg_timezone);
        } else {
            snprintf(buf, buf_len, "NTP not synced yet");
        }
        return true;
    }
    if (strcmp(cmd, "history") == 0) { /// history查看简略的聊天历史
        if (historyCount == 0) {
            snprintf(buf, buf_len, "No conversation history");
            return true;
        }
        int w = snprintf(buf, buf_len, "History: %d turns\n", historyCount);
        for (int i = 0; i < historyCount && w < buf_len - 80; i++) {
            int alen = strlen(history[i].assistant);
            w += snprintf(buf + w, buf_len - w,
                          "[%d] %.40s%s\n  -> %.60s%s\n",
                          i + 1, history[i].user,
                          strlen(history[i].user) > 40 ? "..." : "",
                          history[i].assistant,
                          alen > 60 ? "..." : "");
        }
        return true;
    }
    if (strncmp(cmd, "model", 5) == 0) { /// model不带参数时查看当前模型/model 新模型名直接把当前模型切换成新值例如：/model MiniMax-M2.5
        if (cmd[5] == '\0') {
            snprintf(buf, buf_len, "Model: %s", cfg_model);
        } else if (cmd[5] == ' ' && cmd[6] != '\0') {
            strncpy(cfg_model, cmd + 6, sizeof(cfg_model) - 1);
            cfg_model[sizeof(cfg_model) - 1] = '\0';
            snprintf(buf, buf_len, "Model changed to: %s", cfg_model);
        } else {
            snprintf(buf, buf_len, "Usage: /model [model-name]");
        }
        return true;
    }
    if (strcmp(cmd, "help") == 0) { /// help显示支持的命令列表
        snprintf(buf, buf_len,
                 "Commands: /status /clear /heap /debug /devices /rules "
                 "/memory /time /history /model /reboot /help");
        return true;
    }
    if (strcmp(cmd, "reboot") == 0) { /// reboot延迟几秒后重启设备
        if (g_nats_connected) {
            natsClient.publish(natsSubjectEvents, "Rebooting...");
        }
        g_reboot_pending = true;
        g_reboot_at = millis() + 8000; /* 8s: enough for TG response + ACK poll */
        snprintf(buf, buf_len, "Rebooting in a few seconds...");
        return true;
    }
    return false;
}

/**
 * 通过调用 `handleCommand()` 处理 NATS 命令请求。
 *
 * 无论成功还是失败，结果都会返回给请求方；
 * 如果条件允许，还会同步发布到设备的事件主题。
 */
static void onNatsCmd(nats_client_t *client, const nats_msg_t *msg,
                      void *userdata) {
    (void)client;
    (void)userdata;
    if (msg->data_len == 0) return;

    static char cmdBuf[64];
    size_t len = msg->data_len < sizeof(cmdBuf) - 1 ? msg->data_len : sizeof(cmdBuf) - 1;
    memcpy(cmdBuf, msg->data, len);
    cmdBuf[len] = '\0';

    Serial.printf("\n[NATS] cmd: %s\n", cmdBuf);

    if (!handleCommand(cmdBuf, cmdResponseBuf, sizeof(cmdResponseBuf))) {
        snprintf(cmdResponseBuf, sizeof(cmdResponseBuf),
                 "Unknown command: %s (try /help)", cmdBuf);
    }

    Serial.printf("[NATS] -> %s\n> ", cmdResponseBuf);

    if (msg->reply_len > 0) {
        nats_msg_respond_str(natsClient.core(), msg, cmdResponseBuf);
    }
    if (g_nats_connected) {
        natsClient.publish(natsSubjectEvents, cmdResponseBuf);
    }
}

/*============================================================================
 * OpenClaw: Direct Tool Execution via NATS
 *============================================================================*/

/**
 * 执行通过 NATS 收到的直接工具调用。
 *
 * 这条路径会完全绕过 LLM，适用于 OpenClaw 或已经明确知道要执行哪个工具的自动化流程。
 */
static void onNatsToolExec(nats_client_t *client, const nats_msg_t *msg,
                           void *userdata) {
    (void)userdata;
    if (msg->data_len == 0) {
        if (msg->reply_len > 0)
            nats_msg_respond_str(client, msg,
                                 "{\"ok\":false,\"error\":\"empty payload\"}");
        return;
    }

    /* Copy payload into toolCallJsonBuf (idle — only used by chatWithLLM,
     * which we never call from this callback). */
    size_t len = msg->data_len < sizeof(toolCallJsonBuf) - 1 ? msg->data_len : sizeof(toolCallJsonBuf) - 1;
    memcpy(toolCallJsonBuf, msg->data, len);
    toolCallJsonBuf[len] = '\0';

    Serial.printf("\n[NATS] tool_exec: %s\n", toolCallJsonBuf);

    /* Extract tool name */
    static char toolName[32];
    if (!jsonGetString(toolCallJsonBuf, "tool", toolName, sizeof(toolName))) {
        Serial.printf("[NATS] tool_exec: missing 'tool' key\n");
        if (msg->reply_len > 0)
            nats_msg_respond_str(client, msg,
                                 "{\"ok\":false,\"error\":\"missing 'tool' key\"}");
        return;
    }

    /* Blocklist: remote_chat — re-entrant NATS processing */
    if (strcmp(toolName, "remote_chat") == 0) {
        Serial.printf("[NATS] tool_exec: blocked tool '%s'\n", toolName);
        if (msg->reply_len > 0)
            nats_msg_respond_str(client, msg,
                                 "{\"ok\":false,\"error\":\"remote_chat not available via tool_exec\"}");
        return;
    }

    /* Blocklist: file_write to /memory.txt — internal AI memory */
    if (strcmp(toolName, "file_write") == 0) {
        char pathBuf[64]; /* stack — only 64 bytes, brief use */
        if (jsonGetString(toolCallJsonBuf, "path", pathBuf, sizeof(pathBuf))
            && strcmp(pathBuf, "/memory.txt") == 0) {
            Serial.printf("[NATS] tool_exec: blocked write to /memory.txt\n");
            if (msg->reply_len > 0)
                nats_msg_respond_str(client, msg,
                                     "{\"ok\":false,\"error\":\"cannot write to /memory.txt via tool_exec\"}");
            return;
        }
    }

    /* Execute tool — result into cmdResponseBuf (idle — only used by
     * handleCommand, which we never call from this callback). */
    bool found = toolExecute(toolName, toolCallJsonBuf,
                             cmdResponseBuf, sizeof(cmdResponseBuf));

    /* Determine success: unknown tool or "Error:" prefix */
    bool ok = found && strncmp(cmdResponseBuf, "Error:", 6) != 0;

    /* Build JSON reply — escape directly into reply buffer (no intermediate) */
    static char reply[768];
    int w;
    if (ok) {
        w = snprintf(reply, sizeof(reply), "{\"ok\":true,\"result\":\"");
    } else {
        w = snprintf(reply, sizeof(reply), "{\"ok\":false,\"error\":\"");
    }
    w += jsonEscape(reply + w, sizeof(reply) - w - 3, cmdResponseBuf);
    snprintf(reply + w, sizeof(reply) - w, "\"}");

    Serial.printf("[NATS] tool_exec -> %s\n> ", ok ? "ok" : "error");

    if (msg->reply_len > 0) {
        nats_msg_respond_str(client, msg, reply);
    }

    /* Publish brief event for observability */
    if (g_nats_connected) {
        static char evtBuf[128];
        snprintf(evtBuf, sizeof(evtBuf),
                 "{\"event\":\"tool_exec\",\"tool\":\"%s\",\"ok\":%s}",
                 toolName, ok ? "true" : "false");
        natsClient.publish(natsSubjectEvents, evtBuf);
    }
}

/**
 * 发布一份设备当前暴露能力的 JSON 快照。
 *
 * OpenClaw 会用它来发现该 WireClaw 节点支持的工具、已注册设备、
 * 活动规则以及基础 HAL 能力。
 */
static void onNatsCapabilities(nats_client_t *client, const nats_msg_t *msg,
                               void *userdata) {
    (void)userdata;

    /* Reuse toolCallJsonBuf[4096] — idle here (only used by chatWithLLM,
     * which we never call from this callback). */
    int w = 0;

    w += snprintf(toolCallJsonBuf + w, sizeof(toolCallJsonBuf) - w,
                  "{\"device\":\"%s\",\"version\":\"%s\",\"free_heap\":%u,",
                  cfg_device_name, WIRECLAW_VERSION, ESP.getFreeHeap());

    /* Tools list */
    w += snprintf(toolCallJsonBuf + w, sizeof(toolCallJsonBuf) - w,
                  "\"tools\":[\"led_set\",\"gpio_write\",\"gpio_read\",\"device_info\","
                  "\"file_read\",\"file_write\",\"nats_publish\",\"temperature_read\","
                  "\"device_register\",\"device_list\",\"device_remove\",\"sensor_read\","
                  "\"actuator_set\",\"rule_create\",\"rule_list\",\"rule_delete\","
                  "\"rule_enable\",\"serial_send\",\"chain_create\"],");

    /* Devices */
    w += snprintf(toolCallJsonBuf + w, sizeof(toolCallJsonBuf) - w, "\"devices\":[");
    Device *devs = deviceGetAll();
    bool firstDev = true;
    for (int i = 0; i < MAX_DEVICES && w < (int)sizeof(toolCallJsonBuf) - 200; i++) {
        if (!devs[i].used) continue;
        Device *d = &devs[i];
        if (!firstDev) toolCallJsonBuf[w++] = ',';
        firstDev = false;
        if (deviceIsSensor(d->kind)) {
            float val = deviceReadSensor(d);
            w += snprintf(toolCallJsonBuf + w, sizeof(toolCallJsonBuf) - w,
                          "{\"name\":\"%s\",\"kind\":\"%s\",\"value\":%.1f,\"unit\":\"%s\"}",
                          d->name, deviceKindName(d->kind), val, d->unit);
        } else {
            w += snprintf(toolCallJsonBuf + w, sizeof(toolCallJsonBuf) - w,
                          "{\"name\":\"%s\",\"kind\":\"%s\",\"pin\":%d}",
                          d->name, deviceKindName(d->kind), d->pin);
        }
    }
    w += snprintf(toolCallJsonBuf + w, sizeof(toolCallJsonBuf) - w, "],");

    /* Rules */
    w += snprintf(toolCallJsonBuf + w, sizeof(toolCallJsonBuf) - w, "\"rules\":[");
    const Rule *rules = ruleGetAll();
    bool firstRule = true;
    for (int i = 0; i < MAX_RULES && w < (int)sizeof(toolCallJsonBuf) - 200; i++) {
        if (!rules[i].used) continue;
        const Rule *r = &rules[i];
        if (!firstRule) toolCallJsonBuf[w++] = ',';
        firstRule = false;
        w += snprintf(toolCallJsonBuf + w, sizeof(toolCallJsonBuf) - w,
                      "{\"id\":\"%s\",\"name\":\"%s\",\"enabled\":%s,\"condition\":\"%s\","
                      "\"sensor\":\"%s\",\"fired\":%s}",
                      r->id, r->name,
                      r->enabled ? "true" : "false",
                      conditionOpName(r->condition),
                      r->sensor_name,
                      r->fired ? "true" : "false");
    }
    w += snprintf(toolCallJsonBuf + w, sizeof(toolCallJsonBuf) - w,
                  "],\"hal\":{\"gpio\":true,\"adc\":true,\"pwm\":true,"
                  "\"dac\":false,\"uart\":true,\"system_temp\":true}}");

    Serial.printf("[NATS] capabilities: %d bytes\n> ", w);

    if (msg->reply_len > 0) {
        nats_msg_respond_str(client, msg, toolCallJsonBuf);
    }
}

/*============================================================================
 * NATS Virtual Sensor Subscriptions
 *============================================================================*/

static void onNatsValue(nats_client_t *client, const nats_msg_t *msg,
                        void *userdata) {
    (void)client;
    Device *dev = (Device *)userdata;
    if (!dev || !dev->used) return;
    parseNatsPayload(msg->data, msg->data_len,
                     &dev->nats_value, dev->nats_msg, sizeof(dev->nats_msg));
    if (g_debug) Serial.printf("[NATS] %s = %.1f (msg='%s')\n",
                               dev->name, dev->nats_value, dev->nats_msg);
}

/**
 * 在主 NATS 客户端连上后，订阅所有已注册的 NATS 虚拟传感器。
 *
 * 每个设备都会记录自己的订阅 SID，便于在断线重连或设备删除时正确取消和恢复订阅。
 */
void natsSubscribeDeviceSensors() {
    if (!g_nats_connected) return;
    Device *devs = deviceGetAllMutable();
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (!devs[i].used) continue;
        if (devs[i].kind != DEV_SENSOR_NATS_VALUE) continue;
        if (devs[i].nats_subject[0] == '\0') continue;
        if (devs[i].nats_sid != 0) continue; /* already subscribed */
        uint16_t sid = 0;
        nats_err_t err = natsClient.subscribe(devs[i].nats_subject,
                                              onNatsValue, &devs[i], &sid);
        if (err == NATS_OK) {
            devs[i].nats_sid = sid;
            Serial.printf("[NATS] Subscribed '%s' -> %s (sid=%d)\n",
                          devs[i].name, devs[i].nats_subject, sid);
        } else {
            Serial.printf("[NATS] Subscribe '%s' failed: %s\n",
                          devs[i].nats_subject, nats_err_str(err));
        }
    }
}

/**
 * 取消指定名称的 NATS 虚拟传感器订阅。
 *
 * 这在设备被删除或运行时重新配置时使用。
 */
void natsUnsubscribeDevice(const char *name) {
    if (!g_nats_connected) return;
    Device *devs = deviceGetAllMutable();
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (!devs[i].used) continue;
        if (strcmp(devs[i].name, name) != 0) continue;
        if (devs[i].nats_sid != 0) {
            natsClient.unsubscribe(devs[i].nats_sid);
            Serial.printf("[NATS] Unsubscribed '%s' (sid=%d)\n",
                          name, devs[i].nats_sid);
            devs[i].nats_sid = 0;
        }
        break;
    }
}

/**
 * 构建当前设备完整的 NATS subject 命名空间。
 *
 * 这些主题名都基于 `cfg_device_name` 生成，
 * 从而让多个 WireClaw 节点可以无冲突地共存于同一个 broker。
 */
static void buildNatsSubjects() {
    snprintf(natsSubjectChat, sizeof(natsSubjectChat),
             "%s.chat", cfg_device_name);
    snprintf(natsSubjectCmd, sizeof(natsSubjectCmd),
             "%s.cmd", cfg_device_name);
    snprintf(natsSubjectEvents, sizeof(natsSubjectEvents),
             "%s.events", cfg_device_name);
    snprintf(natsSubjectToolExec, sizeof(natsSubjectToolExec),
             "%s.tool_exec", cfg_device_name);
    snprintf(natsSubjectCapabilities, sizeof(natsSubjectCapabilities),
             "%s.capabilities", cfg_device_name);
    snprintf(natsSubjectHal, sizeof(natsSubjectHal),
             "%s.hal.>", cfg_device_name);
}

/**
 * 建立主 NATS 连接并挂载所有核心订阅。
 *
 * 该函数会注册聊天、命令、工具执行、能力发现、HAL 和虚拟传感器订阅，
 * 然后发送一条上线事件。
 */
static bool connectNats() {
    Serial.printf("NATS: connecting to %s:%d...\n", cfg_nats_host, cfg_nats_port);

    natsClient.onEvent(onNatsEvent, nullptr);

    if (!natsClient.connect(cfg_nats_host, (uint16_t)cfg_nats_port, 2000)) {
        Serial.printf("NATS: connection failed\n");
        return false;
    }

    /* Subscribe to chat and cmd */
    nats_err_t err;
    err = natsClient.subscribe(natsSubjectChat, onNatsChat, nullptr);
    if (err != NATS_OK) {
        Serial.printf("NATS: subscribe %s failed: %s\n",
                      natsSubjectChat, nats_err_str(err));
    }

    err = natsClient.subscribe(natsSubjectCmd, onNatsCmd, nullptr);
    if (err != NATS_OK) {
        Serial.printf("NATS: subscribe %s failed: %s\n",
                      natsSubjectCmd, nats_err_str(err));
    }

    err = natsClient.subscribe(natsSubjectToolExec, onNatsToolExec, nullptr);
    if (err != NATS_OK) {
        Serial.printf("NATS: subscribe %s failed: %s\n",
                      natsSubjectToolExec, nats_err_str(err));
    }

    err = natsClient.subscribe(natsSubjectCapabilities, onNatsCapabilities, nullptr);
    if (err != NATS_OK) {
        Serial.printf("NATS: subscribe %s failed: %s\n",
                      natsSubjectCapabilities, nats_err_str(err));
    }

    err = natsClient.subscribe(natsSubjectDiscover, onNatsCapabilities, nullptr);
    if (err != NATS_OK) {
        Serial.printf("NATS: subscribe %s failed: %s\n",
                      natsSubjectDiscover, nats_err_str(err));
    }

    err = natsClient.subscribe(natsSubjectHal, onNatsHal, nullptr);
    if (err != NATS_OK) {
        Serial.printf("NATS: subscribe %s failed: %s\n",
                      natsSubjectHal, nats_err_str(err));
    }

    /* Publish online event */
    static char onlineMsg[256];
    snprintf(onlineMsg, sizeof(onlineMsg),
             "{\"event\":\"online\",\"device\":\"%s\",\"version\":\"%s\","
             "\"ip\":\"%s\",\"tool_exec\":\"%s\",\"capabilities\":\"%s\","
             "\"hal\":\"%s\"}",
             cfg_device_name, WIRECLAW_VERSION,
             WiFi.localIP().toString().c_str(),
             natsSubjectToolExec, natsSubjectCapabilities,
             natsSubjectHal);
    natsClient.publish(natsSubjectEvents, onlineMsg);

    Serial.printf("NATS: subscribed to %s, %s, %s, %s, %s\n",
                  natsSubjectChat, natsSubjectCmd,
                  natsSubjectToolExec, natsSubjectCapabilities,
                  natsSubjectHal);

    /* Subscribe NATS virtual sensors */
    natsSubscribeDeviceSensors();

    return true;
}

/*============================================================================
 * Telegram Bot
 *============================================================================*/

static WiFiClientSecure tgClient;
bool g_telegram_enabled = false;
static int tgLastUpdateId = 0;
static unsigned long tgLastPoll = 0;

/* Long-poll state machine */
enum TgState { TG_IDLE,
               TG_WAITING };
static TgState tgState = TG_IDLE;
static unsigned long tgWaitStart = 0;
#define TG_RECONNECT_MS 5000  /* 5s between long-poll cycles */
#define TG_LONG_POLL_S 30     /* Telegram server hold time (seconds) */
#define TG_WAIT_TIMEOUT 35000 /* client-side timeout: long-poll + 5s grace */

static const char *TG_HOST = "api.telegram.org";
static const int TG_PORT = 443;

/**
 * 执行一次原始的 Telegram Bot API POST 请求，并抓取响应体。
 *
 * 该辅助函数会管理 TLS 连接生命周期，
 * 返回写入 `buf` 的字节数；失败时返回 `-1`。
 */
static int tgApiCall(const char *method, const char *body, int body_len,
                     char *buf, int buf_len) {
    tgClient.stop(); /* Ensure clean state from any previous connection */

    if (!tgClient.connect(TG_HOST, TG_PORT)) {
        if (g_debug) Serial.printf("[TG] Connect failed\n");
        return -1;
    }

    /* Build full request in one buffer to send at once */
    static char httpReq[512];
    int hdr_len = snprintf(httpReq, sizeof(httpReq),
                           "POST /bot%s/%s HTTP/1.1\r\n"
                           "Host: %s\r\n"
                           "Content-Type: application/json\r\n"
                           "Content-Length: %d\r\n"
                           "Connection: close\r\n\r\n",
                           cfg_telegram_token, method, TG_HOST, body_len);

    tgClient.write((uint8_t *)httpReq, hdr_len);
    if (body_len > 0) {
        tgClient.write((uint8_t *)body, body_len);
    }

    /* Wait for response */
    unsigned long wait_start = millis();
    while (!tgClient.available()) {
        if (!tgClient.connected()) {
            Serial.printf("[TG] Disconnected while waiting\n");
            tgClient.stop();
            return -1;
        }
        if (millis() - wait_start > 15000) {
            Serial.printf("[TG] Response timeout\n");
            tgClient.stop();
            return -1;
        }
        delay(100);
    }

    /* Read status line and headers, extract Content-Length */
    int content_length = -1;
    while (tgClient.connected()) {
        String line = tgClient.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) break;
        if (line.startsWith("Content-Length:") || line.startsWith("content-length:")) {
            content_length = line.substring(15).toInt();
        }
    }

    /* Read body - use Content-Length to avoid SSL error on server close */
    int total = 0;
    if (content_length > 0) {
        int to_read = content_length < buf_len - 1 ? content_length : buf_len - 1;
        total = tgClient.readBytes(buf, to_read);
    } else {
        /* Fallback: read until disconnect */
        unsigned long last_data = millis();
        while (total < buf_len - 1) {
            int avail = tgClient.available();
            if (avail > 0) {
                int rd = tgClient.readBytes(buf + total, min(avail, buf_len - 1 - total));
                total += rd;
                last_data = millis();
            } else if (!tgClient.connected()) {
                break;
            } else if (millis() - last_data > 5000) {
                break;
            } else {
                delay(10);
            }
        }
    }

    tgClient.stop();
    buf[total] = '\0';
    if (g_debug) Serial.printf("[TG] %s: %d bytes\n", method, total);
    return total;
}

/**
 * 向配置中的 Telegram 会话发送一条纯文本消息。
 *
 * 文本会先在本地做 JSON 转义，再发给 Telegram Bot API。
 */
bool tgSendMessage(const char *text) {
    static char req[LLM_MAX_RESPONSE_LEN + 256];
    static char escaped[LLM_MAX_RESPONSE_LEN + 128];

    /* Escape the text for JSON */
    int w = 0;
    for (int i = 0; text[i] && w < (int)sizeof(escaped) - 2; i++) {
        char c = text[i];
        if (c == '"' || c == '\\') {
            escaped[w++] = '\\';
            escaped[w++] = c;
        } else if (c == '\n') {
            escaped[w++] = '\\';
            escaped[w++] = 'n';
        } else if ((uint8_t)c >= 0x20) {
            escaped[w++] = c;
        }
    }
    escaped[w] = '\0';

    int req_len = snprintf(req, sizeof(req),
                           "{\"chat_id\":%s,\"text\":\"%s\"}", cfg_telegram_chat_id, escaped);

    static char resp[256];
    int rlen = tgApiCall("sendMessage", req, req_len, resp, sizeof(resp));
    if (rlen < 0) {
        Serial.printf("[TG] sendMessage failed\n");
        return false;
    }
    return true;
}

/**
 * 在一次 `loop()` 迭代中推进 Telegram 长轮询状态机。
 *
 * 这样可以让 Telegram 接收逻辑尽量保持非阻塞，同时仍能处理命令、
 * 分发聊天请求以及配合延迟重启流程。
 */
static void telegramTick() {
    unsigned long now = millis();

    switch (tgState) {
    case TG_IDLE: {
        /* Skip reconnect delay when reboot pending (fast ACK) */
        unsigned long wait = g_reboot_pending ? 500 : TG_RECONNECT_MS;
        if (now - tgLastPoll < wait) return;

        tgClient.stop();
        if (!tgClient.connect(TG_HOST, TG_PORT)) {
            if (g_debug) Serial.printf("[TG] Connect failed\n");
            tgLastPoll = now;
            return;
        }

        /* Build getUpdates request */
        static char body[128];
        int body_len;
        body_len = snprintf(body, sizeof(body),
                            "{\"offset\":%d,\"limit\":1,\"timeout\":%d}",
                            tgLastUpdateId + 1, g_reboot_pending ? 0 : TG_LONG_POLL_S);

        static char httpReq[512];
        int hdr_len = snprintf(httpReq, sizeof(httpReq),
                               "POST /bot%s/getUpdates HTTP/1.1\r\n"
                               "Host: %s\r\n"
                               "Content-Type: application/json\r\n"
                               "Content-Length: %d\r\n"
                               "Connection: close\r\n\r\n",
                               cfg_telegram_token, TG_HOST, body_len);

        tgClient.write((uint8_t *)httpReq, hdr_len);
        tgClient.write((uint8_t *)body, body_len);

        tgState = TG_WAITING;
        tgWaitStart = now;
        if (g_debug) Serial.printf("[TG] Long poll started\n");
        return;
    }

    case TG_WAITING: {
        if (!tgClient.available()) {
            /* Still waiting — check for errors or timeout */
            if (!tgClient.connected()) {
                if (g_debug) Serial.printf("[TG] Disconnected during wait\n");
                tgClient.stop();
                tgState = TG_IDLE;
                tgLastPoll = now;
                return;
            }
            if (now - tgWaitStart > TG_WAIT_TIMEOUT) {
                if (g_debug) Serial.printf("[TG] Long poll timeout\n");
                tgClient.stop();
                tgState = TG_IDLE;
                tgLastPoll = now;
                return;
            }
            return; /* Still waiting — return to loop() */
        }

        /* Data arrived — read response (brief blocking OK, data is TCP-buffered) */
        static char resp[2048];

        /* Read headers */
        int content_length = -1;
        while (tgClient.connected()) {
            String line = tgClient.readStringUntil('\n');
            line.trim();
            if (line.length() == 0) break;
            if (line.startsWith("Content-Length:") || line.startsWith("content-length:")) {
                content_length = line.substring(15).toInt();
            }
        }

        /* Read body */
        int total = 0;
        if (content_length > 0) {
            int to_read = content_length < (int)sizeof(resp) - 1 ? content_length : (int)sizeof(resp) - 1;
            total = tgClient.readBytes(resp, to_read);
        } else {
            unsigned long last_data = millis();
            while (total < (int)sizeof(resp) - 1) {
                int avail = tgClient.available();
                if (avail > 0) {
                    int rd = tgClient.readBytes(resp + total, min(avail, (int)sizeof(resp) - 1 - total));
                    total += rd;
                    last_data = millis();
                } else if (!tgClient.connected()) {
                    break;
                } else if (millis() - last_data > 5000) {
                    break;
                } else {
                    delay(10);
                }
            }
        }

        tgClient.stop();
        resp[total] = '\0';
        tgState = TG_IDLE;
        tgLastPoll = millis();

        if (g_debug) Serial.printf("[TG] poll: %d bytes\n", total);
        if (total <= 0) return;
        if (g_debug) Serial.printf("[TG] poll: %.200s\n", resp);

        /* Quick check: is there a result with "update_id"? */
        const char *uid_str = strstr(resp, "\"update_id\"");
        if (!uid_str) return; /* No updates - normal */

        /* Parse update_id */
        const char *p = uid_str + 11;
        while (*p == ':' || *p == ' ') p++;
        int update_id = atoi(p);
        if (g_debug) Serial.printf("[TG] update_id=%d (last=%d)\n", update_id, tgLastUpdateId);
        if (update_id <= tgLastUpdateId) return;
        tgLastUpdateId = update_id;

        /* Extract chat_id from message.chat.id */
        const char *chat_id_str = strstr(resp, "\"chat\"");
        if (!chat_id_str) {
            if (g_debug) Serial.printf("[TG] no chat field\n");
            return;
        }
        const char *id_str = strstr(chat_id_str, "\"id\"");
        if (!id_str) {
            if (g_debug) Serial.printf("[TG] no id in chat\n");
            return;
        }
        p = id_str + 4;
        while (*p == ':' || *p == ' ') p++;
        char incoming_chat_id[16];
        int cw = 0;
        while ((*p >= '0' && *p <= '9') || *p == '-') {
            if (cw < (int)sizeof(incoming_chat_id) - 1)
                incoming_chat_id[cw++] = *p;
            p++;
        }
        incoming_chat_id[cw] = '\0';

        if (g_debug) Serial.printf("[TG] chat_id=%s (allowed=%s)\n", incoming_chat_id, cfg_telegram_chat_id);

        /* Security: only allow configured chat_id */
        if (strcmp(incoming_chat_id, cfg_telegram_chat_id) != 0) {
            Serial.printf("[TG] Rejected chat %s\n", incoming_chat_id);
            return;
        }

        /* Extract message text - find "text":"..." */
        const char *text_key = strstr(resp, "\"text\"");
        if (!text_key) {
            if (g_debug) Serial.printf("[TG] no text field\n");
            return;
        }
        p = text_key + 6;
        while (*p == ':' || *p == ' ') p++;
        if (*p != '"') {
            if (g_debug) Serial.printf("[TG] text not a string\n");
            return;
        }
        p++;

        static char msgBuf[512];
        int mw = 0;
        while (*p && *p != '"' && mw < (int)sizeof(msgBuf) - 1) {
            if (*p == '\\' && *(p + 1)) {
                p++;
                if (*p == 'n')
                    msgBuf[mw++] = '\n';
                else
                    msgBuf[mw++] = *p;
            } else {
                msgBuf[mw++] = *p;
            }
            p++;
        }
        msgBuf[mw] = '\0';

        if (mw == 0) {
            if (g_debug) Serial.printf("[TG] empty text\n");
            return;
        }

        Serial.printf("\n[TG] Message from %s: %s\n", incoming_chat_id, msgBuf);

        /* Slash command? Execute locally, no LLM call */
        if (msgBuf[0] == '/') {
            const char *cmd = msgBuf + 1;
            /* Strip @botname suffix (Telegram sends "/status@MyBot" in groups) */
            static char cmdCopy[64];
            strncpy(cmdCopy, cmd, sizeof(cmdCopy) - 1);
            cmdCopy[sizeof(cmdCopy) - 1] = '\0';
            char *at = strchr(cmdCopy, '@');
            if (at) *at = '\0';

            if (handleCommand(cmdCopy, cmdResponseBuf, sizeof(cmdResponseBuf))) {
                Serial.printf("[TG] cmd: /%s -> %s\n", cmdCopy, cmdResponseBuf);
                tgSendMessage(cmdResponseBuf);
            } else {
                snprintf(cmdResponseBuf, sizeof(cmdResponseBuf),
                         "Unknown command: /%s (try /help)", cmdCopy);
                tgSendMessage(cmdResponseBuf);
            }
            Serial.printf("> ");
            return;
        }

        /* Run chat */
        const char *response = chatWithLLM(msgBuf);

        /* Send response back to Telegram */
        if (response) {
            tgSendMessage(response);
        } else {
            tgSendMessage("[error: LLM call failed]");
        }

        Serial.printf("> ");
        return;
    }
    }
}

/** 如果 Telegram TLS 连接仍然活跃，则主动释放，以便给 LLM 腾出堆内存。 */
static void tgYield() {
    if (tgState != TG_IDLE) {
        tgClient.stop();
        tgState = TG_IDLE;
        tgLastPoll = millis();
    }
}

/*============================================================================
 * QQ Bot (via QQ Official Bot API)
 * Uses HTTPS API for sending, WebSocket for receiving messages
 *============================================================================*/

static WiFiClientSecure qqHttpClient;
static WiFiClientSecure qqWsClient;
bool g_qq_enabled = false;
static unsigned long qqLastPoll = 0;
static unsigned long qqLastTokenRefresh = 0;

/* QQ API endpoints */
static const char *QQ_TOKEN_HOST = "bots.qq.com";
static const char *QQ_API_HOST = "api.sgroup.qq.com";
static const int QQ_API_PORT = 443;
static char qqGatewayUrl[256] = "";
static char qqWsHost[96] = "";
static char qqWsPath[192] = "/";
static uint16_t qqWsPort = 443;

/* WebSocket state */
enum QqWsState { QQ_WS_DISCONNECTED,
                 QQ_WS_CONNECTING,
                 QQ_WS_CONNECTED,
                 QQ_WS_READY };
static QqWsState qqWsState = QQ_WS_DISCONNECTED;
static unsigned long qqWsConnectStart = 0;
static unsigned long qqLastHeartbeat = 0;
static unsigned long qqHeartbeatInterval = 41250;
static bool qqHeartbeatAcked = true;
static int qqLastSeq = -1;
static char qqSessionId[96] = "";
#define QQ_WS_RECONNECT_DELAY 30000 /* 30s */
#define QQ_WS_TIMEOUT 60000         /* 60s */

static char qqWsRxBuf[4096];
static int qqWsRxLen = 0;

enum QqReplyTargetKind {
    QQ_REPLY_NONE = 0,
    QQ_REPLY_CHANNEL,
    QQ_REPLY_DM,
    QQ_REPLY_GROUP,
    QQ_REPLY_C2C
};

struct QqIncomingMessage {
    char content[256];
    char eventType[40];
    char replyTarget[64];
    QqReplyTargetKind replyKind;
};

static const uint32_t QQ_INTENT_GUILD_MESSAGES = (1UL << 9);
static const uint32_t QQ_INTENT_DIRECT_MESSAGE = (1UL << 12);
static const uint32_t QQ_INTENT_GROUP_AND_C2C_EVENT = (1UL << 25);
static const uint32_t QQ_INTENT_PUBLIC_GUILD_MESSAGES = (1UL << 30);
static const uint32_t QQ_DEFAULT_INTENTS =
    QQ_INTENT_GUILD_MESSAGES | QQ_INTENT_DIRECT_MESSAGE | QQ_INTENT_GROUP_AND_C2C_EVENT | QQ_INTENT_PUBLIC_GUILD_MESSAGES;

/**
 * 从 QQ HTTPS 接口读取 HTTP 响应到缓冲区中。
 *
 * 当对端关闭、超时或缓冲区写满时读取结束。
 * 结果始终会补上结尾空字符。
 */
static int qqReadHttpResponse(WiFiClientSecure &client, char *buf, int bufSize,
                              unsigned long timeoutMs = 10000) {
    int len = 0;
    unsigned long start = millis();
    while ((millis() - start) < timeoutMs) {
        bool gotData = false;
        while (client.available() && len < bufSize - 1) {
            buf[len++] = client.read();
            gotData = true;
        }
        if (len >= bufSize - 1) break;
        if (!client.connected() && !client.available()) break;
        if (!gotData) delay(1);
    }
    buf[len] = '\0';
    return len;
}

/** 返回原始 HTTP 响应缓冲区中响应体部分的指针。 */
static const char *qqHttpBody(const char *resp) {
    const char *body = strstr(resp, "\r\n\r\n");
    return body ? body + 4 : resp;
}

/** 检查原始 HTTP 响应是否包含 2xx 状态码。 */
static bool qqHttpStatusOk(const char *resp) {
    const char *p = strchr(resp, ' ');
    if (!p) return false;
    int status = atoi(p + 1);
    return status >= 200 && status < 300;
}

/**
 * 从 QQ API 返回的 JSON 载荷中提取字符串字段。
 *
 * 这是一个小型、无动态分配的解析器，只面向 QQ gateway 与 REST
 * 返回里实际会用到的那部分 JSON 结构。
 */
static bool qqJsonExtractString(const char *json, const char *key,
                                char *dst, int dstLen) {
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(json, pattern);
    if (!p) return false;
    p += strlen(pattern);
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (*p != '"') return false;
    p++;

    int w = 0;
    while (*p && *p != '"' && w < dstLen - 1) {
        if (*p == '\\' && *(p + 1)) {
            p++;
            if (*p == 'n') {
                dst[w++] = '\n';
            } else if (*p == 'r') {
                dst[w++] = '\r';
            } else if (*p == 't') {
                dst[w++] = '\t';
            } else {
                dst[w++] = *p;
            }
            p++;
            continue;
        }
        dst[w++] = *p++;
    }
    dst[w] = '\0';
    return w > 0;
}

/** 从 QQ JSON 载荷中提取整数值；若不存在则返回 `fallback`。 */
static int qqJsonExtractInt(const char *json, const char *key, int fallback) {
    char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    const char *p = strstr(json, pattern);
    if (!p) return fallback;
    p += strlen(pattern);
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    return atoi(p);
}

/**
 * 向 QQ gateway 发送一个带掩码的 WebSocket 帧。
 *
 * WebSocket 协议要求客户端发往服务端的数据带掩码，
 * 因此这里会构造帧头、应用随机 mask 并写入网络连接。
 */
static bool qqWsSendFrame(uint8_t opcode, const uint8_t *payload, size_t len) {
    if (!qqWsClient.connected()) return false;
    if (len > 65535) return false;

    uint8_t header[8];
    size_t headerLen = 0;
    header[headerLen++] = 0x80 | (opcode & 0x0F);
    if (len < 126) {
        header[headerLen++] = 0x80 | (uint8_t)len;
    } else {
        header[headerLen++] = 0x80 | 126;
        header[headerLen++] = (uint8_t)((len >> 8) & 0xFF);
        header[headerLen++] = (uint8_t)(len & 0xFF);
    }

    uint32_t mask = esp_random();
    uint8_t maskBytes[4] = {
        (uint8_t)((mask >> 24) & 0xFF),
        (uint8_t)((mask >> 16) & 0xFF),
        (uint8_t)((mask >> 8) & 0xFF),
        (uint8_t)(mask & 0xFF),
    };

    if (qqWsClient.write(header, headerLen) != (int)headerLen) return false;
    if (qqWsClient.write(maskBytes, sizeof(maskBytes)) != (int)sizeof(maskBytes)) return false;

    for (size_t i = 0; i < len; i++) {
        uint8_t b = payload[i] ^ maskBytes[i & 3];
        if (qqWsClient.write(&b, 1) != 1) return false;
    }
    return true;
}

/** 向 QQ gateway 发送文本 WebSocket 帧。 */
static bool qqWsSendText(const char *text) {
    return qqWsSendFrame(0x1, (const uint8_t *)text, strlen(text));
}

/** 在收到服务端 ping 后发送 WebSocket pong 帧。 */
static bool qqWsSendPong(const uint8_t *payload, size_t len) {
    return qqWsSendFrame(0xA, payload, len);
}

/**
 * 带着最近一次看到的序列号向 QQ gateway 发送心跳。
 *
 * 如果心跳确认缺失，会被视为连接已失效，并在 `qqTick()` 中触发重连。
 *
 * 鉴权成功之后，就需要按照周期进行心跳发送。d 为客户端收到的最新的消息的 s，如果是首次连接，d 为传 null
 */
static bool qqWsSendHeartbeat() {
    char payload[64];
    if (qqLastSeq >= 0) {
        snprintf(payload, sizeof(payload), "{\"op\":1,\"d\":%d}", qqLastSeq);
    } else {
        snprintf(payload, sizeof(payload), "{\"op\":1,\"d\":null}");
    }
    if (!qqWsSendText(payload)) {
        return false;
    }
    qqLastHeartbeat = millis();
    qqHeartbeatAcked = false;
    if (g_debug) Serial.printf("[QQ] -> heartbeat seq=%d\n", qqLastSeq);
    return true;
}

/**
 * 关闭当前 QQ WebSocket 会话并重置运行时状态。
 *
 * 如果提供了断开原因就会记录日志，同时重连计时器也会从当前时刻重新开始。
 */
static void qqWsDisconnect(const char *reason) {
    if (reason && *reason) {
        Serial.printf("[QQ] WS disconnect: %s\n", reason);
    }
    qqWsClient.stop();
    qqWsState = QQ_WS_DISCONNECTED;
    qqWsRxLen = 0;
    qqHeartbeatAcked = true;
    qqHeartbeatInterval = 41250;
    qqLastHeartbeat = 0;
    qqWsConnectStart = 0;
    qqLastPoll = millis();
}

/**
 * 将 QQ 返回的 gateway URL 解析为 host、path 和 port。
 *
 * 这里同时支持 `ws://` 和 `wss://`，
 * 从而让传输层连接逻辑无需依赖完整 URL 解析器。
 */
static bool qqParseGatewayUrl() {
    const char *url = qqGatewayUrl;
    bool secure = false;
    if (strncmp(url, "wss://", 6) == 0) {
        secure = true;
        url += 6;
    } else if (strncmp(url, "ws://", 5) == 0) {
        url += 5;
    } else {
        return false;
    }

    const char *path = strchr(url, '/');
    const char *portSep = strchr(url, ':');
    if (!path) path = url + strlen(url);

    qqWsPort = secure ? 443 : 80;
    int hostLen = path - url;
    if (portSep && portSep < path) {
        hostLen = portSep - url;
        qqWsPort = atoi(portSep + 1);
    }
    if (hostLen <= 0 || hostLen >= (int)sizeof(qqWsHost)) return false;

    memcpy(qqWsHost, url, hostLen);
    qqWsHost[hostLen] = '\0';
    strncpy(qqWsPath, *path ? path : "/", sizeof(qqWsPath) - 1);
    qqWsPath[sizeof(qqWsPath) - 1] = '\0';
    return true;
}

/**
 * 获取或复用 QQ bot 的访问令牌，供 REST 与 gateway 共用。
 *
 * 令牌会缓存到临近过期前，避免每次发消息或重连时都重新刷新。
 */
static bool qqGetAccessToken() {
    unsigned long now = millis();

    /* Check if token is still valid */
    if (cfg_qq_access_token[0] != '\0' && now < cfg_qq_token_expires) {
        return true;
    }

    Serial.printf("[QQ] Refreshing access token...\n");

    qqHttpClient.stop();
    if (!qqHttpClient.connect(QQ_TOKEN_HOST, QQ_API_PORT)) {
        Serial.printf("[QQ] Failed to connect to token API\n");
        return false;
    } else {
        Serial.printf("[QQ] Connected to token API\n");
    }

    char body[160];
    snprintf(body, sizeof(body),
             "{\"appId\":\"%s\",\"clientSecret\":\"%s\"}",
             cfg_qq_app_id, cfg_qq_app_secret);
    char req[512];
    snprintf(req, sizeof(req),
             "POST /app/getAppAccessToken HTTP/1.1\r\n"
             "Host: %s\r\n"
             "Content-Type: application/json\r\n"
             "Content-Length: %d\r\n"
             "Connection: close\r\n\r\n"
             "%s",
             QQ_TOKEN_HOST, strlen(body), body);
    qqHttpClient.print(req);

    static char resp[2048];
    qqReadHttpResponse(qqHttpClient, resp, sizeof(resp));
    qqHttpClient.stop();

    /* Parse JSON response */
    /* Expected: {"access_token":"xxx","expires_in":86400} */
    const char *bodyResp = qqHttpBody(resp);
    Serial.printf("[QQ] Access Token Resp: %s\n", bodyResp);
    const char *token = strstr(bodyResp, "\"access_token\":\"");
    if (!token) {
        const char *err = strstr(bodyResp, "Trpc-Error-Msg");
        if (err) {
            Serial.printf("[QQ] Token API error: %.180s\n", err);
        } else {
            Serial.printf("[QQ] Failed to parse token response body: %s\n", bodyResp);
        }
        return false;
    }
    token += 16;

    int i = 0;
    while (*token && *token != '"' && i < 127) {
        cfg_qq_access_token[i++] = *token++;
    }
    cfg_qq_access_token[i] = '\0';

    /* Parse expires_in */
    const char *expires = strstr(bodyResp, "\"expires_in\":");
    if (expires) {
        cfg_qq_token_expires = now + (atoi(expires + 12) - 300) * 1000UL; /* Buffer 5 min */
    } else {
        cfg_qq_token_expires = now + 24 * 3600 * 1000UL; /* Default 24h */
    }

    qqLastTokenRefresh = now;
    Serial.printf("[QQ] Access token obtained: %.10s...\n", cfg_qq_access_token);
    return true;
}

/** 通过 QQ REST API 查询当前的 gateway WebSocket 地址。 */
static bool qqGetGateway() {
    if (!qqGetAccessToken()) return false;

    qqHttpClient.stop();
    if (!qqHttpClient.connect(QQ_API_HOST, QQ_API_PORT)) {
        Serial.printf("[QQ] Gateway connect failed\n");
        return false;
    }

    char path[384];
    snprintf(path, sizeof(path),
             "GET /gateway HTTP/1.1\r\n"
             "Host: %s\r\n"
             "Authorization: QQBot %s\r\n"
             "Connection: close\r\n\r\n",
             QQ_API_HOST, cfg_qq_access_token);

    qqHttpClient.print(path);

    static char resp[1500];
    qqReadHttpResponse(qqHttpClient, resp, sizeof(resp));
    qqHttpClient.stop();
    if (!qqHttpStatusOk(resp)) {
        Serial.printf("[QQ] Gateway HTTP failed: %.120s\n", resp);
        return false;
    }
    /* Parse URL from response: {"url":"wss://.../gateway"} */
    const char *bodyResp = qqHttpBody(resp);
    if (!qqJsonExtractString(bodyResp, "url", qqGatewayUrl, sizeof(qqGatewayUrl))) {
        Serial.printf("[QQ] Gateway parse failed, body: %.180s\n", bodyResp);
        return false;
    }

    Serial.printf("[QQ] Gateway: %s\n", qqGatewayUrl);
    return true;
}

/**
 * 使用任意 REST 路径模板发送 QQ 消息。
 *
 * 上层辅助函数会复用这里的 JSON 序列化与 HTTP 发送逻辑，
 * 以支持频道、私信、群聊和 C2C 等不同消息接口。
 */
static bool qqSendMessageRaw(const char *pathFmt, const char *target_id,
                             const char *content) {
    /* Keep large send buffers out of loopTask stack. */
    static char body[2304];
    static char escaped[2048];
    static char req[4096];
    static char resp[2048];

    if (!qqGetAccessToken()) return false;

    qqHttpClient.stop();
    if (!qqHttpClient.connect(QQ_API_HOST, QQ_API_PORT)) {
        Serial.printf("[QQ] Send: connect failed\n");
        return false;
    }

    /* Build JSON body */

    /* Simple JSON escape */
    int j = 0;
    bool contentTruncated = false;
    int srcIndex = 0;
    for (; content[srcIndex] && j < (int)sizeof(escaped) - 2; srcIndex++) {
        char c = content[srcIndex];
        if (c == '"' || c == '\\') {
            if (j + 2 >= (int)sizeof(escaped)) {
                contentTruncated = true;
                break;
            }
            escaped[j++] = '\\';
            escaped[j++] = c;
        } else if (c == '\n') {
            if (j + 2 >= (int)sizeof(escaped)) {
                contentTruncated = true;
                break;
            }
            escaped[j++] = '\\';
            escaped[j++] = 'n';
        } else {
            escaped[j++] = c;
        }
    }
    if (content[srcIndex] != '\0') {
        contentTruncated = true;
    }
    if (contentTruncated && j < (int)sizeof(escaped) - 4) {
        escaped[j++] = '.';
        escaped[j++] = '.';
        escaped[j++] = '.';
    }
    escaped[j] = '\0';

    int bodyLen = snprintf(body, sizeof(body),
                           "{\"content\":\"%s\"}", escaped);
    if (bodyLen < 0 || bodyLen >= (int)sizeof(body)) {
        Serial.printf("[QQ] Send aborted: request body too large (%d)\n", bodyLen);
        qqHttpClient.stop();
        return false;
    }

    char apiPath[128];
    int apiPathLen = snprintf(apiPath, sizeof(apiPath), pathFmt, target_id);
    if (apiPathLen < 0 || apiPathLen >= (int)sizeof(apiPath)) {
        Serial.printf("[QQ] Send aborted: api path too large\n");
        qqHttpClient.stop();
        return false;
    }
    int reqLen = snprintf(req, sizeof(req),
                          "POST %s HTTP/1.1\r\n"
                          "Host: %s\r\n"
                          "Authorization: QQBot %s\r\n"
                          "Content-Type: application/json\r\n"
                          "Content-Length: %d\r\n"
                          "Connection: close\r\n\r\n%s",
                          apiPath, QQ_API_HOST, cfg_qq_access_token, bodyLen, body);
    if (reqLen < 0 || reqLen >= (int)sizeof(req)) {
        Serial.printf("[QQ] Send aborted: HTTP request too large (%d)\n", reqLen);
        qqHttpClient.stop();
        return false;
    }

    qqHttpClient.write((const uint8_t *)req, reqLen);

    qqReadHttpResponse(qqHttpClient, resp, sizeof(resp));
    qqHttpClient.stop();

    /* Treat any 2xx response as success. Some QQ endpoints may return an
     * empty body instead of a JSON payload with id/msg_id. */
    const char *bodyResp = qqHttpBody(resp);
    if (qqHttpStatusOk(resp)) {
        return true;
    }
    Serial.printf("[QQ] Send response: %s\n", resp);

    Serial.printf("[QQ] Send failed (%s): %.120s | raw=%.120s\n",
                  target_id, bodyResp, resp);
    return false;
}

/** 向 QQ 频道发送消息。 */
bool qqSendMessage(const char *channel_id, const char *content) {
    return qqSendMessageRaw("/api/v1/channels/%s/messages", channel_id, content);
}

/** 向 QQ 群会话发送消息。 */
static bool qqSendGroupMessage(const char *group_openid, const char *content) {
    return qqSendMessageRaw("/v2/groups/%s/messages", group_openid, content);
}

/** 发送与 QQ guild 会话关联的私信消息。 */
static bool qqSendDmMessage(const char *guild_id, const char *content) {
    return qqSendMessageRaw("/dms/%s/messages", guild_id, content);
}

/** 发送一对一 QQ C2C 消息。 */
static bool qqSendC2cMessage(const char *user_openid, const char *content) {
    return qqSendMessageRaw("/v2/users/%s/messages", user_openid, content);
}

/**
 * 根据消息来源，把回复路由到正确的 QQ 发送通道。
 *
 * gateway 对频道、群聊、私信和 C2C 使用的标识不同，
 * 所以这里集中处理回复分发逻辑。
 */
static bool qqReplyMessage(QqReplyTargetKind kind, const char *targetId,
                           const char *content) {
    if (!targetId || targetId[0] == '\0') return false;
    switch (kind) {
    case QQ_REPLY_CHANNEL:
        return qqSendMessage(targetId, content);
    case QQ_REPLY_DM:
        return qqSendDmMessage(targetId, content);
    case QQ_REPLY_GROUP:
        return qqSendGroupMessage(targetId, content);
    case QQ_REPLY_C2C:
        return qqSendC2cMessage(targetId, content);
    default:
        return false;
    }
}

/**
 * 将 QQ gateway 事件转换成统一的入站消息结构。
 *
 * 只有携带聊天内容的事件才会被映射；
 * 不支持的事件会返回 `false`，由调用方直接忽略。
 */
static bool qqParseIncomingMessage(const char *eventType, const char *data,
                                   QqIncomingMessage *msg) {
    memset(msg, 0, sizeof(*msg));
    strncpy(msg->eventType, eventType, sizeof(msg->eventType) - 1);
    if (!qqJsonExtractString(data, "content", msg->content, sizeof(msg->content))) { // 拷贝消息内容
        return false;
    }

    if (strcmp(eventType, "AT_MESSAGE_CREATE") == 0 || strcmp(eventType, "MESSAGE_CREATE") == 0) {
        msg->replyKind = QQ_REPLY_CHANNEL;
        qqJsonExtractString(data, "channel_id", msg->replyTarget, sizeof(msg->replyTarget));
    } else if (strcmp(eventType, "DIRECT_MESSAGE_CREATE") == 0) {
        msg->replyKind = QQ_REPLY_DM;
        qqJsonExtractString(data, "guild_id", msg->replyTarget, sizeof(msg->replyTarget));
    } else if (strcmp(eventType, "GROUP_AT_MESSAGE_CREATE") == 0) {
        msg->replyKind = QQ_REPLY_GROUP;
        qqJsonExtractString(data, "group_openid", msg->replyTarget, sizeof(msg->replyTarget));
    } else if (strcmp(eventType, "C2C_MESSAGE_CREATE") == 0) {
        msg->replyKind = QQ_REPLY_C2C;
        /* C2C send API expects the user_openid when available. */
        qqJsonExtractString(data, "user_openid", msg->replyTarget, sizeof(msg->replyTarget));
        const char *authorId = strstr(data, "\"author\":{\"id\":\""); // 获取这条消息来源的人的id
        if (msg->replyTarget[0] == '\0' && authorId) {
            authorId += 16;
            int w = 0;
            while (authorId[w] && authorId[w] != '"' && w < (int)sizeof(msg->replyTarget) - 1) {
                msg->replyTarget[w] = authorId[w];
                w++;
            }
            msg->replyTarget[w] = '\0';
        }
        // 如果 author.id 没取到，再尝试备用字段
        if (msg->replyTarget[0] == '\0') {
            qqJsonExtractString(data, "author_openid", msg->replyTarget, sizeof(msg->replyTarget));
        }
        if (msg->replyTarget[0] == '\0') {
            qqJsonExtractString(data, "openid", msg->replyTarget, sizeof(msg->replyTarget));
        }
    }
    return msg->content[0] != '\0';
}

/**
 * 将已解析的 QQ 消息作为命令或普通聊天来处理。
 *
 * 回复会通过该消息原本进入的同类通道发回去。
 */
static void qqHandleIncomingMessage(const QqIncomingMessage *msg) {
    Serial.printf("\n[QQ] %s: %s\n", msg->eventType, msg->content);

    // 如果用户发的是斜杆命令，就本地处理，不走 LLM，直接回复结果
    if (msg->content[0] == '/') {
        // 提取命令名,/status -> status
        const char *cmd = msg->content + 1;
        static char cmdCopy[64];
        strncpy(cmdCopy, cmd, sizeof(cmdCopy) - 1);
        cmdCopy[sizeof(cmdCopy) - 1] = '\0';
        char *space = strchr(cmdCopy, ' ');
        if (space) *space = '\0';

        if (handleCommand(cmdCopy, cmdResponseBuf, sizeof(cmdResponseBuf))) { // 处理命令
            Serial.printf("[QQ] cmd: /%s -> %s\n", cmdCopy, cmdResponseBuf);
        } else {
            snprintf(cmdResponseBuf, sizeof(cmdResponseBuf), "Unknown command: /%s", cmdCopy);
        }

        if (msg->replyKind != QQ_REPLY_NONE && msg->replyTarget[0] != '\0') {
            qqReplyMessage(msg->replyKind, msg->replyTarget, cmdResponseBuf);
        }
        return;
    }

    // 对于普通消息，先释放 Telegram 连接（如果有的话），再调用 LLM 处理并回复
    tgYield();
    const char *response = chatWithLLM(msg->content);
    if (response && msg->replyKind != QQ_REPLY_NONE && msg->replyTarget[0] != '\0') {
        qqReplyMessage(msg->replyKind, msg->replyTarget, response);
    }
}

/**
 * 在收到 QQ gateway 的 HELLO 事件后发送初始 IDENTIFY 载荷。
 *
 * 这样会把 WebSocket 会话从“已连接”推进到“已鉴权”，
 * 使 bot 可以开始接收后续分发事件。
 */
static bool qqSendIdentify() {
    char identify[256];
    snprintf(identify, sizeof(identify),
             "{\"op\":2,\"d\":{\"token\":\"QQBot %s\",\"intents\":%lu,"
             "\"properties\":{\"$os\":\"linux\",\"$browser\":\"wireclaw\",\"$device\":\"wireclaw\"}}}",
             cfg_qq_access_token, (unsigned long)QQ_DEFAULT_INTENTS);
    if (!qqWsSendText(identify)) {
        Serial.printf("[QQ] IDENTIFY send failed\n");
        return false;
    }
    qqWsState = QQ_WS_CONNECTED;
    if (g_debug) Serial.printf("[QQ] -> identify intents=%lu\n", (unsigned long)QQ_DEFAULT_INTENTS);
    return true;
}

/**
 * 处理已经解码完成的 QQ gateway JSON 载荷。
 *
 * 该处理器会根据 opcode 和事件类型，更新序列号、连接状态、
 * 心跳时序以及聊天分发逻辑。
 */
static void qqHandleGatewayPayload(const char *data) {
    if (g_debug) Serial.printf("[QQ] receive data:%s \n", data);
    int seq = qqJsonExtractInt(data, "s", -1);
    if (seq >= 0) qqLastSeq = seq;

    int op = qqJsonExtractInt(data, "op", -1);
    switch (op) {
    case 10: { // 收到服务器下发的 HELLO 事件，当客户端与网关建立ws连接之后，网关下发的第一条消息,里面包含了心跳间隔
        int interval = qqJsonExtractInt(data, "heartbeat_interval", 41250);
        if (interval > 1000) qqHeartbeatInterval = (unsigned long)interval;
        qqHeartbeatAcked = true;
        qqLastHeartbeat = millis();
        Serial.printf("[QQ] <- hello interval=%lu ms\n", qqHeartbeatInterval);
        if (!qqSendIdentify()) {
            qqWsDisconnect("identify failed");
        }
        return;
    }
    case 11: // Heartbeat ACK	Receive/Reply	当发送心跳成功之后，就会收到该消息
        qqHeartbeatAcked = true;
        if (g_debug) Serial.printf("[QQ] <- heartbeat ack\n");
        return;
    case 7: // Reconnect	服务端通知客户端重新连接
        qqWsDisconnect("server requested reconnect");
        return;
    case 9: // Invalid Session	Receive	当identify或resume的时候，如果参数有错，服务端会返回该消息
        qqSessionId[0] = '\0';
        qqLastSeq = -1;
        qqWsDisconnect("invalid session");
        return;
    case 0:    // Dispatch	Receive	服务端进行消息推送
        break; // 这里先不处理，延迟到后面处理
    default:
        if (g_debug) Serial.printf("[QQ] <- op=%d payload=%.120s\n", op, data);
        return;
    }

    char eventType[40];
    if (!qqJsonExtractString(data, "t", eventType, sizeof(eventType))) {
        return;
    }

    if (strcmp(eventType, "READY") == 0) {
        qqJsonExtractString(data, "session_id", qqSessionId, sizeof(qqSessionId));
        qqWsState = QQ_WS_READY;
        Serial.printf("[QQ] READY session=%s\n", qqSessionId[0] ? qqSessionId : "<none>");
        return;
    }

    if (strcmp(eventType, "RESUMED") == 0) {
        qqWsState = QQ_WS_READY;
        Serial.printf("[QQ] RESUMED\n");
        return;
    }

    QqIncomingMessage msg;
    if (qqParseIncomingMessage(eventType, data, &msg)) { // 解析其他事件
        qqHandleIncomingMessage(&msg);                   // 解析到事件就处理
    } else if (g_debug) {
        Serial.printf("[QQ] Ignored event %s\n", eventType);
    }
}

/**
 * 消费并解码网络缓冲区中的 QQ WebSocket 帧。
 *
 * 文本帧会被当成 gateway JSON 载荷处理；
 * 控制帧则用于驱动 ping/pong 和连接断开逻辑。
 *
 * 前半段是在实现 RFC 6455 WebSocket 帧解析：RFC 6455 Section 5.2-》https://datatracker.ietf.org/doc/html/rfc6455#section-5.2
 *  后半段才是在处理 QQ gateway 的业务消息
 */
static void qqWsProcess() {
    if (!qqWsClient.available()) return;

    /* Read available data */
    while (qqWsClient.available() && qqWsRxLen < sizeof(qqWsRxBuf) - 1) {
        qqWsRxBuf[qqWsRxLen++] = qqWsClient.read();
    }

    if (qqWsRxLen >= (int)sizeof(qqWsRxBuf) - 1) {
        qqWsDisconnect("rx buffer overflow");
        return;
    }

    /* -------帧解析------- */
    while (qqWsRxLen >= 2) {
        const uint8_t *buf = (const uint8_t *)qqWsRxBuf;
        bool masked = (buf[1] & 0x80) != 0;
        uint64_t payloadLen = buf[1] & 0x7F;
        int headerLen = 2;

        if (payloadLen == 126) {
            if (qqWsRxLen < 4) return;
            payloadLen = ((uint16_t)buf[2] << 8) | buf[3];
            headerLen = 4;
        } else if (payloadLen == 127) {
            if (qqWsRxLen < 10) return;
            payloadLen = 0;
            for (int i = 0; i < 8; i++) {
                payloadLen = (payloadLen << 8) | buf[2 + i];
            }
            headerLen = 10;
        }

        if (masked) headerLen += 4;
        if ((uint64_t)qqWsRxLen < (uint64_t)headerLen + payloadLen) return;

        uint8_t opcode = buf[0] & 0x0F;
        const uint8_t *payload = buf + headerLen;
        uint8_t maskBytes[4] = {0, 0, 0, 0};
        if (masked) {
            memcpy(maskBytes, buf + headerLen - 4, 4);
        }

        static uint8_t framePayload[2048];
        size_t copyLen = payloadLen < sizeof(framePayload) - 1 ? (size_t)payloadLen : sizeof(framePayload) - 1;
        for (size_t i = 0; i < copyLen; i++) {
            framePayload[i] = masked ? (payload[i] ^ maskBytes[i & 3]) : payload[i];
        }
        framePayload[copyLen] = '\0';

        int consumed = headerLen + (int)payloadLen;
        memmove(qqWsRxBuf, qqWsRxBuf + consumed, qqWsRxLen - consumed);
        qqWsRxLen -= consumed;

        /* ======处理 QQ gateway 的业务消息====== */

        if (opcode == 0x1) { // 0x1 是 WebSocket text frameQQ gateway 发来的业务事件是 JSON 文本，
            qqHandleGatewayPayload((const char *)framePayload);
            /* ================== */
        } else if (opcode == 0x8) { // 0x8 是 Close frame
            qqWsDisconnect("close frame");
            return;
        } else if (opcode == 0x9) { // 0x9 是 Ping frame，收到后应该尽快回复 Pong。
            qqWsSendPong(framePayload, copyLen);
        } else if (opcode == 0xA) { // 0xA 是 Pong frame，通常是对 Ping 的回应，可以用来确认连接活跃。
            if (g_debug) Serial.printf("[QQ] <- pong\n");
        } else if (g_debug) {
            Serial.printf("[QQ] Ignored opcode=%u len=%u\n", opcode, (unsigned)payloadLen);
        }
    }
    /* ---------------- */
}

/**
 * 打开 QQ gateway WebSocket，并完成 HTTP Upgrade 握手。
 *
 * 握手成功后，代码会先等待 HELLO 事件，再发送 IDENTIFY，
 * 以符合 gateway 的协议流程。
 */
static bool qqWsConnect() {
    if (qqGatewayUrl[0] == '\0') {
        if (!qqGetGateway()) return false;
    }
    if (!qqParseGatewayUrl()) {
        Serial.printf("[QQ] Invalid gateway URL: %s\n", qqGatewayUrl);
        return false;
    }

    Serial.printf("[QQ] Connecting to WebSocket...\n");

    qqWsClient.stop();
    qqWsRxLen = 0;
    delay(50);
    if (!qqWsClient.connect(qqWsHost, qqWsPort)) {
        Serial.printf("[QQ] WS connect failed\n");
        return false;
    } else {
        Serial.printf("[QQ] WS connect success\n");
    }

    /* Build WebSocket handshake request */
    static char wsReq[512];
    snprintf(wsReq, sizeof(wsReq),
             "GET %s HTTP/1.1\r\n"
             "Host: %s\r\n"
             "Upgrade: websocket\r\n"
             "Connection: Upgrade\r\n"
             "Sec-WebSocket-Version: 13\r\n"
             "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
             "\r\n",
             qqWsPath,
             qqWsHost);

    qqWsClient.print(wsReq);
    qqWsState = QQ_WS_CONNECTING;
    qqWsConnectStart = millis();

    /* Wait for handshake response */
    unsigned long start = millis();
    char resp[1024];
    int len = 0;
    resp[0] = '\0';
    while (qqWsClient.connected() && (millis() - start) < 10000) {
        bool gotData = false;
        while (qqWsClient.available() && len < (int)sizeof(resp) - 1) {
            resp[len++] = qqWsClient.read();
            resp[len] = '\0';
            gotData = true;
            if (len >= 4 && strstr(resp, "\r\n\r\n")) break;
        }
        if (len >= (int)sizeof(resp) - 1) break;
        if (len >= 4 && strstr(resp, "\r\n\r\n")) break;
        if (!gotData) delay(1);
    }
    resp[len] = '\0';
    Serial.printf("[QQ] WebSocket Handshake Resp:%s\n", resp);
    if (!strstr(resp, "101")) {
        Serial.printf("[QQ] WS handshake failed: %s\n", resp);
        qqWsClient.stop();
        return false;
    }
    qqHeartbeatAcked = true;
    qqHeartbeatInterval = 41250;
    qqLastHeartbeat = millis();
    Serial.printf("[QQ] WebSocket connected, waiting for HELLO\n");

    return true;
}

/**
 * 在一次 `loop()` 迭代中推进 QQ 连接状态。
 *
 * 该函数负责处理重连时机、令牌可用性、WebSocket I/O、
 * 心跳调度以及失效连接检测。
 */
static void qqTick() {
    unsigned long now = millis();
    /* Check if we should reconnect */
    if (qqWsState == QQ_WS_DISCONNECTED) {
        if (now - qqLastPoll < QQ_WS_RECONNECT_DELAY) return;
        if (!qqGetAccessToken()) {
            if (g_debug) Serial.printf("[QQ] Tick: token unavailable\n");
            qqLastPoll = now;
            return;
        }

        if (!qqWsConnect()) {
            Serial.printf("[QQ] Tick: WS connect failed, retry in %d ms\n", QQ_WS_RECONNECT_DELAY);
        }
        qqLastPoll = now;
        return;
    }

    if (qqWsState == QQ_WS_CONNECTING && now - qqWsConnectStart > QQ_WS_TIMEOUT) {
        qqWsDisconnect("hello timeout");
        return;
    }

    /* Send heartbeat */
    if ((qqWsState == QQ_WS_CONNECTED || qqWsState == QQ_WS_READY) && now - qqLastHeartbeat > qqHeartbeatInterval) {
        if (!qqHeartbeatAcked) {
            qqWsDisconnect("heartbeat ack timeout");
            return;
        }
        if (!qqWsSendHeartbeat()) {
            qqWsDisconnect("heartbeat send failed");
            return;
        }
    }

    /* Process incoming data */
    if (qqWsState != QQ_WS_DISCONNECTED) {
        qqWsProcess();
    }

    /* Check connection status */
    if (!qqWsClient.connected() && qqWsState != QQ_WS_DISCONNECTED) {
        Serial.printf("[QQ] WebSocket disconnected\n");
        qqWsDisconnect("socket closed");
    }
}

/**
 * 从 QQ 获取基础 bot 身份信息，以校验 access token 是否有效。
 *
 * 该响应只在启动阶段用作一次轻量级的连通性与凭据有效性检查。
 */
static bool qqGetBotInfo() {
    if (!qqGetAccessToken()) return false;

    qqHttpClient.stop();
    if (!qqHttpClient.connect(QQ_API_HOST, QQ_API_PORT)) {
        return false;
    }

    char path[256];
    snprintf(path, sizeof(path),
             "GET /api/v1/users/me HTTP/1.1\r\n"
             "Host: %s\r\n"
             "Authorization: QQBot %s\r\n"
             "Connection: close\r\n\r\n",
             QQ_API_HOST, cfg_qq_access_token);

    qqHttpClient.print(path);

    static char resp[512];
    qqReadHttpResponse(qqHttpClient, resp, sizeof(resp));
    qqHttpClient.stop();

    return qqHttpStatusOk(resp) && strstr(qqHttpBody(resp), "\"id\":") != nullptr;
}

/** 释放当前 QQ WebSocket 会话，为 LLM 工作腾出堆内存。 */
static void qqYield() {
    if (qqWsState != QQ_WS_DISCONNECTED) {
        qqWsDisconnect("yield");
    }
}

/*============================================================================
 * Serial Commands
 *============================================================================*/

/**
 * 处理串口控制台收到的一整行输入。
 *
 * 某些命令只在串口下提供以方便调试；
 * 其余输入则转交给共享命令处理器或普通聊天流程。
 */
void handleSerialCommand(const char *input) {
    /* Serial-only commands (not available via Telegram/NATS) */
    if (strcmp(input, "/config") == 0) {
        Serial.printf("--- config ---\n");
        Serial.printf("WiFi SSID: %s\n", cfg_wifi_ssid);
        Serial.printf("API key:   %.8s...\n", cfg_api_key);
        Serial.printf("Model:     %s\n", cfg_model);
        Serial.printf("Device:    %s\n", cfg_device_name);
        Serial.printf("NATS:      %s:%d (%s)\n", cfg_nats_host, cfg_nats_port,
                      g_nats_enabled ? "enabled" : "disabled");
        Serial.printf("Telegram:  %s\n", g_telegram_enabled ? "enabled" : "disabled");
        Serial.printf("QQ:        %s\n", g_qq_enabled ? "enabled" : "disabled");
        Serial.printf("Prompt:    %d chars\n", (int)strlen(cfg_system_prompt));
        Serial.printf("> ");
        return;
    }

    if (strcmp(input, "/prompt") == 0) {
        Serial.printf("--- system prompt ---\n%s\n---\n> ", cfg_system_prompt);
        return;
    }

    if (strcmp(input, "/history full") == 0) {
        if (historyCount == 0) {
            Serial.printf("No conversation history.\n> ");
            return;
        }
        Serial.printf("--- history (%d turns) ---\n", historyCount);
        for (int i = 0; i < historyCount; i++) {
            Serial.printf("[%d] User: %s\n", i + 1, history[i].user);
            Serial.printf("[%d] Assistant: %s\n\n", i + 1, history[i].assistant);
        }
        Serial.printf("---\n> ");
        return;
    }

    if (strcmp(input, "/setup") == 0) {
        Serial.printf("Starting setup portal...\n");
        runSetupPortal(); /* blocks until config saved + reboot */
        return;
    }

    /* Shared commands — delegate to handleCommand() */
    if (input[0] == '/') {
        const char *cmd = input + 1;
        if (handleCommand(cmd, cmdResponseBuf, sizeof(cmdResponseBuf))) {
            Serial.printf("%s\n> ", cmdResponseBuf);
            return;
        }
    }

    /* Unknown command - treat as chat */
    tgYield(); /* Free Telegram TLS so LLM can allocate */
    chatWithLLM(input);
    Serial.printf("> ");
}

/*============================================================================
 * Setup & Loop
 *============================================================================*/

/**
 * 执行一次性的固件初始化，并启动所有已启用服务。
 *
 * 包括加载配置与历史记录、初始化硬件抽象、
 * 连接网络后端以及启动运行时 Web 配置界面。
 */
void setup() {
    Serial.begin(115200);
    delay(5000);

    Serial.printf("\n\n");
    Serial.printf("========================================\n");
    Serial.printf("  WireClaw v%s\n", WIRECLAW_VERSION);
    Serial.printf("========================================\n\n");

    /* Load config from LittleFS */
    loadConfig();
    historyLoad();
    Serial.printf("Model: %s\n", cfg_model);

    /* Initialize temperature sensor (not available on classic ESP32) */
#if !defined(CONFIG_IDF_TARGET_ESP32)
    initTempSensor();
    if (g_temp_sensor) {
        float temp = 0.0f;
        temperature_sensor_get_celsius(g_temp_sensor, &temp);
        Serial.printf("Chip temp: %.1f C\n", temp);
    }
#endif

    /* Initialize device registry and rule engine */
    devicesInit();
    rulesInit();

    if (cfg_wifi_ssid[0] == '\0') {
        Serial.printf("\n[!] No WiFi config — starting setup portal\n");
        runSetupPortal(); /* blocks until config saved + reboot */
    }

    /* Connect WiFi */
    if (!connectWiFi()) {
        Serial.printf("[!] WiFi failed — starting setup portal\n");
        runSetupPortal(); /* blocks until config saved + reboot */
    }

    /* NTP time sync */
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    setenv("TZ", cfg_timezone, 1);
    tzset();
    Serial.printf("NTP: syncing (TZ=%s)...\n", cfg_timezone);

    /* Init LLM client */
    llm.begin(cfg_api_key, cfg_model, cfg_api_base_url);

    /* Watchdog - reconfigure to 60s (Arduino already inits WDT at 5s) */
    esp_task_wdt_config_t wdt_cfg = {.timeout_ms = 60000, .idle_core_mask = 0, .trigger_panic = true};
    esp_task_wdt_reconfigure(&wdt_cfg);
    esp_task_wdt_add(NULL); /* Add loop task */

    /* Connect NATS (optional) */
    if (cfg_nats_host[0] != '\0') {
        g_nats_enabled = true;
        buildNatsSubjects();
        if (!connectNats()) {
            Serial.printf("NATS: will retry in background\n");
        }
    } else {
        Serial.printf("NATS: disabled (no nats_host in config)\n");
    }

    /* Telegram (optional) */
    if (cfg_telegram_token[0] != '\0' && cfg_telegram_chat_id[0] != '\0') {
        g_telegram_enabled = true;
        tgClient.setInsecure();
        tgClient.setTimeout(30); /* seconds - matches LLM client pattern */
        tgLastPoll = millis();   /* delay first poll by one interval */
        Serial.printf("Telegram: enabled (chat_id %s)\n", cfg_telegram_chat_id);
        char startMsg[160];
        snprintf(startMsg, sizeof(startMsg),
                 "WireClaw v%s started\nConfig: http://%s/\nmDNS: http://%s.local/",
                 WIRECLAW_VERSION, WiFi.localIP().toString().c_str(), cfg_device_name);
        tgSendMessage(startMsg);
    } else {
        Serial.printf("Telegram: disabled (no telegram_token/telegram_chat_id in config)\n");
    }

    /* QQ Bot via Official API (optional) */
    if (cfg_qq_app_id[0] != '\0' && cfg_qq_app_secret[0] != '\0') {
        g_qq_enabled = true;
        qqHttpClient.setInsecure();
        qqHttpClient.setTimeout(30);
        qqWsClient.setInsecure();
        qqWsClient.setTimeout(30);
        qqLastPoll = millis() - QQ_WS_RECONNECT_DELAY; /* allow immediate first connect */
        Serial.printf("QQ: enabled (AppID: %s)\n", cfg_qq_app_id);
        /* Get access token and verify connection */
        if (qqGetAccessToken()) {
            Serial.printf("QQ: access token obtained\n");
            if (qqGetBotInfo()) {
                Serial.printf("QQ: bot info verified\n");
            }
            /* Try to connect WebSocket for receiving messages */
            qqTick();
        }
    } else {
        Serial.printf("QQ: disabled (no qq_app_id/qq_app_secret in config)\n");
    }

    /* Web config portal (HTTP on port 80 + mDNS) */
    webConfigSetup();

    Serial.printf("\nReady! Free heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("Type a message and press Enter. /help for commands.\n\n");
    Serial.printf("> ");
}

/* LED heartbeat state */
static unsigned long lastHeartbeat = 0;
#define HEARTBEAT_INTERVAL_MS 3000

/**
 * 固件主事件循环。
 *
 * 循环内会持续处理连接状态、Web 配置、消息后端、自动化逻辑、
 * 串口输入和延迟重启，同时持续喂狗。

 */
void loop() {
    esp_task_wdt_reset(); /* Feed the watchdog */

    /* LED heartbeat - brief dim green blink when idle */
    if (!g_led_user) {
        unsigned long now = millis();
        if (now - lastHeartbeat > HEARTBEAT_INTERVAL_MS) {
            lastHeartbeat = now;
            led(0, 40, 0); /* dim green */
            delay(50);
            ledOff();
        }
    }

    /* Check WiFi */
    if (WiFi.status() != WL_CONNECTED) {
        Serial.printf("\nWiFi disconnected! Reconnecting...\n");
        ledRed();
        if (!connectWiFi()) {
            delay(5000);
            return;
        }
        Serial.printf("> ");
    }

    /* Process web config requests */
    webConfigLoop();

    /* Process NATS */
    if (g_nats_enabled) {
        if (natsClient.connected()) {
            nats_err_t err = natsClient.process();
            if (err != NATS_OK && err != NATS_ERR_WOULD_BLOCK) {
                if (g_debug) Serial.printf("NATS: process error: %s\n",
                                           nats_err_str(err));
            }
        } else {
            /* Reconnect with backoff */
            unsigned long now = millis();
            if (now - natsLastReconnect > NATS_RECONNECT_DELAY_MS) {
                natsLastReconnect = now;
                connectNats();
            }
        }
    }

    /* Poll Telegram */
    if (g_telegram_enabled) {
        telegramTick();
    }

    /* Poll QQ via go-cqhttp */
    if (g_qq_enabled) {
        qqTick();
    }

    /* Keep sensor EMA values warm (every 10s) */
    sensorsPoll();

    /* Evaluate automation rules */
    rulesEvaluate();

    /* Poll serial_text UART for incoming data */
    serialTextPoll();

    /* Deferred reboot (allows Telegram ACK cycle to complete) */
    if (g_reboot_pending && millis() >= g_reboot_at) {
        Serial.printf("[Reboot] Deferred restart now\n");
        delay(200);
        ESP.restart();
    }

    /* Read serial input character by character */
    while (Serial.available()) {
        char c = Serial.read();

        /* Handle backspace */
        if (c == '\b' || c == 127) {
            if (serialPos > 0) {
                serialPos--;
                Serial.print("\b \b");
            }
            continue;
        }

        /* Handle enter */
        if (c == '\n' || c == '\r') {
            if (serialPos == 0) continue; /* Ignore empty lines */

            serialBuf[serialPos] = '\0';
            serialPos = 0;
            Serial.println(); /* Echo newline */

            /* Trim whitespace */
            char *input = serialBuf;
            while (*input == ' ') input++;
            int len = strlen(input);
            while (len > 0 && input[len - 1] == ' ') input[--len] = '\0';

            if (len == 0) {
                Serial.printf("> ");
                continue;
            }

            /* Process input */
            if (input[0] == '/') {
                handleSerialCommand(input);
            } else {
                tgYield(); /* Free Telegram TLS so LLM can allocate */
                chatWithLLM(input);
                Serial.printf("> ");
            }
            continue;
        }

        /* Buffer character */
        if (serialPos < SERIAL_BUF_SIZE - 1) {
            serialBuf[serialPos++] = c;
            Serial.print(c); /* Echo */
        }
    }

    delay(10); /* Yield */
}
