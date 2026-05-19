// ============================================================
// drv_zce_ws_cmd.c — ZCE EM backend WebSocket command client
//
// Purpose:
//   - Open one outbound WSS connection to the backend command endpoint.
//   - Send the hello handshake required by DEVICE_INTEGRATION_V1.4.
//   - Receive relay commands: {"cmd":"relay","value":0|1}
//   - Apply the command through the existing TuyaMCU channel mapping.
//   - ACK the command and request an immediate telemetry publish.
//
// Important architecture rule:
//   MQTT is kept for outbound telemetry/status/availability only.
//   This file is the only cloud inbound command path.
// ============================================================

#include "drv_zce_ws_cmd.h"
#include "drv_zce_bin.h"
#include "../new_common.h"
#include "../obk_config.h"
#include "../logging/logging.h"
#include "../cmnds/cmd_public.h"

#ifdef ENABLE_DRIVER_ZCE_BIN

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include "lwip/sockets.h"
#ifndef WINDOWS
#include "lwip/netdb.h"
#endif

// ZCE command WebSocket is always WSS in protocol v1.4.
// Do not depend on MQTT_USE_TLS here: that macro belongs to the MQTT client
// and is not reliably propagated to every driver source in this build system.
#define ZCE_WS_CMD_USE_TLS 1

#if ZCE_WS_CMD_USE_TLS
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#endif

#define ZCE_WS_HOST       "mqtt-service-staging-894834276803.europe-west1.run.app"
#define ZCE_WS_PORT       443
#define ZCE_WS_PATH       "/ws/device"
#define ZCE_WS_RECONNECT_DELAY_MS 5000
#define ZCE_WS_STACK_SIZE 8192
#define ZCE_WS_RX_MAX     512
#define ZCE_WS_HTTP_MAX   768

extern int Main_HasWiFiConnected(void);
extern int g_secondsElapsed;

static beken_thread_t s_ws_thread = NULL;
static volatile bool s_ws_stop = false;
static volatile bool s_ws_connected = false;

// ------------------------------------------------------------
// TCP helpers
// ------------------------------------------------------------
static int zce_tcp_connect(const char *host, int port) {
    struct addrinfo hints;
    struct addrinfo *addrInfoList = NULL;
    struct addrinfo *cur = NULL;
    int fd = -1;
    char service[8];

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    snprintf(service, sizeof(service), "%d", port);

    if (getaddrinfo(host, service, &hints, &addrInfoList) != 0) {
        addLogAdv(LOG_WARN, LOG_FEATURE_MAIN, "ZCE_WS: DNS failed for %s", host);
        return -1;
    }

    for (cur = addrInfoList; cur != NULL; cur = cur->ai_next) {
        fd = socket(cur->ai_family, cur->ai_socktype, cur->ai_protocol);
        if (fd < 0) continue;

        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        if (connect(fd, cur->ai_addr, cur->ai_addrlen) == 0) {
            break;
        }

        lwip_close(fd);
        fd = -1;
    }

    if (addrInfoList) freeaddrinfo(addrInfoList);
    return fd;
}

#if ZCE_WS_CMD_USE_TLS
// ------------------------------------------------------------
// Minimal mbedTLS wrapper around an lwIP socket.
// Certificate verification is optional because the existing firmware does not
// carry a CA bundle. SNI is still set to the backend hostname.
// ------------------------------------------------------------
typedef struct {
    int fd;
    mbedtls_ssl_context ssl;
    mbedtls_ssl_config conf;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    bool ready;
} zce_tls_t;

static int zce_tls_send_cb(void *ctx, const unsigned char *buf, size_t len) {
    int fd = *((int *)ctx);
    int ret = send(fd, buf, len, 0);
    if (ret < 0) return MBEDTLS_ERR_SSL_WANT_WRITE;
    return ret;
}

static int zce_tls_recv_cb(void *ctx, unsigned char *buf, size_t len) {
    int fd = *((int *)ctx);
    int ret = recv(fd, buf, len, 0);
    if (ret < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) return MBEDTLS_ERR_SSL_WANT_READ;
        return -1;
    }
    return ret;
}

static void zce_tls_init(zce_tls_t *t) {
    memset(t, 0, sizeof(*t));
    t->fd = -1;
    mbedtls_ssl_init(&t->ssl);
    mbedtls_ssl_config_init(&t->conf);
    mbedtls_entropy_init(&t->entropy);
    mbedtls_ctr_drbg_init(&t->ctr_drbg);
}

static void zce_tls_free(zce_tls_t *t) {
    if (t->ready) {
        mbedtls_ssl_close_notify(&t->ssl);
    }
    mbedtls_ssl_free(&t->ssl);
    mbedtls_ssl_config_free(&t->conf);
    mbedtls_ctr_drbg_free(&t->ctr_drbg);
    mbedtls_entropy_free(&t->entropy);
    if (t->fd >= 0) {
        lwip_close(t->fd);
        t->fd = -1;
    }
    t->ready = false;
}

static int zce_tls_connect(zce_tls_t *t, const char *host, int port) {
    int ret;
    const char *pers = "zce_ws_cmd";

    t->fd = zce_tcp_connect(host, port);
    if (t->fd < 0) return -1;

    ret = mbedtls_ctr_drbg_seed(&t->ctr_drbg, mbedtls_entropy_func, &t->entropy,
                                (const unsigned char *)pers, strlen(pers));
    if (ret != 0) return ret;

    ret = mbedtls_ssl_config_defaults(&t->conf,
                                      MBEDTLS_SSL_IS_CLIENT,
                                      MBEDTLS_SSL_TRANSPORT_STREAM,
                                      MBEDTLS_SSL_PRESET_DEFAULT);
    if (ret != 0) return ret;

    mbedtls_ssl_conf_authmode(&t->conf, MBEDTLS_SSL_VERIFY_OPTIONAL);
    mbedtls_ssl_conf_rng(&t->conf, mbedtls_ctr_drbg_random, &t->ctr_drbg);

    ret = mbedtls_ssl_setup(&t->ssl, &t->conf);
    if (ret != 0) return ret;

    mbedtls_ssl_set_hostname(&t->ssl, host);
    mbedtls_ssl_set_bio(&t->ssl, &t->fd, zce_tls_send_cb, zce_tls_recv_cb, NULL);

    int loops = 0;
    while ((ret = mbedtls_ssl_handshake(&t->ssl)) != 0) {
        if (ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
            return ret;
        }
        if (++loops > 100) return ret;
        rtos_delay_milliseconds(50);
        if (s_ws_stop) return -1;
    }

    t->ready = true;
    return 0;
}

static int zce_net_write(zce_tls_t *t, const uint8_t *buf, int len) {
    int done = 0;
    while (done < len && !s_ws_stop) {
        int ret = mbedtls_ssl_write(&t->ssl, buf + done, len - done);
        if (ret > 0) {
            done += ret;
        } else if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
            rtos_delay_milliseconds(10);
        } else {
            return ret;
        }
    }
    return done;
}

static int zce_net_read(zce_tls_t *t, uint8_t *buf, int len) {
    while (!s_ws_stop) {
        int ret = mbedtls_ssl_read(&t->ssl, buf, len);
        if (ret > 0) return ret;
        if (ret == 0) return 0;
        if (ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) {
            rtos_delay_milliseconds(10);
            continue;
        }
        return ret;
    }
    return -1;
}
#endif

// ------------------------------------------------------------
// WebSocket helpers
// ------------------------------------------------------------
static int zce_ws_write_all(void *ctx, const uint8_t *buf, int len) {
#if ZCE_WS_CMD_USE_TLS
    return zce_net_write((zce_tls_t *)ctx, buf, len);
#else
    (void)ctx; (void)buf; (void)len;
    return -1;
#endif
}

static int zce_ws_read_some(void *ctx, uint8_t *buf, int len) {
#if ZCE_WS_CMD_USE_TLS
    return zce_net_read((zce_tls_t *)ctx, buf, len);
#else
    (void)ctx; (void)buf; (void)len;
    return -1;
#endif
}

static int zce_ws_read_exact(void *ctx, uint8_t *buf, int len) {
    int done = 0;
    while (done < len && !s_ws_stop) {
        int r = zce_ws_read_some(ctx, buf + done, len - done);
        if (r <= 0) return r;
        done += r;
    }
    return done;
}

static uint32_t zce_ws_mask_seed(void) {
    return ((uint32_t)g_secondsElapsed * 1103515245u) ^ 0x5A31C0DEu;
}

static int zce_ws_send_frame(void *ctx, uint8_t opcode, const char *payload, int payloadLen) {
    uint8_t hdr[14];
    uint8_t mask[4];
    int hlen = 0;
    uint32_t seed = zce_ws_mask_seed();

    if (payloadLen < 0) payloadLen = payload ? strlen(payload) : 0;
    if (payloadLen > 1024) return -1;

    hdr[hlen++] = 0x80 | (opcode & 0x0F);
    if (payloadLen < 126) {
        hdr[hlen++] = 0x80 | (uint8_t)payloadLen;
    } else {
        hdr[hlen++] = 0x80 | 126;
        hdr[hlen++] = (payloadLen >> 8) & 0xFF;
        hdr[hlen++] = payloadLen & 0xFF;
    }

    mask[0] = (seed >> 24) & 0xFF;
    mask[1] = (seed >> 16) & 0xFF;
    mask[2] = (seed >> 8) & 0xFF;
    mask[3] = seed & 0xFF;
    memcpy(&hdr[hlen], mask, 4);
    hlen += 4;

    if (zce_ws_write_all(ctx, hdr, hlen) != hlen) return -1;

    if (payloadLen > 0) {
        uint8_t tmp[256];
        int off = 0;
        while (off < payloadLen) {
            int chunk = payloadLen - off;
            if (chunk > (int)sizeof(tmp)) chunk = sizeof(tmp);
            for (int i = 0; i < chunk; i++) {
                tmp[i] = ((const uint8_t *)payload)[off + i] ^ mask[(off + i) & 3];
            }
            if (zce_ws_write_all(ctx, tmp, chunk) != chunk) return -1;
            off += chunk;
        }
    }
    return 0;
}

static int zce_ws_recv_frame(void *ctx, uint8_t *opcode, char *payload, int payloadMax) {
    uint8_t h[2];
    uint64_t len;
    bool masked;
    uint8_t mask[4] = {0,0,0,0};

    int r = zce_ws_read_exact(ctx, h, 2);
    if (r <= 0) return r;

    *opcode = h[0] & 0x0F;
    masked = (h[1] & 0x80) != 0;
    len = h[1] & 0x7F;

    if (len == 126) {
        uint8_t e[2];
        if (zce_ws_read_exact(ctx, e, 2) <= 0) return -1;
        len = ((uint16_t)e[0] << 8) | e[1];
    } else if (len == 127) {
        uint8_t e[8];
        if (zce_ws_read_exact(ctx, e, 8) <= 0) return -1;
        return -1; // not expected for command messages
    }

    if (masked) {
        if (zce_ws_read_exact(ctx, mask, 4) <= 0) return -1;
    }

    if (len >= (uint64_t)payloadMax) {
        // Drain the oversized frame and drop it.
        uint8_t drop[64];
        uint64_t left = len;
        while (left > 0) {
            int chunk = left > sizeof(drop) ? sizeof(drop) : (int)left;
            if (zce_ws_read_exact(ctx, drop, chunk) <= 0) return -1;
            left -= chunk;
        }
        payload[0] = 0;
        return -2;
    }

    if (len > 0) {
        if (zce_ws_read_exact(ctx, (uint8_t *)payload, (int)len) <= 0) return -1;
        if (masked) {
            for (uint64_t i = 0; i < len; i++) payload[i] ^= mask[i & 3];
        }
    }
    payload[len] = 0;
    return (int)len;
}

static int zce_ws_handshake(void *ctx) {
    char req[512];
    char resp[ZCE_WS_HTTP_MAX];
    int used = 0;

    snprintf(req, sizeof(req),
        "GET %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
        "Sec-WebSocket-Version: 13\r\n"
        "\r\n",
        ZCE_WS_PATH, ZCE_WS_HOST);

    if (zce_ws_write_all(ctx, (const uint8_t *)req, strlen(req)) != (int)strlen(req)) return -1;

    memset(resp, 0, sizeof(resp));
    while (used < (int)sizeof(resp) - 1 && !s_ws_stop) {
        int r = zce_ws_read_some(ctx, (uint8_t *)&resp[used], 1);
        if (r <= 0) return -1;
        used += r;
        resp[used] = 0;
        if (strstr(resp, "\r\n\r\n")) break;
    }

    if (strstr(resp, "101") == NULL || strstr(resp, "Upgrade") == NULL) {
        addLogAdv(LOG_WARN, LOG_FEATURE_MAIN, "ZCE_WS: bad handshake response");
        return -1;
    }
    return 0;
}

// ------------------------------------------------------------
// Command processing
// ------------------------------------------------------------
static bool zce_json_get_relay_value(const char *json, int *outValue) {
    const char *cmd = strstr(json, "\"cmd\"");
    const char *relay = strstr(json, "relay");
    const char *value = strstr(json, "\"value\"");
    const char *colon;

    if (!cmd || !relay || !value) return false;
    colon = strchr(value, ':');
    if (!colon) return false;
    while (*colon && (*colon == ':' || *colon == ' ' || *colon == '\t')) colon++;
    if (*colon == '1') {
        *outValue = 1;
        return true;
    }
    if (*colon == '0') {
        *outValue = 0;
        return true;
    }
    return false;
}

static bool zce_json_is_ping(const char *json) {
    return strstr(json, "\"type\"") && strstr(json, "ping");
}

static void zce_apply_relay_command(int value) {
    char cmd[32];

    // Single relay command path: set the relay channel. The existing TuyaMCU
    // channel mapping sends DP 0x10 to the MCU, so we do not call a second
    // tuyaMcu_sendState command here.
    snprintf(cmd, sizeof(cmd), "setChannel 1 %d", value ? 1 : 0);
    CMD_ExecuteCommand(cmd, 0);

    ZCE_BIN_SetRelayCommand(value ? 1 : 0);
    ZCE_BIN_RequestTelemetryNow();
}

static void zce_send_hello(void *ctx) {
    char msg[128];
    snprintf(msg, sizeof(msg), "{\"type\":\"hello\",\"device_id\":\"%s\"}", ZCE_BIN_GetDeviceId());
    if (zce_ws_send_frame(ctx, 0x1, msg, -1) == 0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_MAIN, "ZCE_WS: hello sent device_id=%s", ZCE_BIN_GetDeviceId());
    }
}

static void zce_send_relay_ack(void *ctx, int value) {
    char msg[128];
    unsigned long ts = (unsigned long)g_secondsElapsed * 1000UL;
    snprintf(msg, sizeof(msg), "{\"type\":\"ack\",\"cmd\":\"relay\",\"value\":%d,\"ts\":%lu}", value ? 1 : 0, ts);
    zce_ws_send_frame(ctx, 0x1, msg, -1);
}

static void zce_ws_command_loop(void *ctx) {
    char payload[ZCE_WS_RX_MAX];
    uint8_t opcode;

    zce_send_hello(ctx);

    while (!s_ws_stop) {
        int len = zce_ws_recv_frame(ctx, &opcode, payload, sizeof(payload));
        if (len == -2) continue;
        if (len <= 0) break;

        if (opcode == 0x8) { // close
            break;
        }
        if (opcode == 0x9) { // WebSocket ping
            zce_ws_send_frame(ctx, 0xA, payload, len);
            continue;
        }
        if (opcode != 0x1) { // text only for backend command protocol
            continue;
        }

        if (zce_json_is_ping(payload)) {
            zce_ws_send_frame(ctx, 0x1, "{\"type\":\"pong\"}", -1);
            continue;
        }

        int relayValue;
        if (zce_json_get_relay_value(payload, &relayValue)) {
            addLogAdv(LOG_INFO, LOG_FEATURE_MAIN, "ZCE_WS: relay command value=%d", relayValue);
            zce_apply_relay_command(relayValue);
            zce_send_relay_ack(ctx, relayValue);
        }
    }
}

static void zce_ws_thread_main(beken_thread_arg_t arg) {
    (void)arg;

    while (!s_ws_stop) {
        if (!Main_HasWiFiConnected()) {
            rtos_delay_milliseconds(1000);
            continue;
        }

#if ZCE_WS_CMD_USE_TLS
        zce_tls_t tls;
        zce_tls_init(&tls);
        addLogAdv(LOG_INFO, LOG_FEATURE_MAIN, "ZCE_WS: connecting wss://%s%s", ZCE_WS_HOST, ZCE_WS_PATH);

        if (zce_tls_connect(&tls, ZCE_WS_HOST, ZCE_WS_PORT) == 0 && zce_ws_handshake(&tls) == 0) {
            s_ws_connected = true;
            addLogAdv(LOG_INFO, LOG_FEATURE_MAIN, "ZCE_WS: connected");
            zce_ws_command_loop(&tls);
        } else {
            addLogAdv(LOG_WARN, LOG_FEATURE_MAIN, "ZCE_WS: connect/handshake failed");
        }

        s_ws_connected = false;
        zce_tls_free(&tls);
#else
        addLogAdv(LOG_WARN, LOG_FEATURE_MAIN, "ZCE_WS: TLS disabled, WSS command channel unavailable");
#endif

        if (!s_ws_stop) rtos_delay_milliseconds(ZCE_WS_RECONNECT_DELAY_MS);
    }

    s_ws_thread = NULL;
    rtos_delete_thread(NULL);
}

void ZCE_WS_CMD_Init(void) {
    if (s_ws_thread != NULL) return;
    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN, "ZCE_WS: init command client");
    s_ws_stop = false;
    s_ws_connected = false;

    OSStatus err = rtos_create_thread(&s_ws_thread,
                                      BEKEN_APPLICATION_PRIORITY,
                                      "zce_ws_cmd",
                                      (beken_thread_function_t)zce_ws_thread_main,
                                      ZCE_WS_STACK_SIZE,
                                      (beken_thread_arg_t)0);
    if (err != kNoErr) {
        s_ws_thread = NULL;
        addLogAdv(LOG_WARN, LOG_FEATURE_MAIN, "ZCE_WS: failed to start thread err=%d", err);
    }
}

void ZCE_WS_CMD_Stop(void) {
    s_ws_stop = true;
    s_ws_connected = false;
}

int ZCE_WS_CMD_IsConnected(void) {
    return s_ws_connected ? 1 : 0;
}

#endif // ENABLE_DRIVER_ZCE_BIN
