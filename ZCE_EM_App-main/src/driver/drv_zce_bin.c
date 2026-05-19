// ============================================================
// drv_zce_bin.c — Driver ZCE-BIN v1 pour ZCE EM
// Wattel_EM / T1-U-HL (BK7238)
//
// Startup command requis :
// backlog startDriver TuyaMCU; tuyaMcu_defWiFiState 4;
// linkTuyaMCUOutputToChannel 0x10 bool 1; setChannelType 1 toggle;
// linkTuyaMCUOutputToChannel 0x06 RAW_TAC2121C_VCP;
// setChannelType 2 Voltage_div10;
// setChannelType 3 Power;
// setChannelType 4 Current_div1000;
// linkTuyaMCUOutputToChannel 0x0D val 5; setChannelType 5 EnergyTotal_kWh_div100;
// linkTuyaMCUOutputToChannel 0x68 val 6;
// linkTuyaMCUOutputToChannel 0x69 val 7;
// linkTuyaMCUOutputToChannel 0x0B bool 8;
// startDriver ZCE_BIN
//
// Mapping channels :
//   CH 1 → relay_state  (DP 0x10 bool)
//   CH 2 → tension      (DP 0x06 RAW V×10)
//   CH 3 → puissance    (DP 0x06 RAW W)
//   CH 4 → courant      (DP 0x06 RAW A×1000)
//   CH 5 → energie      (DP 0x0D kWh×100)
//   CH 6 → power factor (DP 0x68 PF×1000)
//   CH 7 → frequence    (DP 0x69 Hz×10)
//   CH 8 → relay_cmd    (DP 0x0B bool)
// ============================================================

#include "drv_zce_bin.h"
#include "../logging/logging.h"
#include "../obk_config.h"

#ifdef ENABLE_DRIVER_ZCE_BIN

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "../new_common.h"
#include "../mqtt/new_mqtt.h"
#include "../cmnds/cmd_public.h"
#include "../new_cfg.h"

// ---- API channels OBK ----
extern int CHANNEL_Get(int index);

// ---- Constantes ZCE-BIN v1 ----
#define ZCE_MAGIC_0     0x5A
#define ZCE_MAGIC_1     0xE1
#define ZCE_VERSION     0x01
#define ZCE_FRAME_SIZE  41
#define ZCE_DATA_LEN    28

// ---- Topics MQTT ----
#define MQTT_TOPIC_PREFIX   "zce"
#define PRODUCT_ID_SIZE     26

// ---- Mapping channels TuyaMCU ----
#define CH_RELAY_STATE  1   // DP 0x10 bool
#define CH_VOLTAGE      2   // DP 0x06 RAW V×10
#define CH_POWER        3   // DP 0x06 RAW W (direct)
#define CH_CURRENT      4   // DP 0x06 RAW A×1000
#define CH_ENERGY       5   // DP 0x0D kWh×100
#define CH_POWERFACT    6   // DP 0x68 PF×1000
#define CH_FREQ         7   // DP 0x69 Hz×10
#define CH_RELAY_CMD    8   // DP 0x0B bool

// ---- Buffers globaux ----
static char g_deviceId[PRODUCT_ID_SIZE] = {0};
static char g_topicTelemetry[96]        = {0};
static char g_uuid[40]                  = {0};
static char g_model[16]                 = {0};
static bool g_initialized               = false;
static int  g_zce_seconds               = 0;

// The original Tuya Wi-Fi firmware for JIUJI JJMVPD-63LWS sends this command
// periodically to the MCU:
//   55 AA 00 06 00 05 6A 01 00 01 01 77
// It is DP 0x6A / 106, type bool, value true.
// Without it, some MCU firmwares only report one snapshot of DP 0x06 and then
// the voltage/current/power channels can remain frozen.
#define ZCE_TUYA_DP_MEASURE_ENABLE 106
#define ZCE_TUYA_MEASURE_ENABLE_PERIOD_S 2
#define ZCE_TUYA_QUERY_PERIOD_S 5


extern char g_wifi_bssid[33];
extern int  g_secondsElapsed;

// ============================================================
// CRC8 / SMBUS polynomial 0x07
// ============================================================
static void crc8_process(uint8_t in_data, uint8_t* crc) {
    uint8_t i = 0;
    do {
        uint8_t tmp = in_data ^ (*crc);
        *crc <<= 1;
        if (tmp & 0x80) *crc ^= 0x07;
        in_data <<= 1;
    } while (8 > ++i);
}

static uint8_t crc8_calc(const uint8_t* buf, int len) {
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc8_process(buf[i], &crc);
    }
    return crc;
}

// ============================================================
// Parse MAC depuis "AA:BB:CC:DD:EE:FF"
// ============================================================
static void parseMacStr(const char* macStr, uint8_t* mac) {
    unsigned int v[6] = {0, 0, 0, 0, 0, 0};
    if (macStr && strlen(macStr) >= 17) {
        sscanf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]);
    }
    for (int i = 0; i < 6; i++) {
        mac[i] = (uint8_t)v[i];
    }
}

// ============================================================
// Construction du device_id
// ============================================================
static void buildDeviceId(void) {
    if (g_uuid[0] != '\0' && g_model[0] != '\0') {
        char uuid_nohyph[33] = {0};
        int j = 0;
        for (int i = 0; g_uuid[i] != '\0' && i < 36 && j < 32; i++) {
            if (g_uuid[i] != '-') uuid_nohyph[j++] = g_uuid[i];
        }
        snprintf(g_deviceId, PRODUCT_ID_SIZE,
                 "ZCE-%.4s-%.18s", g_model, uuid_nohyph);
    } else {
        snprintf(g_deviceId, PRODUCT_ID_SIZE,
                 "ZCE-EM-%s", CFG_GetShortDeviceName());
        addLogAdv(LOG_WARN, LOG_FEATURE_MAIN,
            "ZCE_BIN: UUID/model non definis, fallback=%s", g_deviceId);
    }
    g_deviceId[PRODUCT_ID_SIZE - 1] = '\0';

    snprintf(g_topicTelemetry, sizeof(g_topicTelemetry),
             "%s/%s/telemetry", MQTT_TOPIC_PREFIX, g_deviceId);

    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN,
        "ZCE_BIN: device_id=%s topic=%s", g_deviceId, g_topicTelemetry);
}

// ============================================================
// Initialisation / maintien de la session TuyaMCU JJMVPD-63LWS
// ============================================================
static void zceKickTuyaMCU(bool fullInit) {
    // Commands are routed through OBK command layer because TuyaMCU APIs are
    // owned by drv_tuyaMCU.c. This keeps ZCE_BIN independent from TuyaMCU internals.
    if (fullInit) {
        CMD_ExecuteCommand("tuyaMcu_sendProductInformation", 0);
        CMD_ExecuteCommand("tuyaMcu_sendMCUConf", 0);
        CMD_ExecuteCommand("tuyaMcu_sendQueryState", 0);
    }

    // Reproduce the original Tuya module behaviour observed on UART:
    // DP 0x6A = true enables/refreshes metering reports.
    CMD_ExecuteCommand("tuyaMcu_sendState 106 bool 1", 0);
}

// ============================================================
// Construction de la trame ZCE-BIN v1 (41 octets)
// ============================================================
static int buildBinaryFrame(uint8_t* frame) {
    memset(frame, 0, ZCE_FRAME_SIZE);

    // Lecture des channels TuyaMCU
    int relay_state = CHANNEL_Get(CH_RELAY_STATE);
    int v_raw       = CHANNEL_Get(CH_VOLTAGE);     // V×10
    int p_w         = CHANNEL_Get(CH_POWER);        // W direct
    int i_raw       = CHANNEL_Get(CH_CURRENT);      // A×1000
    int energy_kwh100 = CHANNEL_Get(CH_ENERGY);     // kWh×100
    int pf_raw      = CHANNEL_Get(CH_POWERFACT);    // PF×1000
    int f_raw       = CHANNEL_Get(CH_FREQ);         // Hz×10
    int relay_cmd   = CHANNEL_Get(CH_RELAY_CMD);

    // Conversion puissance : W → W×10 pour la trame
    int p_raw = p_w * 10;

    // Conversion energie : kWh×100 → Wh×100
    // kWh×100 → Wh = ×1000 → Wh×100 = ×100000
    uint64_t e_wh100 = (uint64_t)energy_kwh100 * 1000ULL;

    // Flags relay
    uint8_t flags = 0;
    flags |= (1 << 0);
    if (relay_state) flags |= (1 << 1);
    flags |= (1 << 2);
    if (relay_cmd)   flags |= (1 << 3);

    // MAC
    uint8_t mac[6] = {0};
    parseMacStr(g_wifi_bssid, mac);

    // Header
    frame[0] = ZCE_MAGIC_0;
    frame[1] = ZCE_MAGIC_1;
    frame[2] = ZCE_VERSION;
    frame[3] = flags;
    frame[4] = 0x00;

    // MAC (offsets 5-10)
    memcpy(&frame[5], mac, 6);

    // Length uint16 LE (offsets 11-12)
    frame[11] = ZCE_DATA_LEN & 0xFF;
    frame[12] = (ZCE_DATA_LEN >> 8) & 0xFF;

    // Voltage uint16 LE V×10 (offsets 13-14)
    frame[13] = (uint8_t)(v_raw & 0xFF);
    frame[14] = (uint8_t)((v_raw >> 8) & 0xFF);

    // Current uint24 LE A×1000 (offsets 15-17)
    frame[15] = (uint8_t)(i_raw & 0xFF);
    frame[16] = (uint8_t)((i_raw >> 8) & 0xFF);
    frame[17] = (uint8_t)((i_raw >> 16) & 0xFF);

    // Power uint24 LE W×10 (offsets 18-20)
    frame[18] = (uint8_t)(p_raw & 0xFF);
    frame[19] = (uint8_t)((p_raw >> 8) & 0xFF);
    frame[20] = (uint8_t)((p_raw >> 16) & 0xFF);

    // PowerFactor uint16 LE PF×1000 (offsets 21-22)
    frame[21] = (uint8_t)(pf_raw & 0xFF);
    frame[22] = (uint8_t)((pf_raw >> 8) & 0xFF);

    // Frequency uint16 LE Hz×10 (offsets 23-24)
    frame[23] = (uint8_t)(f_raw & 0xFF);
    frame[24] = (uint8_t)((f_raw >> 8) & 0xFF);

    // Energy uint64 LE Wh×100 (offsets 25-32)
    frame[25] = (uint8_t)(e_wh100      );
    frame[26] = (uint8_t)(e_wh100 >>  8);
    frame[27] = (uint8_t)(e_wh100 >> 16);
    frame[28] = (uint8_t)(e_wh100 >> 24);
    frame[29] = (uint8_t)(e_wh100 >> 32);
    frame[30] = (uint8_t)(e_wh100 >> 40);
    frame[31] = (uint8_t)(e_wh100 >> 48);
    frame[32] = (uint8_t)(e_wh100 >> 56);

    // Uptime uint32 LE secondes (offsets 33-36)
    uint32_t uptime = (uint32_t)g_secondsElapsed;
    frame[33] = (uint8_t)(uptime      );
    frame[34] = (uint8_t)(uptime >>  8);
    frame[35] = (uint8_t)(uptime >> 16);
    frame[36] = (uint8_t)(uptime >> 24);

    // Reserved (37-39)
    frame[37] = 0x00;
    frame[38] = 0x00;
    frame[39] = 0x00;

    // CRC8
    frame[40] = crc8_calc(frame, 40);

    addLogAdv(LOG_DEBUG, LOG_FEATURE_MAIN,
        "ZCE_BIN: V=%d I=%d P=%d PF=%d F=%d E=%d",
        v_raw, i_raw, p_w, pf_raw, f_raw, energy_kwh100);

    return ZCE_FRAME_SIZE;
}

// ============================================================
// Publication MQTT binaire
// ============================================================
static void publishBinaryFrame(const uint8_t* frame, int len) {
#if ENABLE_MQTT
    if (!mqtt_client) return;
    if (!Main_HasMQTTConnected()) return;

    err_t err = mqtt_publish(
        mqtt_client,
        g_topicTelemetry,
        (const void*)frame,
        (u16_t)len,
        0, 0, NULL, NULL
    );

    if (err != ERR_OK) {
        addLogAdv(LOG_WARN, LOG_FEATURE_MAIN,
            "ZCE_BIN: mqtt_publish failed err=%d", (int)err);
    }
#endif
}

// ============================================================
// Commandes OBK
// ============================================================
static commandResult_t ZCE_Cmd_SetUUID(const void* ctx,
    const char* cmd, const char* args, int flags)
{
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1))
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    const char* uuid = Tokenizer_GetArg(0);
    if (strlen(uuid) != 36) {
        addLogAdv(LOG_WARN, LOG_FEATURE_MAIN,
            "ZCE_BIN: UUID doit faire 36 chars avec tirets");
        return CMD_RES_BAD_ARGUMENT;
    }
    strncpy(g_uuid, uuid, sizeof(g_uuid) - 1);
    g_uuid[sizeof(g_uuid) - 1] = '\0';
    buildDeviceId();
    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN,
        "ZCE_BIN: UUID defini = %s", g_uuid);
    return CMD_RES_OK;
}

static commandResult_t ZCE_Cmd_SetModel(const void* ctx,
    const char* cmd, const char* args, int flags)
{
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1))
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    strncpy(g_model, Tokenizer_GetArg(0), sizeof(g_model) - 1);
    g_model[sizeof(g_model) - 1] = '\0';
    buildDeviceId();
    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN,
        "ZCE_BIN: model defini = %s", g_model);
    return CMD_RES_OK;
}

static commandResult_t ZCE_Cmd_GetInfo(const void* ctx,
    const char* cmd, const char* args, int flags)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN,
        "ZCE_BIN: uuid=%s model=%s device_id=%s topic=%s",
        g_uuid[0]  ? g_uuid  : "(vide)",
        g_model[0] ? g_model : "(vide)",
        g_deviceId, g_topicTelemetry);
    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN,
        "ZCE_BIN: V=%d I=%d P=%d PF=%d F=%d",
        CHANNEL_Get(CH_VOLTAGE),
        CHANNEL_Get(CH_CURRENT),
        CHANNEL_Get(CH_POWER),
        CHANNEL_Get(CH_POWERFACT),
        CHANNEL_Get(CH_FREQ));
    return CMD_RES_OK;
}

// ============================================================
// ZCE_BIN_Init
// ============================================================
void ZCE_BIN_Init(void) {
    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN, "ZCE_BIN: init");
    buildDeviceId();
    CMD_RegisterCommand("ZCE_SetUUID",  ZCE_Cmd_SetUUID,  NULL);
    CMD_RegisterCommand("ZCE_SetModel", ZCE_Cmd_SetModel, NULL);
    CMD_RegisterCommand("ZCE_GetInfo",  ZCE_Cmd_GetInfo,  NULL);
    g_zce_seconds = 0;
    g_initialized = true;

    // Start the same kind of TuyaMCU session as the original Tuya firmware.
    // autoexec.bat must start TuyaMCU before ZCE_BIN.
    zceKickTuyaMCU(true);

    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN,
        "ZCE_BIN: init OK device_id=%s", g_deviceId);
}

// ============================================================
// ZCE_BIN_Every1Second
// ============================================================
void ZCE_BIN_Every1Second(void) {
    if (!g_initialized) return;

    g_zce_seconds++;

    // Keep MCU metering alive even when MQTT is temporarily disconnected.
    // This is important: otherwise CHANNEL_Get can keep publishing an old DP 0x06 snapshot.
    if ((g_zce_seconds % ZCE_TUYA_MEASURE_ENABLE_PERIOD_S) == 0) {
        CMD_ExecuteCommand("tuyaMcu_sendState 106 bool 1", 0);
    }

    // Ask for a full DP refresh from time to time.
    if ((g_zce_seconds % ZCE_TUYA_QUERY_PERIOD_S) == 0) {
        CMD_ExecuteCommand("tuyaMcu_sendQueryState", 0);
    }

#if ENABLE_MQTT
    if (!Main_HasMQTTConnected()) return;

    uint8_t frame[ZCE_FRAME_SIZE];
    buildBinaryFrame(frame);
    publishBinaryFrame(frame, ZCE_FRAME_SIZE);
#endif
}

// ============================================================
// ZCE_BIN_RunQuickTick
// ============================================================
void ZCE_BIN_RunQuickTick(void) {
}

// ============================================================
// ZCE_BIN_Stop
// ============================================================
void ZCE_BIN_Stop(void) {
    g_initialized = false;
    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN, "ZCE_BIN: stopped");
}

#endif // ENABLE_DRIVER_ZCE_BIN


