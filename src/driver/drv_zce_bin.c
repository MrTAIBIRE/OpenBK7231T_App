// ============================================================
// drv_zce_bin.c — Driver ZCE-BIN v1 pour OpenBeken
// Wattel_EM / T1-U-HL (BK7238)
//
// Publie une trame binaire de 41 octets toutes les secondes
// sur le topic MQTT : zce/<device_id>/telemetry
//
// Device ID construit depuis UUID + model stockés via
// la commande OBK "ZCE_SetUUID" / "ZCE_SetModel"
// (interface production sur UART2 / console OBK)
//
// Format trame ZCE-BIN v1 (41 octets) :
//   [0-1]   Magic      0x5A 0xE1
//   [2]     Version    0x01
//   [3]     Flags      relay_state/relay_cmd
//   [4]     Réservé    0x00
//   [5-10]  MAC        6 octets big-endian
//   [11-12] Length     uint16 LE = 28
//   [13-14] Voltage    uint16 LE  V×10
//   [15-17] Current    uint24 LE  A×1000
//   [18-20] Power      uint24 LE  W×10
//   [21-22] PowerFact  uint16 LE  PF×1000
//   [23-24] Frequency  uint16 LE  Hz×10
//   [25-32] Energy     uint64 LE  Wh×100
//   [33-36] Uptime     uint32 LE  secondes
//   [37-39] Réservé    0x00
//   [40]    CRC8/SMBUS polynomial 0x07
//
// Valeurs lues via DRV_GetReading() (drv_public.h) :
//   OBK_VOLTAGE      → V
//   OBK_CURRENT      → A
//   OBK_POWER        → W
//   OBK_POWER_FACTOR → sans unité (0.0-1.0)
//   OBK_FREQUENCY    → Hz
//
// Energie lue depuis channel OBK (TuyaMCU DP 0x0D → CH_ENERGY)
//
// Relay state/cmd lus depuis channels OBK (TuyaMCU DP 0x10/0x0B)
//
// Interface production via commandes OBK (console/MQTT/HTTP) :
//   ZCE_SetUUID <uuid-36-chars>
//   ZCE_SetModel <model>
//   ZCE_GetInfo
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
#include "../driver/drv_public.h"
#include "../cmnds/cmd_public.h"

// ---- Headers OBK ----
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

// ---- Channels TuyaMCU (configurés dans autoexec.bat) ----
// linkTuyaMCUOutputToChannel 0x10 bool 1  → relay_state
// linkTuyaMCUOutputToChannel 0x0B bool 2  → relay_cmd
// linkTuyaMCUOutputToChannel 0x0D val  3  → energy (Wh)
#define CH_RELAY_STATE  1
#define CH_RELAY_CMD    2
#define CH_ENERGY       3


// ---- Buffers globaux ----
static char g_deviceId[PRODUCT_ID_SIZE]   = {0};
static char g_topicTelemetry[96]          = {0};
static char g_uuid[40]                    = {0};
static char g_model[16]                   = {0};
static bool g_initialized                 = false;

// MAC lu depuis wifi_bssid (exporté dans new_common.h)
// Format "AA:BB:CC:DD:EE:FF"
extern char g_wifi_bssid[33];

// Uptime depuis new_common.h
extern int g_secondsElapsed;

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
// Format : ZCE-<model>-<18 premiers chars UUID sans tirets>
// ============================================================
static void buildDeviceId(void) {
    if (g_uuid[0] != '\0' && g_model[0] != '\0') {
        // Supprimer les tirets de l'UUID
        char uuid_nohyph[33] = {0};
        int j = 0;
        for (int i = 0; g_uuid[i] != '\0' && i < 36 && j < 32; i++) {
            if (g_uuid[i] != '-') uuid_nohyph[j++] = g_uuid[i];
        }
        snprintf(g_deviceId, PRODUCT_ID_SIZE,
                 "ZCE-%.4s-%.18s", g_model, uuid_nohyph);
    } else {
        // Fallback : utiliser le nom court OBK
        snprintf(g_deviceId, PRODUCT_ID_SIZE,
                 "ZCE-EM-%s", CFG_GetShortDeviceName());
        addLogAdv(LOG_WARN, LOG_FEATURE_MAIN,
            "ZCE_BIN: UUID/model non definis, fallback=%s", g_deviceId);
    }
    g_deviceId[PRODUCT_ID_SIZE - 1] = '\0';

    // Construire le topic telemetry
    snprintf(g_topicTelemetry, sizeof(g_topicTelemetry),
             "%s/%s/telemetry", MQTT_TOPIC_PREFIX, g_deviceId);

    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN,
        "ZCE_BIN: device_id=%s", g_deviceId);
    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN,
        "ZCE_BIN: topic=%s", g_topicTelemetry);
}

// ============================================================
// Construction de la trame ZCE-BIN v1 (41 octets)
// ============================================================
static int buildBinaryFrame(uint8_t* frame) {
    memset(frame, 0, ZCE_FRAME_SIZE);

    // --- Lecture des mesures via DRV_GetReading() ---
    // Retourne float dans les unités SI
    float voltage    = DRV_GetReading(OBK_VOLTAGE);       // V
    float current    = DRV_GetReading(OBK_CURRENT);       // A
    float power      = DRV_GetReading(OBK_POWER);         // W
    float powerfact  = DRV_GetReading(OBK_POWER_FACTOR);  // 0.0-1.0
    float frequency  = DRV_GetReading(OBK_FREQUENCY);     // Hz

    // Energie depuis channel TuyaMCU (DP 0x0D = Wh)
    int energy_wh    = CHANNEL_Get(CH_ENERGY);

    // Relay state/cmd depuis channels TuyaMCU
    int relay_state  = CHANNEL_Get(CH_RELAY_STATE);
    int relay_cmd    = CHANNEL_Get(CH_RELAY_CMD);

    // --- Flags ---
    uint8_t flags = 0;
    // bit0 = relay_state_known, bit1 = relay_state_value
    flags |= (1 << 0);  // toujours connu si TuyaMCU actif
    if (relay_state) flags |= (1 << 1);
    // bit2 = relay_cmd_known, bit3 = relay_cmd_value
    flags |= (1 << 2);
    if (relay_cmd)   flags |= (1 << 3);

    // --- Lecture MAC ---
    uint8_t mac[6] = {0};
    parseMacStr(g_wifi_bssid, mac);

    // --- Header ---
    frame[0] = ZCE_MAGIC_0;
    frame[1] = ZCE_MAGIC_1;
    frame[2] = ZCE_VERSION;
    frame[3] = flags;
    frame[4] = 0x00;  // réservé

    // MAC big-endian (offsets 5-10)
    memcpy(&frame[5], mac, 6);

    // Length uint16 LE (offsets 11-12)
    frame[11] = ZCE_DATA_LEN & 0xFF;
    frame[12] = (ZCE_DATA_LEN >> 8) & 0xFF;

    // Voltage uint16 LE V×10 (offsets 13-14)
    uint16_t v_raw = (uint16_t)(voltage * 10.0f);
    frame[13] = v_raw & 0xFF;
    frame[14] = (v_raw >> 8) & 0xFF;

    // Current uint24 LE A×1000 (offsets 15-17)
    uint32_t i_raw = (uint32_t)(current * 1000.0f);
    frame[15] = i_raw & 0xFF;
    frame[16] = (i_raw >> 8) & 0xFF;
    frame[17] = (i_raw >> 16) & 0xFF;

    // Power uint24 LE W×10 (offsets 18-20)
    uint32_t p_raw = (uint32_t)(power * 10.0f);
    frame[18] = p_raw & 0xFF;
    frame[19] = (p_raw >> 8) & 0xFF;
    frame[20] = (p_raw >> 16) & 0xFF;

    // PowerFactor uint16 LE PF×1000 (offsets 21-22)
    uint16_t pf_raw = (uint16_t)(powerfact * 1000.0f);
    frame[21] = pf_raw & 0xFF;
    frame[22] = (pf_raw >> 8) & 0xFF;

    // Frequency uint16 LE Hz×10 (offsets 23-24)
    uint16_t f_raw = (uint16_t)(frequency * 10.0f);
    frame[23] = f_raw & 0xFF;
    frame[24] = (f_raw >> 8) & 0xFF;

    // Energy uint64 LE Wh×100 (offsets 25-32)
    uint64_t e_raw = (uint64_t)energy_wh * 100ULL;
    frame[25] = (uint8_t)(e_raw      );
    frame[26] = (uint8_t)(e_raw >>  8);
    frame[27] = (uint8_t)(e_raw >> 16);
    frame[28] = (uint8_t)(e_raw >> 24);
    frame[29] = (uint8_t)(e_raw >> 32);
    frame[30] = (uint8_t)(e_raw >> 40);
    frame[31] = (uint8_t)(e_raw >> 48);
    frame[32] = (uint8_t)(e_raw >> 56);

    // Uptime uint32 LE secondes (offsets 33-36)
    uint32_t uptime = (uint32_t)g_secondsElapsed;
    frame[33] = (uint8_t)(uptime      );
    frame[34] = (uint8_t)(uptime >>  8);
    frame[35] = (uint8_t)(uptime >> 16);
    frame[36] = (uint8_t)(uptime >> 24);

    // Réservé (offsets 37-39)
    frame[37] = 0x00;
    frame[38] = 0x00;
    frame[39] = 0x00;

    // CRC8 sur bytes[0..39]
    frame[40] = crc8_calc(frame, 40);

    return ZCE_FRAME_SIZE;
}

// ============================================================
// Publication MQTT binaire via lwIP mqtt_publish directement
// mqtt_client est exporté dans new_mqtt.h
// ============================================================
static void publishBinaryFrame(const uint8_t* frame, int len) {
#if ENABLE_MQTT
    if (!mqtt_client) return;
    if (!Main_HasMQTTConnected()) return;

    // lwIP mqtt_publish(client, topic, payload, payload_len,
    //                   qos, retain, callback, callback_arg)
    err_t err = mqtt_publish(
        mqtt_client,
        g_topicTelemetry,
        (const void*)frame,
        (u16_t)len,
        0,    // QoS 0
        0,    // retain = 0
        NULL, // no callback
        NULL  // no callback arg
    );

    if (err != ERR_OK) {
        addLogAdv(LOG_WARN, LOG_FEATURE_MAIN,
            "ZCE_BIN: mqtt_publish failed err=%d", (int)err);
    }
#endif
}

// ============================================================
// Commandes OBK enregistrées au startDriver
// ============================================================

// ZCE_SetUUID <36-char-uuid>
static commandResult_t ZCE_Cmd_SetUUID(const void* ctx,
    const char* cmd, const char* args, int flags)
{
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }
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

// ZCE_SetModel <model>  ex: EM
static commandResult_t ZCE_Cmd_SetModel(const void* ctx,
    const char* cmd, const char* args, int flags)
{
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }
    const char* model = Tokenizer_GetArg(0);
    strncpy(g_model, model, sizeof(g_model) - 1);
    g_model[sizeof(g_model) - 1] = '\0';
    buildDeviceId();
    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN,
        "ZCE_BIN: model defini = %s", g_model);
    return CMD_RES_OK;
}

// ZCE_GetInfo — affiche dans les logs OBK (visible en console/MQTT)
static commandResult_t ZCE_Cmd_GetInfo(const void* ctx,
    const char* cmd, const char* args, int flags)
{
    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN,
        "ZCE_BIN: uuid=%s model=%s device_id=%s topic=%s",
        g_uuid[0]   ? g_uuid   : "(vide)",
        g_model[0]  ? g_model  : "(vide)",
        g_deviceId,
        g_topicTelemetry);
    return CMD_RES_OK;
}

// ============================================================
// ZCE_BIN_Init — appelé par OBK au startDriver ZCE_BIN
// ============================================================
void ZCE_BIN_Init(void) {
    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN, "ZCE_BIN: init");

    // Construire device_id (avec les valeurs déjà en RAM si présentes)
    buildDeviceId();

    // Enregistrer les commandes OBK
    // → utilisables depuis console web, MQTT cmnd/, HTTP, script
    CMD_RegisterCommand("ZCE_SetUUID",  ZCE_Cmd_SetUUID,  NULL);
    CMD_RegisterCommand("ZCE_SetModel", ZCE_Cmd_SetModel, NULL);
    CMD_RegisterCommand("ZCE_GetInfo",  ZCE_Cmd_GetInfo,  NULL);

    g_initialized = true;

    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN,
        "ZCE_BIN: init OK — device_id=%s", g_deviceId);
    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN,
        "ZCE_BIN: cmds dispo: ZCE_SetUUID, ZCE_SetModel, ZCE_GetInfo");
}

// ============================================================
// ZCE_BIN_Every1Second — appelé chaque seconde par OBK
// ============================================================
void ZCE_BIN_Every1Second(void) {
#if ENABLE_MQTT
    if (!g_initialized) return;
    if (!Main_HasMQTTConnected()) return;

    uint8_t frame[ZCE_FRAME_SIZE];
    buildBinaryFrame(frame);
    publishBinaryFrame(frame, ZCE_FRAME_SIZE);
#endif
}

// ============================================================
// ZCE_BIN_RunQuickTick — appelé à chaque loop OBK
// (rien à faire ici : les commandes passent par CMD_RegisterCommand)
// ============================================================
void ZCE_BIN_RunQuickTick(void) {
    // Vide : la gestion production passe par les commandes OBK
    // enregistrées dans ZCE_BIN_Init()
}

// ============================================================
// ZCE_BIN_Stop — appelé au stopDriver ZCE_BIN
// ============================================================
void ZCE_BIN_Stop(void) {
    g_initialized = false;
    addLogAdv(LOG_INFO, LOG_FEATURE_MAIN, "ZCE_BIN: stopped");
}

#endif // ENABLE_DRIVER_ZCE_BIN
