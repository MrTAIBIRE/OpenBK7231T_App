#pragma once

// ============================================================
// Driver ZCE-BIN v1 — Wattel_EM / T1-U-HL (BK7238)
//
// Fonctions exposées au système de drivers OBK (drv_main.c) :
//   ZCE_BIN_Init()         — appelé au startDriver ZCE_BIN
//   ZCE_BIN_Every1Second() — appelé chaque seconde par OBK
//   ZCE_BIN_RunQuickTick() — appelé à chaque loop (UART2 prod)
//   ZCE_BIN_Stop()         — appelé au stopDriver ZCE_BIN
// ============================================================

void ZCE_BIN_Init(void);
void ZCE_BIN_Every1Second(void);
void ZCE_BIN_RunQuickTick(void);
void ZCE_BIN_Stop(void);

const char *ZCE_BIN_GetDeviceId(void);

void ZCE_BIN_RequestTelemetryNow(void);

void ZCE_BIN_SetRelayCommand(int value);
