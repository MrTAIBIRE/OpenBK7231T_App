#pragma once

// ZCE EM backend command WebSocket client.
// This is the single inbound cloud command path for relay commands.
// MQTT remains outbound only for telemetry/status/availability.

void ZCE_WS_CMD_Init(void);
void ZCE_WS_CMD_Stop(void);
int  ZCE_WS_CMD_IsConnected(void);
