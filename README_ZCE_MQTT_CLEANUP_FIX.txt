Fix compilation error:
undefined reference to `mqtt_client_cleanup`

Root cause:
src/mqtt/new_mqtt.c called mqtt_client_cleanup() when MQTT_CLIENT_CLEANUP is defined, but the lwIP MQTT client used in the BK7238 SDK / patched mqtt.c does not provide this symbol.

Change:
Removed the unsupported mqtt_client_cleanup() call. The reconnect path now only calls mqtt_disconnect(mqtt_client), then reuses the same mqtt_client_t instance for MQTT_do_connect(). This matches the existing lwIP MQTT API available in this SDK and avoids adding a duplicate or fake cleanup function.

Modified file:
src/mqtt/new_mqtt.c
