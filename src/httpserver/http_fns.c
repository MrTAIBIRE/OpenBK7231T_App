#include "../new_common.h"
#include "http_fns.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../hal/hal_ota.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../cmnds/cmd_enums.h"
#include "../driver/drv_tuyaMCU.h"
#include "../driver/drv_girierMCU.h"
#include "../driver/drv_public.h"
#include "../driver/drv_bl_shared.h"
#include "../hal/hal_wifi.h"
#include "../hal/hal_pins.h"
#include "../hal/hal_flashConfig.h"
#include "../logging/logging.h"
#include "../devicegroups/deviceGroups_public.h"
#include "../mqtt/new_mqtt.h"
#include "hass.h"
#include "../cJSON/cJSON.h"
#include <time.h>
#include "../driver/drv_ntp.h"
#include "../driver/drv_deviceclock.h"		// to set clock via Javascript in pmntp
#include "../driver/drv_local.h"
#ifdef PLATFORM_BEKEN
#include "start_type_pub.h"
#endif

#ifdef WINDOWS
// nothing
#elif PLATFORM_BL602 && !PLATFORM_BL_NEW
#include <bl_sys.h>
#include <bl_adc.h>     //  For BL602 ADC HAL
#include <bl602_adc.h>  //  For BL602 ADC Standard Driver
#include <bl602_glb.h>  //  For BL602 Global Register Standard Driver
#include <wifi_mgmr_ext.h> //For BL602 WiFi AP Scan
#elif PLATFORM_W600 || PLATFORM_W800

#elif PLATFORM_XRADIO
#include <image/flash.h>
#include <ota/ota.h>
#elif defined(PLATFORM_BK7231N)
// tuya-iotos-embeded-sdk-wifi-ble-bk7231n/sdk/include/tuya_hal_storage.h
#include "tuya_hal_storage.h"
#include "BkDriverFlash.h"
#include "temp_detect_pub.h"
#elif defined(PLATFORM_LN882H)
#elif defined(PLATFORM_TR6260)
#elif defined(PLATFORM_REALTEK) && !PLATFORM_REALTEK_NEW
	#include "wifi_structures.h"
	#include "wifi_constants.h"
	#include "wifi_conf.h"
	extern uint32_t current_fw_idx;
	#ifdef PLATFORM_RTL87X0C
	#include "hal_sys_ctrl.h"
	extern hal_reset_reason_t reset_reason;
	#endif
	SemaphoreHandle_t scan_hdl;
#elif PLATFORM_REALTEK_NEW
#include "lwip_netconf.h"
#include "ameba_soc.h"
#include "ameba_ota.h"
//SemaphoreHandle_t scan_hdl;
extern uint32_t current_fw_idx;
#elif defined(PLATFORM_ESPIDF) || PLATFORM_ESP8266
#include "esp_wifi.h"
#include "esp_system.h"
#elif defined(PLATFORM_BK7231T)
// REALLY? A typo in Tuya SDK? Storge?
// tuya-iotos-embeded-sdk-wifi-ble-bk7231t/platforms/bk7231t/tuya_os_adapter/include/driver/tuya_hal_storge.h
#include "tuya_hal_storge.h"
#include "BkDriverFlash.h"
#include "temp_detect_pub.h"
#elif defined(PLATFORM_ECR6600)
#include "hal_system.h"
#endif

#if (defined(PLATFORM_BK7231T) || defined(PLATFORM_BK7231N)) && !defined(PLATFORM_BEKEN_NEW)
int tuya_os_adapt_wifi_all_ap_scan(AP_IF_S** ap_ary, unsigned int* num);
int tuya_os_adapt_wifi_release_ap(AP_IF_S* ap);
#endif

static const char SUBMIT_AND_END_FORM[] = "<br><input type=\"submit\" value=\"Submit\"></form>";


const char* g_typesOffLowMidHigh[] = { "Off","Low","Mid","High" };
const char* g_typesOffLowMidHighHighest[] = { "Off", "Low","Mid","High","Highest" };
const char* g_typesOffLowestLowMidHighHighest[] = { "Off", "Lowest", "Low", "Mid", "High", "Highest" };
const char* g_typesLowMidHighHighest[] = { "Low","Mid","High","Highest" };
const char* g_typesOffOnRemember[] = { "Off", "On", "Remember" };
const char* g_typeLowMidHigh[] = { "Low","Mid","High" };
const char* g_typesLowestLowMidHighHighest[] = { "Lowest", "Low", "Mid", "High", "Highest" };;
const char* g_typeOpenStopClose[] = { "Open","Stop","Close" };
const char* g_typeStopUpDown[] = { "Stop","Up","Down" };

#define ADD_OPTION(t,a) if(type == t) { *numTypes = sizeof(a)/sizeof(a[0]); return a; }

const char **Channel_GetOptionsForChannelType(int type, int *numTypes) {
	ADD_OPTION(ChType_OffLowMidHigh, g_typesOffLowMidHigh);
	ADD_OPTION(ChType_OffLowestLowMidHighHighest, g_typesOffLowestLowMidHighHighest);
	ADD_OPTION(ChType_LowestLowMidHighHighest, g_typesLowestLowMidHighHighest);
	ADD_OPTION(ChType_OffLowMidHighHighest, g_typesOffLowMidHighHighest);
	ADD_OPTION(ChType_LowMidHighHighest, g_typesLowMidHighHighest);
	ADD_OPTION(ChType_OffOnRemember, g_typesOffOnRemember);
	ADD_OPTION(ChType_LowMidHigh, g_typeLowMidHigh);
	ADD_OPTION(ChType_OpenStopClose, g_typeOpenStopClose);
	ADD_OPTION(ChType_StopUpDown, g_typeStopUpDown);
	
	*numTypes = 0;
	return 0;
}

unsigned char hexdigit(char hex) {
	return (hex <= '9') ? hex - '0' :
		toupper((unsigned char)hex) - 'A' + 10;
}

unsigned char hexbyte(const char* hex) {
	return (hexdigit(*hex) << 4) | hexdigit(*(hex + 1));
}

int http_fn_empty_url(http_request_t* request) {
	poststr(request, "HTTP/1.1 302 OK\nLocation: /index\nConnection: close\n\n");
	poststr(request, NULL);
	return 0;
}

void postFormAction(http_request_t* request, char* action, char* value) {
	//"<form action=\"cfg_pins\"><input type=\"submit\" value=\"Configure Module\"/></form>"
	hprintf255(request, "<form action=\"%s\"><input type=\"submit\" value=\"%s\"/></form>", action, value);
}

void poststr_h2(http_request_t* request, const char* content) {
	hprintf255(request, "<h2>%s</h2>", content);
}
void poststr_h4(http_request_t* request, const char* content) {
	hprintf255(request, "<h4>%s</h4>", content);
}

/// @brief Generate a pair of label and field elements for Name type entry. The field is limited to entry of a-zA-Z0-9_- characters.
/// @param request 
/// @param label 
/// @param fieldId This also gets used as the field name
/// @param value 
/// @param preContent
void add_label_name_field(http_request_t* request, char* label, char* fieldId, const char* value, char* preContent) {
	if (strlen(preContent) > 0) {
		poststr(request, preContent);
	}

	hprintf255(request, "<label for=\"%s\">%s:</label><br>", fieldId, label);
	hprintf255(request, "<input type=\"text\" id=\"%s\" name=\"%s\" value=\"%s\" ", fieldId, fieldId, value);
	poststr(request, "pattern=\"^[a-zA-Z0-9_-]+$\" title=\"Only alphanumerics, underscore and hyphen characters allowed.\">");
}

/// @brief Generate a pair of label and field elements.
/// @param request 
/// @param label 
/// @param fieldId This also gets used as the field name
/// @param value 
/// @param preContent 
void add_label_input(http_request_t* request, char* inputType, char* label, char* fieldId, const char* value, char* preContent) {
	if (strlen(preContent) > 0) {
		poststr(request, preContent);
	}

	hprintf255(request, "<label for=\"%s\">%s:</label><br>", fieldId, label);
	hprintf255(request, "<input type=\"%s\" id=\"%s\" name=\"%s\" value=\"", inputType, fieldId, fieldId);
	poststr(request, value);
	hprintf255(request, "\">");
}

/// @brief Generates a pair of label and text field elements.
/// @param request 
/// @param label Label for the field
/// @param fieldId Field id, this also gets used as the name
/// @param value String value
/// @param preContent Content before the label
void add_label_text_field(http_request_t* request, char* label, char* fieldId, const char* value, char* preContent) {
	add_label_input(request, "text", label, fieldId, value, preContent);
}

/// @brief Generates a pair of label and text field elements.
/// @param request 
/// @param label Label for the field
/// @param fieldId Field id, this also gets used as the name
/// @param value String value
/// @param preContent Content before the label
void add_label_password_field(http_request_t* request, char* label, char* fieldId, const char* value, char* preContent) {
	add_label_input(request, "password", label, fieldId, value, preContent);
}

/// @brief Generate a pair of label and numeric field elements.
/// @param request 
/// @param label Label for the field
/// @param fieldId Field id, this also gets used as the name
/// @param value Integer value
/// @param preContent Content before the label
void add_label_numeric_field(http_request_t* request, char* label, char* fieldId, int value, char* preContent) {
	char strValue[32];
	sprintf(strValue, "%i", value);
	add_label_input(request, "number", label, fieldId, strValue, preContent);
}

int http_fn_testmsg(http_request_t* request) {
	poststr(request, "This is just a test msg\n\n");
	poststr(request, NULL);
	return 0;

}

#if ENABLE_TIME_PMNTP
// poor mans NTP
int http_fn_pmntp(http_request_t* request) {
	char tmpA[128];
	uint32_t actepoch=0;
	// javascripts "getTime()" should return time since 01.01.1970 (UTC)
	if (http_getArg(request->url, "EPOCH", tmpA, sizeof(tmpA))) {
		actepoch = (uint32_t)strtoul(tmpA,0,10);
		TIME_setDeviceTime(actepoch);
		addLogAdv(LOG_DEBUG, LOG_FEATURE_HTTP,"Set clock to %u!",actepoch);	
	}
#if ENABLE_TIME_DST
	if (! IsDST_initialized()) {
#endif
		if (http_getArg(request->url, "OFFSET", tmpA, sizeof(tmpA)) && actepoch != 0 ) {
		// if actual time is during DST period, javascript will return 
		// an offset including the one additional hour of DST  
		// if we don't handle DST, simply accept this as "offset"
		TIME_setDeviceTimeOffset(atoi(tmpA));
		addLogAdv(LOG_DEBUG, LOG_FEATURE_HTTP,"Clock - set g_UTCoffset to %i!",
			atoi(tmpA));	
		}
#if ENABLE_TIME_DST
	// ignore JS offset, if we can/will calculate DST on our own
	} else setDST();
#endif
	poststr(request, "HTTP/1.1 302 OK\nLocation: /index\nConnection: close\n\n");
	poststr(request, NULL);
	return 0;
}
#endif

// bit mask telling which channels are hidden from HTTP
// If given bit is set, then given channel is hidden
extern int g_hiddenChannels;

int http_fn_index(http_request_t* request) {
	char tmpA[128];

	http_setup(request, httpMimeTypeHTML);

	if (http_getArg(request->url, "restart", tmpA, sizeof(tmpA))) {
		poststr(request, "<h3>ZCE EM will restart...</h3>");
		RESET_ScheduleModuleReset(1);
	}

	http_html_start(request, "Home");
	poststr(request, "<h2>ZCE EM setup</h2>");
	poststr(request, "<p>This page is limited to Wi-Fi configuration and OTA firmware update.</p>");
	poststr(request, "<fieldset><legend>Status</legend>");
	hprintf255(request, "Device name: <b>%s</b><br>", CFG_GetDeviceName());
	hprintf255(request, "Configured SSID: <b>%s</b><br>", CFG_GetWiFiSSID());
	hprintf255(request, "Uptime: <span id=\"onlineFor\" data-initial=\"%i\">-</span><br>", g_secondsElapsed);
	poststr(request, "</fieldset>");
	postFormAction(request, "cfg_wifi", "Configure Wi-Fi");
	postFormAction(request, "ota", "OTA firmware update");
	poststr(request, "<form action=\"index\"><input type=\"hidden\" name=\"restart\" value=\"1\"><input class=\"bred\" type=\"submit\" value=\"Restart device\" onclick=\"return confirm('Restart ZCE EM?')\"></form>");
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}


int http_fn_about(http_request_t* request) {
	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "About");
	poststr_h2(request, "ZCE EM firmware");
	poststr(request, htmlFooterReturnToMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}
#if ENABLE_HTTP_MQTT

int http_fn_cfg_mqtt_set(http_request_t* request) {
	char tmpA[128];
	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Saving MQTT");

	if (http_getArg(request->url, "host", tmpA, sizeof(tmpA))) {
	}
	// FIX: always set, so people can clear field
	CFG_SetMQTTHost(tmpA);
	if (http_getArg(request->url, "port", tmpA, sizeof(tmpA))) {
		CFG_SetMQTTPort(atoi(tmpA));
	}

#if MQTT_USE_TLS
	CFG_SetMQTTUseTls(http_getArg(request->url, "mqtt_use_tls", tmpA, sizeof(tmpA)));
	CFG_SetMQTTVerifyTlsCert(http_getArg(request->url, "mqtt_verify_tls_cert", tmpA, sizeof(tmpA)));
	http_getArg(request->url, "mqtt_cert_file", tmpA, sizeof(tmpA));
	CFG_SetMQTTCertFile(tmpA);
#endif

	if (http_getArg(request->url, "user", tmpA, sizeof(tmpA))) {
		CFG_SetMQTTUserName(tmpA);
	}
	if (http_getArg(request->url, "password", tmpA, sizeof(tmpA))) {
		CFG_SetMQTTPass(tmpA);
	}
	if (http_getArg(request->url, "client", tmpA, sizeof(tmpA))) {
		CFG_SetMQTTClientId(tmpA);
	}
	if (http_getArg(request->url, "group", tmpA, sizeof(tmpA))) {
		CFG_SetMQTTGroupTopic(tmpA);
	}

	CFG_Save_SetupTimer();

	poststr(request, "Please wait for module to connect... if there is problem, restart it from Index html page...");
#if ENABLE_MQTT
	g_mqtt_bBaseTopicDirty = 1;
#endif
	poststr(request, "<br><a href=\"cfg_mqtt\">Return to MQTT settings</a><br>");
	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}
int http_fn_cfg_mqtt(http_request_t* request) {
	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "MQTT");
	poststr_h2(request, "Use this to connect to your MQTT");
	poststr_h4(request, "To disable MQTT, clear the host field.");
	hprintf255(request, "<h4>Command topic: cmnd/%s/[Command]</h4>", CFG_GetMQTTClientId());
	hprintf255(request, "<h4>Publish data topic: %s/[Channel]/get</h4>", CFG_GetMQTTClientId());
	hprintf255(request, "<h4>Receive data topic:  %s/[Channel]/set</h4>", CFG_GetMQTTClientId());

	add_label_text_field(request, "Host", "host", CFG_GetMQTTHost(), "<form action=\"/cfg_mqtt_set\">");
	add_label_numeric_field(request, "Port", "port", CFG_GetMQTTPort(), "<br>");
#if MQTT_USE_TLS
	hprintf255(request, "<input type=\"checkbox\" id=\"mqtt_use_tls\" name=\"mqtt_use_tls\" value=\"1\"");
	if (CFG_GetMQTTUseTls()) {
		hprintf255(request, " checked>");
	}
	hprintf255(request, "<label for=\"mqtt_use_tls\">Use TLS</label><br>");

	hprintf255(request, "<input type=\"checkbox\" id=\"mqtt_verify_tls_cert\" name=\"mqtt_verify_tls_cert\" value=\"1\"");
	if (CFG_GetMQTTVerifyTlsCert()) {
		hprintf255(request, " checked>");
	}
	hprintf255(request, "<label for=\"mqtt_use_tls\">Verify TLS Certificate</label><br>");

	add_label_text_field(request, "Certificate File (CA Root or Public Certificate PEM format)", "mqtt_cert_file", CFG_GetMQTTCertFile(), "<br>");
#endif
	add_label_text_field(request, "Client Topic (Base Topic)", "client", CFG_GetMQTTClientId(), "<br><br>");
	add_label_text_field(request, "Group Topic (Secondary Topic to only receive cmnds)", "group", CFG_GetMQTTGroupTopic(), "<br>");
	add_label_text_field(request, "User", "user", CFG_GetMQTTUserName(), "<br>");
	add_label_password_field(request, "Password", "password", CFG_GetMQTTPass(), "<br>");
	poststr(request, "<span style=\"float:right;\"><input type=\"checkbox\" onclick=\"e=getElement('password');if(this.checked){e.type='text'}else e.type='password'\" > enable clear text password</span><br>");

	poststr(request, "<br><input type=\"submit\" value=\"Submit\" onclick=\"return confirm('Are you sure? Please check MQTT data twice?')\"></form> ");
	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}
#endif
#if ENABLE_HTTP_IP
int http_fn_cfg_ip(http_request_t* request) {
	char tmp[64];
	int g_changes = 0;
	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "IP");
	poststr_h2(request, "Here you can set static IP or DHCP");
	hprintf255(request, "<h4>This setting applies only to WiFi client mode.</h4>");
	hprintf255(request, "<h4>You must restart manually for changes to take place.</h4>");
	hprintf255(request, "<h4>Currently, DHCP is enabled by default and works when you set IP to 0.0.0.0.</h4>");

	if (http_getArg(request->url, "IP", tmp, sizeof(tmp))) {
		str_to_ip(tmp, g_cfg.staticIP.localIPAddr);
		hprintf255(request, "<br>IP=%s (%02x %02x %02x %02x)<br>",tmp,g_cfg.staticIP.localIPAddr[0],g_cfg.staticIP.localIPAddr[1],g_cfg.staticIP.localIPAddr[2],g_cfg.staticIP.localIPAddr[3]);
		g_changes++;
	}
	if (http_getArg(request->url, "mask", tmp, sizeof(tmp))) {
		str_to_ip(tmp, g_cfg.staticIP.netMask);
		hprintf255(request, "<br>Mask=%s (%02x %02x %02x %02x)<br>",tmp, g_cfg.staticIP.netMask[0], g_cfg.staticIP.netMask[1], g_cfg.staticIP.netMask[2], g_cfg.staticIP.netMask[3]);
		g_changes++;
	}
	if (http_getArg(request->url, "dns", tmp, sizeof(tmp))) {
		str_to_ip(tmp, g_cfg.staticIP.dnsServerIpAddr);
		hprintf255(request, "<br>DNS=%s (%02x %02x %02x %02x)<br>",tmp, g_cfg.staticIP.dnsServerIpAddr[0], g_cfg.staticIP.dnsServerIpAddr[1], g_cfg.staticIP.dnsServerIpAddr[2], g_cfg.staticIP.dnsServerIpAddr[3]);
		g_changes++;
	}
	if (http_getArg(request->url, "gate", tmp, sizeof(tmp))) {
		str_to_ip(tmp, g_cfg.staticIP.gatewayIPAddr);
		hprintf255(request, "<br>GW=%s (%02x %02x %02x %02x)<br>",tmp, g_cfg.staticIP.gatewayIPAddr[0], g_cfg.staticIP.gatewayIPAddr[1], g_cfg.staticIP.gatewayIPAddr[2], g_cfg.staticIP.gatewayIPAddr[3]);
		g_changes++;
	}
	if (g_changes) {
		CFG_MarkAsDirty();
		hprintf255(request, "<h4>Saved.</h4>");
	}
	convert_IP_to_string(tmp, g_cfg.staticIP.localIPAddr);
	add_label_text_field(request, "IP", "IP", tmp, "<form action=\"/cfg_ip\">");
	convert_IP_to_string(tmp, g_cfg.staticIP.netMask);
	add_label_text_field(request, "Mask", "mask", tmp, "<br><br>");
	convert_IP_to_string(tmp, g_cfg.staticIP.dnsServerIpAddr);
	add_label_text_field(request, "DNS", "dns", tmp, "<br>");
	convert_IP_to_string(tmp, g_cfg.staticIP.gatewayIPAddr);
	add_label_text_field(request, "Gate", "gate", tmp, "<br>");

	poststr(request, "<br><input type=\"submit\" value=\"Submit\" onclick=\"return confirm('Are you sure? Remember that you need to reboot manually to apply changes')\"></form> ");
	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}
#endif

#if ENABLE_HTTP_WEBAPP
int http_fn_cfg_webapp(http_request_t* request) {
	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Set Webapp");
	add_label_text_field(request, "URL of the Webapp", "url", CFG_GetWebappRoot(), "<form action=\"/cfg_webapp_set\">");

#if MQTT_USE_TLS
	hprintf255(request, "<input type=\"checkbox\" id=\"enable_web_server\" name=\"enable_web_server\" value=\"1\"");
	if (!CFG_GetDisableWebServer()) {
		hprintf255(request, " checked>");
	}
	hprintf255(request, "<label for=\"enable_web_server\">Web Server Enabled</label><br>");
#endif

	poststr(request, "<br><input type=\"submit\" value=\"Submit\">");
	poststr(request, "<br><input class=\"bgrn\" type=\"submit\" value=\"Reset to default\" onclick=\"if(!confirm('Reset WebApp URL to default?')) return false; document.getElementById('url').value='https://'; return true;\">");
	poststr(request, "</form>");
	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}

int http_fn_cfg_webapp_set(http_request_t* request) {
	char tmpA[128];
	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Saving Webapp");

	if (http_getArg(request->url, "url", tmpA, sizeof(tmpA))) {
		CFG_SetWebappRoot(tmpA);
		CFG_Save_IfThereArePendingChanges();
		hprintf255(request, "Webapp url set to %s", tmpA);
	}
	else {
		poststr(request, "Webapp url not set because you didn't specify the argument.");
	}

#if MQTT_USE_TLS
	CFG_SetDisableWebServer(!http_getArg(request->url, "enable_web_server", tmpA, sizeof(tmpA)));
	if (CFG_GetDisableWebServer()) {
		poststr(request, "<br>");
		poststr(request, "Webapp will be disabled on next boot!");
	}
#endif

	poststr(request, "<br>");
	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}
#endif




#if ENABLE_HTTP_PING
int http_fn_cfg_ping(http_request_t* request) {
	char tmpA[128];
	int bChanged;

	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Set Watchdog");
	bChanged = 0;
	poststr(request, "<h3>Ping watchdog (backup reconnect mechanism)</h3>");
	poststr(request, "<p> By default, all ZCE EM devices automatically try to reconnect to WiFi when a connection is lost.");
	poststr(request, " I have tested the reconnect mechanism many times by restarting my router and it always worked reliably.");
	poststr(request, " However, according to some reports, there are still some edge cases where a device fails to reconnect to WiFi.");
	poststr(request, " This is why <b>this mechanism</b> has been added.</p>");
	poststr(request, "<p>This mechanism continuously pings a specified host and reconnects to WiFi if it doesn't respond for the specified number of seconds.</p>");
	poststr(request, "<p>USAGE: For the host, choose the main address of your router and ensure it responds to pings. The interval is around 1 second, and the timeout can be set by the user, for example, to 60 seconds.</p>");
	if (http_getArg(request->url, "host", tmpA, sizeof(tmpA))) {
		CFG_SetPingHost(tmpA);
		poststr_h4(request, "New ping host set!");
		bChanged = 1;
	}
	/* if(http_getArg(request->url,"interval",tmpA,sizeof(tmpA))) {
		 CFG_SetPingIntervalSeconds(atoi(tmpA));
		 poststr(request,"<h4> New ping interval set!</h4>");
		 bChanged = 1;
	 }*/
	if (http_getArg(request->url, "disconnectTime", tmpA, sizeof(tmpA))) {
		CFG_SetPingDisconnectedSecondsToRestart(atoi(tmpA));
		poststr_h4(request, "New ping disconnectTime set!");
		bChanged = 1;
	}
	if (http_getArg(request->url, "clear", tmpA, sizeof(tmpA))) {
		CFG_SetPingDisconnectedSecondsToRestart(0);
		CFG_SetPingIntervalSeconds(0);
		CFG_SetPingHost("");
		poststr_h4(request, "Ping watchdog disabled!");
		bChanged = 1;
	}
	if (bChanged) {
		CFG_Save_IfThereArePendingChanges();
		poststr_h4(request, "Changes will be applied after restarting");
	}
	poststr(request, "<form action=\"/cfg_ping\">\
<input type=\"hidden\" id=\"clear\" name=\"clear\" value=\"1\">\
<input type=\"submit\" value=\"Disable ping watchdog!\">\
</form>");
	poststr_h2(request, "Use this to enable pinger");
	add_label_text_field(request, "Host", "host", CFG_GetPingHost(), "<form action=\"/cfg_ping\">");
	add_label_numeric_field(request, "Take action after this number of seconds with no reply", "disconnectTime",
		CFG_GetPingDisconnectedSecondsToRestart(), "<br>");
	poststr(request, "<br><br>\
<input type=\"submit\" value=\"Submit\" onclick=\"return confirm('Are you sure?')\">\
</form>");
	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}
#endif
int http_fn_cfg_wifi(http_request_t* request) {
	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Wi-Fi setup");
	poststr(request, "<h2>ZCE EM Wi-Fi setup</h2>");
	poststr(request, "<p>Enter the Wi-Fi network used by this device. After saving, ZCE EM restarts and reconnects automatically.</p>");
	poststr(request, "<form action=\"/cfg_wifi_set\">");
	add_label_text_field(request, "SSID", "ssid", CFG_GetWiFiSSID(), "");
	add_label_password_field(request, "Password", "pass", "", "");
	poststr(request, "<br><input type=\"submit\" value=\"Save Wi-Fi configuration\" onclick=\"return confirm('Save Wi-Fi settings and restart ZCE EM?')\">");
	poststr(request, "</form>");
	poststr(request, "<form action=\"/cfg_wifi\"><input type=\"hidden\" name=\"scan\" value=\"1\"><input type=\"submit\" value=\"Scan Wi-Fi networks\"></form>");
	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}

#if ENABLE_HTTP_NAMES
int http_fn_cfg_name(http_request_t* request) {
	// for a test, show password as well...
	char tmpA[128];

	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Set name");

	poststr_h2(request, "Change device names for display");
	if (http_getArg(request->url, "shortName", tmpA, sizeof(tmpA))) {
		if (STR_ReplaceWhiteSpacesWithUnderscore(tmpA)) {
			poststr_h2(request, "You cannot have whitespaces in short name!");
		}
		CFG_SetShortDeviceName(tmpA);
	}
	if (http_getArg(request->url, "name", tmpA, sizeof(tmpA))) {
		CFG_SetDeviceName(tmpA);
	}
	CFG_Save_IfThereArePendingChanges();

	poststr_h2(request, "Use this to change device names");
	add_label_name_field(request, "ShortName", "shortName", CFG_GetShortDeviceName(), "<form action=\"/cfg_name\">");
	add_label_name_field(request, "Full Name", "name", CFG_GetDeviceName(), "<br>");

	poststr(request, "<br><br>");
	poststr(request, "<input type=\"submit\" value=\"Submit\" "
		"onclick=\"return confirm('Are you sure? "
		"Short name might be used by Home Assistant, "
		"so you will have to reconfig some stuff.')\">");
	poststr(request, "</form>");
	//poststr(request,htmlReturnToCfg);
	//HTTP_AddBuildFooter(request);
	//poststr(request,htmlEnd);
	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}
#endif
int http_fn_cfg_wifi_set(http_request_t* request) {
	char tmpA[128];
	int bChanged = 0;

	addLogAdv(LOG_INFO, LOG_FEATURE_HTTP, "ZCE EM: saving Wi-Fi configuration");
	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Saving Wi-Fi");

	if (http_getArg(request->url, "ssid", tmpA, sizeof(tmpA))) {
		bChanged |= CFG_SetWiFiSSID(tmpA);
	}
	if (http_getArg(request->url, "pass", tmpA, sizeof(tmpA))) {
		bChanged |= CFG_SetWiFiPass(tmpA);
	}
	if (bChanged) {
		HAL_DisableEnhancedFastConnect();
		CFG_Save_SetupTimer();
		poststr(request, "<p>Wi-Fi configuration saved. ZCE EM will restart...</p>");
		RESET_ScheduleModuleReset(3);
	} else {
		poststr(request, "<p>No Wi-Fi changes detected.</p>");
	}
	poststr(request, "<br><a href=\"cfg_wifi\">Return to Wi-Fi setup</a><br>");
	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}


int http_fn_cfg_loglevel_set(http_request_t* request) {
	char tmpA[128];
	addLogAdv(LOG_INFO, LOG_FEATURE_HTTP, "HTTP_ProcessPacket: generating cfg_loglevel_set ");

	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Set log level");
	if (http_getArg(request->url, "loglevel", tmpA, sizeof(tmpA))) {
#if WINDOWS
#else
		g_loglevel = atoi(tmpA);
#endif
		poststr(request, "LOG level changed.");
	}

	tmpA[0] = 0;
#if WINDOWS
	add_label_text_field(request, "Loglevel", "loglevel", "", "<form action=\"/cfg_loglevel_set\">");
#else
	add_label_numeric_field(request, "Loglevel", "loglevel", g_loglevel, "<form action=\"/cfg_loglevel_set\">");
#endif
	poststr(request, "<br><br>\
<input type=\"submit\" value=\"Submit\" >\
</form>");

	poststr(request, "<br><a href=\"cfg\">Return to config settings</a><br>");
	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}

#if ENABLE_HTTP_MAC
int http_fn_cfg_mac(http_request_t* request) {
	// must be unsigned, else print below prints negatives as e.g. FFFFFFFe
	unsigned char mac[6];
	char tmpA[128];
	int i;
	char macStr[16];

	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Set MAC address");

	if (http_getArg(request->url, "mac", tmpA, sizeof(tmpA))) {
		for (i = 0; i < 6; i++)
		{
			mac[i] = hexbyte(&tmpA[i * 2]);
		}
		//sscanf(tmpA,"%02X%02X%02X%02X%02X%02X",&mac[0],&mac[1],&mac[2],&mac[3],&mac[4],&mac[5]);
		if (WiFI_SetMacAddress((char*)mac)) {
			poststr_h4(request, "New MAC set!");
		}
		else {
			poststr_h4(request, "MAC change error?");
		}
		CFG_Save_IfThereArePendingChanges();
	}

	WiFI_GetMacAddress((char*)mac);

	poststr_h2(request, "Here you can change MAC address.");

	sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	add_label_text_field(request, "MAC", "mac", macStr, "<form action=\"/cfg_mac\">");
	poststr(request, "<br><br>\
<input type=\"submit\" value=\"Submit\" onclick=\"return confirm('Are you sure? Please check MAC hex string twice?')\">\
</form>");
	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}
#endif

const char* CMD_GetResultString(commandResult_t r) {
	if (r == CMD_RES_OK)
		return "OK";
	if (r == CMD_RES_EMPTY_STRING)
		return "No command entered";
	if (r == CMD_RES_ERROR)
		return "Command found but returned error";
	if (r == CMD_RES_NOT_ENOUGH_ARGUMENTS)
		return "Not enough arguments for this command";
	if (r == CMD_RES_UNKNOWN_COMMAND)
		return "Unknown command";
	if (r == CMD_RES_BAD_ARGUMENT)
		return "Bad argument";
	return "Unknown error";
}
// all log printfs made by command will be sent also to request
void LOG_SetCommandHTTPRedirectReply(http_request_t* request);

int http_fn_cmd_tool(http_request_t* request) {
	commandResult_t res;
	const char* resStr;
	char tmpA[128];
	char* long_str_alloced = 0;
	int commandLen;

	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Command tool");
	poststr_h4(request, "Command Tool");
	poststr(request, "This is a basic command line. <br>");
	poststr(request, "Please consider using the 'Web Application' console for more options and real-time log viewing. <br>");
	poststr(request, "Remember that some commands are added after a restart when a driver is activated. <br>");

	commandLen = http_getArg(request->url, "cmd", tmpA, sizeof(tmpA));
	addLogAdv(LOG_ERROR, LOG_FEATURE_HTTP, "http_fn_cmd_tool: len %i",commandLen);
	if (commandLen) {
		poststr(request, "<br>");
		// all log printfs made by command will be sent also to request
		LOG_SetCommandHTTPRedirectReply(request);
		if (commandLen > (sizeof(tmpA) - 5)) {
			commandLen += 8;
			long_str_alloced = (char*)malloc(commandLen);
			if (long_str_alloced) {
				http_getArg(request->url, "cmd", long_str_alloced, commandLen);
				res = CMD_ExecuteCommand(long_str_alloced, COMMAND_FLAG_SOURCE_CONSOLE);
				free(long_str_alloced);
			}
			else {
				res = CMD_RES_ERROR;
			}
		}
		else {
			res = CMD_ExecuteCommand(tmpA, COMMAND_FLAG_SOURCE_CONSOLE);
		}
		LOG_SetCommandHTTPRedirectReply(0);
		resStr = CMD_GetResultString(res);
		hprintf255(request, "<h3>%s</h3>", resStr);
		poststr(request, "<br>");
	}
	add_label_text_field(request, "Command", "cmd", tmpA, "<form action=\"/cmd_tool\">");
	poststr(request, SUBMIT_AND_END_FORM);

	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}

#if ENABLE_HTTP_STARTUP
int http_fn_startup_command(http_request_t* request) {
	char tmpA[8];
	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Set startup command");
	poststr_h4(request, "Set/Change/Clear startup command line");
	poststr(request, "<p>Startup command is a shorter, smaller alternative to LittleFS autoexec.bat. "
		"The startup commands are run at device startup. "
		"You can use them to init peripherals and drivers, like BL0942 energy sensor. "
		"Use backlog cmd1; cmd2; cmd3; etc to enter multiple commands</p>");

	if (http_getArg(request->url, "startup_cmd", tmpA, sizeof(tmpA))) {
		// direct config access to remove buffer on stack
		int realSize = http_getArg(request->url, "data", g_cfg.initCommandLine, sizeof(g_cfg.initCommandLine));
		// mark as dirty (value has changed)
		g_cfg_pendingChanges++;
		if (realSize >= sizeof(g_cfg.initCommandLine)) {
			hprintf255(request, "<h3 style='color:red'>Command trimmed from %i to %i!</h3>",realSize, sizeof(g_cfg.initCommandLine));
		} else {
			hprintf255(request, "<h3>Command changed!</h3>");
		}
		CFG_Save_IfThereArePendingChanges();
	}

#if ENABLE_OBK_SCRIPTING
	poststr(request, "<form action=\"/startup_command\">");
	poststr(request, "<label for='data'>Startup command</label><br>");
	poststr(request, "<textarea id='data' name='data' rows='15' cols='40'>");
	poststr(request, CFG_GetShortStartupCommand());
	poststr(request, "</textarea><br>");
#else
	add_label_text_field(request, "Startup command", "data", CFG_GetShortStartupCommand(), "<form action=\"/startup_command\">");
#endif
	poststr(request, "<input type='hidden' name='startup_cmd' value='1'>");
	poststr(request, SUBMIT_AND_END_FORM);

	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}
#endif

#if ENABLE_HA_DISCOVERY
HassDeviceInfo *hass_createEnumChannelInfo(int i) {
	HassDeviceInfo *dev_info = 0;
	channelEnum_t *en;
	if (g_enums != NULL && g_enums[i]->numOptions != 0) {
		en = g_enums[i];
	}
	else {
		// revert to textfield if no enums are defined
		dev_info = hass_init_textField_info(i);
		return dev_info;;
	}

	char **options = (char**)malloc(en->numOptions * sizeof(char *));
	for (int o = 0; o < en->numOptions; o++) {
		options[o] = en->options[o].label;
	}

	if (en->options != NULL && en->numOptions > 0) {
		// backlog setChannelType 1 Enum; setChannelEnum 0:red 2:blue 3:green; scheduleHADiscovery 1
		char stateTopic[32];
		char cmdTopic[32];
		char title[64];
		char value_tmp[1024];
		char command_tmp[1024];

		CMD_GenEnumValueTemplate(en, value_tmp, sizeof(value_tmp));
		CMD_GenEnumCommandTemplate(en, command_tmp, sizeof(command_tmp));

		strcpy(title, CHANNEL_GetLabel(i));
		sprintf(stateTopic, "~/%i/get", i);
		sprintf(cmdTopic, "~/%i/set", i);
		dev_info = hass_createSelectEntityIndexedCustom(
			stateTopic,
			cmdTopic,
			en->numOptions,
			(const char**)options,
			title,
			value_tmp,
			command_tmp
		);
	}
	os_free(options);
	return dev_info;
}

#if PLATFORM_BL_NEW
extern void* _os_malloc(size_t size);
extern void _os_free(void* ptr);
#endif

void doHomeAssistantDiscovery(const char* topic, http_request_t* request) {
	int i;
	int relayCount;
	int pwmCount;
	int dInputCount;
	int excludedCount = 0;
	bool ledDriverChipRunning;
	HassDeviceInfo* dev_info = NULL;
	bool measuringPower = false;
	bool measuringBattery = false;
	struct cJSON_Hooks hooks;
	bool discoveryQueued = false;
	int type;
	// warning - this is 32 bit
	int flagsChannelPublished;
	int ch;
	int dimmer, toggle, brightness_scale = 0;

	// no channels published yet
	flagsChannelPublished = 0;

	for (i = 0; i < CHANNEL_MAX; i++) {
		if (CHANNEL_HasNeverPublishFlag(i)) {
			BIT_SET(flagsChannelPublished, i);
			excludedCount++;
		}
	}
	
	if (topic == 0 || *topic == 0) {
		topic = "homeassistant";
	}

#ifdef ENABLE_DRIVER_BL0937
	measuringPower = DRV_IsMeasuringPower();
#endif
	measuringBattery = DRV_IsMeasuringBattery();

	PIN_get_Relay_PWM_Count(&relayCount, &pwmCount, &dInputCount);
	addLogAdv(LOG_INFO, LOG_FEATURE_HTTP, "HASS counts: %i rels, %i pwms, %i inps, %i excluded", relayCount, pwmCount, dInputCount, excludedCount);

#if ENABLE_LED_BASIC
	ledDriverChipRunning = LED_IsLedDriverChipRunning();
#else
	ledDriverChipRunning = 0;
#endif

#if PLATFORM_TXW81X || PLATFORM_BL_NEW
	hooks.malloc_fn = _os_malloc;
	hooks.free_fn = _os_free;
#else
	hooks.malloc_fn = os_malloc;
	hooks.free_fn = os_free;
#endif
	cJSON_InitHooks(&hooks);

	DRV_OnHassDiscovery(topic);
	EventHandlers_FireEvent(CMD_EVENT_ON_DISCOVERY, 0);

#if ENABLE_ADVANCED_CHANNELTYPES_DISCOVERY
	// try to pair toggles with dimmers. This is needed only for TuyaMCU, 
	// where custom channel types are used. This is NOT used for simple
	// CW/RGB/RGBCW/etc lights.
	if (CFG_HasFlag(OBK_FLAG_DISCOVERY_DONT_MERGE_LIGHTS) == false) {
		while (true) {
			// find first dimmer
			dimmer = -1;
			for (i = 0; i < CHANNEL_MAX; i++) {
				type = g_cfg.pins.channelTypes[i];
				if (BIT_CHECK(flagsChannelPublished, i)) {
					continue;
				}
				if (type == ChType_Dimmer) {
					brightness_scale = 100;
					dimmer = i;
					break;
				}
				if (type == ChType_Dimmer1000) {
					brightness_scale = 1000;
					dimmer = i;
					break;
				}
				if (type == ChType_Dimmer256) {
					brightness_scale = 256;
					dimmer = i;
					break;
				}
			}
			// find first togle
			toggle = -1;
			for (i = 0; i < CHANNEL_MAX; i++) {
				type = g_cfg.pins.channelTypes[i];
				if (BIT_CHECK(flagsChannelPublished, i)) {
					continue;
				}
				if (type == ChType_Toggle) {
					toggle = i;
					break;
				}
			}
			// if nothing found, stop
			if (toggle == -1 || dimmer == -1) {
				break;
			}

			BIT_SET(flagsChannelPublished, toggle);
			BIT_SET(flagsChannelPublished, dimmer);
			dev_info = hass_init_light_singleColor_onChannels(toggle, dimmer, brightness_scale);
			MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
			hass_free_device_info(dev_info);
			discoveryQueued = true;
		}
	}
#endif


#if ENABLE_LED_BASIC
	if (ledDriverChipRunning) {
		pwmCount = CFG_CountLEDRemapChannels();
	}
	if (pwmCount == 5 || (pwmCount == 4 && CFG_HasFlag(OBK_FLAG_LED_EMULATE_COOL_WITH_RGB))) {
		if (dev_info == NULL) {
			dev_info = hass_init_light_device_info(LIGHT_RGBCW);
		}
		// Enable + RGB control + CW control
		MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
		hass_free_device_info(dev_info);
		dev_info = NULL;
		discoveryQueued = true;
	}
	else if (pwmCount > 0) {
		if (pwmCount == 4) {
			addLogAdv(LOG_ERROR, LOG_FEATURE_HTTP, "4 PWM device not yet handled");
		}
		else if (pwmCount == 3) {
			// Enable + RGB control
			dev_info = hass_init_light_device_info(LIGHT_RGB);
		}
		else if (pwmCount == 2) {
			// PWM + Temperature (https://github.com/zce/ZCE_EM7231T_App/issues/279)
			dev_info = hass_init_light_device_info(LIGHT_PWMCW);
		}
		else {
			dev_info = hass_init_light_device_info(LIGHT_PWM);
		}

		if (dev_info != NULL) {
			MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
			hass_free_device_info(dev_info);
			dev_info = NULL;
			discoveryQueued = true;
		}
	}
#endif

#ifdef ENABLE_DRIVER_BL0937
	if (measuringPower == true) {
		for (i = OBK__FIRST; i <= OBK__LAST; i++)
		{
			dev_info = hass_init_energy_sensor_device_info(i, BL_SENSORS_IX_0);
			if (dev_info) {
				MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
				hass_free_device_info(dev_info);
				discoveryQueued = true;
			}
			if (i == OBK_VOLTAGE) {
				//20250319 XJIKKA to simplify and save space in flash frequency together with voltage
				dev_info = hass_init_sensor_device_info(FREQUENCY_SENSOR, SPECIAL_CHANNEL_OBK_FREQUENCY, -1, -1, -1);
				if (dev_info) {
					MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
					hass_free_device_info(dev_info);
					discoveryQueued = true;
				}
			}
		}
#if ENABLE_BL_TWIN
		//BL_SENSORS_IX_1 - mqtt hass discovery using hass_uniq_id_suffix (_b) from drv_bl_shared.c
		if (BL_IsMeteringDeviceIndexActive(BL_SENSORS_IX_1)) {
			for (i = OBK__FIRST; i <= OBK__LAST; i++)
			{
				dev_info = hass_init_energy_sensor_device_info(i, BL_SENSORS_IX_1);
				if (dev_info) {
					MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
					hass_free_device_info(dev_info);
					discoveryQueued = true;
				}
			}
		}
#endif
	}
#endif

	if (measuringBattery == true) {
		dev_info = hass_init_sensor_device_info(BATTERY_SENSOR, 0, -1, -1, 1);
		MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
		hass_free_device_info(dev_info);

		dev_info = hass_init_sensor_device_info(BATTERY_VOLTAGE_SENSOR, 0, -1, -1, 1);
		MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
		hass_free_device_info(dev_info);

		discoveryQueued = true;
	}

	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		if (IS_PIN_DHT_ROLE(g_cfg.pins.roles[i]) || IS_PIN_TEMP_HUM_SENSOR_ROLE(g_cfg.pins.roles[i])) {
			ch = PIN_GetPinChannelForPinIndex(i);
			// TODO: flags are 32 bit and there are 64 max channels
			BIT_SET(flagsChannelPublished, ch);
			dev_info = hass_init_sensor_device_info(TEMPERATURE_SENSOR, ch, 2, 1, 1);
			MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
			hass_free_device_info(dev_info);

			ch = PIN_GetPinChannel2ForPinIndex(i);
			// TODO: flags are 32 bit and there are 64 max channels
			BIT_SET(flagsChannelPublished, ch);
			dev_info = hass_init_sensor_device_info(HUMIDITY_SENSOR, ch, -1, -1, 1);
			MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
			hass_free_device_info(dev_info);

			discoveryQueued = true;
		}
		else if (IS_PIN_AIR_SENSOR_ROLE(g_cfg.pins.roles[i])) {
			ch = PIN_GetPinChannelForPinIndex(i);
			// TODO: flags are 32 bit and there are 64 max channels
			BIT_SET(flagsChannelPublished, ch);
			dev_info = hass_init_sensor_device_info(CO2_SENSOR, ch, -1, -1, 1);
			MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
			hass_free_device_info(dev_info);

			ch = PIN_GetPinChannel2ForPinIndex(i);
			// TODO: flags are 32 bit and there are 64 max channels
			BIT_SET(flagsChannelPublished, ch);
			dev_info = hass_init_sensor_device_info(TVOC_SENSOR, ch, -1, -1, 1);
			MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
			hass_free_device_info(dev_info);

			discoveryQueued = true;
		}
	}
	//{
	//	HassDeviceInfo*dev_info = hass_createGarageEntity("~/1/get", "~/1/set",
	//	 "Main Door");
	//	MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
	//	hass_free_device_info(dev_info);
	//	discoveryQueued = true;
	//}
#if ENABLE_ADVANCED_CHANNELTYPES_DISCOVERY
	for (i = 0; i < CHANNEL_MAX; i++) {
		type = g_cfg.pins.channelTypes[i];
		// TODO: flags are 32 bit and there are 64 max channels
		if (BIT_CHECK(flagsChannelPublished, i)) {
			continue;
		}
		dev_info = 0;
		switch (type)
		{
			case ChType_Motion:
			{
				dev_info = hass_init_binary_sensor_device_info(i, true);
				cJSON_AddStringToObject(dev_info->root, "dev_cla", "motion");
			}
			break;
			case ChType_Motion_n:
			{
				dev_info = hass_init_binary_sensor_device_info(i, false);
				cJSON_AddStringToObject(dev_info->root, "dev_cla", "motion");
			}
			break;
			case ChType_OpenClosed:
			{
				dev_info = hass_init_binary_sensor_device_info(i, false);
			}
			break;
			case ChType_OpenClosed_Inv:
			{
				dev_info = hass_init_binary_sensor_device_info(i, true);
			}
			break;
			case ChType_Voltage_div10:
			{
				dev_info = hass_init_sensor_device_info(VOLTAGE_SENSOR, i, 2, 1, 1);
			}
			break;
			case ChType_Voltage_div100:
			{
				dev_info = hass_init_sensor_device_info(VOLTAGE_SENSOR, i, 2, 2, 1);
			}
			break;
			case ChType_ReadOnlyLowMidHigh:
			{
				dev_info = hass_init_sensor_device_info(READONLYLOWMIDHIGH_SENSOR, i, -1, -1, 1);
			}
			break;
			case ChType_BatteryLevelPercent:
			{
				dev_info = hass_init_sensor_device_info(BATTERY_CHANNEL_SENSOR, i, -1, -1, 1);
			}
			break;
			case ChType_SmokePercent:
			{
				dev_info = hass_init_sensor_device_info(SMOKE_SENSOR, i, -1, -1, 1);
			}
			break;
			case ChType_Illuminance:
			{
				dev_info = hass_init_sensor_device_info(ILLUMINANCE_SENSOR, i, -1, -1, 1);
			}
			break;
			case ChType_Custom:
			case ChType_ReadOnly:
			{
				dev_info = hass_init_sensor_device_info(CUSTOM_SENSOR, i, -1, -1, 1);
			}
			break;
			case ChType_Temperature:
			{
				dev_info = hass_init_sensor_device_info(TEMPERATURE_SENSOR, i, -1, -1, 1);
			}
			break;
			case ChType_Temperature_div2:
			{
				dev_info = hass_init_sensor_device_info(TEMPERATURE_SENSOR, i, 2, 1, 5);
			}
			break;
			case ChType_Temperature_div10:
			{
				dev_info = hass_init_sensor_device_info(TEMPERATURE_SENSOR, i, 2, 1, 1);
			}
			break;
			case ChType_ReadOnly_div10:
			{
				dev_info = hass_init_sensor_device_info(CUSTOM_SENSOR, i, 2, 1, 1);
			}
			break;
			case ChType_Temperature_div100:
			{
				dev_info = hass_init_sensor_device_info(TEMPERATURE_SENSOR, i, 2, 2, 1);
			}
			break;
			case ChType_ReadOnly_div100:
			{
				dev_info = hass_init_sensor_device_info(CUSTOM_SENSOR, i, 2, 2, 1);
			}
			break;
			case ChType_Humidity:
			{
				dev_info = hass_init_sensor_device_info(HUMIDITY_SENSOR, i, -1, -1, 1);
			}
			break;
			case ChType_Humidity_div10:
			{
				dev_info = hass_init_sensor_device_info(HUMIDITY_SENSOR, i, 2, 1, 1);
			}
			break;
			case ChType_Current_div100:
			{
				dev_info = hass_init_sensor_device_info(CURRENT_SENSOR, i, 3, 2, 1);
			}
			break;
			case ChType_ReadOnly_div1000:
			{
				dev_info = hass_init_sensor_device_info(CUSTOM_SENSOR, i, 3, 3, 1);
			}
			break;
			case ChType_LeakageCurrent_div1000:
			case ChType_Current_div1000:
			{
				dev_info = hass_init_sensor_device_info(CURRENT_SENSOR, i, 3, 3, 1);
			}
			break;
			case ChType_Power:
			{
				dev_info = hass_init_sensor_device_info(POWER_SENSOR, i, -1, -1, 1);
			}
			break;
			case ChType_Power_div10:
			{
				dev_info = hass_init_sensor_device_info(POWER_SENSOR, i, 2, 1, 1);
			}
			break;
			case ChType_Power_div100:
			{
				dev_info = hass_init_sensor_device_info(POWER_SENSOR, i, 3, 2, 1);
			}
			break;
			case ChType_PowerFactor_div100:
			{
				dev_info = hass_init_sensor_device_info(POWERFACTOR_SENSOR, i, 3, 2, 1);
			}
			break;
			case ChType_Pressure_div100:
			{
				dev_info = hass_init_sensor_device_info(PRESSURE_SENSOR, i, 3, 2, 1);
			}
			break;
			case ChType_PowerFactor_div1000:
			{
				dev_info = hass_init_sensor_device_info(POWERFACTOR_SENSOR, i, 4, 3, 1);
			}
			break;
			case ChType_Frequency_div100:
			{
				dev_info = hass_init_sensor_device_info(FREQUENCY_SENSOR, i, 3, 2, 1);
			}
			break;
			case ChType_Percent:
			{
				dev_info = hass_init_sensor_device_info(HASS_PERCENT, i, 3, 2, 1);
			}
			break;
			case ChType_Frequency_div1000:
			{
				dev_info = hass_init_sensor_device_info(FREQUENCY_SENSOR, i, 4, 3, 1);
			}
			break;
			case ChType_Frequency_div10:
			{
				dev_info = hass_init_sensor_device_info(FREQUENCY_SENSOR, i, 3, 1, 1);
			}
			break;
			case ChType_EnergyTotal_kWh_div100:
			{
				dev_info = hass_init_sensor_device_info(ENERGY_SENSOR, i, 3, 2, 1);
			}
			break;
			case ChType_EnergyExport_kWh_div1000:
			{
				dev_info = hass_init_sensor_device_info(ENERGY_SENSOR, i, 3, 3, 1);
			}
			break;
			case ChType_EnergyImport_kWh_div1000:
			{
				dev_info = hass_init_sensor_device_info(ENERGY_SENSOR, i, 3, 3, 1);
			}
			break;
			case ChType_EnergyTotal_kWh_div1000:
			{
				dev_info = hass_init_sensor_device_info(ENERGY_SENSOR, i, 3, 3, 1);
			}
			break;
			case ChType_Ph:
			{
				dev_info = hass_init_sensor_device_info(WATER_QUALITY_PH, i, 2, 2, 1);
			}
			break;
			case ChType_Orp:
			{
				dev_info = hass_init_sensor_device_info(WATER_QUALITY_ORP, i, -1, 2, 1);
			}
			break;
			case ChType_Tds:
			{
				dev_info = hass_init_sensor_device_info(WATER_QUALITY_TDS, i, -1, 2, 1);
			}
			break;
			case ChType_TextField:
			{
				dev_info = hass_init_textField_info(i);
			}
			break;
			case ChType_ReadOnlyEnum:
			{
				dev_info = hass_init_sensor_device_info(HASS_READONLYENUM, i, -1, -1, -1);
			}
			break;
			case ChType_Enum:
			{			
				dev_info = hass_createEnumChannelInfo(i);
			}
			break;
			case ChType_Illuminance_div10:
			{
				dev_info = hass_init_sensor_device_info(ILLUMINANCE_SENSOR, i, 2, 1, 1);
			}
			break;
			default:
			{
				int numOptions;
				const char **options = Channel_GetOptionsForChannelType(type, &numOptions);
				if (options && numOptions) {
					// backlog setChannelType 2 LowMidHigh; scheduleHADiscovery 1
					// backlog setChannelType 3 OpenStopClose; scheduleHADiscovery 1
					char stateTopic[16];
					char cmdTopic[16];
					// TODO: lengths
					sprintf(stateTopic, "~/%i/get", i);
					sprintf(cmdTopic, "~/%i/set", i);
					dev_info = hass_createSelectEntityIndexed(
						stateTopic,
						cmdTopic,
						numOptions,
						options,
						CHANNEL_GetLabel(i)
					);
				}
			}
			break;
		}
		if (dev_info) {
			MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
			hass_free_device_info(dev_info);

			BIT_SET(flagsChannelPublished, i);
			discoveryQueued = true;
		}
	}
#endif

	//if (relayCount > 0) {
	for (i = 0; i < CHANNEL_MAX; i++) {
		// if already included by light, skip
		if (BIT_CHECK(flagsChannelPublished, i)) {
			continue;
		}
		bool bToggleInv = g_cfg.pins.channelTypes[i] == ChType_Toggle_Inv;
		if (h_isChannelRelay(i) || g_cfg.pins.channelTypes[i] == ChType_Toggle || bToggleInv) {
			// TODO: flags are 32 bit and there are 64 max channels
			BIT_SET(flagsChannelPublished, i);
			if (CFG_HasFlag(OBK_FLAG_MQTT_HASS_ADD_RELAYS_AS_LIGHTS)) {
				dev_info = hass_init_relay_device_info(i, LIGHT_ON_OFF, bToggleInv);
			}
			else {
				dev_info = hass_init_relay_device_info(i, RELAY, bToggleInv);
			}
			MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
			hass_free_device_info(dev_info);
			dev_info = NULL;
			discoveryQueued = true;
		}
	}
	//}
	if (dInputCount > 0) {
		for (i = 0; i < CHANNEL_MAX; i++) {
			if (h_isChannelDigitalInput(i)) {
				if (BIT_CHECK(flagsChannelPublished, i)) {
					continue;
				}
				// TODO: flags are 32 bit and there are 64 max channels
				BIT_SET(flagsChannelPublished, i);
				dev_info = hass_init_binary_sensor_device_info(i, false);
				MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
				hass_free_device_info(dev_info);
				dev_info = NULL;
				discoveryQueued = true;
			}
		}
	}
	if(CFG_HasFlag(OBK_FLAG_MQTT_BROADCASTSELFSTATEPERMINUTE) || CFG_HasFlag(OBK_FLAG_MQTT_BROADCASTSELFSTATEONCONNECT)) {
		//use -1 for channel as these don't correspond to channels
#ifndef NO_CHIP_TEMPERATURE
		dev_info = hass_init_sensor_device_info(HASS_TEMP, -1, -1, -1, 1);
		MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
		hass_free_device_info(dev_info);
#endif
		dev_info = hass_init_sensor_device_info(HASS_RSSI, -1, -1, -1, 1);
		MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
		hass_free_device_info(dev_info);
		dev_info = hass_init_sensor_device_info(HASS_UPTIME, -1, -1, -1, 1);
		MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
		hass_free_device_info(dev_info);
		dev_info = hass_init_sensor_device_info(HASS_BUILD, -1, -1, -1, 1);
		MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
		hass_free_device_info(dev_info);
		dev_info = hass_init_sensor_device_info(HASS_SSID, -1, -1, -1, 1);
		MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
		hass_free_device_info(dev_info);
		dev_info = hass_init_sensor_device_info(HASS_IP, -1, -1, -1, 1);
		MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
		hass_free_device_info(dev_info);
		discoveryQueued = true;

	}
	if (discoveryQueued) {
		MQTT_InvokeCommandAtEnd(PublishChannels);
	}
	else {
		const char* msg = "No relay, PWM, sensor or power driver running.";
		if (request) {
			poststr(request, msg);
			poststr(request, NULL);
		}
		else {
			addLogAdv(LOG_ERROR, LOG_FEATURE_HTTP, "HA discovery: %s", msg);
		}
	}
}

/// @brief Sends HomeAssistant discovery MQTT messages.
/// @param request 
/// @return 
int http_fn_ha_discovery(http_request_t* request) {
	char topic[32];

	http_setup(request, httpMimeTypeText);

	if (MQTT_IsReady() == false) {
		poststr(request, "MQTT not running.");
		poststr(request, NULL);
		return 0;
	}

	// even if it returns the empty HA topic,
	// the function call below will set default
	http_getArg(request->url, "prefix", topic, sizeof(topic));
	doHomeAssistantDiscovery(topic, request);

	poststr(request, "MQTT discovery queued.");
	poststr(request, NULL);
	return 0;
}
#endif

#if ENABLE_OLD_YAML_GENERATOR
void http_generate_singleColor_cfg(http_request_t* request, const char* clientId) {
	hprintf255(request, "    command_topic: \"cmnd/%s/led_enableAll\"\n", clientId);
	hprintf255(request, "    state_topic: \"%s/led_enableAll/get\"\n", clientId);
	hprintf255(request, "    availability_topic: \"%s/connected\"\n", clientId);
	hprintf255(request, "    payload_on: 1\n");
	hprintf255(request, "    payload_off: 0\n");
	hprintf255(request, "    brightness_command_topic: \"cmnd/%s/led_dimmer\"\n", clientId);
	hprintf255(request, "    brightness_state_topic: \"%s/led_dimmer/get\"\n", clientId);
	hprintf255(request, "    brightness_scale: 100\n");
}
void http_generate_rgb_cfg(http_request_t* request, const char* clientId) {
	hprintf255(request, "    rgb_command_template: \"{{ '#%%02x%%02x%%02x0000' | format(red, green, blue)}}\"\n");
	hprintf255(request, "    rgb_value_template: \"{{ value[0:2]|int(base=16) }},{{ value[2:4]|int(base=16) }},{{ value[4:6]|int(base=16) }}\"\n");
	hprintf255(request, "    rgb_state_topic: \"%s/led_basecolor_rgb/get\"\n", clientId);
	hprintf255(request, "    rgb_command_topic: \"cmnd/%s/led_basecolor_rgb\"\n", clientId);

	http_generate_singleColor_cfg(request, clientId);
}
void http_generate_cw_cfg(http_request_t* request, const char* clientId) {
	hprintf255(request, "    color_temp_command_topic: \"cmnd/%s/led_temperature\"\n", clientId);
	hprintf255(request, "    color_temp_state_topic: \"%s/led_temperature/get\"\n", clientId);
	http_generate_singleColor_cfg(request, clientId);
}

void hprintf_qos_payload(http_request_t* request, const char* clientId) {
	poststr(request, "    qos: 1\n");
	poststr(request, "    payload_on: 1\n");
	poststr(request, "    payload_off: 0\n");
	poststr(request, "    retain: true\n");
	hprintf255(request, "    availability:\n");
	hprintf255(request, "      - topic: \"%s/connected\"\n", clientId);
}
#endif
#if ENABLE_HA_DISCOVERY
int http_fn_ha_cfg(http_request_t* request) {
	int relayCount;
	int pwmCount;
	int dInputCount;
	const char* shortDeviceName;
	const char* clientId;
	int i;
	char mqttAdded = 0;
	char switchAdded = 0;
	char lightAdded = 0;

	i = 0;

	shortDeviceName = CFG_GetShortDeviceName();
	clientId = CFG_GetMQTTClientId();

	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Home Assistant Setup");
	poststr_h4(request, "Home Assistant Cfg");
	hprintf255(request, "<h4>Note that your short device name is: %s</h4>", shortDeviceName);
#if ENABLE_OLD_YAML_GENERATOR
	poststr_h4(request, "Paste this to configuration yaml");
	poststr(request, "<h5>Make sure that you have \"switch:\" keyword only once! Home Assistant doesn't like dup keywords.</h5>");
	poststr(request, "<h5>You can also use \"switch MyDeviceName:\" to avoid keyword duplication!</h5>");

	poststr(request, "<textarea rows=\"40\" cols=\"50\">");

	PIN_get_Relay_PWM_Count(&relayCount, &pwmCount, &dInputCount);

	if (relayCount > 0) {

		for (i = 0; i < CHANNEL_MAX; i++) {
			if (h_isChannelRelay(i)) {
				if (mqttAdded == 0) {
					poststr(request, "mqtt:\n");
					mqttAdded = 1;
				}
				if (switchAdded == 0) {
					poststr(request, "  switch:\n");
					switchAdded = 1;
				}

				hass_print_unique_id(request, "  - unique_id: \"%s\"\n", RELAY, i, 0);
				hprintf255(request, "    name: %i\n", i);
				hprintf255(request, "    state_topic: \"%s/%i/get\"\n", clientId, i);
				hprintf255(request, "    command_topic: \"%s/%i/set\"\n", clientId, i);
				hprintf_qos_payload(request, clientId);
			}
		}
	}
	if (dInputCount > 0) {
		for (i = 0; i < CHANNEL_MAX; i++) {
			if (h_isChannelDigitalInput(i)) {
				if (mqttAdded == 0) {
					poststr(request, "mqtt:\n");
					mqttAdded = 1;
				}
				if (switchAdded == 0) {
					poststr(request, "  binary_sensor:\n");
					switchAdded = 1;
				}

				hass_print_unique_id(request, "  - unique_id: \"%s\"\n", BINARY_SENSOR, i, 0);
				hprintf255(request, "    name: %i\n", i);
				hprintf255(request, "    state_topic: \"%s/%i/get\"\n", clientId, i);
				hprintf_qos_payload(request, clientId);
			}
		}
	}
#if ENABLE_LED_BASIC
	if (pwmCount == 5 || LED_IsLedDriverChipRunning()) {
		// Enable + RGB control + CW control
		if (mqttAdded == 0) {
			poststr(request, "mqtt:\n");
			mqttAdded = 1;
		}
		if (switchAdded == 0) {
			poststr(request, "  light:\n");
			switchAdded = 1;
		}

		hass_print_unique_id(request, "  - unique_id: \"%s\"\n", LIGHT_RGBCW, i, 0);
		hprintf255(request, "    name: %i\n", i);
		http_generate_rgb_cfg(request, clientId);
		//hprintf255(request, "    #brightness_value_template: \"{{ value }}\"\n");
		hprintf255(request, "    color_temp_command_topic: \"cmnd/%s/led_temperature\"\n", clientId);
		hprintf255(request, "    color_temp_state_topic: \"%s/led_temperature/get\"\n", clientId);
		//hprintf255(request, "    #color_temp_value_template: \"{{ value }}\"\n");
	}
	else
		if (pwmCount == 3) {
			// Enable + RGB control
			if (mqttAdded == 0) {
				poststr(request, "mqtt:\n");
				mqttAdded = 1;
			}
			if (switchAdded == 0) {
				poststr(request, "  light:\n");
				switchAdded = 1;
			}

			hass_print_unique_id(request, "  - unique_id: \"%s\"\n", LIGHT_RGB, i, 0);
			hprintf255(request, "    name: Light\n");
			http_generate_rgb_cfg(request, clientId);
		}
		else if (pwmCount == 1) {
			// single color
			if (mqttAdded == 0) {
				poststr(request, "mqtt:\n");
				mqttAdded = 1;
			}
			if (switchAdded == 0) {
				poststr(request, "  light:\n");
				switchAdded = 1;
			}

			hass_print_unique_id(request, "  - unique_id: \"%s\"\n", LIGHT_PWM, i, 0);
			hprintf255(request, "    name: Light\n");
			http_generate_singleColor_cfg(request, clientId);
		}
		else if (pwmCount == 2) {
			// CW
			if (mqttAdded == 0) {
				poststr(request, "mqtt:\n");
				mqttAdded = 1;
			}
			if (switchAdded == 0) {
				poststr(request, "  light:\n");
				switchAdded = 1;
			}

			hass_print_unique_id(request, "  - unique_id: \"%s\"\n", LIGHT_PWMCW, i, 0);
			hprintf255(request, "    name: Light\n");
			http_generate_cw_cfg(request, clientId);
		}
		else if (pwmCount > 0) {

			for (i = 0; i < CHANNEL_MAX; i++) {
				if (h_isChannelPWM(i)) {
					if (mqttAdded == 0) {
						poststr(request, "mqtt:\n");
						mqttAdded = 1;
					}
					if (lightAdded == 0) {
						poststr(request, "  light:\n");
						lightAdded = 1;
					}

					hass_print_unique_id(request, "  - unique_id: \"%s\"\n", LIGHT_PWM, i, 0);
					hprintf255(request, "    name: %i\n", i);
					hprintf255(request, "    state_topic: \"%s/%i/get\"\n", clientId, i);
					hprintf255(request, "    command_topic: \"%s/%i/set\"\n", clientId, i);
					hprintf255(request, "    brightness_command_topic: \"%s/%i/set\"\n", clientId, i);
					poststr(request, "    on_command_type: \"brightness\"\n");
					poststr(request, "    brightness_scale: 99\n");
					poststr(request, "    qos: 1\n");
					poststr(request, "    payload_on: 99\n");
					poststr(request, "    payload_off: 0\n");
					poststr(request, "    retain: true\n");
					poststr(request, "    optimistic: true\n");
					hprintf255(request, "    availability:\n");
					hprintf255(request, "      - topic: \"%s/connected\"\n", clientId);
				}
			}
		}
#endif

	poststr(request, "</textarea>");
#endif
	poststr(request, "<br/><div><label for=\"ha_disc_topic\">Discovery topic:</label><input id=\"ha_disc_topic\" value=\"homeassistant\"><button onclick=\"send_ha_disc();\">Start Home Assistant Discovery</button>&nbsp;<form action=\"cfg_mqtt\" class='disp-inline'><button type=\"submit\">Configure MQTT</button></form></div><br/>");
	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, ha_discovery_script);
	poststr(request, NULL);
	return 0;
}

#endif

void runHTTPCommandInternal(http_request_t* request, const char *cmd) {
	bool bEchoHack = strncmp(cmd, "echo", 4) == 0;
	CMD_ExecuteCommand(cmd, COMMAND_FLAG_SOURCE_HTTP);
#if ENABLE_TASMOTA_JSON
	if (!bEchoHack) {
		JSON_ProcessCommandReply(cmd, skipToNextWord(cmd), request, (jsonCb_t)hprintf255, COMMAND_FLAG_SOURCE_HTTP);
	}
	else {
		const char *s = Tokenizer_GetArg(0);
		poststr(request, s);
	}
#endif
}
int http_fn_cm(http_request_t* request) {
	char tmpA[128];
	char* long_str_alloced = 0;
	int commandLen = 0;

	http_setup(request, httpMimeTypeJson);
	// exec command
	if (request->method == HTTP_GET) {
		commandLen = http_getArg(request->url, "cmnd", tmpA, sizeof(tmpA));
		//ADDLOG_INFO(LOG_FEATURE_HTTP, "Got here (GET) %s;%s;%d", request->url, tmpA, commandLen);
    } else if (request->method == HTTP_POST || request->method == HTTP_PUT) {
		commandLen = http_getRawArg(request->bodystart, "cmnd", tmpA, sizeof(tmpA));
		//ADDLOG_INFO(LOG_FEATURE_HTTP, "Got here (POST) %s;%s;%d", request->bodystart, tmpA, commandLen);
    }
	if (commandLen) {
		if (commandLen > (sizeof(tmpA) - 5)) {
			commandLen += 8;
			long_str_alloced = (char*)malloc(commandLen);
			if (long_str_alloced) {
				if (request->method == HTTP_GET) {
					http_getArg(request->url, "cmnd", long_str_alloced, commandLen);
				} else if (request->method == HTTP_POST || request->method == HTTP_PUT) {
					http_getRawArg(request->bodystart, "cmnd", long_str_alloced, commandLen);
				}
				CMD_ExecuteCommand(long_str_alloced, COMMAND_FLAG_SOURCE_HTTP);

				runHTTPCommandInternal(request, long_str_alloced);

				free(long_str_alloced);
			}
		}
		else {
			runHTTPCommandInternal(request, tmpA);
		}
	}

	poststr(request, NULL);


	return 0;
}

int http_fn_cfg(http_request_t* request) {
	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Configuration");
	poststr(request, "<h2>ZCE EM configuration</h2>");
	poststr(request, "<p>Only Wi-Fi setup and OTA update are available in this product firmware.</p>");
	postFormAction(request, "cfg_wifi", "Configure Wi-Fi");
	postFormAction(request, "ota", "OTA firmware update");
	poststr(request, htmlFooterReturnToMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}


int http_fn_cfg_pins(http_request_t* request) {
	int iChanged = 0;
	int iChangedRequested = 0;
	int i;
	char tmpA[128];
	char tmpB[64];

	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Pin config");

#if 0
	poststr(request, "<script src=\"https://test1.js\"></script>");
	//poststr(request, "<script src=\"http://localhost:8080/test1.js\"></script>");
	poststr(request, "<script>createBeforeMain();</script>");
#endif

	poststr(request, "<p>The first field assigns a role to the given pin. The next field is used to enter channel index (relay index), used to support multiple relays and buttons. ");
	poststr(request, "So, first button and first relay should have channel 1, second button and second relay have channel 2, etc.</p>");
	poststr(request, "<p>Only for button roles another field will be provided to enter channel to toggle when doing double click. ");
	poststr(request, "It shows up when you change role to button.</p>");
#if PLATFORM_BK7231N || PLATFORM_BK7231T
	poststr(request, "<p>BK7231N/BK7231T supports PWM only on pins 6, 7, 8, 9, 24 and 26!</p>");
#endif
	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		sprintf(tmpA, "%i", i);
		if (http_getArg(request->url, tmpA, tmpB, sizeof(tmpB))) {
			int role;
			int pr;

			iChangedRequested++;

			role = atoi(tmpB);

			pr = PIN_GetPinRoleForPinIndex(i);
			if (pr != role) {
				PIN_SetPinRoleForPinIndex(i, role);
				iChanged++;
			}
		}
		sprintf(tmpA, "r%i", i);
		if (http_getArg(request->url, tmpA, tmpB, sizeof(tmpB))) {
			int rel;
			int prevRel;

			iChangedRequested++;

			rel = atoi(tmpB);

			prevRel = PIN_GetPinChannelForPinIndex(i);
			if (prevRel != rel) {
				PIN_SetPinChannelForPinIndex(i, rel);
				iChanged++;
			}
		}
		sprintf(tmpA, "e%i", i);
		if (http_getArg(request->url, tmpA, tmpB, sizeof(tmpB))) {
			int rel;
			int prevRel;

			iChangedRequested++;

			rel = atoi(tmpB);

			prevRel = PIN_GetPinChannel2ForPinIndex(i);
			if (prevRel != rel) {
				PIN_SetPinChannel2ForPinIndex(i, rel);
				iChanged++;
			}
		}
	}
	if (iChangedRequested > 0) {
		// Anecdotally, if pins are configured badly, the
		// second-timer breaks. To reconfigure, force
		// saving the configuration instead of waiting.
		//CFG_Save_SetupTimer(); 
		CFG_Save_IfThereArePendingChanges();

		// Invoke Hass discovery if configuration has changed and not in safe mode.
#if ENABLE_HA_DISCOVERY
		if (!bSafeMode && CFG_HasFlag(OBK_FLAG_AUTOMAIC_HASS_DISCOVERY)) {
			Main_ScheduleHomeAssistantDiscovery(1);
		}
#endif
		hprintf255(request, "Pins update - %i reqs, %i changed!<br><br>", iChangedRequested, iChanged);
	}
	//	strcat(outbuf,"<button type=\"button\">Click Me!</button>");
	poststr(request, "<form action=\"cfg_pins\" id=\"x\">");


	poststr(request, "<script> var r = [");
	for (i = 0; i < IOR_Total_Options; i++) {
		if (i) {
			poststr(request, ",");
		}
		// print array with ["name_of_role",<Number of channnels for this role>]
		hprintf255(request, "[\"%s\",%i]", htmlPinRoleNames[i],PIN_IOR_NofChan(i));
	}
	poststr(request, "];");

	poststr(request, "var  sr = r.map((e,i)=>{return e[0]+'#'+i}).sort(Intl.Collator().compare).map(e=>e.split('#'));");
	
	poststr(request, "function hide_show() {"
		"n=this.name;"
		"er=getElement('r'+n);"
		"ee=getElement('e'+n);"
		"ch=r[this.value][1];"		// since we might have skiped PWM entries in options list, don't use "selectedIndex" but "value" (it's even shorter ;-)
		"er.disabled = (ch<1); er.style.display= ch<1 ? 'none' : 'inline';"
		"ee.disabled = (ch<2); ee.style.display= ch<2 ? 'none' : 'inline';"
		"}");

	poststr(request, "function f(alias, id, c, b, ch1, ch2) {"
		"let f = document.getElementById(\"x\");"
		"let d = document.createElement(\"div\");"
		"d.className = \"hdiv\";"
		"d.innerHTML = \"<span class='disp-inline' style='min-width: 15ch'>\"+alias+\"</span>\";"
		"f.appendChild(d);"
		"let s = document.createElement(\"select\");"
		"s.className = \"hele\";"
		"s.name = id;"
		"d.appendChild(s);"
		"	for (var i = 0; i < sr.length; i++) {"
		"	if(b && sr[i][0].startsWith(\"PWM\")) continue; "
		"var o = document.createElement(\"option\");"
		"	o.text = sr[i][0];"
		"	o.value = sr[i][1];"
		"	o.selected = (sr[i][1] == c);"
		"s.add(o);s.onchange = hide_show;"
		"}"
		"var y = document.createElement(\"input\");"
		"y.className = \"hele\";"
		"y.name = \"r\"+id;"
		"y.id = \"r\"+id;"
		"y.disabled = ch1==null;"
		"y.style.display = ch1==null ? 'none' :'inline' ;"
		"y.value = ch1==null ? 0 : ch1;"
		"d.appendChild(y);"
		"y = document.createElement(\"input\");"
		"y.className = \"hele\";"
		"y.name = \"e\"+id;"
		"y.id = \"e\"+id;"
		"y.disabled = ch2==null ;"
		"y.style.display = ch2==null ? 'none' :'inline' ;"
		"y.value = ch2==null ? 0 : ch2;"
		"d.appendChild(y);"
		" }");

	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		// On BL602, any GPIO can be mapped to one of 5 PWM channels
		// But on Beken chips, only certain pins can be PWM
		int bCanThisPINbePWM;
		int si, ch, ch2;
		const char* alias;

		si = PIN_GetPinRoleForPinIndex(i);
		ch = PIN_GetPinChannelForPinIndex(i);
		ch2 = PIN_GetPinChannel2ForPinIndex(i);

		// if available..
		alias = HAL_PIN_GetPinNameAlias(i);

		bCanThisPINbePWM = HAL_PIN_CanThisPinBePWM(i);
		si = PIN_GetPinRoleForPinIndex(i);
		hprintf255(request, "f(\"");
		if (alias) {
#if defined(PLATFORM_BEKEN) || defined(WINDOWS)
			hprintf255(request, "P%i (%s) ", i, alias);
#else
			poststr(request, alias);
			poststr(request, " ");
#endif
		}
		else {
			hprintf255(request, "P%i ", i);
		}
		hprintf255(request, "\",%i,%i, %i,", i, si, !bCanThisPINbePWM);
		// Primary linked channel
		int NofC = PIN_IOR_NofChan(si);
		if (NofC >= 1)
		{
			hprintf255(request, "%i,", ch);
		}
		// Some roles do not need any channels
		else {
			hprintf255(request, "null,", ch);
		}
		// Secondary linked channel
		if (NofC > 1)
		{
			hprintf255(request, "%i,", ch2);
		}
		else {
			hprintf255(request, "null,", ch);
		}
		hprintf255(request, ");");

	}
	poststr(request, " </script>");
	poststr(request, "<input type=\"submit\" value=\"Save\"/></form>");

	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}

#if ENABLE_HTTP_FLAGS

const char* g_obk_flagNames[] = {
	"[MQTT] Broadcast led params together (send dimmer and color when dimmer or color changes, topic name: YourDevName/led_basecolor_rgb/get, YourDevName/led_dimmer/get)",
	"[MQTT] Broadcast led final color (topic name: YourDevName/led_finalcolor_rgb/get)",
	"[MQTT] Broadcast self state every N (def: 60) seconds (delay configurable by 'mqtt_broadcastInterval' and 'mqtt_broadcastItemsPerSec' commands)",
	"[LED][Debug] Show raw PWM controller on WWW index instead of new LED RGB/CW/etc picker",
	"[LED] Force show RGBCW controller (for example, for SM2135 LEDs, or for DGR sender)",
	"[CMD] Enable TCP console command server (for PuTTY, etc)",
	"[BTN] Instant touch reaction instead of waiting for release (aka SetOption 13)",
	"[MQTT] [Debug] Always set Retain flag to all published values",
	"[LED] Alternate CW light mode (first PWM for warm/cold slider, second for brightness)",
	"[SM2135] Use separate RGB/CW modes instead of writing all 5 values as RGB",
	"[MQTT] Broadcast self state on MQTT connect",
	"[PWM] BK7231 use 600hz instead of 1khz default",
	"[LED] Remember LED driver state (RGBCW, enable, brightness, temperature) after reboot",
	"[HTTP] Show actual PIN logic level for unconfigured pins",
	"[IR] Do MQTT publish (RAW STRING) for incoming IR data",
	"[IR] Allow 'unknown' protocol",
	"[MQTT] Broadcast led final color RGBCW (topic name: YourDevName/led_finalcolor_rgbcw/get)",
	"[LED] Automatically enable Light when changing brightness, color or temperature on WWW panel",
	"[LED] Smooth transitions for LED (EXPERIMENTAL)",
	"[MQTT] Always publish channels used by TuyaMCU",
	"[LED] Force RGB mode (3 PWMs for LEDs) and ignore further PWMs if they are set",
	"[MQTT] Retain power channels (Relay channels, etc)",
	"[IR] Do MQTT publish (Tasmota JSON format) for incoming IR data",
	"[LED] Automatically enable Light on any change of brightness, color or temperature",
	"[LED] Emulate Cool White with RGB in device with four PWMs - Red is 0, Green 1, Blue 2, and Warm is 4",
	"[POWER] Allow negative current/power for power measurement (all chips, BL0937, BL0942, etc)",
	// On BL602, if marked, uses /dev/ttyS1, otherwise S0
	// On Beken, if marked, uses UART2, otherwise UART1
	"[UART] Use alternate UART for BL0942, CSE, TuyaMCU, etc",
	"[HASS] Invoke HomeAssistant discovery on change to ip address, configuration",
	"[LED] Setting RGB white (FFFFFF) enables temperature mode",
	"[NETIF] Use short device name as a hostname instead of a long name",
	"[MQTT] Enable Tasmota TELE etc publishes (for ioBroker etc)",
	"[UART] Enable UART command line",
	"[LED] Use old linear brightness mode, ignore gamma ramp",
	"[MQTT] Apply channel type multiplier on (if any) on channel value before publishing it",
	"[MQTT] In HA discovery, add relays as lights",
	"[HASS] Deactivate avty_t flag when publishing to HASS (permit to keep value). You must restart HASS discovery for change to take effect.",
	"[DRV] Deactivate Autostart of all drivers",
	"[WiFi] Quick connect to WiFi on reboot (TODO: check if it works for you and report on github)",
	"[Power] Set power and current to zero if all relays are open",
	"[MQTT] [Debug] Publish all channels (don't enable it, it will be publish all 64 possible channels on connect)",
	"[MQTT] Use kWh unit for energy consumption (total, last hour, today) instead of Wh",
	"[BTN] Ignore all button events (aka child lock)",
	"[DoorSensor] Invert state",
	"[TuyaMCU] Use queue",
	"[HTTP] Disable authentication in safe mode (not recommended)",
	"[MQTT Discovery] Don't merge toggles and dimmers into lights",
	"[TuyaMCU] Store raw data",
	"[TuyaMCU] Store ALL data",
	"[PWR] Invert AC dir",
	"[HTTP] Hide ON/OFF for relays (only red/green buttons)",
	"[MQTT] Never add GET suffix",
	"[WiFi] (RTL/BK/BL602) Enhanced fast connect by saving AP data to flash (preferable with Flag 37 & static ip). Quick reset 3 times to connect normally",
	"error",
	"error",
	"error",
	"error",
}; 

void uint64_to_str(uint64_t num, char* str) {
	char temp[21];  // uint64_t 20 numbers + \0
	int i = 0;
	if (num == 0) {
		temp[i++] = '0';
	} else {
		while (num > 0) {
			temp[i++] = '0' + (num % 10);
			num /= 10;
		}
	}
	temp[i] = '\0';
	int j;
	for (j = 0; j < i; j++) {
		str[j] = temp[i - j - 1];
	}
	str[j] = '\0';
}

int http_fn_cfg_generic(http_request_t* request) {
	int i;
	char tmpA[64];
	char tmpB[64];

	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Generic config");

	if (http_getArg(request->url, "boot_ok_delay", tmpA, sizeof(tmpA))) {
		i = atoi(tmpA);
		if (i <= 0) {
			poststr(request, "<h5>Boot ok delay must be at least 1 second<h5>");
			i = 1;
		}
		hprintf255(request, "<h5>Setting boot OK delay to %i<h5>", i);
		CFG_SetBootOkSeconds(i);
	}

	if (http_getArg(request->url, "setFlags", tmpA, sizeof(tmpA))) {
		for (i = 0; i < OBK_TOTAL_FLAGS; i++) {
			int ni;
			sprintf(tmpB, "flag%i", i);

			if (http_getArg(request->url, tmpB, tmpA, sizeof(tmpA))) {
				ni = atoi(tmpA);
			}
			else {
				ni = 0;
			}
			//hprintf255(request, "<h5>Setting flag %i to %i<h5>", i, ni);
			CFG_SetFlag(i, ni);
		}
	}

	CFG_Save_IfThereArePendingChanges();

	// 32 bit type
	//hprintf255(request, "<h4>Flags (Current value=%i)</h4>", CFG_GetFlags());
	// 64 bit - TODO fixme
	//hprintf255(request, "<h4>Flags (Current value=%llu)</h4>", CFG_GetFlags64());
	char buf[21];
	uint64_to_str(CFG_GetFlags64(), buf);
	hprintf255(request, "<h4>Flags (Current value=%s)</h4>", buf);
	poststr(request, "<form action=\"/cfg_generic\">");

	for (i = 0; i < OBK_TOTAL_FLAGS; i++) {
		const char* flagName = g_obk_flagNames[i];
		/*
		<div><input type="checkbox" name="flag0" id="flag0" value="1" checked>
		<label for="flag0">Flag 0 - [MQTT] Broadcast led params together (send dimmer and color when dimmer or color changes, topic name: YourDevName/led_basecolor_rgb/get, YourDevName/led_dimmer/get)</label>
		</div>
		*/
		hprintf255(request, "<div><input type=\"checkbox\" name=\"flag%i\" id=\"flag%i\" value=\"1\"%s>",
			i, i, (CFG_HasFlag(i) ? " checked" : "")); //this is less that 128 char

		hprintf255(request, "<label for=\"flag%i\">Flag %i - ", i, i);
		poststr(request, flagName);
		poststr(request, "</label></div>");
	}
	poststr(request, "<input type=\"hidden\" id=\"setFlags\" name=\"setFlags\" value=\"1\">");
	poststr(request, SUBMIT_AND_END_FORM);

	add_label_numeric_field(request, "Uptime in seconds required to mark boot as OK", "boot_ok_delay",
		CFG_GetBootOkSeconds(), "<form action=\"/cfg_generic\">");
	poststr(request, "<br><input type=\"submit\" value=\"Save\"/></form>");

	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}
#endif
#if ENABLE_HTTP_STARTUP
int http_fn_cfg_startup(http_request_t* request) {
	int channelIndex;
	int newValue;
	int i;
	char tmpA[128];

	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Config startup");
	poststr_h4(request, "Here you can set pin start values");
	poststr(request, "<ul><li>For relays, use 1 or 0</li>");
	poststr(request, "<li>To 'remember last power state', use -1 as a special value</li>");
	poststr(request, "<li>For dimmers, range is 0 to 100</li>");
	poststr(request, "<li>For custom values, you can set any numeric value</li>");
	poststr(request, "<li>Remember that you can also use short <a href='startup_command'>startup command</a> to run commands like led_baseColor #FF0000 and led_enableAll 1 etc</li>");
	hprintf255(request, "<li>To remember last state of LED driver, set ");
	hprintf255(request, "<a href='cfg_generic'>Flag 12 - %s</a>", g_obk_flagNames[12]);
	poststr(request, "</li></ul>");

	if (http_getArg(request->url, "idx", tmpA, sizeof(tmpA))) {
		channelIndex = atoi(tmpA);
		if (http_getArg(request->url, "value", tmpA, sizeof(tmpA))) {
			newValue = atoi(tmpA);


			CFG_SetChannelStartupValue(channelIndex, newValue);
			// also save current value if marked as saved
			Channel_SaveInFlashIfNeeded(channelIndex);
			hprintf255(request, "<h5>Setting channel %i start value to %i<h5>", channelIndex, newValue);

			CFG_Save_IfThereArePendingChanges();
		}
	}

	poststr_h4(request, "New start values");

	for (i = 0; i < CHANNEL_MAX; i++) {
		if (CHANNEL_IsInUse(i)) {
			int startValue = CFG_GetChannelStartupValue(i);

			poststr(request, "<form action='/cfg_startup' class='indent'>");
			hprintf255(request, "<input type=\"hidden\" id=\"idx\" name=\"idx\" value=\"%i\"/>", i);

			sprintf(tmpA, "Channel %i", i);
			add_label_numeric_field(request, tmpA, "value", startValue, "");

			poststr(request, "<input type=\"submit\" value=\"Save\"/></form><br/>");
		}
	}

	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}
#endif
#if ENABLE_HTTP_DGR
int http_fn_cfg_dgr(http_request_t* request) {
	char tmpA[128];
	bool bForceSet;

	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "Device groups");

	hprintf255(request, "<h5>Here you can configure Tasmota Device Groups<h5>");

	if (http_getArg(request->url, "bSet", tmpA, sizeof(tmpA))) {
		bForceSet = true;
	}
	else {
		bForceSet = false;
	}

	if (http_getArg(request->url, "name", tmpA, sizeof(tmpA)) || bForceSet) {
		int newSendFlags;
		int newRecvFlags;

		newSendFlags = 0;
		newRecvFlags = 0;

		if (http_getArgInteger(request->url, "s_pwr"))
			newSendFlags |= DGR_SHARE_POWER;
		if (http_getArgInteger(request->url, "r_pwr"))
			newRecvFlags |= DGR_SHARE_POWER;
		if (http_getArgInteger(request->url, "s_lbr"))
			newSendFlags |= DGR_SHARE_LIGHT_BRI;
		if (http_getArgInteger(request->url, "r_lbr"))
			newRecvFlags |= DGR_SHARE_LIGHT_BRI;
		if (http_getArgInteger(request->url, "s_lcl"))
			newSendFlags |= DGR_SHARE_LIGHT_COLOR;
		if (http_getArgInteger(request->url, "r_lcl"))
			newRecvFlags |= DGR_SHARE_LIGHT_COLOR;

		CFG_DeviceGroups_SetName(tmpA);
		CFG_DeviceGroups_SetSendFlags(newSendFlags);
		CFG_DeviceGroups_SetRecvFlags(newRecvFlags);

		if (tmpA[0] != 0) {
#ifndef OBK_DISABLE_ALL_DRIVERS
			DRV_StartDriver("DGR");
#endif
		}
		CFG_Save_IfThereArePendingChanges();
	}
	{
		int newSendFlags;
		int newRecvFlags;
		const char* groupName = CFG_DeviceGroups_GetName();


		newSendFlags = CFG_DeviceGroups_GetSendFlags();
		newRecvFlags = CFG_DeviceGroups_GetRecvFlags();

		add_label_text_field(request, "Group name", "name", groupName, "<form action=\"/cfg_dgr\">");
		poststr(request, "<br><table><tr><th>Name</th><th>Tasmota Code</th><th>Receive</th><th>Send</th></tr><tr><td>Power</td><td>1</td>");

		poststr(request, "<td><input type=\"checkbox\" name=\"r_pwr\" value=\"1\"");
		if (newRecvFlags & DGR_SHARE_POWER)
			poststr(request, " checked");
		poststr(request, "></td><td><input type=\"checkbox\" name=\"s_pwr\" value=\"1\"");
		if (newSendFlags & DGR_SHARE_POWER)
			poststr(request, " checked");
		poststr(request, "></td> ");

		poststr(request, "</tr><tr><td>Light Brightness</td><td>2</td>");

		poststr(request, "<td><input type=\"checkbox\" name=\"r_lbr\" value=\"1\"");
		if (newRecvFlags & DGR_SHARE_LIGHT_BRI)
			poststr(request, " checked");
		poststr(request, "></td><td><input type=\"checkbox\" name=\"s_lbr\" value=\"1\"");
		if (newSendFlags & DGR_SHARE_LIGHT_BRI)
			poststr(request, " checked");
		poststr(request, "></td> ");

		poststr(request, "</tr><tr><td>Light Color</td><td>16</td>");
		poststr(request, "<td><input type=\"checkbox\" name=\"r_lcl\" value=\"1\"");
		if (newRecvFlags & DGR_SHARE_LIGHT_COLOR)
			poststr(request, " checked");
		poststr(request, "></td><td><input type=\"checkbox\" name=\"s_lcl\" value=\"1\"");
		if (newSendFlags & DGR_SHARE_LIGHT_COLOR)
			poststr(request, " checked");
		poststr(request, "></td> ");

		poststr(request, "<input type=\"hidden\" name=\"bSet\" value=\"1\">");

		poststr(request, "</tr></table>");
		poststr(request, SUBMIT_AND_END_FORM);
	}

	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}
#endif

void OTA_RequestDownloadFromHTTP(const char* s) {
#if PLATFORM_BEKEN
	otarequest(s);
#elif PLATFORM_ECR6600
	extern int http_client_download_file(const char* url);
	extern int ota_done(bool reset);
	delay_ms(100);
	int ret = http_client_download_file(s);
	if(ret != -1) ota_done(1);
	else ota_done(0);
#elif PLATFORM_W600 || PLATFORM_W800
	t_http_fwup(s);
#elif PLATFORM_XRADIO
	uint32_t* verify_value;
	ota_verify_t      verify_type;
	ota_verify_data_t verify_data;

	if(ota_get_image(OTA_PROTOCOL_HTTP, s) != OTA_STATUS_OK)
	{
		addLogAdv(LOG_ERROR, LOG_FEATURE_HTTP, "OTA http get image failed");
		return;
	}

	if(ota_get_verify_data(&verify_data) != OTA_STATUS_OK)
	{
		verify_type = OTA_VERIFY_NONE;
		verify_value = NULL;
	}
	else
	{
		verify_type = verify_data.ov_type;
		verify_value = (uint32_t*)(verify_data.ov_data);
	}

	if(ota_verify_image(verify_type, verify_value) != OTA_STATUS_OK)
	{
		addLogAdv(LOG_ERROR, LOG_FEATURE_HTTP, "OTA http verify image failed");
		return;
	}

	ota_reboot();
#elif PLATFORM_REALTEK_NEW
	ota_context* ctx = NULL;
	ctx = (ota_context*)malloc(sizeof(ota_context));
	if(ctx == NULL) goto exit;
	memset(ctx, 0, sizeof(ota_context));
	char url[256] = { 0 };
	char resource[256] = { 0 };
	uint16_t port;
	parser_url(s, &url, &port, &resource, 256);
	int ret = ota_update_init(ctx, &url, port, &resource, OTA_HTTP);
	if(ret != 0)
	{
		addLogAdv(LOG_ERROR, LOG_FEATURE_HTTP, "ota_update_init failed");
		goto exit;
	}
	ret = ota_update_start(ctx);
	if(!ret)
	{
		addLogAdv(LOG_INFO, LOG_FEATURE_HTTP, "OTA finished");
		sys_clear_ota_signature();
		delay_ms(50);
		sys_reset();
	}
exit:
	ota_update_deinit(ctx);
	addLogAdv(LOG_ERROR, LOG_FEATURE_HTTP, "OTA failed");
	if(ctx) free(ctx);
#endif
}
int http_fn_ota_exec(http_request_t* request) {
	char tmpA[128];
	//char tmpB[64];

	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "OTA request");
	if (http_getArg(request->url, "host", tmpA, sizeof(tmpA))) {
		hprintf255(request, "<h3>OTA requested for %s!</h3>", tmpA);
		addLogAdv(LOG_INFO, LOG_FEATURE_HTTP, "http_fn_ota_exec: will try to do OTA for %s", tmpA);
		OTA_RequestDownloadFromHTTP(tmpA);
	}
	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}

int http_fn_ota(http_request_t* request) {
	http_setup(request, httpMimeTypeHTML);
	http_html_start(request, "OTA firmware update");
#ifndef OBK_OTA_EXTENSION
	poststr(request, "<h3>Sorry, OTA update not implemented for " DEVICENAME_PREFIX_FULL " </h3>");
#else
	poststr(request, "<p>Upload a ZCE EM firmware file from this page or provide an OTA URL below. "
#if PLATFORM_BEKEN
	" The .rbl file is used for OTA updates on this target."
#endif
	"</p>");
	add_label_text_field(request, "URL for ota firmware file", "host", "", "<form action=\"/ota_exec\">");
	poststr(request, "<br>\
<input type=\"submit\" value=\"Submit\" onclick=\"return confirm('Are you sure?')\">\
</form>");

	const char htmlOTA[] = "<script>var o=document.getElementById('otafile'),d=document.querySelector('dialog'),h=document.getElementById('hint'),D='OTA started! Please wait ',R=/" DEVICENAME_PREFIX_FULL "_.*" 
#ifdef OBK_OTA_NAME_EXTENSION
	OBK_OTA_NAME_EXTENSION
#endif
	OBK_OTA_EXTENSION "/,SR=R.source,mr=(e)=>e.name.match(R);doota=()=>{f=o.files[0];if(f&&(f)){d.showModal();var t=30;setTimeout(()=>{d.close(),location.href='/'},1e3*t),setInterval(()=>d.innerHTML=D+t--+' secs',1e3),fetch('/api/ota',{method:'POST',body:f}).then((e)=>{e.ok&&fetch('/index?restart=1')})}else alert(f?'filename invalid':'no file selected')};d.innerHTML=D,o.addEventListener('change',((e)=>{const t=e.target.files[0];if(!t)return;h.innerHTML=mr(t)?'':'Selected file does <b>not</b> match required format '+SR+'!'}))</script>";

	poststr(request, "<br><br><br>Expert feature: Upload firmware OTA file.<br>Use only firmware validated for ZCE EM.<br><span id='hint' style='color: yellow;'></span><br><br>");
	poststr(request, "<input id='otafile' type='file' accept='" OBK_OTA_EXTENSION "'>");
	poststr(request, "<input type='button' class='bred' onclick='doota();' value='START OTA - device will reboot after OTA'><dialog></dialog>");
	poststr(request, htmlOTA);
#endif
	poststr(request, htmlFooterReturnToCfgOrMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}

int http_fn_other(http_request_t* request) {
	http_setup(request, httpMimeTypeHTML);
#if ENABLE_OBK_BERRY
	if (CMD_Berry_RunEventHandlers_StrPtr(CMD_EVENT_ON_HTTP, request->url, request)) {
		return 0;
	}
#endif
	http_html_start(request, "Not found");
	poststr(request, "Not found.<br/>");
	poststr(request, htmlFooterReturnToMainPage);
	http_html_end(request);
	poststr(request, NULL);
	return 0;
}
