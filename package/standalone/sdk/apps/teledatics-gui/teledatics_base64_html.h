#ifndef TELEDATICS_BASE64_HTML
#define TELEDATICS_BASE64_HTML

#define WIFI_MODE_SUBST 0
#define WIFI_SSID_SUBST 1
#define WIFI_SECURITY_SUBST 2
#define WIFI_PASSWORD_SUBST 3
#define WIFI_COUNTRY_SUBST 4
#define WIFI_CHANNEL_SUBST 5
#define WIFI_IPMODE_SUBST 6
#define WIFI_STATICIP_SUBST 7
#define WIFI_DHCPSERVER_SUBST 8
#define WIFI_INTERVAL_SUBST 9
#define WIFI_SHORT_BCN_SUBST 10
#define WIFI_TXPOWER_SUBST 11
#define WIFI_COUNT_SUBST 12
#define WIFI_DURATION_SUBST 13
#define PPP_ENABLE_SUBST 14
#define TDXPAH_ADDON_SUBST 15

const char* html_subst[] = {
  "%%WIFI_MODE%%",     "%%WIFI_SSID%%",      "%%WIFI_SECURITY%%",
  "%%WIFI_PASSWORD%%", "%%WIFI_COUNTRY%%",   "%%WIFI_CHANNEL%%",
  "%%WIFI_IP_MODE%%",  "%%WIFI_STATIC_IP%%", "%%WIFI_DHCP_SERVER%%",
  "%%WIFI_INTERVAL%%", "%%WIFI_SHORT_BCN%%", "%%WIFI_TXPOWER%%",
  "%%WIFI_COUNT%%",    "%%WIFI_DURATION%%",  "%%PPP_ENABLE%%",
  "%%TDXPAH_ADDON%%",
};

#define WIFI_SUBST_SIZE sizeof(html_subst) / sizeof(html_subst[0])

// NOTE: see Makefile
const char html_base64[] = {
#include "teledatics_gui_html_lzo.h"
};

#endif /* TELEDATICS_BASE64_HTML */
