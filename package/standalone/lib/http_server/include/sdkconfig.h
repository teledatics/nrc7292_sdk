#undef BOOTLOADER_BUILD
#define CONFIG_HTTPD_MAX_REQ_HDR_LEN 1024
#define CONFIG_HTTPD_MAX_URI_LEN 1024
#define CONFIG_HTTPD_ERR_RESP_NO_DELAY 1
#define CONFIG_HTTPD_PURGE_BUF_LEN 32
#define CONFIG_LOG_DEFAULT_LEVEL 0
#define CONFIG_LWIP_MAX_SOCKETS 16
#define CONFIG_LOG_MAXIMUM_LEVEL ESP_LOG_NONE
#define CONFIG_BOOTLOADER_LOG_LEVEL ESP_LOG_NONE
#define CONFIG_LOG_TIMESTAMP_SOURCE_RTOS 1
