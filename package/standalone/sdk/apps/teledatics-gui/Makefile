# NOTE: max firmware binary size is ~934K, compress HTML to save space
COMPRESS_HTML := $(shell `lzop -9 -c sdk/apps/teledatics-gui/teledatics_gui.html | base64 - | sed 's/$$/\"/' | sed 's/^/\"/'   > sdk/apps/teledatics-gui/teledatics_gui_html_lzo.h`)

CSRCS += \
	teledatics_gui_accessories.c \
	teledatics_gui_air_quality.c \
	teledatics_gui_ethernet.c \
	teledatics_gui_http.c \
	teledatics_gui_main.c \
	teledatics_gui_ppp.c \
	teledatics_gui_util.c \
	teledatics_gui_wifi.c \
	teledatics_gui_wifi_gw.c \
	teledatics_lzo_decompress.c \
	pb_common.c \
	pb_decode.c \
	pb_encode.c \
	esp_hosted_config.pb.c
	
ifeq ($(CONFIG_DEBUG),y)
DEFINE	+=	-DDO_PROFILING
CSRCS += \
	teledatics_gui_metrics.c 
endif

include $(SDK_WIFI_COMMON)/wifi_common.make
