CSRCS += \
	teledatics_gui_accessories.c \
	teledatics_gui_air_quality.c \
	teledatics_gui_ethernet.c \
	teledatics_gui_http.c \
	teledatics_gui_main.c \
	teledatics_gui_ppp.c \
	teledatics_gui_util.c \
	teledatics_gui_wifi.c \
	teledatics_gui_wifi_gw.c 
	
ifeq ($(CONFIG_DEBUG),y)
DEFINE	+=	-DDO_PROFILING
CSRCS += \
	teledatics_gui_metrics.c 
endif

include $(SDK_WIFI_COMMON)/wifi_common.make
