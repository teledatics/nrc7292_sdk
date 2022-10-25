DEFINE	+= -DCONFIG_HTTPD_WS_SUPPORT 
DEFINE	+= -DCONFIG_LOG_DEFAULT_LEVEL=0
CSRCS += \
	httpd_ws.c \
	httpd_sess.c \
	httpd_parse.c \
	httpd_uri.c \
	httpd_txrx.c \
	httpd_main.c \
	ctrl_sock.c 
