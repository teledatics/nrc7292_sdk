# COREFILES, CORE4FILES: The minimum set of files needed for lwIP.
COREFILES	= \
	init.c \
	def.c \
	dns.c \
	inet_chksum.c \
	ip.c \
	mem.c \
	memp.c \
	netif.c \
	pbuf.c \
	raw.c \
	stats.c \
	sys.c \
	altcp.c \
	altcp_alloc.c \
	altcp_tcp.c \
	tcp.c \
	tcp_in.c \
	tcp_out.c \
	timeouts.c \
	udp.c

CORE4FILES	= \
	autoip.c \
	dhcp.c \
	etharp.c \
	icmp.c \
	igmp.c \
	ip4_frag.c \
	ip4.c \
	ip4_addr.c

# APIFILES: The files which implement the sequential and socket APIs.
APIFILES= \
	api_lib.c \
	api_msg.c \
	err.c \
	if_api.c \
	netbuf.c \
	netdb.c \
	netifapi.c \
	sockets.c \
	tcpip.c

# NETIFFILES: Files implementing various generic network interface functions
NETIFFILES= \
	ethernet.c \
	bridgeif.c \
	bridgeif_fdb.c \
	slipif.c

# PPPFILES: PPP
PPPFILES= \
	auth.c \
	ccp.c \
	chap-md5.c \
	chap_ms.c \
	chap-new.c \
	demand.c \
	eui64.c \
	fsm.c \
	ipcp.c \
	ipv6cp.c \
	lcp.c \
	magic.c \
	mppe.c \
	multilink.c \
	ppp.c \
	pppapi.c \
	pppcrypt.c \
	pppoe.c \
	pppol2tp.c \
	pppos.c \
	upap.c \
	vj.c

# PORTING_FILES
LWIP_PORTING = \
	sys_arch.c \
	wlif.c \
	nrc_ping.c \
	nrc_iperf.c \
	nrc_wifi.c

ifeq ($(CONFIG_ETHERNET), y)
LWIP_PORTING_ETH = \
	nrc_eth_if.c
endif

ifeq ($(CONFIG_ESP_HOSTED), y)
LWIP_PORTING_ESP_HOSTED = \
	nrc_esp32_if.c
endif

LWIP_APPS= \
	dhcpserver.c \
	captdns.c \
	nrc_iperf_tcp.c\
	nrc_iperf_udp.c\
	ping.c

# add files to CSRCS list
CSRCS += \
	$(COREFILES) \
	$(CORE4FILES) \
	$(APIFILES) \
	$(NETIFFILES) \
	$(PPPFILES) \
	$(LWIP_PORTING)\
	$(LWIP_APPS) \
	$(LWIP_PORTING_ETH) \
	$(LWIP_PORTING_ESP_HOSTED)
