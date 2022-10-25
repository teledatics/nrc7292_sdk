# crypto
CRYPTO_SRCS	+= \
    aes.c \
    aesni.c \
    arc4.c \
    aria.c \
    asn1parse.c \
    asn1write.c \
    tls_base64.c \
    bignum.c \
    blowfish.c \
    camellia.c \
    ccm.c \
    chacha20.c \
    chachapoly.c \
    cipher.c \
    cipher_wrap.c \
    cmac.c \
    ctr_drbg.c \
    des.c \
    dhm.c \
    ecdh.c \
    ecdsa.c \
    ecjpake.c \
    ecp.c \
    ecp_curves.c \
    entropy.c \
    entropy_poll.c \
    error.c \
    gcm.c \
    havege.c \
    hkdf.c \
    hmac_drbg.c \
    md.c \
    md2.c \
    md4.c \
    md5.c \
    md_wrap.c \
    memory_buffer_alloc.c \
    nist_kw.c \
    oid.c \
    padlock.c \
    pem.c \
    pk.c \
    pk_wrap.c \
    pkcs12.c \
    pkcs5.c \
    pkparse.c \
    pkwrite.c \
    platform.c \
    platform_util.c \
    poly1305.c \
    ripemd160.c \
    rsa.c \
    rsa_internal.c \
    sha1.c \
    mbed_sha256.c \
    mbed_sha512.c \
    threading.c \
    timing.c \
    version.c \
    version_features.c \
    xtea.c \

# x509
X509_SRCS += \
    certs.c \
    pkcs11.c \
    x509.c \
    x509_create.c \
    x509_crl.c \
    x509_crt.c \
    x509_csr.c \
    x509write_crt.c \
    x509write_csr.c

# tls
TLS_SRCS += \
    debug.c \
    net_sockets.c \
    ssl_cache.c \
    ssl_ciphersuites.c \
    ssl_cli.c \
    ssl_cookie.c \
    ssl_srv.c \
    ssl_ticket.c \
    ssl_tls.c

# porting layer
PORTING_SRCS += \
	timing_alt.c

CSRCS += $(CRYPTO_SRCS)
CSRCS += $(X509_SRCS)
CSRCS += $(PORTING_SRCS)
ifeq ($(CONFIG_LWIP), y)
CSRCS += $(TLS_SRCS)
endif #CONFIG_LWIP
