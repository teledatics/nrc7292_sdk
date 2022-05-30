#include "nrc_sdk.h"
#include "esp_err.h"

#ifndef ETH_H
#define ETH_H

/**
 * @brief Default configuration for Ethernet driver
 *
 */
#define ETH_DEFAULT_CONFIG(emac, ephy)   \
    {                                    \
        .mac = emac,                     \
        .phy = ephy,                     \
        .check_link_period_ms = 2000,    \
        .stack_input = NULL,             \
        .on_lowlevel_init_done = NULL,   \
        .on_lowlevel_deinit_done = NULL, \
        .read_phy_reg = NULL,            \
        .write_phy_reg = NULL,           \
    }
    
/**
* @brief Ethernet interface
*
*/
typedef enum {
    EMAC_DATA_INTERFACE_RMII,    /*!< Reduced Media Independent Interface */
    EMAC_DATA_INTERFACE_MII,     /*!< Media Independent Interface */
} eth_data_interface_t;

/**
* @brief Ethernet link status
*
*/
typedef enum {
    ETH_LINK_UP,  /*!< Ethernet link is up */
    ETH_LINK_DOWN /*!< Ethernet link is down */
} eth_link_t;

/**
* @brief Ethernet speed
*
*/
typedef enum {
    ETH_SPEED_10M,  /*!< Ethernet speed is 10Mbps */
    ETH_SPEED_100M, /*!< Ethernet speed is 100Mbps */
    ETH_SPEED_MAX   /*!< Max speed mode (for checking purpose) */
} eth_speed_t;

/**
* @brief Ethernet duplex mode
*
*/
typedef enum {
    ETH_DUPLEX_HALF,    /*!< Ethernet is in half duplex */
    ETH_DUPLEX_FULL,    /*!< Ethernet is in full duplex */
} eth_duplex_t;

/**
* @brief Ethernet Checksum
*/
typedef enum {
    ETH_CHECKSUM_SW, /*!< Ethernet checksum calculate by software */
    ETH_CHECKSUM_HW  /*!< Ethernet checksum calculate by hardware */
} eth_checksum_t;

/**
* @brief Internal ethernet EMAC's DMA available burst sizes
*/
typedef enum {
    ETH_DMA_BURST_LEN_32,
    ETH_DMA_BURST_LEN_16,
    ETH_DMA_BURST_LEN_8,
    ETH_DMA_BURST_LEN_4,
    ETH_DMA_BURST_LEN_2,
    ETH_DMA_BURST_LEN_1,
} eth_mac_dma_burst_len_t;

#define ETH_CRC_LEN         (4)  /* Ethernet frame CRC length */

/**
 * @brief Maximum Ethernet payload size
 *
 */
#define ETH_MAX_PAYLOAD_LEN (1500)

/**
 * @brief Minimum Ethernet payload size
 *
 */
#define ETH_MIN_PAYLOAD_LEN (46)

/**
 * @brief Ethernet frame header size: Dest addr(6 Bytes) + Src addr(6 Bytes) + length/type(2 Bytes)
 *
 */
#define ETH_HEADER_LEN (14)

/**
 * @brief Optional 802.1q VLAN Tag length
 *
 */
#define ETH_VLAN_TAG_LEN (4)

/**
 * @brief Jumbo frame payload size
 *
 */
#define ETH_JUMBO_FRAME_PAYLOAD_LEN (9000)

/**
 * @brief Maximum frame size (1522 Bytes)
 *
 */
#define ETH_MAX_PACKET_SIZE (ETH_HEADER_LEN + ETH_VLAN_TAG_LEN + ETH_MAX_PAYLOAD_LEN + ETH_CRC_LEN)

/**
 * @brief Minimum frame size (64 Bytes)
 *
 */
#define ETH_MIN_PACKET_SIZE (ETH_HEADER_LEN + ETH_MIN_PAYLOAD_LEN + ETH_CRC_LEN)

/**
* @brief Ethernet driver state
*
*/
typedef enum {
    ETH_STATE_LLINIT, /*!< Lowlevel init done */
    ETH_STATE_DEINIT, /*!< Deinit done */
    ETH_STATE_LINK,   /*!< Link status changed */
    ETH_STATE_SPEED,  /*!< Speed updated */
    ETH_STATE_DUPLEX, /*!< Duplex updated */
    ETH_STATE_PAUSE,  /*!< Pause ability updated */
} esp_eth_state_t;

#define ESP_ETH_PHY_ADDR_AUTO (-1)

/**
 * @brief Auto-negotiation controll commands
 *
 */
typedef enum {
    ESP_ETH_PHY_AUTONEGO_RESTART,
    ESP_ETH_PHY_AUTONEGO_EN,
    ESP_ETH_PHY_AUTONEGO_DIS,
    ESP_ETH_PHY_AUTONEGO_G_STAT,
} eth_phy_autoneg_cmd_t;

/**
* @brief Ethernet mediator
*
*/
typedef struct esp_eth_mediator_s esp_eth_mediator_t;

/**
* @brief Ethernet mediator
*
*/
struct esp_eth_mediator_s {
    /**
    * @brief Read PHY register
    *
    * @param[in] eth: mediator of Ethernet driver
    * @param[in] phy_addr: PHY Chip address (0~31)
    * @param[in] phy_reg: PHY register index code
    * @param[out] reg_value: PHY register value
    *
    * @return
    *       - ESP_OK: read PHY register successfully
    *       - ESP_FAIL: read PHY register failed because some error occurred
    *
    */
    esp_err_t (*phy_reg_read)(esp_eth_mediator_t *eth, uint32_t phy_addr, uint32_t phy_reg, uint32_t *reg_value);

    /**
    * @brief Write PHY register
    *
    * @param[in] eth: mediator of Ethernet driver
    * @param[in] phy_addr: PHY Chip address (0~31)
    * @param[in] phy_reg: PHY register index code
    * @param[in] reg_value: PHY register value
    *
    * @return
    *       - ESP_OK: write PHY register successfully
    *       - ESP_FAIL: write PHY register failed because some error occurred
    */
    esp_err_t (*phy_reg_write)(esp_eth_mediator_t *eth, uint32_t phy_addr, uint32_t phy_reg, uint32_t reg_value);

    /**
    * @brief Deliver packet to upper stack
    *
    * @param[in] eth: mediator of Ethernet driver
    * @param[in] buffer: packet buffer
    * @param[in] length: length of the packet
    *
    * @return
    *       - ESP_OK: deliver packet to upper stack successfully
    *       - ESP_FAIL: deliver packet failed because some error occurred
    *
    */
    esp_err_t (*stack_input)(esp_eth_mediator_t *eth, uint8_t *buffer, uint32_t length);

    /**
    * @brief Callback on Ethernet state changed
    *
    * @param[in] eth: mediator of Ethernet driver
    * @param[in] state: new state
    * @param[in] args: optional argument for the new state
    *
    * @return
    *       - ESP_OK: process the new state successfully
    *       - ESP_FAIL: process the new state failed because some error occurred
    *
    */
    esp_err_t (*on_state_changed)(esp_eth_mediator_t *eth, esp_eth_state_t state, void *args);
};

/**
* @brief Ethernet MAC
*
*/
typedef struct esp_eth_mac_s esp_eth_mac_t;

/**
* @brief Ethernet MAC
*
*/
struct esp_eth_mac_s {
    /**
    * @brief Set mediator for Ethernet MAC
    *
    * @param[in] mac: Ethernet MAC instance
    * @param[in] eth: Ethernet mediator
    *
    * @return
    *      - ESP_OK: set mediator for Ethernet MAC successfully
    *      - ESP_ERR_INVALID_ARG: set mediator for Ethernet MAC failed because of invalid argument
    *
    */
    esp_err_t (*set_mediator)(esp_eth_mac_t *mac, esp_eth_mediator_t *eth);

    /**
    * @brief Initialize Ethernet MAC
    *
    * @param[in] mac: Ethernet MAC instance
    *
    * @return
    *      - ESP_OK: initialize Ethernet MAC successfully
    *      - ESP_ERR_TIMEOUT: initialize Ethernet MAC failed because of timeout
    *      - ESP_FAIL: initialize Ethernet MAC failed because some other error occurred
    *
    */
    esp_err_t (*init)(esp_eth_mac_t *mac);

    /**
    * @brief Deinitialize Ethernet MAC
    *
    * @param[in] mac: Ethernet MAC instance
    *
    * @return
    *      - ESP_OK: deinitialize Ethernet MAC successfully
    *      - ESP_FAIL: deinitialize Ethernet MAC failed because some error occurred
    *
    */
    esp_err_t (*deinit)(esp_eth_mac_t *mac);

    /**
    * @brief Start Ethernet MAC
    *
    * @param[in] mac: Ethernet MAC instance
    *
    * @return
    *      - ESP_OK: start Ethernet MAC successfully
    *      - ESP_FAIL: start Ethernet MAC failed because some other error occurred
    *
    */
    esp_err_t (*start)(esp_eth_mac_t *mac);

    /**
    * @brief Stop Ethernet MAC
    *
    * @param[in] mac: Ethernet MAC instance
    *
    * @return
    *      - ESP_OK: stop Ethernet MAC successfully
    *      - ESP_FAIL: stop Ethernet MAC failed because some error occurred
    *
    */
    esp_err_t (*stop)(esp_eth_mac_t *mac);

    /**
    * @brief Transmit packet from Ethernet MAC
    *
    * @param[in] mac: Ethernet MAC instance
    * @param[in] buf: packet buffer to transmit
    * @param[in] length: length of packet
    *
    * @return
    *      - ESP_OK: transmit packet successfully
    *      - ESP_ERR_INVALID_ARG: transmit packet failed because of invalid argument
    *      - ESP_ERR_INVALID_STATE: transmit packet failed because of wrong state of MAC
    *      - ESP_FAIL: transmit packet failed because some other error occurred
    *
    */
    esp_err_t (*transmit)(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length);

    /**
    * @brief Receive packet from Ethernet MAC
    *
    * @param[in] mac: Ethernet MAC instance
    * @param[out] buf: packet buffer which will preserve the received frame
    * @param[out] length: length of the received packet
    *
    * @note Memory of buf is allocated in the Layer2, make sure it get free after process.
    * @note Before this function got invoked, the value of "length" should set by user, equals the size of buffer.
    *       After the function returned, the value of "length" means the real length of received data.
    *
    * @return
    *      - ESP_OK: receive packet successfully
    *      - ESP_ERR_INVALID_ARG: receive packet failed because of invalid argument
    *      - ESP_ERR_INVALID_SIZE: input buffer size is not enough to hold the incoming data.
    *                              in this case, value of returned "length" indicates the real size of incoming data.
    *      - ESP_FAIL: receive packet failed because some other error occurred
    *
    */
    esp_err_t (*receive)(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length);

    /**
    * @brief Read PHY register
    *
    * @param[in] mac: Ethernet MAC instance
    * @param[in] phy_addr: PHY chip address (0~31)
    * @param[in] phy_reg: PHY register index code
    * @param[out] reg_value: PHY register value
    *
    * @return
    *      - ESP_OK: read PHY register successfully
    *      - ESP_ERR_INVALID_ARG: read PHY register failed because of invalid argument
    *      - ESP_ERR_INVALID_STATE: read PHY register failed because of wrong state of MAC
    *      - ESP_ERR_TIMEOUT: read PHY register failed because of timeout
    *      - ESP_FAIL: read PHY register failed because some other error occurred
    *
    */
    esp_err_t (*read_phy_reg)(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t *reg_value);

    /**
    * @brief Write PHY register
    *
    * @param[in] mac: Ethernet MAC instance
    * @param[in] phy_addr: PHY chip address (0~31)
    * @param[in] phy_reg: PHY register index code
    * @param[in] reg_value: PHY register value
    *
    * @return
    *      - ESP_OK: write PHY register successfully
    *      - ESP_ERR_INVALID_STATE: write PHY register failed because of wrong state of MAC
    *      - ESP_ERR_TIMEOUT: write PHY register failed because of timeout
    *      - ESP_FAIL: write PHY register failed because some other error occurred
    *
    */
    esp_err_t (*write_phy_reg)(esp_eth_mac_t *mac, uint32_t phy_addr, uint32_t phy_reg, uint32_t reg_value);

    /**
    * @brief Set MAC address
    *
    * @param[in] mac: Ethernet MAC instance
    * @param[in] addr: MAC address
    *
    * @return
    *      - ESP_OK: set MAC address successfully
    *      - ESP_ERR_INVALID_ARG: set MAC address failed because of invalid argument
    *      - ESP_FAIL: set MAC address failed because some other error occurred
    *
    */
    esp_err_t (*set_addr)(esp_eth_mac_t *mac, uint8_t *addr);

    /**
    * @brief Get MAC address
    *
    * @param[in] mac: Ethernet MAC instance
    * @param[out] addr: MAC address
    *
    * @return
    *      - ESP_OK: get MAC address successfully
    *      - ESP_ERR_INVALID_ARG: get MAC address failed because of invalid argument
    *      - ESP_FAIL: get MAC address failed because some other error occurred
    *
    */
    esp_err_t (*get_addr)(esp_eth_mac_t *mac, uint8_t *addr);

    /**
    * @brief Set speed of MAC
    *
    * @param[in] ma:c Ethernet MAC instance
    * @param[in] speed: MAC speed
    *
    * @return
    *      - ESP_OK: set MAC speed successfully
    *      - ESP_ERR_INVALID_ARG: set MAC speed failed because of invalid argument
    *      - ESP_FAIL: set MAC speed failed because some other error occurred
    *
    */
    esp_err_t (*set_speed)(esp_eth_mac_t *mac, eth_speed_t speed);

    /**
    * @brief Set duplex mode of MAC
    *
    * @param[in] mac: Ethernet MAC instance
    * @param[in] duplex: MAC duplex
    *
    * @return
    *      - ESP_OK: set MAC duplex mode successfully
    *      - ESP_ERR_INVALID_ARG: set MAC duplex failed because of invalid argument
    *      - ESP_FAIL: set MAC duplex failed because some other error occurred
    *
    */
    esp_err_t (*set_duplex)(esp_eth_mac_t *mac, eth_duplex_t duplex);

    /**
    * @brief Set link status of MAC
    *
    * @param[in] mac: Ethernet MAC instance
    * @param[in] link: Link status
    *
    * @return
    *      - ESP_OK: set link status successfully
    *      - ESP_ERR_INVALID_ARG: set link status failed because of invalid argument
    *      - ESP_FAIL: set link status failed because some other error occurred
    *
    */
    esp_err_t (*set_link)(esp_eth_mac_t *mac, eth_link_t link);

    /**
    * @brief Set promiscuous of MAC
    *
    * @param[in] mac: Ethernet MAC instance
    * @param[in] enable: set true to enable promiscuous mode; set false to disable promiscuous mode
    *
    * @return
    *      - ESP_OK: set promiscuous mode successfully
    *      - ESP_FAIL: set promiscuous mode failed because some error occurred
    *
    */
    esp_err_t (*set_promiscuous)(esp_eth_mac_t *mac, bool enable);

    /**
    * @brief Enable flow control on MAC layer or not
    *
    * @param[in] mac: Ethernet MAC instance
    * @param[in] enable: set true to enable flow control; set false to disable flow control
    *
    * @return
    *      - ESP_OK: set flow control successfully
    *      - ESP_FAIL: set flow control failed because some error occurred
    *
    */
    esp_err_t (*enable_flow_ctrl)(esp_eth_mac_t *mac, bool enable);

    /**
    * @brief Set the PAUSE ability of peer node
    *
    * @param[in] mac: Ethernet MAC instance
    * @param[in] ability: zero indicates that pause function is supported by link partner; non-zero indicates that pause function is not supported by link partner
    *
    * @return
    *      - ESP_OK: set peer pause ability successfully
    *      - ESP_FAIL: set peer pause ability failed because some error occurred
    */
    esp_err_t (*set_peer_pause_ability)(esp_eth_mac_t *mac, uint32_t ability);

    /**
    * @brief Free memory of Ethernet MAC
    *
    * @param[in] mac: Ethernet MAC instance
    *
    * @return
    *      - ESP_OK: free Ethernet MAC instance successfully
    *      - ESP_FAIL: free Ethernet MAC instance failed because some error occurred
    *
    */
    esp_err_t (*del)(esp_eth_mac_t *mac);
};

/**
 * @brief RMII Clock Mode Options
 *
 */
typedef enum {
    /**
     * @brief Default values configured using Kconfig are going to be used when "Default" selected.
     *
     */
    EMAC_CLK_DEFAULT,

    /**
     * @brief Input RMII Clock from external. EMAC Clock GPIO number needs to be configured when this option is selected.
     *
     * @note MAC will get RMII clock from outside. Note that ESP32 only supports GPIO0 to input the RMII clock.
     *
     */
    EMAC_CLK_EXT_IN,

    /**
     * @brief Output RMII Clock from internal APLL Clock. EMAC Clock GPIO number needs to be configured when this option is selected.
     *
     */
    EMAC_CLK_OUT
} emac_rmii_clock_mode_t;

/**
 * @brief RMII Clock GPIO number Options
 *
 */
typedef enum {
    /**
     * @brief MAC will get RMII clock from outside at this GPIO.
     *
     * @note ESP32 only supports GPIO0 to input the RMII clock.
     *
     */
    EMAC_CLK_IN_GPIO = 0,

    /**
     * @brief Output RMII Clock from internal APLL Clock available at GPIO0
     *
     * @note GPIO0 can be set to output a pre-divided PLL clock (test only!). Enabling this option will configure GPIO0 to output a 50MHz clock.
     * In fact this clock doesn’t have directly relationship with EMAC peripheral. Sometimes this clock won’t work well with your PHY chip.
     * You might need to add some extra devices after GPIO0 (e.g. inverter). Note that outputting RMII clock on GPIO0 is an experimental practice.
     * If you want the Ethernet to work with WiFi, don’t select GPIO0 output mode for stability.
     *
     */
    EMAC_APPL_CLK_OUT_GPIO = 0,

    /**
     * @brief Output RMII Clock from internal APLL Clock available at GPIO16
     *
     */
    EMAC_CLK_OUT_GPIO = 16,

    /**
     * @brief Inverted Output RMII Clock from internal APLL Clock available at GPIO17
     *
     */
    EMAC_CLK_OUT_180_GPIO = 17
} emac_rmii_clock_gpio_t;

/**
 * @brief Ethernet MAC Clock Configuration
 *
 */
typedef union {
    struct {
        // MII interface is not fully implemented...
        // Reserved for GPIO number, clock source, etc. in MII mode
    } mii; /*!< EMAC MII Clock Configuration */
    struct {
        emac_rmii_clock_mode_t clock_mode; /*!< RMII Clock Mode Configuration */
        emac_rmii_clock_gpio_t clock_gpio; /*!< RMII Clock GPIO Configuration */
    } rmii; /*!< EMAC RMII Clock Configuration */
} eth_mac_clock_config_t;

/**
* @brief Ethernet PHY
*
*/
typedef struct esp_eth_phy_s esp_eth_phy_t;

/**
* @brief Ethernet PHY
*
*/
struct esp_eth_phy_s {
    /**
    * @brief Set mediator for PHY
    *
    * @param[in] phy: Ethernet PHY instance
    * @param[in] mediator: mediator of Ethernet driver
    *
    * @return
    *      - ESP_OK: set mediator for Ethernet PHY instance successfully
    *      - ESP_ERR_INVALID_ARG: set mediator for Ethernet PHY instance failed because of some invalid arguments
    *
    */
    esp_err_t (*set_mediator)(esp_eth_phy_t *phy, esp_eth_mediator_t *mediator);

    /**
    * @brief Software Reset Ethernet PHY
    *
    * @param[in] phy: Ethernet PHY instance
    *
    * @return
    *      - ESP_OK: reset Ethernet PHY successfully
    *      - ESP_FAIL: reset Ethernet PHY failed because some error occurred
    *
    */
    esp_err_t (*reset)(esp_eth_phy_t *phy);

    /**
    * @brief Hardware Reset Ethernet PHY
    *
    * @note Hardware reset is mostly done by pull down and up PHY's nRST pin
    *
    * @param[in] phy: Ethernet PHY instance
    *
    * @return
    *      - ESP_OK: reset Ethernet PHY successfully
    *      - ESP_FAIL: reset Ethernet PHY failed because some error occurred
    *
    */
    esp_err_t (*reset_hw)(esp_eth_phy_t *phy);

    /**
    * @brief Initialize Ethernet PHY
    *
    * @param[in] phy: Ethernet PHY instance
    *
    * @return
    *      - ESP_OK: initialize Ethernet PHY successfully
    *      - ESP_FAIL: initialize Ethernet PHY failed because some error occurred
    *
    */
    esp_err_t (*init)(esp_eth_phy_t *phy);

    /**
    * @brief Deinitialize Ethernet PHY
    *
    * @param[in] phy: Ethernet PHY instance
    *
    * @return
    *      - ESP_OK: deinitialize Ethernet PHY successfully
    *      - ESP_FAIL: deinitialize Ethernet PHY failed because some error occurred
    *
    */
    esp_err_t (*deinit)(esp_eth_phy_t *phy);

    /**
    * @brief Configure auto negotiation
    *
    * @param[in] phy: Ethernet PHY instance
    * @param[in] cmd: Configuration command, it is possible to Enable (restart), Disable or get current status
    *                   of PHY auto negotiation
    * @param[out] autonego_en_stat: Address where to store current status of auto negotiation configuration
    *
    * @return
    *      - ESP_OK: restart auto negotiation successfully
    *      - ESP_FAIL: restart auto negotiation failed because some error occurred
    *      - ESP_ERR_INVALID_ARG: invalid command
    *
    */
    esp_err_t (*autonego_ctrl)(esp_eth_phy_t *phy, eth_phy_autoneg_cmd_t cmd, bool *autonego_en_stat);

    /**
    * @brief Get Ethernet PHY link status
    *
    * @param[in] phy: Ethernet PHY instance
    *
    * @return
    *      - ESP_OK: get Ethernet PHY link status successfully
    *      - ESP_FAIL: get Ethernet PHY link status failed because some error occurred
    *
    */
    esp_err_t (*get_link)(esp_eth_phy_t *phy);

    /**
    * @brief Power control of Ethernet PHY
    *
    * @param[in] phy: Ethernet PHY instance
    * @param[in] enable: set true to power on Ethernet PHY; ser false to power off Ethernet PHY
    *
    * @return
    *      - ESP_OK: control Ethernet PHY power successfully
    *      - ESP_FAIL: control Ethernet PHY power failed because some error occurred
    *
    */
    esp_err_t (*pwrctl)(esp_eth_phy_t *phy, bool enable);

    /**
    * @brief Set PHY chip address
    *
    * @param[in] phy: Ethernet PHY instance
    * @param[in] addr: PHY chip address
    *
    * @return
    *      - ESP_OK: set Ethernet PHY address successfully
    *      - ESP_FAIL: set Ethernet PHY address failed because some error occurred
    *
    */
    esp_err_t (*set_addr)(esp_eth_phy_t *phy, uint32_t addr);

    /**
    * @brief Get PHY chip address
    *
    * @param[in] phy: Ethernet PHY instance
    * @param[out] addr: PHY chip address
    *
    * @return
    *      - ESP_OK: get Ethernet PHY address successfully
    *      - ESP_ERR_INVALID_ARG: get Ethernet PHY address failed because of invalid argument
    *
    */
    esp_err_t (*get_addr)(esp_eth_phy_t *phy, uint32_t *addr);

    /**
    * @brief Advertise pause function supported by MAC layer
    *
    * @param[in] phy: Ethernet PHY instance
    * @param[out] addr: Pause ability
    *
    * @return
    *      - ESP_OK: Advertise pause ability successfully
    *      - ESP_ERR_INVALID_ARG: Advertise pause ability failed because of invalid argument
    *
    */
    esp_err_t (*advertise_pause_ability)(esp_eth_phy_t *phy, uint32_t ability);

    /**
    * @brief Sets the PHY to loopback mode
    *
    * @param[in] phy: Ethernet PHY instance
    * @param[in] enable: enables or disables PHY loopback
    *
    * @return
    *      - ESP_OK: PHY instance loopback mode has been configured successfully
    *      - ESP_FAIL: PHY instance loopback configuration failed because some error occurred
    *
    */
    esp_err_t (*loopback)(esp_eth_phy_t *phy, bool enable);

    /**
    * @brief Sets PHY speed mode
    *
    * @note Autonegotiation feature needs to be disabled prior to calling this function for the new
    *       setting to be applied
    *
    * @param[in] phy: Ethernet PHY instance
    * @param[in] speed: Speed mode to be set
    *
    * @return
    *      - ESP_OK: PHY instance speed mode has been configured successfully
    *      - ESP_FAIL: PHY instance speed mode configuration failed because some error occurred
    *
    */
    esp_err_t (*set_speed)(esp_eth_phy_t *phy, eth_speed_t speed);

    /**
    * @brief Sets PHY duplex mode
    *
    * @note Autonegotiation feature needs to be disabled prior to calling this function for the new
    *       setting to be applied
    *
    * @param[in] phy: Ethernet PHY instance
    * @param[in] duplex: Duplex mode to be set
    *
    * @return
    *      - ESP_OK: PHY instance duplex mode has been configured successfully
    *      - ESP_FAIL: PHY instance duplex mode configuration failed because some error occurred
    *
    */
    esp_err_t (*set_duplex)(esp_eth_phy_t *phy, eth_duplex_t duplex);

    /**
    * @brief Free memory of Ethernet PHY instance
    *
    * @param[in] phy: Ethernet PHY instance
    *
    * @return
    *      - ESP_OK: free PHY instance successfully
    *      - ESP_FAIL: free PHY instance failed because some error occurred
    *
    */
    esp_err_t (*del)(esp_eth_phy_t *phy);
};

/**
* @brief Ethernet PHY configuration
*
*/
typedef struct {
    int32_t phy_addr;             /*!< PHY address, set -1 to enable PHY address detection at initialization stage */
    uint32_t reset_timeout_ms;    /*!< Reset timeout value (Unit: ms) */
    uint32_t autonego_timeout_ms; /*!< Auto-negotiation timeout value (Unit: ms) */
    int reset_gpio_num;           /*!< Reset GPIO number, -1 means no hardware reset */
} eth_phy_config_t;

/**
 * @brief Default configuration for Ethernet PHY object
 *
 */
#define ETH_PHY_DEFAULT_CONFIG()           \
    {                                      \
        .phy_addr = ESP_ETH_PHY_ADDR_AUTO, \
        .reset_timeout_ms = 100,           \
        .autonego_timeout_ms = 4000,       \
        .reset_gpio_num = 5,               \
    }


/**
* @brief Handle of Ethernet driver
*
*/
typedef void *esp_eth_handle_t;

/**
* @brief Configuration of Ethernet driver
*
*/
typedef struct {
    /**
    * @brief Ethernet MAC object
    *
    */
    esp_eth_mac_t *mac;

    /**
    * @brief Ethernet PHY object
    *
    */
    esp_eth_phy_t *phy;

    /**
    * @brief Period time of checking Ethernet link status
    *
    */
    uint32_t check_link_period_ms;

    /**
     * @brief Configuration status of PHY autonegotiation feature
     *
     */
    bool auto_nego_en;

    /**
    * @brief Input frame buffer to user's stack
    *
    * @param[in] eth_handle: handle of Ethernet driver
    * @param[in] buffer: frame buffer that will get input to upper stack
    * @param[in] length: length of the frame buffer
    *
    * @return
    *      - ESP_OK: input frame buffer to upper stack successfully
    *      - ESP_FAIL: error occurred when inputting buffer to upper stack
    *
    */
//     esp_err_t (*stack_input)(esp_eth_handle_t eth_handle, uint8_t *buffer, uint32_t length, void *priv);
    nrc_err_t (*stack_input)(esp_eth_handle_t eth_handle, uint8_t *buffer, uint32_t length, void *priv);
    /**
    * @brief Callback function invoked when lowlevel initialization is finished
    *
    * @param[in] eth_handle: handle of Ethernet driver
    *
    * @return
    *       - ESP_OK: process extra lowlevel initialization successfully
    *       - ESP_FAIL: error occurred when processing extra lowlevel initialization
    */
    esp_err_t (*on_lowlevel_init_done)(esp_eth_handle_t eth_handle);

    /**
    * @brief Callback function invoked when lowlevel deinitialization is finished
    *
    * @param[in] eth_handle: handle of Ethernet driver
    *
    * @return
    *       - ESP_OK: process extra lowlevel deinitialization successfully
    *       - ESP_FAIL: error occurred when processing extra lowlevel deinitialization
    */
    esp_err_t (*on_lowlevel_deinit_done)(esp_eth_handle_t eth_handle);

    /**
    * @brief Read PHY register
    *
    * @note Usually the PHY register read/write function is provided by MAC (SMI interface),
    *       but if the PHY device is managed by other interface (e.g. I2C), then user needs to
    *       implement the corresponding read/write.
    *       Setting this to NULL means your PHY device is managed by MAC's SMI interface.
    *
    * @param[in] eth_handle: handle of Ethernet driver
    * @param[in] phy_addr: PHY chip address (0~31)
    * @param[in] phy_reg: PHY register index code
    * @param[out] reg_value: PHY register value
    *
    * @return
    *      - ESP_OK: read PHY register successfully
    *      - ESP_ERR_INVALID_ARG: read PHY register failed because of invalid argument
    *      - ESP_ERR_TIMEOUT: read PHY register failed because of timeout
    *      - ESP_FAIL: read PHY register failed because some other error occurred
    */
    esp_err_t (*read_phy_reg)(esp_eth_handle_t eth_handle, uint32_t phy_addr, uint32_t phy_reg, uint32_t *reg_value);

    /**
    * @brief Write PHY register
    *
    * @note Usually the PHY register read/write function is provided by MAC (SMI interface),
    *       but if the PHY device is managed by other interface (e.g. I2C), then user needs to
    *       implement the corresponding read/write.
    *       Setting this to NULL means your PHY device is managed by MAC's SMI interface.
    *
    * @param[in] eth_handle: handle of Ethernet driver
    * @param[in] phy_addr: PHY chip address (0~31)
    * @param[in] phy_reg: PHY register index code
    * @param[in] reg_value: PHY register value
    *
    * @return
    *      - ESP_OK: write PHY register successfully
    *      - ESP_ERR_INVALID_ARG: read PHY register failed because of invalid argument
    *      - ESP_ERR_TIMEOUT: write PHY register failed because of timeout
    *      - ESP_FAIL: write PHY register failed because some other error occurred
    */
    esp_err_t (*write_phy_reg)(esp_eth_handle_t eth_handle, uint32_t phy_addr, uint32_t phy_reg, uint32_t reg_value);
} esp_eth_config_t;

/**
* @brief Configuration of Ethernet MAC object
*
*/
typedef struct {
    uint32_t sw_reset_timeout_ms;        /*!< Software reset timeout value (Unit: ms) */
    uint32_t rx_task_stack_size;         /*!< Stack size of the receive task */
    uint32_t rx_task_prio;               /*!< Priority of the receive task */
    int smi_mdc_gpio_num;                /*!< SMI MDC GPIO number, set to -1 could bypass the SMI GPIO configuration */
    int smi_mdio_gpio_num;               /*!< SMI MDIO GPIO number, set to -1 could bypass the SMI GPIO configuration */
    uint32_t flags;                      /*!< Flags that specify extra capability for mac driver */
    eth_data_interface_t interface;      /*!< EMAC Data interface to PHY (MII/RMII) */
    eth_mac_clock_config_t clock_config; /*!< EMAC Interface clock configuration */
} eth_mac_config_t;

/**
 * @brief Default configuration for Ethernet MAC object
 *
 */
#define ETH_MAC_DEFAULT_CONFIG()                          \
    {                                                     \
        .sw_reset_timeout_ms = 100,                       \
        .rx_task_stack_size = 2048,                       \
        .rx_task_prio = 15,                               \
        .smi_mdc_gpio_num = 23,                           \
        .smi_mdio_gpio_num = 18,                          \
        .flags = 0,                                       \
        .interface = EMAC_DATA_INTERFACE_RMII,            \
        .clock_config =                                   \
        {                                                 \
            .rmii =                                       \
            {                                             \
                .clock_mode = EMAC_CLK_DEFAULT,           \
                .clock_gpio = EMAC_CLK_IN_GPIO            \
            }                                             \
        }                                                 \
    }

/**
* @brief Install Ethernet driver
*
* @param[in]  config: configuration of the Ethernet driver
* @param[out] out_hdl: handle of Ethernet driver
*
* @return
*       - ESP_OK: install esp_eth driver successfully
*       - ESP_ERR_INVALID_ARG: install esp_eth driver failed because of some invalid argument
*       - ESP_ERR_NO_MEM: install esp_eth driver failed because there's no memory for driver
*       - ESP_FAIL: install esp_eth driver failed because some other error occurred
*/
esp_err_t esp_eth_driver_install(const esp_eth_config_t *config, esp_eth_handle_t *out_hdl);

/**
* @brief Start Ethernet driver **ONLY** in standalone mode (i.e. without TCP/IP stack)
*
* @note This API will start driver state machine and internal software timer (for checking link status).
*
* @param[in] hdl handle of Ethernet driver
*
* @return
*       - ESP_OK: start esp_eth driver successfully
*       - ESP_ERR_INVALID_ARG: start esp_eth driver failed because of some invalid argument
*       - ESP_ERR_INVALID_STATE: start esp_eth driver failed because driver has started already
*       - ESP_FAIL: start esp_eth driver failed because some other error occurred
*/
esp_err_t esp_eth_start(esp_eth_handle_t hdl);

#ifdef ETH_DRIVER_ENC28J60

typedef spi_device_t spi_device_handle_t;

/**
 * @brief ENC28J60 specific configuration
 *
 */
typedef struct {
    spi_device_handle_t spi_hdl; /*!< Handle of SPI device driver */
    int int_gpio_num;            /*!< Interrupt GPIO number */
} eth_enc28j60_config_t;

esp_eth_mac_t *esp_eth_mac_new_enc28j60(const eth_enc28j60_config_t *enc28j60_config, const eth_mac_config_t *mac_config);
esp_eth_phy_t *esp_eth_phy_new_enc28j60(const eth_phy_config_t *config);

#endif

#endif /* ETH_H */
