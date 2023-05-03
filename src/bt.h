#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h> // host controller interface
#include <zephyr/bluetooth/services/bas.h> // battery service GATT
#include <zephyr/settings/settings.h>

/* Bluetooth */
/* UUID of the Remote Service */
// Project ID: 065 (3rd entry)
// MFG ID = 0x02DF (4th entry)
#define BT_UUID_REMOTE_SERV_VAL \
        BT_UUID_128_ENCODE(0xe9ea0000, 0xe19b, 0x0065, 0x02DF, 0xc7907585fc48)
/* UUID of the Data Characteristic */
#define BT_UUID_REMOTE_DATA_CHRC_VAL \
        BT_UUID_128_ENCODE(0xe9ea0001, 0xe19b, 0x0065, 0x02DF, 0xc7907585fc48)
/* UUID of the Message Characteristic */
#define BT_UUID_REMOTE_MESSAGE_CHRC_VAL \
        BT_UUID_128_ENCODE(0xe9ea0002, 0xe19b, 0x0065, 0x02DF, 0xc7907585fc48)
#define BT_UUID_REMOTE_SERVICE 			BT_UUID_DECLARE_128(BT_UUID_REMOTE_SERV_VAL)
#define BT_UUID_REMOTE_DATA_CHRC 		BT_UUID_DECLARE_128(BT_UUID_REMOTE_DATA_CHRC_VAL)
#define BT_UUID_REMOTE_MESSAGE_CHRC 	BT_UUID_DECLARE_128(BT_UUID_REMOTE_MESSAGE_CHRC_VAL)

// capture notifications state
enum bt_data_notifications_enabled {
  BT_DATA_NOTIFICATIONS_ENABLED,
  BT_DATA_NOTIFICATIONS_DISABLED,
};

// create a struct of all BLE callbacks
struct bt_remote_srv_cb {
  void (*notif_changed)(enum bt_data_notifications_enabled status);
  void (*data_rx)(struct bt_conn *conn, const uint8_t *const data, uint16_t len);
};