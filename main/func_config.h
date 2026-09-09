#define SINGLE_ACTION_TIME_MS 3000
#define IMU_DATA_F 26
#define IMU_DATA_LEN IMU_DATA_F*3
// TODO: pending update
#define CENTRAL_MAC_ADDR      {0x38, 0x18, 0x2b, 0x13, 0x4f, 0xbc}
#define LEFT_LOWER_MAC_ADDR   {0x38, 0x18, 0x2b, 0x13, 0x4f, 0xbc}
#define LEFT_UPPER_MAC_ADDR   {0x38, 0x18, 0x2b, 0x18, 0xe4, 0x38}
#define RIGHT_LOWER_MAC_ADDR  {0x38, 0x18, 0x2b, 0x18, 0x4e, 0x50}
#define RIGHT_UPPER_MAC_ADDR  {0x38, 0x18, 0x2b, 0x18, 0x70, 0x18}

// 38:18:2b:18:4e:50
// 38:18:2b:18:e4:38
// 38:18:2b:13:4f:bc
// 38:18:2b:18:70:18
#define DATA_COLLECT_WIFI_SSID "csy"
#define DATA_COLLECT_WIFI_PASSWORD "55555555"
#define DATA_COLLECT_IP "172.20.10.3"
#define DATA_COLLECT_PORT "5000"
#define DATA_COLLECT_URL "http://" DATA_COLLECT_IP ":" DATA_COLLECT_PORT "/imu"