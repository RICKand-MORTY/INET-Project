void ble_init(void);
// initialize BLE module, concrete implementation depends on BlueNRG HAL
// In PlatformIO, need to manually port ST's BLE library

void ble_update(int state);
// 0: Normal, 1: Tremor, 2: Dyskinesia, 3: FOG
