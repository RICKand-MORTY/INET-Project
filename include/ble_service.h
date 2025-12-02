// void ble_init(void);
// initialize BLE module, concrete implementation depends on BlueNRG HAL
// In PlatformIO, need to manually port ST's BLE library

// void ble_update(int state);
// 0: Normal, 1: Tremor, 2: Dyskinesia, 3: FOG

#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 BLE（创建 service / characteristic 并开始广播）
void ble_init(void);

// 主循环里周期性调用，用于处理 BLE 事件（连接/断开等）
void ble_process(void);

/*
 * 更新 BLE 中的 4 个 characteristic：
 *   state:
 *      0 = Normal
 *      1 = Tremor
 *      2 = Dyskinesia
 *      3 = FOG
 *
 *   tremor_flag / dyskinesia_flag / fog_flag:
 *      0 = 未检测到
 *      1 = 检测到
 *
 * 约定：
 *   - 三个 flag 都是 0 时，代表“正常”
 *   - 可以同时多个为 1，表示多个症状同时出现
 */
void ble_update(int state,
                int tremor_flag,
                int dyskinesia_flag,
                int fog_flag);

#ifdef __cplusplus
}
#endif

#endif // BLE_SERVICE_H