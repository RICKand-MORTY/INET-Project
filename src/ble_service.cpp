#include "mbed.h"
#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/GattServer.h"
#include "ble/GattCharacteristic.h"
#include "ble/GattService.h"

#include "ble_service.h"
#include "events/EventQueue.h"

// 使用 mbed BLE 的命名空间
using namespace ble;
using namespace events;

// =======================================================
//  全局 BLE 相关对象
// =======================================================

// 注意：变量名不要叫 ble，避免和命名空间 ble 冲突
static BLE &ble_instance = BLE::Instance();

// BLE 事件队列：专门用来处理 BLE 协议栈的异步事件
static EventQueue ble_event_queue(16 * EVENTS_EVENT_SIZE);

// 把 BLE 内部事件挂到队列里，后面 ble_process() 会定期 dispatch
static void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context)
{
    (void) context;
    ble_event_queue.call(mbed::callback(&ble_instance, &BLE::processEvents));
}

// 当前是否已连接（仅作状态记录和调试输出）
static bool ble_connected = false;

// =======================================================
//  UUID 规划：1 个 Service + 4 个 Characteristic
// =======================================================
//
// Service：0xA000
//   - 0xA010: state       (0=Normal, 1=Tremor, 2=Dyskinesia, 3=FOG)
//   - 0xA011: tremor_flag (0/1)
//   - 0xA012: dysk_flag   (0/1)
//   - 0xA013: fog_flag    (0/1)
//
static const uint16_t PARKINSON_SERVICE_UUID = 0xA000;

static const uint16_t STATE_CHAR_UUID       = 0xA010;
static const uint16_t TREMOR_CHAR_UUID      = 0xA011;
static const uint16_t DYSKINESIA_CHAR_UUID  = 0xA012;
static const uint16_t FOG_CHAR_UUID         = 0xA013;

// 当前缓存值
static uint8_t state_value       = 0;  // 0–3
static uint8_t tremor_value      = 0;  // 0/1
static uint8_t dyskinesia_value  = 0;  // 0/1
static uint8_t fog_value         = 0;  // 0/1

// 4 个 Characteristic，全都是 Read + Notify
static ReadOnlyGattCharacteristic<uint8_t> state_char(
    STATE_CHAR_UUID,
    &state_value,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

static ReadOnlyGattCharacteristic<uint8_t> tremor_char(
    TREMOR_CHAR_UUID,
    &tremor_value,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

static ReadOnlyGattCharacteristic<uint8_t> dyskinesia_char(
    DYSKINESIA_CHAR_UUID,
    &dyskinesia_value,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

static ReadOnlyGattCharacteristic<uint8_t> fog_char(
    FOG_CHAR_UUID,
    &fog_value,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

// 把 4 个特征放到同一个 service 里
static GattCharacteristic *parkinsons_chars[] = {
    (GattCharacteristic *)&state_char,
    (GattCharacteristic *)&tremor_char,
    (GattCharacteristic *)&dyskinesia_char,
    (GattCharacteristic *)&fog_char
};

static GattService parkinsons_service(
    PARKINSON_SERVICE_UUID,
    parkinsons_chars,
    sizeof(parkinsons_chars) / sizeof(parkinsons_chars[0])
);

// =======================================================
//  GAP 事件处理（连接 / 断开）
// =======================================================

class SimpleGapEventHandler : public Gap::EventHandler {
public:
    void onConnectionComplete(const ConnectionCompleteEvent &event) override
    {
        (void)event;
        ble_connected = true;
        printf("[BLE] Device connected\r\n");
    }

    void onDisconnectionComplete(const DisconnectionCompleteEvent &event) override
    {
        (void)event;
        ble_connected = false;
        printf("[BLE] Device disconnected, restart advertising\r\n");

        // 断开后重新广播
        ble_instance.gap().startAdvertising(LEGACY_ADVERTISING_HANDLE);
    }
};

static SimpleGapEventHandler gap_event_handler;

// =======================================================
//  BLE 初始化完成回调
// =======================================================

static void on_ble_init_complete(BLE::InitializationCompleteCallbackContext *params)
{
    if (params->error != BLE_ERROR_NONE) {
        printf("[BLE] init failed with error code %d\r\n", params->error);
        return;
    }

    printf("[BLE] init complete\r\n");

    // 注册 GAP 事件处理
    ble_instance.gap().setEventHandler(&gap_event_handler);

    // 注册 Service（带 4 个 characteristic）
    ble_error_t err = ble_instance.gattServer().addService(parkinsons_service);
    if (err != BLE_ERROR_NONE) {
        printf("[BLE] addService failed: %d\r\n", err);
        return;
    }

    // ---------- 设置广播 ----------
    AdvertisingParameters adv_params(
        advertising_type_t::CONNECTABLE_UNDIRECTED,
        adv_interval_t(millisecond_t(200))
    );

    uint8_t adv_buffer[64] = {0};
    AdvertisingDataBuilder adv_data_builder(adv_buffer);

    adv_data_builder.clear();
    adv_data_builder.setFlags();
    adv_data_builder.setName("PD-State");   // 手机上看到的设备名

    err = ble_instance.gap().setAdvertisingParameters(
        LEGACY_ADVERTISING_HANDLE, adv_params);
    printf("[BLE] setAdvertisingParameters err=%d\r\n", err);

    err = ble_instance.gap().setAdvertisingPayload(
        LEGACY_ADVERTISING_HANDLE,
        adv_data_builder.getAdvertisingData()
    );
    printf("[BLE] setAdvertisingPayload err=%d\r\n", err);

    err = ble_instance.gap().startAdvertising(LEGACY_ADVERTISING_HANDLE);
    printf("[BLE] Advertising started, err=%d\r\n", err);
}

// =======================================================
//  对外接口实现
// =======================================================

void ble_init(void)
{
    if (ble_instance.hasInitialized()) {
        printf("[BLE] ble_instance already initialized\r\n");
        return;
    }

    // 告诉 BLE 协议栈：有事件就调用 schedule_ble_events()，丢进 ble_event_queue
    ble_instance.onEventsToProcess(schedule_ble_events);


    ble_error_t err = ble_instance.init(on_ble_init_complete);
    if (err != BLE_ERROR_NONE) {
        printf("[BLE] ble.init() failed with error: %d\r\n", err);
        return;
    }

    // 主动跑一小段事件循环，把初始化“催”出来
    // （纯 CPU 自旋，不加延时）
    for (int i = 0; i < 1000 && !ble_instance.hasInitialized(); ++i) {
        ble_instance.processEvents();
    }

    printf("[BLE] after spin, hasInitialized() = %d\r\n",ble_instance.hasInitialized() ? 1 : 0);
    printf("[BLE] init() called, waiting for completion...\r\n");
}

void ble_process(void)
{
    // 非阻塞处理 BLE 事件：如果队列里有事件就处理一个，没有就立刻返回
    ble_event_queue.dispatch(0);

    // 保险起见，再手动跑一下底层事件循环
    ble_instance.processEvents();
}

void ble_update(int state,
                int tremor_flag,
                int dyskinesia_flag,
                int fog_flag)
{
    // 1) 归一化输入参数
    if (state < 0) state = 0;
    if (state > 3) state = 3;

    state_value      = (uint8_t)state;
    tremor_value     = tremor_flag     ? 1 : 0;
    dyskinesia_value = dyskinesia_flag ? 1 : 0;
    fog_value        = fog_flag        ? 1 : 0;

    if (!ble_instance.hasInitialized()) {
        return;
    }

    GattServer &server = ble_instance.gattServer();

    // 2) 依次写入 4 个 characteristic
    server.write(state_char.getValueHandle(),
                 &state_value, sizeof(state_value));
    server.write(tremor_char.getValueHandle(),
                 &tremor_value, sizeof(tremor_value));
    server.write(dyskinesia_char.getValueHandle(),
                 &dyskinesia_value, sizeof(dyskinesia_value));
    server.write(fog_char.getValueHandle(),
                 &fog_value, sizeof(fog_value));

    printf("[BLE] Update: state=%d, T=%d, D=%d, F=%d\r\n",
           state_value, tremor_value, dyskinesia_value, fog_value);
}
