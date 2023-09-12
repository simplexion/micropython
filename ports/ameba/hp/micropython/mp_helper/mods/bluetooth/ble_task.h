#include <gap.h>

extern void *ble_task_handle;
void ble_task_init(void);
void ble_task_deinit(void);
T_APP_RESULT ble_gap_callback(uint8_t cb_type, void* p_cb_data);
