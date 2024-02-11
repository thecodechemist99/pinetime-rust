//! Bindings for closed-source methods from libheart.a
//!
//! Based on an implementation for Arduino: https://github.com/atc1441/HRS3300-Arduino-Library/tree/master

#[repr(C)]
pub enum hrs3300_msg_code_t {
    MSG_ALG_NOT_OPEN = 0x01,
    MSG_NO_TOUCH = 0x02,
    MSG_PPG_LEN_TOO_SHORT = 0x03,
    MSG_HR_READY = 0x04,
    MSG_ALG_TIMEOUT = 0x05,
    MSG_SETTLE = 0x06,
}

#[repr(C)]
pub enum hrs3300_bp_msg_code_t {
    MSG_BP_ALG_NOT_OPEN = 0x01,
    MSG_BP_NO_TOUCH = 0x02,
    MSG_BP_PPG_LEN_TOO_SHORT = 0x03,
    MSG_BP_READY = 0x04,
    MSG_BP_ALG_TIMEOUT = 0x05,
    MSG_BP_SETTLE = 0x06,
}

#[repr(C)]
pub struct hrs3300_results_t {
    pub alg_status: hrs3300_msg_code_t,
    pub data_cnt: cty::uint32_t,
    pub hr_result: cty::uint8_t,
    pub hr_result_qual: cty::uint8_t,
    pub object_flg: bool,
}

#[repr(C)]
pub struct hrs3300_bp_results_t {
    pub bp_alg_status: hrs3300_bp_msg_code_t,
    pub sbp: cty::uint8_t,
    pub dbp: cty::uint8_t,
    pub data_cnt: cty::uint32_t,
    pub hr_result: cty::uint8_t,
    pub object_flg: bool,
}

extern "C" {
    pub fn Hrs3300_alg_open();
    pub fn Hrs3300_bp_alg_open();
    pub fn Hrs3300_set_exinf(
        age: cty::uint8_t,
        height: cty::uint8_t,
        weight: cty::uint8_t,
        gender: cty::uint8_t,
        ref_sbp: cty::uint8_t,
        ref_dbp: cty::uint8_t,
    );
    pub fn Hrs3300_alg_get_results() -> hrs3300_results_t;
    pub fn Hrs3300_alg_get_bp_results() -> hrs3300_bp_results_t;
    pub fn Hrs3300_alg_send_data(
        new_raw_data: cty::int16_t,
        als_raw_data: cty::int16_t,
        gsen_data_x: cty::int16_t,
        gsen_data_y: cty::int16_t,
        gsen_data_z: cty::int16_t,
    ) -> bool;
    pub fn Hrs3300_bp_alg_send_data(new_raw_data: cty::int16_t) -> bool;
    pub fn Hrs3300_alg_close();
}

// typedef enum {
// 	MSG_ALG_NOT_OPEN = 0x01,
// 	MSG_NO_TOUCH = 0x02,
// 	MSG_PPG_LEN_TOO_SHORT = 0x03,
// 	MSG_HR_READY = 0x04,
// 	MSG_ALG_TIMEOUT = 0x05,
// 	MSG_SETTLE = 0x06
// } hrs3300_msg_code_t;

// typedef enum {
// 	MSG_BP_ALG_NOT_OPEN = 0x01,
// 	MSG_BP_NO_TOUCH = 0x02,
// 	MSG_BP_PPG_LEN_TOO_SHORT = 0x03,
// 	MSG_BP_READY = 0x04,
// 	MSG_BP_ALG_TIMEOUT = 0x05,
// 	MSG_BP_SETTLE = 0x06
// } hrs3300_bp_msg_code_t;

// typedef struct {
//   hrs3300_msg_code_t alg_status;
//   uint32_t           data_cnt;
//   uint8_t            hr_result;
//   uint8_t            hr_result_qual;
// 	bool               object_flg;
// } hrs3300_results_t;

// typedef struct {
//   hrs3300_msg_code_t alg_status;
//   uint32_t           data_cnt;
//   uint8_t            hr_result;
//   uint8_t            hr_result_qual;
// 	bool               object_flg;
// } hrs3300_results_t;

// hrs3300_results_t Hrs3300_alg_get_results(void);
// hrs3300_bp_results_t Hrs3300_alg_get_bp_results(void);
// extern bool Hrs3300_alg_open(void);
// extern void Hrs3300_bp_alg_open(void);
// extern bool Hrs3300_alg_send_data(int16_t new_raw_data, int16_t als_raw_data, int16_t gsen_data_x, int16_t gsen_data_y, int16_t gsen_data_z, uint16_t timer_time);
// extern bool Hrs3300_bp_alg_send_data(int16_t new_raw_data);
// extern void Hrs3300_alg_close(void);
