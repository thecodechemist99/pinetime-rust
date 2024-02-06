//! Bluetooth module

// Core
use core::mem;

// BLE
use nrf_softdevice::{
    self,
    ble::advertisement_builder::{
        Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload, ServiceList, ServiceUuid16,
    },
    raw, Config,
};

// Others
use chrono::{NaiveDate, NaiveDateTime, NaiveTime};

pub static ADV_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
    .flags(&[Flag::GeneralDiscovery, Flag::LE_Only])
    .services_16(ServiceList::Complete, &[ServiceUuid16::BATTERY])
    .full_name("PineTime")
    .build();

pub static SCAN_DATA: LegacyAdvertisementPayload = LegacyAdvertisementBuilder::new()
    .services_16(ServiceList::Complete, &[ServiceUuid16::BATTERY])
    .build();

#[nrf_softdevice::gatt_server]
pub struct Server {
    pub bas: BatteryService,
}

#[nrf_softdevice::gatt_service(uuid = "180f")]
pub struct BatteryService {
    #[characteristic(uuid = "2a19", read, notify)]
    pub battery_level: u8,
}

pub async fn generate_config() -> Config {
    Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 6,
            event_length: 24,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 256 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
            attr_tab_size: raw::BLE_GATTS_ATTR_TAB_SIZE_DEFAULT,
        }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 3,
            central_role_count: 3,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: b"PineTime" as *const u8 as _,
            current_len: 9,
            max_len: 9,
            write_perm: unsafe { mem::zeroed() },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                raw::BLE_GATTS_VLOC_STACK as u8,
            ),
        }),
        ..Default::default()
    }
}

pub fn cts_get_epoch(buf: &[u8; 10]) -> NaiveDateTime {
    let year = u16::from_le_bytes(buf[..2].try_into().ok().unwrap()) as i32;
    let month = buf[2] as u32;
    let day = buf[3] as u32;
    let hour = buf[4] as u32;
    let minute = buf[5] as u32;
    let second = buf[6] as u32;
    // let day_of_week = buf[7] as u32;
    let fractions_256 = buf[8] as u32;

    NaiveDateTime::new(
        NaiveDate::from_ymd_opt(year, month, day).unwrap(),
        NaiveTime::from_hms_milli_opt(hour as u32, minute, second, fractions_256 / 256 * 1000)
            .unwrap(),
    )
}
