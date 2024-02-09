//! Bluetooth module

// Core
use core::mem;

// System
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};

// BLE
use nrf_softdevice::{
    self,
    ble::{
        advertisement_builder::{
            Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload, ServiceList,
            ServiceUuid16,
        },
        gatt_client, gatt_server, peripheral, DisconnectedError,
    },
    raw, Config, Softdevice,
};

// Module
use super::time::TimeReference;

#[nrf_softdevice::gatt_client(uuid = "1805")]
struct CurrentTimeServiceClient {
    #[characteristic(uuid = "2a2b", read, write, notify)]
    current_time: [u8; 10],
}

#[nrf_softdevice::gatt_server]
pub struct Server {
    pub bas: BatteryService,
}

#[nrf_softdevice::gatt_service(uuid = "180f")]
pub struct BatteryService {
    #[characteristic(uuid = "2a19", read, notify)]
    pub battery_level: u8,
}

struct BluetoothConfig<'a> {
    /// BLE hardware configuration
    hw_config: peripheral::Config,
    /// Softdevice
    sd: &'a Softdevice,
    /// GATT server
    server: Server,
}

pub struct Bluetooth {
    /// Bluetooth configuration
    config: BluetoothConfig<'static>,
    /// Advertisement data
    adv_data: LegacyAdvertisementPayload,
    /// Scan data
    scan_data: LegacyAdvertisementPayload,
}

impl Bluetooth {
    /// Configure bluetooth on boot
    pub fn init(device_name: &str) -> Self {
        let sd_config = Config {
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
                p_value: device_name.as_bytes().as_ptr() as _,
                current_len: 9,
                max_len: 9,
                write_perm: unsafe { mem::zeroed() },
                _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                    raw::BLE_GATTS_VLOC_STACK as u8,
                ),
            }),
            ..Default::default()
        };
        let sd = Softdevice::enable(&sd_config);
        let server = Server::new(sd).unwrap();

        Self {
            config: BluetoothConfig {
                hw_config: peripheral::Config::default(),
                sd,
                server,
            },
            adv_data: LegacyAdvertisementBuilder::new()
                .flags(&[Flag::GeneralDiscovery, Flag::LE_Only])
                .services_16(ServiceList::Complete, &[ServiceUuid16::BATTERY])
                .full_name(device_name)
                .build(),
            scan_data: LegacyAdvertisementBuilder::new()
                .services_16(ServiceList::Complete, &[ServiceUuid16::BATTERY])
                .build(),
        }
    }
    /// Access softdevice
    pub fn get_sd(&self) -> &'static Softdevice {
        self.config.sd
    }
    /// Start GATT server
    pub async fn run_gatt_server(
        &mut self,
        cts_update_channel: &Signal<ThreadModeRawMutex, TimeReference>,
    ) -> DisconnectedError {
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data: &self.adv_data,
            scan_data: &self.scan_data,
        };
        let conn = peripheral::advertise_connectable(self.config.sd, adv, &self.config.hw_config)
            .await
            .unwrap();

        defmt::info!("advertising done!");

        let client: CurrentTimeServiceClient = gatt_client::discover(&conn).await.unwrap();

        // Update time via CTS
        let bytes = client.current_time_read().await.unwrap();
        cts_update_channel.signal(TimeReference::from_cts_bytes(&bytes));

        // Enable current time notifications from the peripheral
        client.current_time_cccd_write(true).await.unwrap();

        // Receive notifications
        gatt_client::run(&conn, &client, |e| match e {
            CurrentTimeServiceClientEvent::CurrentTimeNotification(bytes) => {
                defmt::info!("current time notification: {}", bytes);
                cts_update_channel.signal(TimeReference::from_cts_bytes(&bytes));
            }
        })
        .await;

        // Run GATT server on the connection, this returns on disconnect.
        // Event enums (ServerEvents) are generated by nrf_softdevice::gatt_server
        // proc macro when applied to the Server struct above
        gatt_server::run(&conn, &self.config.server, |e| match e {
            ServerEvent::Bas(e) => match e {
                BatteryServiceEvent::BatteryLevelCccdWrite { notifications } => {
                    defmt::info!("battery notifications: {}", notifications)
                }
            },
        })
        .await
    }
}
