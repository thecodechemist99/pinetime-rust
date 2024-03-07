//! Bluetooth module

// Core
use core::mem;
use futures::{
    future::{select, Either},
    pin_mut,
};

// System
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, signal::Signal};
use embassy_time::Timer;

// BLE
use nrf_softdevice::{
    self,
    ble::{
        advertisement_builder::{
            Flag, LegacyAdvertisementBuilder, LegacyAdvertisementPayload, ServiceList,
            ServiceUuid16,
        },
        gatt_client, gatt_server, peripheral, Connection, DisconnectedError,
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
    pub hrs: HeartRateService,
    pub bas: BatteryService,
}

#[nrf_softdevice::gatt_service(uuid = "180d")]
pub struct HeartRateService {
    #[characteristic(uuid = "2a37", notify)]
    pub heart_rate_measurement: u8,
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
    const MAX_CONN: u8 = 2;

    /// Configure bluetooth on boot
    pub fn init(device_name: &str) -> Self {
        let sd = Softdevice::enable(&Config {
            clock: Some(raw::nrf_clock_lf_cfg_t {
                // Use external clock to reduce power consumption
                source: raw::NRF_CLOCK_LF_SRC_XTAL as u8,
                rc_ctiv: 0,
                rc_temp_ctiv: 0,
                accuracy: raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
            }),
            conn_gap: Some(raw::ble_gap_conn_cfg_t {
                conn_count: Self::MAX_CONN,
                event_length: 24,
            }),
            conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 256 }),
            gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t {
                // attr_tab_size: raw::BLE_GATTS_ATTR_TAB_SIZE_DEFAULT,
                attr_tab_size: 336,
            }),
            gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
                adv_set_count: 1,
                periph_role_count: Self::MAX_CONN,
            }),
            gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
                p_value: device_name.as_bytes().as_ptr() as _,
                current_len: device_name.len() as u16,
                max_len: device_name.len() as u16,
                write_perm: unsafe { mem::zeroed() },
                _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(
                    raw::BLE_GATTS_VLOC_STACK as u8,
                ),
            }),
            ..Default::default()
        });
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
        bas_update_channel: &Signal<ThreadModeRawMutex, u8>,
        hrs_update_channel: &Signal<ThreadModeRawMutex, u8>,
    ) -> Result<(), DisconnectedError> {
        // Advertise
        let adv = peripheral::ConnectableAdvertisement::ScannableUndirected {
            adv_data: &self.adv_data,
            scan_data: &self.scan_data,
        };

        // Set up connection
        let conn = peripheral::advertise_connectable(self.config.sd, adv, &self.config.hw_config)
            .await
            .unwrap();

        defmt::info!("advertising done!");

        // == Client setup ==

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

        // == Server setup ==

        // Send service notifications
        let hrs_future = self.notify_heart_rate(&hrs_update_channel, &conn);
        let bas_future = self.notify_battery(&bas_update_channel, &conn);

        // Run GATT server on the connection, this returns on disconnect.
        // Event enums (ServerEvents) are generated by nrf_softdevice::gatt_server
        // proc macro when applied to the Server struct above
        let gatt_future = gatt_server::run(&conn, &self.config.server, |e| match e {
            ServerEvent::Bas(e) => match e {
                BatteryServiceEvent::BatteryLevelCccdWrite { notifications } => {
                    defmt::info!("battery notifications: {}", notifications)
                }
            },
            ServerEvent::Hrs(e) => match e {
                HeartRateServiceEvent::HeartRateMeasurementCccdWrite { notifications } => {
                    defmt::info!("heart rate notifications: {}", notifications)
                } // HeartRateServiceEvent::HrmClientCharConfWrite(val) => {
                  //     defmt::info!(
                  //         "heart rate client characteristic configuration write value: {}",
                  //         val
                  //     )
                  // }
            },
        });

        pin_mut!(hrs_future);
        pin_mut!(bas_future);
        pin_mut!(gatt_future);

        // Use "select" to wait for either one of two futures to complete.
        // This means when the GATT server finishes operating, the notification
        // futures are automatically aborted.
        let services_future = select(hrs_future, bas_future);

        match select(services_future, gatt_future).await {
            // match select(bas_future, gatt_future).await {
            Either::Left((f, _)) => match f {
                Either::Left((_, _)) => Ok(()),
                Either::Right((_, _)) => Ok(()),
            },
            // Either::Left((_, _)) => Ok(()),
            Either::Right((e, _)) => Err(e),
        }
    }
    /// Send battery notifications
    #[allow(unused)]
    async fn notify_battery(
        &self,
        bas_update_channel: &Signal<ThreadModeRawMutex, u8>,
        connection: &Connection,
    ) {
        loop {
            // Check for battery level update
            if bas_update_channel.signaled() {
                // Update battery level
                let battery_level = bas_update_channel.wait().await;
                self.config
                    .server
                    .bas
                    .battery_level_set(&battery_level)
                    .unwrap();

                // Send notification
                self.config
                    .server
                    .bas
                    .battery_level_notify(connection, &battery_level);
            }
            // Sleep for five seconds
            Timer::after_secs(5).await
        }
    }
    /// Send heart rate notifications
    #[allow(unused)]
    async fn notify_heart_rate(
        &self,
        hrs_update_channel: &Signal<ThreadModeRawMutex, u8>,
        connection: &Connection,
    ) {
        loop {
            // Check for heart rate update
            if hrs_update_channel.signaled() {
                // Update battery level
                let heart_rate = hrs_update_channel.wait().await;
                self.config
                    .server
                    .hrs
                    .heart_rate_measurement_set(&heart_rate)
                    .unwrap();

                // Send notification
                self.config
                    .server
                    .hrs
                    .heart_rate_measurement_notify(connection, &heart_rate);
            }
            // Sleep for five seconds
            Timer::after_secs(5).await
        }
    }
}
