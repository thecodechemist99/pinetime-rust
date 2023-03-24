use embassy_nrf::{
    pac::{
        FICR,
        RADIO,
    },
    peripherals::TIMER2,
};

use rubble::{config::Config};
use rubble::gatt::BatteryServiceAttrs;
use rubble::l2cap::{BleChannelMap, L2CAPState};
// use rubble::link::ad_structure::AdStructure;
use rubble::link::queue::{PacketQueue, SimpleQueue};
use rubble::link::{DeviceAddress, LinkLayer, Responder, MIN_PDU_BUF};
use rubble::security::NoSecurity;
// use rubble::time::{Duration as RubbleDuration, Timer};
use rubble_nrf5x::radio::{BleRadio, PacketBuffer};
use rubble_nrf5x::timer::BleTimer;
use rubble_nrf5x::utils::get_device_address;

pub struct BleConfig {}

impl Config for BleConfig {
    type Timer = BleTimer<TIMER2>;
    type Transmitter = BleRadio;
    type ChannelMapper = BleChannelMap<BatteryServiceAttrs, NoSecurity>;
}

#[allow(unused)]
pub struct Bluetooth {
    /// BLE timer
    ble_timer: <BleConfig as rubble::config::Config>::Timer,

    /// Bluetooth device address
    device_address: DeviceAddress,

    /// Radio
    radio: BleRadio,

    /// BLE LinkLayer
    ble_ll: LinkLayer<BleConfig>,

    /// BLE Responder
    ble_r: Responder<BleConfig>,
}

impl Bluetooth {
    pub fn init() -> Self {
        // Initialize BLE timer on TIMER2
        let ble_timer = BleTimer::init(TIMER2);

        // Get bluetooth device address
        let device_address = get_device_address();
        defmt::info!("Bluetooth device address: {:?}", device_address);

        // Initialize radio
        let ble_tx_buf: PacketBuffer = [0; MIN_PDU_BUF];
        let ble_rx_buf: PacketBuffer = [0; MIN_PDU_BUF];


        let mut radio = BleRadio::new(
            RADIO,
            &FICR,
            &mut ble_tx_buf,
            &mut ble_rx_buf,
        );
        
        // Create bluetooth TX/RX queues
        let tx_queue = SimpleQueue::new();
        let rx_queue = SimpleQueue::new();

        let (tx, tx_cons) = tx_queue.split();
        let (rx_prod, rx) = rx_queue.split();
        
        // Create the actual BLE stack objects
        let mut ble_ll = LinkLayer::new(device_address, ble_timer);
        let ble_r = Responder::new(
            tx,
            rx,
            L2CAPState::new(BleChannelMap::with_attributes(BatteryServiceAttrs::new())),
        );

        Self { ble_timer, device_address, radio, ble_ll, ble_r }
    }

}