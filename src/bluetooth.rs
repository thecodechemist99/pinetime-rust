use nrf52832_hal::{
    pac::{
        FICR,
        RADIO,
        TIMER2,
    },
};

use rubble::{config::Config};
use rubble::gatt::BatteryServiceAttrs;
use rubble::l2cap::{BleChannelMap, L2CAPState};
use rubble::link::ad_structure::AdStructure;
use rubble::link::queue::{PacketQueue, SimpleQueue};
use rubble::link::{DeviceAddress, LinkLayer, Responder, MIN_PDU_BUF};
use rubble::security::NoSecurity;
use rubble::time::{Duration as RubbleDuration};
use rubble_nrf5x::radio::{BleRadio, PacketBuffer};
use rubble_nrf5x::timer::BleTimer;
use rubble_nrf5x::utils::get_device_address;

pub struct BleConfig {}

impl Config for BleConfig {
    type Timer = BleTimer<TIMER2>;
    type Transmitter = BleRadio;
    type ChannelMapper = BleChannelMap<BatteryServiceAttrs, NoSecurity>;
    type PacketQueue = &'static mut SimpleQueue;
}

#[allow(unused)]
pub struct BLE {
    /// BLE RX buffer
    ble_rx_buf: PacketBuffer,

    /// BLE TX buffer
    ble_tx_buf: PacketBuffer,

    /// Bluetooth device address
    device_address: DeviceAddress,

    /// Radio
    radio: BleRadio,

    /// BLE LinkLayer
    ble_ll: LinkLayer<BleConfig>,

    /// BLE Responder
    ble_r: Responder<BleConfig>,
}

impl BLE {
    /// Initialize BLE
    pub fn init(ficr: &FICR, radio: RADIO, timer: TIMER2) -> Self {
        // Initialize BLE timer on TIMER2
        let ble_timer = BleTimer::init(timer);

        // Get bluetooth device address
        let device_address = get_device_address();
        // defmt::info!("Bluetooth device address: {:?}", device_address);

        // Initialize radio
        let mut ble_tx_buf: PacketBuffer = [0; MIN_PDU_BUF];
        let mut ble_rx_buf: PacketBuffer = [0; MIN_PDU_BUF];

        let radio = BleRadio::new(
            radio,
            ficr,
            & mut ble_tx_buf,
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

        // Send advertisement and set up regular interrupt
        // let next_update = ble_ll
        //     .start_advertise(
        //         RubbleDuration::from_millis(200),
        //         &[AdStructure::CompleteLocalName("Rusty PineTime")],
        //         &mut radio,
        //         tx_cons,
        //         rx_prod,
        //     )
        //     .unwrap();

        // ble_ll.timer().configure_interrupt(next_update);

        Self { ble_rx_buf, ble_tx_buf, device_address, radio, ble_ll, ble_r }
    }
}