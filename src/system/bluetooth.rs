use nrf52832_hal::pac::{FICR, RADIO, TIMER1};
use rubble::{
    att::NoAttributes,
    config::Config,        
    l2cap::{BleChannelMap, L2CAPState},
    link::{
        ad_structure::AdStructure,
        Cmd,
        LinkLayer,
        queue::{PacketQueue, SimpleQueue},
        Responder, DeviceAddress,
    },
    security::NoSecurity,
    time::{Duration as RubbleDuration, Timer},
};
use rubble_nrf5x::{
    radio::{BleRadio, PacketBuffer},
    timer::BleTimer,
    utils::get_device_address,
};

pub struct BleConfig {}

impl Config for BleConfig {
    type Timer = BleTimer<TIMER1>;
    type Transmitter = BleRadio;
    type ChannelMapper = BleChannelMap<NoAttributes, NoSecurity>;
    type PacketQueue = &'static mut SimpleQueue;
}

pub struct Bluetooth {
    ble_ll: LinkLayer<BleConfig>,
    ble_r: Responder<BleConfig>,
    #[allow(unused)]
    device_address: DeviceAddress,
    radio: BleRadio,
}

impl Bluetooth {
    pub fn init(
        ble_timer: BleTimer<TIMER1>,
        radio_peripheral: RADIO,
        ficr_peripheral: &FICR,
        ble_tx_buf: &'static mut PacketBuffer,
        ble_rx_buf: &'static mut PacketBuffer,
        tx_queue: &'static mut SimpleQueue,
        rx_queue: &'static mut SimpleQueue,
    ) -> Self {
        // Get bluetooth device address
        let device_address = get_device_address();
        defmt::info!("Bluetooth device address: {:?}", defmt::Debug2Format(&device_address));
        
        // Initialize radio
        let mut radio = BleRadio::new(
            radio_peripheral,
            ficr_peripheral,
            ble_tx_buf,
            ble_rx_buf,
        );
                
        // Create bluetooth TX/RX queues
        let (tx, tx_cons) = tx_queue.split();
        let (rx_prod, rx) = rx_queue.split();
                
        // Create the actual BLE stack objects
        let mut ble_ll: LinkLayer<BleConfig> = LinkLayer::new(device_address, ble_timer);
        let ble_r: Responder<BleConfig> = Responder::new(
            tx,
            rx,
            L2CAPState::new(BleChannelMap::with_attributes(NoAttributes)),
        );

        // Send BLE advertisement and set up regular interrupt
        let next_update = ble_ll
            .start_advertise(
                // Values below 295ms seem not to work, for those
                // the timer interrupt is only triggered once
                RubbleDuration::millis(200),
                &[AdStructure::CompleteLocalName("Rusty PineTime")],
                &mut radio,
                tx_cons,
                rx_prod,
            )
            .unwrap();
    
        ble_ll.timer().configure_interrupt(next_update);

        Self { ble_ll, ble_r, device_address, radio }
    }

    pub fn handle_radio_interrupt(&mut self) -> bool {
        if let Some(cmd) = 
            self.radio.recv_interrupt(self.ble_ll.timer().now(), &mut self.ble_ll)
        {    
            self.reset_interrupt(cmd)
        } else {
            false
        }
    }

    pub fn handle_timer_interrupt(&mut self) -> bool {
        if self.ble_ll.timer().is_interrupt_pending() {
            self.ble_ll.timer().clear_interrupt();

            let cmd = self.ble_ll.update_timer(&mut self.radio);
            self.reset_interrupt(cmd)
        } else {
            false
        }
    }

    pub fn drain_packets(&mut self) {
        while self.ble_r.has_work() {
            self.ble_r.process_one().unwrap();
        }
    }

    fn reset_interrupt(&mut self, cmd: Cmd) -> bool {
        self.radio.configure_receiver(cmd.radio);
        self.ble_ll.timer().configure_interrupt(cmd.next_update);

        cmd.queued_work
    }
}