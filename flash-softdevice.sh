#! /bin/bash

probe-rs erase --chip nrf52832_xxAA --allow-erase-all
probe-rs download --verify --format hex --chip nRF52840_xxAA ./src/blobs/nrf-softdevice/s132_nrf52_7.3.0_softdevice.hex