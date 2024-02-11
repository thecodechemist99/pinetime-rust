#! /bin/bash

probe-rs erase --chip nrf52832_xxAA --allow-erase-all
probe-rs download --verify --format hex --chip nRF52832_xxAA ./src/blobs/nrf-softdevice/s113_nrf52_7.2.0_softdevice.hex