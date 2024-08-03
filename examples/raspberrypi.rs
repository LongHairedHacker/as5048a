// ------------------------------------------------------------------------------
// Copyright 2018 Uwe Arzt, mail@uwe-arzt.de
// SPDX-License-Identifier: Apache-2.0
// ------------------------------------------------------------------------------

use embedded_hal::digital::PinState;
use linux_embedded_hal as hal;

use as5048a::AS5048A;

use crate::hal::spidev::{self, SpidevOptions};
use crate::hal::{SpidevBus, SysfsPin};

use std::thread;
use std::time::Duration;

fn main() -> Result<(), std::io::Error> {
    let mut spi = SpidevBus::open("/dev/spidev0.0").unwrap();
    let options = SpidevOptions::new()
        .max_speed_hz(1_000_000)
        .mode(spidev::SpiModeFlags::SPI_MODE_1)
        .build();
    spi.configure(&options).unwrap();

    // CS pin on SparkFun Breakout
    let ncs = SysfsPin::new(8);
    ncs.export().unwrap();
    while !ncs.is_exported() {}
    let ncs = ncs.into_output_pin(PinState::High).unwrap();

    let mut as5048 = AS5048A::new(ncs);

    println!("AS5048A Example");
    loop {
        println!("-------------------------------------------------------------------------");

        let (diag, gain) = as5048.diag_gain(&mut spi).unwrap();
        println!("diag: {:08b} gain: {}", diag, gain);
        println!("magnitude: {:?}", as5048.magnitude(&mut spi));
        println!("angle: {:?}", as5048.angle(&mut spi));
        thread::sleep(Duration::from_millis(1000));
    }
}
