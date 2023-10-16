#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

// must be kept first.
mod fmt;

mod modem;

use core::sync::atomic::{compiler_fence, fence, Ordering};
use core::{slice, str};

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::{pac, Peripherals};
use embassy_time::{Duration, Timer};
use futures::future::pending;
use static_cell::make_static;
use {defmt_rtt as _, panic_probe as _};

use crate::modem::*;

#[embassy_executor::task]
async fn modem_task(runner: Runner<'static>) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    info!("Hello World!");

    let mut led = Output::new(p.P0_02, Level::Low, OutputDrive::Standard);
    led.set_high();

    let spu = unsafe { &*pac::SPU::ptr() };
    info!("Setting all RAM as nonsecure...");
    for i in 0..32 {
        spu.ramregion[i].perm.write(|w| {
            w.execute().set_bit();
            w.write().set_bit();
            w.read().set_bit();
            w.secattr().clear_bit();
            w.lock().clear_bit();
            w
        })
    }

    spu.periphid[42].perm.write(|w| w.secattr().non_secure());
    info!("spu ipc perm: {:08x}", spu.periphid[42].perm.read().bits());

    let state = make_static!(State::new());
    let (dev, mut control, runner) = modem::new(state).await;

    let mut buf = *b"AT+CFUN?";

    control.send_at(&mut buf);

    loop {
        compiler_fence(Ordering::SeqCst);
        control.process_ctrl();
        control.process_data();
    }

    info!("handshake complete, modem is alive");
    pending::<()>().await;
}
