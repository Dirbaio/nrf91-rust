#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

// must be kept first.
mod fmt;

mod modem;

use core::mem::MaybeUninit;
use core::slice;

use embassy_executor::Spawner;
use embassy_nrf::gpio::{AnyPin, Level, Output, OutputDrive, Pin};
use embassy_nrf::pac;
use embassy_time::Timer;
use futures::future::pending;
use static_cell::make_static;
use {defmt_rtt as _, panic_probe as _};

use crate::modem::*;

#[embassy_executor::task]
async fn modem_task(runner: Runner<'static>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn blink_task(pin: AnyPin) {
    let mut led = Output::new(pin, Level::Low, OutputDrive::Standard);
    loop {
        led.set_high();
        Timer::after_millis(100).await;
        led.set_low();
        Timer::after_millis(100).await;
    }
}

extern "C" {
    static __start_ipc: u8;
    static __end_ipc: u8;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    info!("Hello World!");

    unwrap!(spawner.spawn(blink_task(p.P0_02.degrade())));

    let spu = unsafe { &*pac::SPU::ptr() };
    info!("Setting IPC RAM as nonsecure...");
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

    let ipc_mem = unsafe {
        let ipc_start = &__start_ipc as *const u8 as *mut MaybeUninit<u8>;
        let ipc_end = &__end_ipc as *const u8 as *mut MaybeUninit<u8>;
        let ipc_len = ipc_end.offset_from(ipc_start) as usize;
        slice::from_raw_parts_mut(ipc_start, ipc_len)
    };

    let state = make_static!(State::new());
    let (dev, mut control, runner) = modem::new(state, ipc_mem).await;
    unwrap!(spawner.spawn(modem_task(runner)));

    let mut buf = *b"AT+CFUN?";

    control.send_at(&mut buf);
    info!("AT command sent");

    // needed so we don't deallocate `buf`, because we're not copying it into IPC MEM yet.
    pending::<()>().await;
}
