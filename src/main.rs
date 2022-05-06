#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use core::mem::{self, MaybeUninit};
use core::slice;
use core::str;
use core::sync::atomic::{compiler_fence, fence, Ordering};

use defmt::assert_eq;
use defmt::unwrap;
use defmt::*;
use embassy::executor::Spawner;
use embassy::time::{Duration, Timer};
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::pac;
use embassy_nrf::Peripherals;
use futures::future::pending;

use defmt_rtt as _; // global logger
use panic_probe as _;

#[embassy::main]
async fn main(_spawner: Spawner, p: Peripherals) {
    info!("Hello World {:x}", core::mem::size_of::<ControlBlock>());

    let mut led = Output::new(p.P0_02, Level::Low, OutputDrive::Standard);
    led.set_high();

    /*
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
     */

    let mut modem = Modem::new();

    let mut buf = *b"AT+CFUN?";

    modem.send_at(&mut buf);

    loop {
        compiler_fence(Ordering::SeqCst);
        modem.process_ctrl();
        modem.process_data();
    }

    info!("handshake complete, modem is alive");
    pending::<()>().await;
}

struct Modem {
    rx_ctrl: *mut List,
    rx_data: *mut List,

    rx_seq_no: u16,
    tx_seq_no: u16,
}

impl Modem {
    pub fn new() -> Self {
        let ipc = unsafe { &*pac::IPC::ptr() };
        let power = unsafe { &*pac::POWER::ptr() };

        let mut cb = unsafe {
            CONTROL_BLOCK.write(mem::zeroed());
            CONTROL_BLOCK.assume_init_mut()
        };

        unsafe {
            cb.flags[2] = 1;
            cb.rx_base = &mut RX as *mut _ as _;
            cb.rx_size = RX_SIZE;
            cb.p_ctrl_list = &mut cb.ctrl_list;
            cb.p_data_list = &mut cb.data_list;
            cb.p_modem_descriptor = &mut cb.modem_descriptor;
            cb.p_trace = &mut cb.trace;
            cb.ctrl_list.len = 8;
            cb.data_list.len = 8;
        }

        info!("rx_base {:x}", cb.rx_base);
        info!("p_ctrl_list {:x}", cb.p_ctrl_list);
        info!("p_data_list {:x}", cb.p_data_list);
        info!("p_modem_descriptor {:x}", cb.p_modem_descriptor);
        info!("ctrl_msgs {:x}", &mut cb.ctrl_msgs as *mut _);
        info!("data_msgs {:x}", &mut cb.data_msgs as *mut _);

        ipc.inten.write(|w| unsafe { w.bits(0xFF) });
        ipc.gpmem[0].write(|w| unsafe { w.bits(cb as *mut _ as u32) });
        ipc.gpmem[1].write(|w| unsafe { w.bits(0) });

        for i in 0..8 {
            ipc.send_cnf[i].write(|w| unsafe { w.bits(1 << i) });
            ipc.receive_cnf[i].write(|w| unsafe { w.bits(1 << i) });
        }

        // POWER.LTEMODEM.STARTN = 0
        // The reg is missing in the PAC??
        // NOTE: 0x4xxx for NS, 0x5xxx for S
        let startn = 0x4000_5610 as *mut u32;
        unsafe {
            info!("startn = {}", startn.read_volatile());
            startn.write_volatile(0);
            info!("startn = {}", startn.read_volatile());
        }

        loop {
            compiler_fence(Ordering::SeqCst);

            let int = ipc.intpend.read();
            if int.receive0().is_pending() {
                info!("IPC 0");
                ipc.events_receive[0].reset();
            }
            if int.receive2().is_pending() {
                info!("IPC 2");
                ipc.events_receive[2].reset();

                let status = ipc.gpmem[1].read().bits();
                info!("IPC status: {:08x}", status);

                info!("version: {:08x}", cb.modem_descriptor.version);
                info!("ctrl_list: {:08x}", cb.modem_descriptor.ctrl_list);
                info!("data_list: {:08x}", cb.modem_descriptor.data_list);
                break;
            }
            if int.receive4().is_pending() {
                info!("IPC 4");
                ipc.events_receive[4].reset();
            }
            if int.receive6().is_pending() {
                info!("IPC 6");
                ipc.events_receive[6].reset();
            }
            if int.receive7().is_pending() {
                info!("IPC 7");
                ipc.events_receive[7].reset();
            }
        }

        assert_eq!(cb.modem_descriptor.version, 1);
        let rx_ctrl = unsafe { &mut *cb.modem_descriptor.ctrl_list };
        let rx_data = unsafe { &mut *cb.modem_descriptor.data_list };
        info!("rx_ctrl_len: {}", rx_ctrl.len);
        info!("rx_data_len: {}", rx_data.len);

        Self {
            rx_ctrl,
            rx_data,
            tx_seq_no: 0,
            rx_seq_no: 0,
        }
    }

    fn process_ctrl(&mut self) {
        let list = unsafe { &mut *self.rx_ctrl };
        self.process(list);
    }

    fn process_data(&mut self) {
        let list = unsafe { &mut *self.rx_data };
        self.process(list);
    }

    fn process(&mut self, list: &mut List) {
        for item in &mut list.items {
            if item.preamble & 0xFF == 0x01 && item.preamble >> 16 == self.rx_seq_no as _ {
                let msg = unsafe { &mut *item.message };
                info!("rx msg: {:?}", msg);

                if msg.id == 0x0003_0003 {
                    let u: &[u8] = unsafe { slice::from_raw_parts(msg.data as _, msg.data_len) };
                    let u = unsafe { str::from_utf8_unchecked(u) };
                    info!("AT command response received: '{}'", u);
                }
                item.preamble = 0x03;
                self.rx_seq_no = self.rx_seq_no.wrapping_add(1);
            }
        }
    }

    fn send(&mut self) {
        let mut cb = unsafe { CONTROL_BLOCK.assume_init_mut() };

        let mut msg = &mut cb.ctrl_msgs[0];
        msg.channel = 1; // ctrl

        let preamble = (self.tx_seq_no as u32) << 16 | 0x01;
        cb.ctrl_list.items[0] = ListItem {
            preamble,
            message: msg as *mut _ as _,
        };

        self.tx_seq_no = self.tx_seq_no.wrapping_add(1);

        fence(Ordering::SeqCst);

        let ipc = unsafe { &*pac::IPC::ptr() };
        ipc.tasks_send[1].write(|w| unsafe { w.bits(1) });
    }

    fn send_at(&mut self, cmd: &mut [u8]) {
        let mut cb = unsafe { CONTROL_BLOCK.assume_init_mut() };

        let mut msg = &mut cb.data_msgs[0];
        msg.channel = 2; // data
        msg.id = 0x0001_0003; // AT CMD
        msg.param_len = 4;
        msg.param[0] = 0xa7;
        msg.param[1] = 0x00;
        msg.param[2] = 0x00;
        msg.param[3] = 0x00;
        msg.data = cmd.as_mut_ptr() as _;
        msg.data_len = cmd.len();

        cb.ctrl_list.items[0].message = msg as *mut _ as _;

        fence(Ordering::SeqCst);

        cb.ctrl_list.items[0].preamble = (self.tx_seq_no as u32) << 16 | 0x01;
        self.tx_seq_no = self.tx_seq_no.wrapping_add(1);

        fence(Ordering::SeqCst);

        let ipc = unsafe { &*pac::IPC::ptr() };
        ipc.tasks_send[1].write(|w| unsafe { w.bits(1) });
    }
}

static mut CONTROL_BLOCK: MaybeUninit<ControlBlock> = MaybeUninit::uninit();

const RX_SIZE: usize = 4096;
const TX_SIZE: usize = 4096;
static mut RX: [u8; RX_SIZE] = [0; RX_SIZE];
static mut TX: [u8; TX_SIZE] = [0; RX_SIZE];

#[repr(C)]
struct ControlBlock {
    flags: [u8; 4],
    rx_base: *mut (),
    rx_size: usize,
    p_ctrl_list: *mut List,
    p_data_list: *mut List,
    p_modem_descriptor: *mut ModemDescriptor,
    p_trace: *mut Trace,
    unk: u32,

    modem_descriptor: ModemDescriptor,
    trace: Trace,
    ctrl_list: List,
    data_list: List,
    ctrl_msgs: [Message; 8],
    data_msgs: [Message; 8],
}

#[repr(C)]
struct ModemDescriptor {
    version: u32,
    ctrl_list: *mut List,
    data_list: *mut List,
    padding: [u32; 5],
}

#[repr(C)]
struct Trace {
    size: usize,
    base: *mut (),
    padding: [u32; 6],
}

#[repr(C)]
struct List {
    len: usize,
    items: [ListItem; 8],
}

#[repr(C)]
struct ListItem {
    /// top 16 bits: seqno
    /// bottom 8 bits:
    ///     0x01: sent
    ///     0x02: held
    ///     0x03: freed
    preamble: u32,
    message: *mut Message,
}

#[repr(C)]
#[derive(defmt::Format)]
struct Message {
    /// top 16 bits: operation
    id: u32,

    /// 1 = ctrl, 2 = data
    channel: u8,
    unk1: u8,
    unk2: u8,
    unk3: u8,

    data: *mut (),
    data_len: usize,
    param_len: usize,
    param: [u8; 44],
}
