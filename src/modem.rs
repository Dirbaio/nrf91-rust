use core::future::poll_fn;
use core::marker::PhantomData;
use core::mem::{self, MaybeUninit};
use core::sync::atomic::{compiler_fence, fence, Ordering};
use core::task::Poll;
use core::{slice, str};

use embassy_net_driver_channel as ch;
use embassy_nrf::interrupt::InterruptExt;
use embassy_nrf::{interrupt, pac};
use embassy_sync::waitqueue::AtomicWaker;

const RX_SIZE: usize = 4096;
const MTU: usize = 1500;

pub type NetDriver<'a> = ch::Device<'a, MTU>;

static WAKER: AtomicWaker = AtomicWaker::new();
#[interrupt]
fn IPC() {
    let ipc = unsafe { &*pac::ns::IPC::ptr() };

    info!("irq");

    ipc.inten.write(|w| w);
    WAKER.wake();
}

struct Allocator<'a> {
    start: *mut u8,
    end: *mut u8,
    _phantom: PhantomData<&'a mut u8>,
}

impl<'a> Allocator<'a> {
    fn alloc_bytes(&mut self, size: usize) -> &'a mut [MaybeUninit<u8>] {
        // safety: both pointers come from the same allocation.
        let available_size = unsafe { self.end.offset_from(self.start) } as usize;
        if size > available_size {
            panic!("out of memory")
        }

        // safety: we've checked above this doesn't go out of bounds.
        let p = self.start;
        self.start = unsafe { p.add(size) };

        // safety: we've checked the pointer is in-bounds.
        unsafe { slice::from_raw_parts_mut(p as *mut _, size) }
    }

    fn alloc<T>(&mut self) -> &'a mut MaybeUninit<T> {
        let align = mem::align_of::<T>();
        let size = mem::size_of::<T>();

        let align_size = match (self.start as usize) % align {
            0 => 0,
            n => align - n,
        };

        // safety: both pointers come from the same allocation.
        let available_size = unsafe { self.end.offset_from(self.start) } as usize;
        if align_size + size > available_size {
            panic!("out of memory")
        }

        // safety: we've checked above this doesn't go out of bounds.
        let p = unsafe { self.start.add(align_size) };
        self.start = unsafe { p.add(size) };

        // safety: we've checked the pointer is aligned and in-bounds.
        unsafe { &mut *(p as *mut _) }
    }
}

pub async fn new<'a>(
    state: &'a mut State,
    shmem: &'a mut [MaybeUninit<u8>],
) -> (NetDriver<'a>, Control<'a>, Runner<'a>) {
    let shmem_len = shmem.len();
    let shmem_ptr = shmem.as_mut_ptr() as *mut u8;
    let mut alloc = Allocator {
        start: shmem_ptr,
        end: unsafe { shmem_ptr.add(shmem_len) },
        _phantom: PhantomData,
    };

    let ipc = unsafe { &*pac::ns::IPC::ptr() };
    let power = unsafe { &*pac::POWER::ptr() };

    let cb: &mut ControlBlock = alloc.alloc().write(unsafe { mem::zeroed() });
    let rx = alloc.alloc_bytes(RX_SIZE);

    cb.flags[2] = 1;
    cb.rx_base = rx.as_mut_ptr() as _;
    cb.rx_size = RX_SIZE;
    cb.p_ctrl_list = &mut cb.ctrl_list;
    cb.p_data_list = &mut cb.data_list;
    cb.p_modem_descriptor = &mut cb.modem_descriptor;
    cb.p_trace = &mut cb.trace;
    cb.ctrl_list.len = 8;
    cb.data_list.len = 8;

    info!("rx_base {:x}", cb.rx_base);
    info!("p_ctrl_list {:x}", cb.p_ctrl_list);
    info!("p_data_list {:x}", cb.p_data_list);
    info!("p_modem_descriptor {:x}", cb.p_modem_descriptor);
    info!("ctrl_msgs {:x}", &mut cb.ctrl_msgs as *mut _);
    info!("data_msgs {:x}", &mut cb.data_msgs as *mut _);

    //ipc.inten.write(|w| unsafe { w.bits(0xFF) });
    ipc.gpmem[0].write(|w| unsafe { w.bits(cb as *mut _ as u32) });
    ipc.gpmem[1].write(|w| unsafe { w.bits(0) });

    // connect task/event i to channel i
    for i in 0..8 {
        ipc.send_cnf[i].write(|w| unsafe { w.bits(1 << i) });
        ipc.receive_cnf[i].write(|w| unsafe { w.bits(1 << i) });
    }

    // POWER.LTEMODEM.STARTN = 0
    // The reg is missing in the PAC??
    let startn = unsafe { (power as *const _ as *mut u32).add(0x610 / 4) };
    unsafe {
        info!("startn = {}", startn.read_volatile());
        startn.write_volatile(0);
        info!("startn = {}", startn.read_volatile());
    }

    loop {
        compiler_fence(Ordering::SeqCst);
        if ipc.events_receive[0].read().bits() != 0 {
            info!("IPC 0");
            ipc.events_receive[0].reset();
        }
        if ipc.events_receive[2].read().bits() != 0 {
            info!("IPC 2");
            ipc.events_receive[2].reset();

            let status = ipc.gpmem[1].read().bits();
            info!("IPC status: {:08x}", status);

            info!("version: {:08x}", cb.modem_descriptor.version);
            info!("ctrl_list: {:08x}", cb.modem_descriptor.ctrl_list);
            info!("data_list: {:08x}", cb.modem_descriptor.data_list);
            break;
        }
    }

    ipc.intenset.write(|w| {
        w.receive0().set_bit();
        w.receive2().set_bit();
        w.receive4().set_bit();
        w
    });
    unsafe { interrupt::IPC.enable() };

    assert_eq!(cb.modem_descriptor.version, 1);
    let rx_ctrl = unsafe { &mut *cb.modem_descriptor.ctrl_list };
    let rx_data = unsafe { &mut *cb.modem_descriptor.data_list };
    info!("rx_ctrl_len: {}", rx_ctrl.len);
    info!("rx_data_len: {}", rx_data.len);

    let control = Control {
        cb,
        rx_ctrl,
        rx_data,
        tx_seq_no: 0,
        _phantom: PhantomData,
    };

    let (ch_runner, device) = ch::new(&mut state.ch, ch::driver::HardwareAddress::Ethernet([0; 6]));
    let state_ch = ch_runner.state_runner();

    let runner = Runner {
        ch: ch_runner,
        rx_ctrl,
        rx_data,
        rx_seq_no: 0,
    };

    (device, control, runner)
}

pub struct State {
    ch: ch::State<MTU, 4, 4>,
}

impl State {
    pub fn new() -> Self {
        Self { ch: ch::State::new() }
    }
}

pub struct Runner<'a> {
    ch: ch::Runner<'a, MTU>,
    rx_ctrl: *mut List,
    rx_data: *mut List,
    rx_seq_no: u16,
}

pub struct Control<'a> {
    cb: &'a mut ControlBlock,
    rx_ctrl: *mut List,
    rx_data: *mut List,

    tx_seq_no: u16,

    _phantom: PhantomData<&'a mut ()>,
}

impl<'a> Runner<'a> {
    pub async fn run(mut self) -> ! {
        poll_fn(|cx| {
            WAKER.register(cx.waker());
            self.poll();
            Poll::Pending
        })
        .await
    }

    fn poll(&mut self) {
        info!("poll!");
        let ipc = unsafe { &*pac::ns::IPC::ptr() };

        if ipc.events_receive[0].read().bits() != 0 {
            ipc.events_receive[0].reset();
            info!("ipc 0");
        }

        if ipc.events_receive[2].read().bits() != 0 {
            ipc.events_receive[2].reset();
            info!("ipc 2");
            let list = unsafe { &mut *self.rx_ctrl };
            self.process(list);
        }

        if ipc.events_receive[4].read().bits() != 0 {
            ipc.events_receive[4].reset();
            info!("ipc 4");
            let list = unsafe { &mut *self.rx_data };
            self.process(list);
        }

        ipc.intenset.write(|w| {
            w.receive0().set_bit();
            w.receive2().set_bit();
            w.receive4().set_bit();
            w
        });
    }

    fn process(&mut self, list: &mut List) {
        for item in &mut list.items {
            if item.preamble & 0xFF == 0x01 && item.preamble >> 16 == self.rx_seq_no as u32 {
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
}

impl<'a> Control<'a> {
    fn send(&mut self) {
        let cb = &mut *self.cb;

        let msg = &mut cb.ctrl_msgs[0];
        msg.channel = 1; // ctrl

        let preamble = (self.tx_seq_no as u32) << 16 | 0x01;
        cb.ctrl_list.items[0] = ListItem {
            preamble,
            message: msg as *mut _ as _,
        };

        self.tx_seq_no = self.tx_seq_no.wrapping_add(1);

        fence(Ordering::SeqCst);

        let ipc = unsafe { &*pac::ns::IPC::ptr() };
        ipc.tasks_send[1].write(|w| unsafe { w.bits(1) });
    }

    pub fn send_at(&mut self, cmd: &mut [u8]) {
        let cb = &mut *self.cb;

        let msg = &mut cb.data_msgs[0];
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

        let ipc = unsafe { &*pac::ns::IPC::ptr() };
        ipc.tasks_send[1].write(|w| unsafe { w.bits(1) });
    }
}

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
