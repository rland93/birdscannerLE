//! Raw HCI bridge: H4 records from the app core go into the SoftDevice
//! Controller (via `sdc-hci`), everything the controller produces goes back
//! over the shared-RAM rings.

use core::future::poll_fn;
use core::task::Poll;

use defmt::{debug, warn};
use embassy_nrf::interrupt::typelevel::{self, Interrupt as _};
use embassy_nrf::pac;
use embassy_sync::waitqueue::AtomicWaker;
use hci_shm::{H4_ACL, H4_CMD, H4_EVT, H4_ISO, MAX_RECORD, Shared};
use nrf_sdc::SoftdeviceController;

/// IPC channel the app core signals on, and the one we signal back on.
const IPC_CH_FROM_APP: usize = 0;
const IPC_CH_TO_APP: usize = 1;

static IPC_WAKER: AtomicWaker = AtomicWaker::new();

pub struct IpcInterruptHandler;

impl typelevel::Handler<typelevel::IPC> for IpcInterruptHandler {
    unsafe fn on_interrupt() {
        let r = pac::IPC;
        if r.events_receive(IPC_CH_FROM_APP).read() != 0 {
            r.events_receive(IPC_CH_FROM_APP).write_value(0);
            IPC_WAKER.wake();
        }
    }
}

fn setup_ipc() {
    let r = pac::IPC;
    // Event n <-> channel n in both directions keeps the mapping obvious.
    r.receive_cnf(IPC_CH_FROM_APP).write(|w| w.0 = 1 << IPC_CH_FROM_APP);
    r.send_cnf(IPC_CH_TO_APP).write(|w| w.0 = 1 << IPC_CH_TO_APP);
    r.events_receive(IPC_CH_FROM_APP).write_value(0);
    r.intenset().write(|w| w.0 = 1 << IPC_CH_FROM_APP);
    typelevel::IPC::unpend();
    unsafe { typelevel::IPC::enable() };
}

fn notify_app() {
    pac::IPC.tasks_send(IPC_CH_TO_APP).write_value(1);
}

/// Forward HCI traffic forever.
pub async fn run(shm: &'static Shared, sdc: &SoftdeviceController<'_>) -> ! {
    setup_ipc();
    let mut rec = [0u8; MAX_RECORD];
    let mut evt = [0u8; sdc_hci::EVENT_BUF];
    // Controller output; large enough for ISO SDUs too.
    let mut out = [0u8; sdc_hci::GET_BUF];
    // A controller packet that did not fit into the ring yet.
    let mut stalled: Option<(u8, usize)> = None;

    poll_fn(|cx| {
        // Register before draining so nothing that arrives during the drain is missed.
        IPC_WAKER.register(cx.waker());
        sdc.register_waker(cx.waker());
        let mut signalled = false;

        loop {
            let mut progress = false;

            // Controller -> host. Never consume a host record whose reply
            // might not fit while controller output is stalled.
            if let Some((ind, len)) = stalled {
                if !shm.to_app.try_push(ind, &[&out[..len]]) {
                    break;
                }
                stalled = None;
                signalled = true;
                progress = true;
            }
            if let Some((ind, len)) = sdc_hci::try_get(&mut out) {
                if shm.to_app.try_push(ind, &[&out[..len]]) {
                    signalled = true;
                } else {
                    stalled = Some((ind, len));
                }
                progress = true;
            }

            // Host -> controller. A command reply needs ring space; check first.
            if stalled.is_none() && shm.to_app.fits(1 + evt.len()) {
                if let Some(n) = shm.to_net.try_pop(&mut rec) {
                    signalled = true; // freed space in to_net
                    progress = true;
                    let (ind, pkt) = (rec[0], &rec[1..n]);
                    match ind {
                        H4_CMD => {
                            debug!("HCI cmd {=u16:#06x}", u16::from_le_bytes([pkt[0], pkt[1]]));
                            if let Some(len) = sdc_hci::run_command(pkt, &mut evt) {
                                assert!(shm.to_app.try_push(H4_EVT, &[&evt[..len]]));
                            }
                        }
                        H4_ACL => {
                            if let Err(e) = sdc.hci_data_put(pkt) {
                                warn!("HCI: ACL rejected: {}", e);
                            }
                        }
                        H4_ISO => {
                            if let Err(e) = sdc.hci_iso_data_put(pkt) {
                                debug!("HCI: ISO SDU rejected: {}", e);
                            }
                        }
                        other => warn!("HCI: dropping record with indicator {=u8}", other),
                    }
                }
            }

            if !progress {
                break;
            }
        }
        if signalled {
            notify_app();
        }
        Poll::<()>::Pending
    })
    .await;
    unreachable!()
}
