//! Bluetooth LE: connectable advertising, optional LE Secure Connections
//! pairing (bonds kept in flash) and an LE Audio BAP Unicast Server with one
//! Source ASE (the microphone).
//!
//! GATT: PACS (what we can capture), ASCS (stream setup state machine, logic
//! in the `bap` crate) and an empty CAS (makes us a CAP Acceptor).
//! The LE Audio characteristics do not require encryption: the nRF52840
//! receiver dongle cannot encrypt isochronous channels, and a CIS created on
//! an encrypted link must be encrypted. Clients that pair (e.g. BlueZ) still
//! get an encrypted link and CIS.
//!
//! When the client enables the Source ASE it creates a CIS; we accept it, set
//! up an HCI input data path, and once the client signals Receiver Start Ready
//! the microphone task starts sending LC3 frames as ISO SDUs.

use bap::ascs::{self, Action, AseState, Ascs, Effects};
use bt_hci::cmd::le::{LeAcceptCisRequest, LeRejectCisRequest, LeSetupIsoDataPath};
use bt_hci::cmd::link_control::Disconnect;
use bt_hci::data::IsoPacket;
use bt_hci::event::le::{LeCisEstablished, LeCisRequest};
use bt_hci::param::{CodecId, ConnHandle, DataPathDirection, DataPathId, DisconnectReason, ExtDuration};
use bt_hci::FromHciBytes;
use defmt::{debug, error, info, warn};
use embassy_futures::select::{Either3, select3};
use embassy_nrf::pac;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use heapless::Vec;
use static_cell::StaticCell;
use trouble_host::prelude::*;
use trouble_host::IoCapabilities;

use crate::audio::{self, StreamConfig};
use crate::hci_link::{DISCONNECTIONS, ShmTransport};

/// Bonds live in the last 16 KiB of app-core flash (excluded in `memory.x`).
pub const BOND_FLASH: core::ops::Range<u32> = 0x000F_C000..0x0010_0000;
pub type BondStore = bond_store::BondStore<
    embassy_embedded_hal::adapter::BlockingAsync<embassy_nrf::nvmc::Nvmc<'static>>,
>;

const DEVICE_NAME: &str = "nrf53bird";
const CONNECTIONS_MAX: usize = 1;
/// Signalling + ATT.
const L2CAP_CHANNELS_MAX: usize = 2;
const SOURCE_ASES: usize = 1;
/// Only Source ASE in this server.
const ASE_ID: u8 = 1;

type Ctrl = ExternalController<ShmTransport, 8>;
type BleStack = Stack<'static, Ctrl, DefaultPacketPool>;

#[gatt_server]
pub struct Server {
    pacs: PacsService,
    ascs: AscsService,
    /// Registered for discovery only; never accessed.
    _cas: CasService,
}

/// Published Audio Capabilities Service.
#[gatt_service(uuid = "1850")]
struct PacsService {
    #[characteristic(uuid = "2bcb", read)]
    source_pac: Vec<u8, { bap::pacs::SOURCE_PAC_MAX_LEN }>,
    #[characteristic(uuid = "2bcc", read)]
    source_audio_locations: [u8; 4],
    #[characteristic(uuid = "2bcd", read, notify)]
    available_audio_contexts: [u8; 4],
    #[characteristic(uuid = "2bce", read)]
    supported_audio_contexts: [u8; 4],
}

/// Audio Stream Control Service.
#[gatt_service(uuid = "184e")]
struct AscsService {
    #[characteristic(uuid = "2bc5", read, notify)]
    source_ase: Vec<u8, { ascs::ASE_VALUE_MAX_LEN }>,
    #[characteristic(uuid = "2bc6", write, write_without_response, notify)]
    control_point: Vec<u8, 128>,
}

/// Common Audio Service: no characteristics unless we were a set member.
#[gatt_service(uuid = "1853")]
struct CasService {}

/// Controller events trouble-host hands to us synchronously; processed by
/// the connection task, which can issue HCI commands.
#[derive(Clone, Copy, defmt::Format)]
enum LinkEvent {
    CisRequest { acl: u16, cis: u16, cig_id: u8, cis_id: u8 },
    CisEstablished { cis: u16, ok: bool },
}

static LINK_EVENTS: Channel<CriticalSectionRawMutex, LinkEvent, 4> = Channel::new();

struct Hooks;

impl EventHandler for Hooks {
    fn on_cis_request(&self, e: &LeCisRequest) {
        let ev = LinkEvent::CisRequest { acl: e.acl_handle.raw(), cis: e.cis_handle.raw(), cig_id: e.cig_id, cis_id: e.cis_id };
        if LINK_EVENTS.try_send(ev).is_err() {
            error!("link event queue full, dropping {}", ev);
        }
    }

    fn on_cis_established(&self, e: &LeCisEstablished) {
        let ev = LinkEvent::CisEstablished { cis: e.handle.raw(), ok: e.status.to_result().is_ok() };
        if LINK_EVENTS.try_send(ev).is_err() {
            error!("link event queue full, dropping {}", ev);
        }
    }
}

/// Random static address derived from the chip's unique FICR device ID.
fn address() -> Address {
    let id0 = pac::FICR.info().deviceid(0).read();
    let id1 = pac::FICR.info().deviceid(1).read();
    let mut a = [0u8; 6];
    a[..4].copy_from_slice(&id0.to_le_bytes());
    a[4..].copy_from_slice(&id1.to_le_bytes()[..2]);
    // Random static: two most significant bits set.
    a[5] |= 0xc0;
    Address::random(a)
}

pub fn start(spawner: embassy_executor::Spawner, transport: ShmTransport, bonds: BondStore) {
    static RESOURCES: StaticCell<HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX>> = StaticCell::new();
    static STACK_CELL: StaticCell<BleStack> = StaticCell::new();
    static SERVER: StaticCell<Server<'static>> = StaticCell::new();

    let resources = RESOURCES.init(HostResources::new());
    let address = address();
    info!("BLE address {}", address);
    let stack = STACK_CELL.init(
        trouble_host::new(ExternalController::new(transport), resources)
            .set_random_address(address)
            .set_io_capabilities(IoCapabilities::NoInputNoOutput)
            .build(),
    );

    let server = SERVER.init(
        Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
            name: DEVICE_NAME,
            appearance: &appearance::audio_source::MICROPHONE,
        }))
        .unwrap(),
    );
    init_values(server);

    spawner.spawn(runner_task(stack).unwrap());
    spawner.spawn(peripheral_task(stack, server, bonds).unwrap());
    spawner.spawn(iso_task(stack).unwrap());
}

fn init_values(server: &Server<'_>) {
    let mut pac_buf = [0u8; bap::pacs::SOURCE_PAC_MAX_LEN];
    let n = bap::pacs::source_pac(&mut pac_buf);
    unwrap_set(server.set(&server.pacs.source_pac, &Vec::from_slice(&pac_buf[..n]).unwrap()));
    unwrap_set(server.set(&server.pacs.source_audio_locations, &bap::pacs::source_audio_locations()));
    unwrap_set(server.set(&server.pacs.available_audio_contexts, &bap::pacs::available_audio_contexts()));
    unwrap_set(server.set(&server.pacs.supported_audio_contexts, &bap::pacs::supported_audio_contexts()));
    let ascs = Ascs::<SOURCE_ASES>::new();
    unwrap_set(server.set(&server.ascs.source_ase, &ase_value(&ascs)));
}

fn unwrap_set(r: Result<(), trouble_host::Error>) {
    if let Err(e) = r {
        defmt::panic!("GATT set failed: {}", e);
    }
}

fn ase_value(ascs: &Ascs<SOURCE_ASES>) -> Vec<u8, { ascs::ASE_VALUE_MAX_LEN }> {
    let mut buf = [0u8; ascs::ASE_VALUE_MAX_LEN];
    let n = ascs.ase_value(ASE_ID, &mut buf);
    Vec::from_slice(&buf[..n]).unwrap()
}

#[embassy_executor::task]
async fn runner_task(stack: &'static BleStack) -> ! {
    let mut runner = stack.runner();
    loop {
        if let Err(e) = runner.run_with_handler(&Hooks).await {
            error!("BLE host error: {}", e);
        }
    }
}

#[embassy_executor::task]
async fn peripheral_task(stack: &'static BleStack, server: &'static Server<'static>, mut bonds: BondStore) -> ! {
    bonds.load(stack).await;
    let mut peripheral = stack.peripheral();
    loop {
        match advertise(&mut peripheral, server).await {
            Ok(conn) => {
                if let Err(e) = conn.raw().set_bondable(true) {
                    warn!("set_bondable: {}", e);
                }
                serve(stack, server, &conn, &mut bonds).await;
            }
            Err(e) => {
                error!("advertising failed: {}", e);
                embassy_time::Timer::after_secs(1).await;
            }
        }
    }
}

async fn advertise<'v>(
    peripheral: &mut Peripheral<'v, Ctrl, DefaultPacketPool>,
    server: &'v Server<'v>,
) -> Result<GattConnection<'v, 'v, DefaultPacketPool>, BleHostError<crate::hci_link::Error>> {
    let ascs_data = bap::adv::ascs_service_data();
    let cas_data = bap::adv::cas_service_data();
    let appearance = appearance::audio_source::MICROPHONE.to_le_bytes();
    let mut adv = [0u8; 31];
    let adv_len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            // bap returns complete Service Data payloads (UUID first), so emit raw AD type 0x16.
            AdStructure::Unknown { ty: 0x16, data: &ascs_data },
            AdStructure::Unknown { ty: 0x16, data: &cas_data },
            AdStructure::Unknown { ty: 0x19, data: &appearance },
        ],
        &mut adv,
    )?;
    let mut scan = [0u8; 31];
    let scan_len = AdStructure::encode_slice(
        &[
            AdStructure::CompleteLocalName(DEVICE_NAME.as_bytes()),
            AdStructure::IncompleteServiceUuids16(&[bap::uuid::PACS.to_le_bytes(), bap::uuid::ASCS.to_le_bytes()]),
        ],
        &mut scan,
    )?;
    let advertiser = peripheral
        .advertise(
            &AdvertisementParameters::default(),
            Advertisement::ConnectableScannableUndirected { adv_data: &adv[..adv_len], scan_data: &scan[..scan_len] },
        )
        .await?;
    info!("advertising as \"{}\"", DEVICE_NAME);
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("connected: {}", conn.raw().peer_address());
    Ok(conn)
}

/// A CIS the controller offered us and we accepted.
#[derive(Clone, Copy)]
struct Cis {
    handle: u16,
    cig_id: u8,
    cis_id: u8,
}

async fn serve(
    stack: &BleStack,
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    bonds: &mut BondStore,
) {
    let acl = conn.raw().handle().raw();
    let mut ascs = Ascs::<SOURCE_ASES>::new();
    let mut cises: Vec<Cis, 2> = Vec::new();
    // Events from a previous connection are meaningless now.
    while LINK_EVENTS.try_receive().is_ok() {}
    while DISCONNECTIONS.try_receive().is_ok() {}

    loop {
        let mut fx = Effects::default();
        match select3(conn.next(), LINK_EVENTS.receive(), DISCONNECTIONS.receive()).await {
            Either3::First(event) => match event {
                GattConnectionEvent::Disconnected { reason } => {
                    info!("disconnected: {}", reason);
                    ascs.acl_disconnected(&mut fx);
                    apply_actions(stack, &fx).await;
                    audio::STREAM.signal(None);
                    unwrap_set(server.set(&server.ascs.source_ase, &ase_value(&ascs)));
                    return;
                }
                GattConnectionEvent::PairingComplete { security_level, bond } => {
                    info!("paired: {}, bonded: {}", security_level, bond.is_some());
                    if let Some(bond) = bond {
                        bonds.save(&bond).await;
                    }
                }
                GattConnectionEvent::PairingFailed(e) => warn!("pairing failed: {}", e),
                GattConnectionEvent::Gatt { event } => {
                    let cp_handle = server.ascs.control_point.handle;
                    let mut rsp = [0u8; ascs::CP_RESPONSE_MAX_LEN];
                    let rsp_len = match &event {
                        GattEvent::Write(w) if w.handle() == cp_handle => {
                            Some(w.with_data(|_, data| ascs.control_point(data, &mut rsp, &mut fx)))
                        }
                        _ => None,
                    };
                    match event.accept() {
                        Ok(reply) => reply.send().await,
                        Err(e) => warn!("GATT reply failed: {}", e),
                    }
                    if let Some(n) = rsp_len {
                        debug!("ASE CP response {=[u8]:02x}", rsp[..n]);
                        let value: Vec<u8, 128> = Vec::from_slice(&rsp[..n]).unwrap();
                        if let Err(e) = server.ascs.control_point.notify(conn, &value, false).await {
                            warn!("ASE CP notify failed: {}", e);
                        }
                    }
                }
                _ => {}
            },
            Either3::Second(LinkEvent::CisRequest { acl: req_acl, cis, cig_id, cis_id }) => {
                if req_acl == acl && !cises.is_full() && ascs.cis_request(cig_id, cis_id) {
                    info!("accepting CIS {=u16:#x} (CIG {=u8}, CIS {=u8})", cis, cig_id, cis_id);
                    let _ = cises.push(Cis { handle: cis, cig_id, cis_id });
                    if let Err(e) = stack.async_command(LeAcceptCisRequest::new(ConnHandle::new(cis))).await {
                        error!("LE Accept CIS Request: {}", e);
                    }
                } else {
                    warn!("rejecting CIS {=u16:#x} (CIG {=u8}, CIS {=u8})", cis, cig_id, cis_id);
                    // 0x0D: Connection Rejected due to Limited Resources.
                    if let Err(e) = stack.command(LeRejectCisRequest::new(ConnHandle::new(cis), 0x0d)).await {
                        error!("LE Reject CIS Request: {}", e);
                    }
                }
            }
            Either3::Second(LinkEvent::CisEstablished { cis, ok }) => {
                let Some(pos) = cises.iter().position(|c| c.handle == cis) else {
                    continue;
                };
                if ok {
                    let c = cises[pos];
                    info!("CIS {=u16:#x} established", cis);
                    ascs.cis_established(c.cig_id, c.cis_id, cis, &mut fx);
                } else {
                    warn!("CIS {=u16:#x} failed to establish", cis);
                    cises.swap_remove(pos);
                    ascs.cis_disconnected(cis, &mut fx);
                }
            }
            Either3::Third(handle) => {
                if let Some(pos) = cises.iter().position(|c| c.handle == handle) {
                    info!("CIS {=u16:#x} disconnected", handle);
                    cises.swap_remove(pos);
                    ascs.cis_disconnected(handle, &mut fx);
                }
            }
        }

        loop {
            apply_actions(stack, &fx).await;
            for id in fx.changed_ases() {
                debug_assert_eq!(id, ASE_ID);
                let value = ase_value(&ascs);
                info!("ASE {=u8} -> {}", id, defmt::Debug2Format(&ascs.state(id).unwrap_or(AseState::Idle)));
                unwrap_set(server.set(&server.ascs.source_ase, &value));
                if let Err(e) = server.ascs.source_ase.notify(conn, &value, false).await {
                    warn!("ASE notify failed: {}", e);
                }
            }
            // A Release without a CIS finishes only after Releasing was notified.
            if !fx.has_deferred() {
                break;
            }
            fx.clear();
            ascs.complete_deferred(&mut fx);
        }
    }
}

async fn apply_actions(stack: &BleStack, fx: &Effects) {
    for action in fx.actions() {
        match *action {
            Action::SetupIsoDataPath { cis_handle } => {
                let transparent = CodecId { coding_format: 0x03, company_id: 0, vendor_specific_codec_id: 0 };
                let cmd = LeSetupIsoDataPath::new(
                    ConnHandle::new(cis_handle),
                    DataPathDirection::Input,
                    DataPathId::HCI,
                    transparent,
                    ExtDuration::from_micros(0),
                    &[],
                );
                match stack.command(cmd).await {
                    Ok(_) => info!("ISO data path up on CIS {=u16:#x}", cis_handle),
                    Err(e) => error!("LE Setup ISO Data Path: {}", e),
                }
            }
            Action::StartStream { ase_id, cis_handle, codec, qos } => {
                info!(
                    "ASE {=u8} streaming: {=u32} Hz, {=u16} B/frame, SDU interval {=u32} us, max SDU {=u16}",
                    ase_id, codec.sampling_hz, codec.octets_per_frame, qos.sdu_interval_us, qos.max_sdu
                );
                audio::STREAM.signal(Some(StreamConfig {
                    cis_handle,
                    sampling_hz: codec.sampling_hz,
                    octets_per_frame: codec.octets_per_frame,
                }));
            }
            Action::StopStream { ase_id } => {
                info!("ASE {=u8} stopped streaming", ase_id);
                audio::STREAM.signal(None);
            }
            Action::DisconnectCis { cis_handle } => {
                let cmd = Disconnect::new(ConnHandle::new(cis_handle), DisconnectReason::RemoteUserTerminatedConn);
                // bt-hci models Disconnect as a sync command; it completes on Command Status.
                if let Err(e) = stack.command(cmd).await {
                    error!("Disconnect CIS: {}", e);
                }
            }
        }
    }
}

/// One HCI ISO data packet ready for the controller.
struct IsoSdu {
    len: usize,
    bytes: [u8; 8 + bap::pacs::MAX_OCTETS_PER_FRAME as usize],
}

static ISO_TX: Channel<CriticalSectionRawMutex, IsoSdu, 3> = Channel::new();

/// Queue one SDU for `cis_handle` (HCI ISO data, single complete fragment,
/// no timestamp). Returns false if the queue is full and the SDU was dropped.
pub fn queue_iso_sdu(cis_handle: u16, seq: u16, sdu: &[u8]) -> bool {
    let mut p = IsoSdu { len: 8 + sdu.len(), bytes: [0; 8 + bap::pacs::MAX_OCTETS_PER_FRAME as usize] };
    // PB_Flag = 0b10 (complete SDU), TS_Flag = 0.
    p.bytes[..2].copy_from_slice(&(cis_handle | (0b10 << 12)).to_le_bytes());
    p.bytes[2..4].copy_from_slice(&(4 + sdu.len() as u16).to_le_bytes());
    p.bytes[4..6].copy_from_slice(&seq.to_le_bytes());
    p.bytes[6..8].copy_from_slice(&(sdu.len() as u16).to_le_bytes());
    p.bytes[8..p.len].copy_from_slice(sdu);
    ISO_TX.try_send(p).is_ok()
}

#[embassy_executor::task]
async fn iso_task(stack: &'static BleStack) -> ! {
    let iso = stack.iso();
    loop {
        let p = ISO_TX.receive().await;
        let (packet, _) = IsoPacket::from_hci_bytes(&p.bytes[..p.len]).unwrap();
        if let Err(e) = iso.send(&packet).await {
            warn!("ISO send failed: {}", e);
        }
    }
}
