//! The mics: scanning, connecting, and driving each mic's Source ASE as a
//! BAP Unicast Client (logic in the `bap` crate's `client` module).
//!
//! Links are not paired or encrypted: the nRF52840 controller cannot encrypt
//! isochronous channels, and a CIS on an encrypted ACL must be encrypted.
//!
//! One task per connected mic ([`MAX_MICS`] slots). Each mic gets its own CIG
//! (CIG_ID = slot) with one CIS, so mics start and stop independently.

use core::cell::{Cell, RefCell};

use bap::ascs::{AseState, CodecConfig};
use bap::client::{self, AseStatus, QosPreferences};
use bt_hci::cmd::le::LeSetupIsoDataPath;
use bt_hci::cmd::link_control::Disconnect;
use bt_hci::param::{AddrKind, BdAddr, CodecId, ConnHandle, DataPathDirection, DataPathId, DisconnectReason, ExtDuration};
use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, Either4, select, select4};
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer, with_timeout};
use garden_link::{Addr, DongleToHost, Failure, HostToDongle, MicId, MicLink, Quality};
use static_cell::StaticCell;
use trouble_host::prelude::*;

use crate::MAX_MICS;
use crate::hci_local::{LocalTransport, raw_command};
use crate::link;

type Ctrl = ExternalController<LocalTransport, 8>;
type BleStack = Stack<'static, Ctrl, DefaultPacketPool>;
type Client<'a> = GattClient<'a, Ctrl, DefaultPacketPool, 8>;

/// Signalling + ATT per connection.
const L2CAP_CHANNELS: usize = 2 * MAX_MICS;
/// The mic firmware exposes one Source ASE with this id.
const ASE_ID: u8 = 1;
/// How long a single ASE step may take before we give up.
const STEP_TIMEOUT: Duration = Duration::from_secs(5);
const OP_LE_SET_CIG_PARAMS: u16 = 0x2062;
const OP_LE_CREATE_CIS: u16 = 0x2064;
const OP_LE_REMOVE_CIG: u16 = 0x2065;

const UUID_PACS: u16 = 0x1850;
const UUID_ASCS: u16 = 0x184E;
const UUID_SOURCE_PAC: u16 = 0x2BCB;
const UUID_SOURCE_AUDIO_LOCATIONS: u16 = 0x2BCC;
const UUID_SOURCE_ASE: u16 = 0x2BC5;
const UUID_ASE_CONTROL_POINT: u16 = 0x2BC6;

enum SlotCmd {
    Start(Quality),
    Stop,
    Disconnect,
}

#[derive(Clone, Copy)]
struct SlotInfo {
    addr: Addr,
    link: MicLink,
    /// CIS handle while a CIG exists for this slot.
    cis: Option<u16>,
    /// What a streaming slot sends. StreamConfig normally goes out once, when
    /// the stream starts, so a host that connects mid-stream would never learn
    /// the codec parameters; keep them for replay on Hello.
    stream: Option<StreamCfg>,
}

/// The codec parameters a slot streams with (one Source ASE, fixed per
/// stream).
#[derive(Clone, Copy)]
struct StreamCfg {
    sampling_hz: u32,
    frame_duration_us: u32,
    octets_per_frame: u16,
}

static SLOTS: BlockingMutex<CriticalSectionRawMutex, RefCell<[Option<SlotInfo>; MAX_MICS]>> =
    BlockingMutex::new(RefCell::new([None; MAX_MICS]));
static SLOT_CMD: [Channel<CriticalSectionRawMutex, SlotCmd, 2>; MAX_MICS] = [const { Channel::new() }; MAX_MICS];
/// LE CIS Established result for the slot's CIS.
static CIS_UP: [Signal<CriticalSectionRawMutex, bool>; MAX_MICS] = [const { Signal::new() }; MAX_MICS];
/// The slot's CIS went down; HCI reason.
static CIS_DOWN: [Signal<CriticalSectionRawMutex, u8>; MAX_MICS] = [const { Signal::new() }; MAX_MICS];
/// Whether `Advert`s are forwarded to the host.
static SCANNING: BlockingMutex<CriticalSectionRawMutex, Cell<bool>> = BlockingMutex::new(Cell::new(false));
/// Recently seen ASCS advertisers, to match their scan responses (names).
static LE_AUDIO_SEEN: BlockingMutex<CriticalSectionRawMutex, RefCell<heapless::Vec<Addr, 8>>> =
    BlockingMutex::new(RefCell::new(heapless::Vec::new()));

fn with_slot<R>(slot: usize, f: impl FnOnce(&mut Option<SlotInfo>) -> R) -> R {
    SLOTS.lock(|s| f(&mut s.borrow_mut()[slot]))
}

fn slot_of_cis(handle: u16) -> Option<usize> {
    SLOTS.lock(|s| s.borrow().iter().position(|i| i.is_some_and(|i| i.cis == Some(handle))))
}

fn report(slot: usize) {
    if let Some(i) = with_slot(slot, |s| *s) {
        link::send(&DongleToHost::MicStatus { mic: MicId(slot as u8), addr: i.addr, link: i.link });
    }
}

fn set_link(slot: usize, link: MicLink) {
    with_slot(slot, |s| {
        if let Some(i) = s {
            i.link = link;
        }
    });
    report(slot);
    if let MicLink::Gone(_) = link {
        with_slot(slot, |s| *s = None);
    }
}

fn to_addr(a: Address) -> Addr {
    let le = a.addr.into_inner();
    Addr { kind: a.kind.into_inner(), bytes: [le[5], le[4], le[3], le[2], le[1], le[0]] }
}

fn to_address(a: Addr) -> Address {
    let b = a.bytes;
    let kind = if a.kind == 0 { AddrKind::PUBLIC } else { AddrKind::RANDOM };
    Address::new(kind, BdAddr::new([b[5], b[4], b[3], b[2], b[1], b[0]]))
}

/// Called by the HCI transport for every Disconnection Complete.
pub fn on_disconnection(handle: u16, reason: u8) {
    if let Some(slot) = slot_of_cis(handle) {
        CIS_DOWN[slot].signal(reason);
    }
}

struct Hooks;

impl EventHandler for Hooks {
    fn on_adv_reports(&self, reports: bt_hci::param::LeAdvReportsIter) {
        if !SCANNING.lock(|s| s.get()) {
            return;
        }
        for report in reports.flatten() {
            let (ascs, name) = parse_adv(report.data);
            let addr = to_addr(Address::new(report.addr_kind, report.addr));
            // The name usually arrives in the scan response, which carries no
            // ASCS data: accept it from addresses seen announcing ASCS.
            let known = LE_AUDIO_SEEN.lock(|seen| {
                let mut seen = seen.borrow_mut();
                let known = seen.contains(&addr);
                if ascs && !known {
                    if seen.is_full() {
                        seen.remove(0);
                    }
                    let _ = seen.push(addr);
                }
                known
            });
            if ascs || (known && !name.is_empty()) {
                link::send(&DongleToHost::Advert { addr, rssi: report.rssi, name });
            }
        }
    }

    fn on_cis_established(&self, e: &bt_hci::event::le::LeCisEstablished) {
        if let Some(slot) = slot_of_cis(e.handle.raw()) {
            CIS_UP[slot].signal(e.status.to_result().is_ok());
        }
    }

    fn on_iso_data(&self, p: &bt_hci::data::IsoPacket<'_>) {
        let Some(slot) = slot_of_cis(p.handle().raw()) else { return };
        let Some(hdr) = p.data_load_header() else { return };
        // Packet_Status_Flag lives in the top two bits of ISO_SDU_Length:
        // anything but 0b00 means the SDU is missing or possibly corrupt.
        let valid = hdr.iso_sdu_len >> 14 == 0;
        let frame = if valid { p.data() } else { &[] };
        link::send(&DongleToHost::Audio { mic: MicId(slot as u8), seq: hdr.sequence_num, frame });
    }
}

/// Whether advertising data announces ASCS (an LE Audio unicast server), and
/// the device name if it carries one.
fn parse_adv(data: &[u8]) -> (bool, &str) {
    let mut ascs = false;
    let mut name = "";
    let mut rest = data;
    while let [len, tail @ ..] = rest {
        let len = usize::from(*len);
        if len == 0 || tail.len() < len {
            break;
        }
        let (field, next) = tail.split_at(len);
        match field {
            [0x16, lo, hi, ..] if u16::from_le_bytes([*lo, *hi]) == UUID_ASCS => ascs = true,
            [0x08 | 0x09, n @ ..] => name = core::str::from_utf8(n).unwrap_or(""),
            _ => {}
        }
        rest = next;
    }
    (ascs, name)
}

pub fn start(spawner: Spawner, transport: LocalTransport) {
    static RESOURCES: StaticCell<HostResources<DefaultPacketPool, MAX_MICS, L2CAP_CHANNELS>> = StaticCell::new();
    static STACK: StaticCell<BleStack> = StaticCell::new();

    let stack = STACK.init(
        trouble_host::new(ExternalController::new(transport), RESOURCES.init(HostResources::new()))
            .set_random_address(own_address())
            .build(),
    );
    spawner.spawn(runner_task(stack).unwrap());
    spawner.spawn(manager_task(spawner, stack).unwrap());
}

fn own_address() -> Address {
    let ficr = embassy_nrf::pac::FICR;
    let mut a = [0u8; 6];
    a[..4].copy_from_slice(&ficr.deviceaddr(0).read().to_le_bytes());
    a[4..].copy_from_slice(&ficr.deviceaddr(1).read().to_le_bytes()[..2]);
    a[5] |= 0xc0; // random static
    Address::random(a)
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

fn hello() {
    link::send(&DongleToHost::Hello {
        protocol: garden_link::PROTOCOL_VERSION,
        firmware: env!("CARGO_PKG_VERSION"),
        max_mics: MAX_MICS as u8,
    });
    for slot in 0..MAX_MICS {
        let Some(i) = with_slot(slot, |s| *s) else { continue };
        report(slot);
        // A host that (re)connected while this slot was already streaming
        // must still learn the codec parameters, or it cannot decode.
        if let Some(c) = i.stream {
            link::send(&DongleToHost::StreamConfig {
                mic: MicId(slot as u8),
                sampling_hz: c.sampling_hz,
                frame_duration_us: c.frame_duration_us,
                octets_per_frame: c.octets_per_frame,
            });
        }
    }
}

#[embassy_executor::task]
async fn manager_task(spawner: Spawner, stack: &'static BleStack) -> ! {
    let mut central = stack.central();
    hello();
    loop {
        let cmd = if SCANNING.lock(|s| s.get()) {
            // Active scanning, so names in scan responses are seen too. The
            // session runs until the next command arrives.
            let config = ScanConfig { active: true, ..Default::default() };
            let mut scanner = Scanner::new(&mut central);
            match scanner.scan(&config).await {
                Ok(_session) => link::FROM_HOST.receive().await,
                Err(e) => {
                    warn!("scan failed: {}", e);
                    SCANNING.lock(|s| s.set(false));
                    link::FROM_HOST.receive().await
                }
            }
        } else {
            link::FROM_HOST.receive().await
        };

        match cmd {
            HostToDongle::Hello => hello(),
            HostToDongle::Scan { enable } => SCANNING.lock(|s| s.set(enable)),
            HostToDongle::Connect { addr } => connect(spawner, stack, &mut central, addr).await,
            HostToDongle::Disconnect { mic } => slot_cmd(mic, SlotCmd::Disconnect),
            HostToDongle::StartStream { mic, quality } => slot_cmd(mic, SlotCmd::Start(quality)),
            HostToDongle::StopStream { mic } => slot_cmd(mic, SlotCmd::Stop),
        }
    }
}

fn slot_cmd(mic: MicId, cmd: SlotCmd) {
    let slot = usize::from(mic.0);
    if slot < MAX_MICS && with_slot(slot, |s| s.is_some()) {
        if SLOT_CMD[slot].try_send(cmd).is_err() {
            link::log("mic busy, command dropped");
        }
    } else {
        link::log("no such mic");
    }
}

async fn connect(
    spawner: Spawner,
    stack: &'static BleStack,
    central: &mut Central<'static, Ctrl, DefaultPacketPool>,
    addr: Addr,
) {
    if let Some(slot) = SLOTS.lock(|s| s.borrow().iter().position(|i| i.is_some_and(|i| i.addr == addr))) {
        report(slot);
        return;
    }
    let Some(slot) = SLOTS.lock(|s| s.borrow().iter().position(|i| i.is_none())) else {
        link::log("all mic slots in use");
        return;
    };
    with_slot(slot, |s| *s = Some(SlotInfo { addr, link: MicLink::Connecting, cis: None, stream: None }));
    report(slot);
    while SLOT_CMD[slot].try_receive().is_ok() {}

    let target = [to_address(addr)];
    let config = ConnectConfig {
        scan_config: ScanConfig { filter_accept_list: &target, timeout: Duration::from_secs(10), ..Default::default() },
        connect_params: Default::default(),
    };
    match central.connect(&config).await {
        Ok(conn) => {
            info!("connected to {}", conn.peer_address());
            spawner.spawn(mic_task(slot, conn, stack).unwrap());
        }
        Err(e) => {
            warn!("connect to {} failed: {}", target[0], e);
            link::logf(format_args!("connect to {addr} failed: {e:?}"));
            set_link(slot, MicLink::Gone(Failure::ConnectTimeout));
        }
    }
}

#[embassy_executor::task(pool_size = MAX_MICS)]
async fn mic_task(slot: usize, conn: Connection<'static, DefaultPacketPool>, stack: &'static BleStack) {
    let end = run_mic(slot, &conn, stack).await;
    // Tear down anything left for this slot.
    if let Some(cis) = with_slot(slot, |s| s.and_then(|i| i.cis)) {
        remove_iso(slot, cis, stack).await;
    }
    if !matches!(end, Failure::Disconnected(_)) {
        conn.disconnect();
    }
    set_link(slot, MicLink::Gone(end));
}

/// Runs a connected mic until it is gone; returns why.
async fn run_mic(slot: usize, conn: &Connection<'static, DefaultPacketPool>, stack: &'static BleStack) -> Failure {
    set_link(slot, MicLink::Discovering);
    let client: Client<'_> = match Client::new(stack, conn).await {
        Ok(c) => c,
        Err(e) => {
            warn!("GATT client: {}", e);
            return Failure::NotAnLeAudioSource;
        }
    };
    match select(client.task(), serve_mic(slot, conn, &client, stack)).await {
        Either::First(_) => {
            // GATT ends only when the link is gone; fetch the reason.
            match with_timeout(Duration::from_millis(500), async {
                loop {
                    if let ConnectionEvent::Disconnected { reason } = conn.next().await {
                        return reason.into_inner();
                    }
                }
            })
            .await
            {
                Ok(reason) => Failure::Disconnected(reason),
                Err(_) => Failure::Disconnected(0x08),
            }
        }
        Either::Second(f) => f,
    }
}

/// Handles of the mic's LE Audio characteristics.
struct Mic {
    ase: Characteristic<[u8]>,
    cp: Characteristic<[u8]>,
    pac: [u8; 64],
    pac_len: usize,
    audio_locations: u32,
}

async fn discover(client: &Client<'_>) -> Result<Mic, BleHostError<crate::hci_local::Error>> {
    let pacs = client.services_by_uuid(&Uuid::new_short(UUID_PACS)).await?;
    let pacs = pacs.first().ok_or(Error::NotFound)?.clone();
    let ascs = client.services_by_uuid(&Uuid::new_short(UUID_ASCS)).await?;
    let ascs = ascs.first().ok_or(Error::NotFound)?.clone();
    let pac_c: Characteristic<[u8]> = client.characteristic_by_uuid(&pacs, &Uuid::new_short(UUID_SOURCE_PAC)).await?;
    let loc_c: Characteristic<[u8]> =
        client.characteristic_by_uuid(&pacs, &Uuid::new_short(UUID_SOURCE_AUDIO_LOCATIONS)).await?;
    let ase: Characteristic<[u8]> = client.characteristic_by_uuid(&ascs, &Uuid::new_short(UUID_SOURCE_ASE)).await?;
    let cp: Characteristic<[u8]> = client.characteristic_by_uuid(&ascs, &Uuid::new_short(UUID_ASE_CONTROL_POINT)).await?;

    let mut pac = [0u8; 64];
    let pac_len = client.read_characteristic(&pac_c, &mut pac).await?;
    let mut loc = [0u8; 4];
    let n = client.read_characteristic(&loc_c, &mut loc).await?;
    let audio_locations = if n == 4 { u32::from_le_bytes(loc) } else { 0 };

    // Enable notifications on both; one catch-all listener reads them.
    for c in [&ase, &cp] {
        client.write_handle(c.cccd_handle.ok_or(Error::NotSupported)?, &[1, 0]).await?;
    }
    Ok(Mic { ase, cp, pac, pac_len, audio_locations })
}

async fn serve_mic(slot: usize, conn: &Connection<'static, DefaultPacketPool>, client: &Client<'_>, stack: &'static BleStack) -> Failure {
    let mic = match discover(client).await {
        Ok(m) => m,
        Err(e) => {
            warn!("mic {=usize}: discovery failed: {}", slot, e);
            return Failure::NotAnLeAudioSource;
        }
    };
    if client::parse_pac(&mic.pac[..mic.pac_len]).is_err() {
        return Failure::NotAnLeAudioSource;
    }
    let mut notes = match client.listen_all() {
        Ok(l) => l,
        Err(e) => {
            warn!("listen: {}", e);
            return Failure::NotAnLeAudioSource;
        }
    };
    set_link(slot, MicLink::Ready);

    loop {
        let rssi_tick = Timer::after_secs(5);
        match select4(conn.next(), SLOT_CMD[slot].receive(), notes.next(), select(rssi_tick, CIS_DOWN[slot].wait())).await {
            Either4::First(ConnectionEvent::Disconnected { reason }) => return Failure::Disconnected(reason.into_inner()),
            Either4::First(ConnectionEvent::RequestConnectionParams(req)) => {
                let _ = req.accept(None, stack).await;
            }
            Either4::First(_) => {}
            Either4::Second(SlotCmd::Start(q)) => {
                if with_slot(slot, |s| s.is_some_and(|i| i.link == MicLink::Ready)) {
                    set_link(slot, MicLink::Starting);
                    match start_stream(slot, conn, client, &mic, &mut notes, stack, q).await {
                        Ok(()) => set_link(slot, MicLink::Streaming),
                        Err(why) => {
                            link::log(why);
                            stop_stream(slot, client, &mic, &mut notes, stack).await;
                            set_link(slot, MicLink::Ready);
                        }
                    }
                }
            }
            Either4::Second(SlotCmd::Stop) => {
                if with_slot(slot, |s| s.is_some_and(|i| i.link == MicLink::Streaming)) {
                    set_link(slot, MicLink::Stopping);
                    stop_stream(slot, client, &mic, &mut notes, stack).await;
                    set_link(slot, MicLink::Ready);
                }
            }
            Either4::Second(SlotCmd::Disconnect) => {
                conn.disconnect();
            }
            Either4::Third(n) => {
                // Unsolicited ASE change while streaming (e.g. the mic released
                // the stream or its CIS dropped): clean up our side.
                if n.handle() == mic.ase.handle
                    && let Ok(v) = client::parse_ase(n.as_ref())
                    && v.status.state() != AseState::Streaming
                    && with_slot(slot, |s| s.is_some_and(|i| i.link == MicLink::Streaming))
                {
                    link::log("mic stopped streaming");
                    stop_stream(slot, client, &mic, &mut notes, stack).await;
                    set_link(slot, MicLink::Ready);
                }
            }
            Either4::Fourth(Either::First(())) => {
                if let Ok(rssi) = conn.rssi(stack).await {
                    link::send(&DongleToHost::Rssi { mic: MicId(slot as u8), rssi });
                }
            }
            Either4::Fourth(Either::Second(reason)) => {
                warn!("mic {=usize}: CIS lost ({=u8:#x})", slot, reason);
                if with_slot(slot, |s| s.is_some_and(|i| i.link == MicLink::Streaming)) {
                    link::log("stream lost");
                    stop_stream(slot, client, &mic, &mut notes, stack).await;
                    set_link(slot, MicLink::Ready);
                }
            }
        }
    }
}

type Notes<'a> = trouble_host::gatt::NotificationListener<'a, { trouble_host::config::GATT_CLIENT_NOTIFICATION_MTU }>;

/// Write one ASE Control Point PDU and wait until the ASE reaches `target`.
/// Returns the server's QoS preferences when the target is Codec Configured.
async fn ase_step(
    client: &Client<'_>,
    mic: &Mic,
    notes: &mut Notes<'_>,
    pdu: &[u8],
    target: AseState,
) -> Result<Option<QosPreferences>, &'static str> {
    client.write_characteristic(&mic.cp, pdu).await.map_err(|_| "ASE control point write failed")?;
    let wait = async {
        loop {
            let n = notes.next().await;
            if n.handle() == mic.cp.handle {
                let Ok(rsp) = client::parse_cp_response(n.as_ref()) else { continue };
                if rsp.opcode == pdu[0] && !rsp.entry(ASE_ID).is_some_and(|e| e.is_success()) {
                    return Err("mic rejected the ASE operation");
                }
            } else if n.handle() == mic.ase.handle {
                // A Codec Configured value may not fit one notification; read it whole then.
                let mut buf = [0u8; 64];
                let value: &[u8] = match client::parse_ase(n.as_ref()) {
                    Err(client::ParseError::Truncated) => {
                        let len = client.read_characteristic(&mic.ase, &mut buf).await.map_err(|_| "ASE read failed")?;
                        &buf[..len]
                    }
                    _ => n.as_ref(),
                };
                let Ok(v) = client::parse_ase(value) else { continue };
                if v.status.state() == target {
                    return Ok(match v.status {
                        AseStatus::CodecConfigured(c) => Some(c.preferences),
                        _ => None,
                    });
                }
            }
        }
    };
    with_timeout(STEP_TIMEOUT, wait).await.unwrap_or(Err("mic did not respond in time"))
}

async fn start_stream(
    slot: usize,
    conn: &Connection<'static, DefaultPacketPool>,
    client: &Client<'_>,
    mic: &Mic,
    notes: &mut Notes<'_>,
    stack: &'static BleStack,
    quality: Quality,
) -> Result<(), &'static str> {
    let pac = client::parse_pac(&mic.pac[..mic.pac_len]).map_err(|_| "bad PAC")?;
    // Mono: one location bit if the mic has any.
    let location = mic.audio_locations & mic.audio_locations.wrapping_neg();
    let codec: CodecConfig = client::choose_codec_config(&pac, quality.sampling_hz(), quality.octets_per_frame(), location)
        .ok_or("mic does not support that quality")?;
    let mut pdu = [0u8; client::CP_PDU_MAX_LEN];

    let n = client::config_codec(&mut pdu, ASE_ID, client::target_latency::LOW_LATENCY, client::target_phy::LE_2M, &codec)
        .ok_or("codec not encodable")?;
    let prefs = ase_step(client, mic, notes, &pdu[..n], AseState::CodecConfigured).await?.ok_or("no QoS preferences")?;

    let qos = client::derive_qos(&prefs, &codec, slot as u8, 0);
    let cis = set_cig(&qos)?;
    with_slot(slot, |s| s.as_mut().map(|i| i.cis = Some(cis)));
    CIS_UP[slot].reset();
    CIS_DOWN[slot].reset();

    let n = client::config_qos(&mut pdu, ASE_ID, &qos);
    ase_step(client, mic, notes, &pdu[..n], AseState::QosConfigured).await?;
    let n = client::enable(&mut pdu, ASE_ID, bap::context::UNSPECIFIED);
    ase_step(client, mic, notes, &pdu[..n], AseState::Enabling).await?;

    let acl = conn.handle().raw();
    let mut p = [0u8; 5];
    p[0] = 1;
    p[1..3].copy_from_slice(&cis.to_le_bytes());
    p[3..5].copy_from_slice(&acl.to_le_bytes());
    let status = raw_command(OP_LE_CREATE_CIS, &p).status;
    if status != 0 {
        link::logf(format_args!("LE Create CIS: HCI status {status:#04x}"));
        return Err("LE Create CIS failed");
    }
    match with_timeout(STEP_TIMEOUT, CIS_UP[slot].wait()).await {
        Ok(true) => {}
        Ok(false) => return Err("CIS could not be established"),
        Err(_) => return Err("CIS establishment timed out"),
    }

    let transparent = CodecId { coding_format: 0x03, company_id: 0, vendor_specific_codec_id: 0 };
    let path = LeSetupIsoDataPath::new(
        ConnHandle::new(cis),
        DataPathDirection::Output,
        DataPathId::HCI,
        transparent,
        ExtDuration::from_micros(0),
        &[],
    );
    stack.command(path).await.map_err(|_| "ISO data path setup failed")?;

    let cfg = StreamCfg {
        sampling_hz: codec.sampling_hz,
        frame_duration_us: codec.frame_duration_us,
        octets_per_frame: codec.octets_per_frame,
    };
    with_slot(slot, |s| s.as_mut().map(|i| i.stream = Some(cfg)));
    link::send(&DongleToHost::StreamConfig {
        mic: MicId(slot as u8),
        sampling_hz: cfg.sampling_hz,
        frame_duration_us: cfg.frame_duration_us,
        octets_per_frame: cfg.octets_per_frame,
    });
    let n = client::receiver_start_ready(&mut pdu, ASE_ID);
    ase_step(client, mic, notes, &pdu[..n], AseState::Streaming).await?;
    info!("mic {=usize} streaming {=u32} Hz / {=u16} B", slot, codec.sampling_hz, codec.octets_per_frame);
    Ok(())
}

/// LE Set CIG Parameters for one unidirectional (mic -> dongle) CIS, built by
/// hand: bt-hci 0.10 lays this command out like its Test variant. Returns the
/// CIS handle.
fn set_cig(qos: &bap::ascs::QosConfig) -> Result<u16, &'static str> {
    let mut p = [0u8; 15 + 9];
    let sdu = qos.sdu_interval_us.to_le_bytes();
    p[0] = qos.cig_id;
    p[1..4].copy_from_slice(&sdu[..3]); // SDU_Interval_C_To_P
    p[4..7].copy_from_slice(&sdu[..3]); // SDU_Interval_P_To_C
    p[7] = 0; // Worst_Case_SCA: 251-500 ppm
    p[8] = 0; // Packing: sequential
    p[9] = qos.framing;
    p[10..12].copy_from_slice(&qos.max_transport_latency_ms.to_le_bytes());
    p[12..14].copy_from_slice(&qos.max_transport_latency_ms.to_le_bytes());
    p[14] = 1; // CIS_Count
    p[15] = qos.cis_id;
    p[16..18].copy_from_slice(&0u16.to_le_bytes()); // Max_SDU_C_To_P: nothing to the mic
    p[18..20].copy_from_slice(&qos.max_sdu.to_le_bytes());
    p[20] = qos.phy;
    p[21] = qos.phy;
    p[22] = 0; // RTN_C_To_P
    p[23] = qos.retransmission_number;
    let r = raw_command(OP_LE_SET_CIG_PARAMS, &p);
    match r.ret() {
        [_cig, 1, lo, hi, ..] if r.status == 0 => Ok(u16::from_le_bytes([*lo, *hi])),
        _ => {
            warn!("LE Set CIG Parameters: status {=u8:#x}", r.status);
            Err("CIG setup failed")
        }
    }
}

/// Bring the ASE back to QoS Configured (or Codec Configured) and free the CIS/CIG.
async fn stop_stream(slot: usize, client: &Client<'_>, mic: &Mic, notes: &mut Notes<'_>, stack: &'static BleStack) {
    let mut pdu = [0u8; client::CP_PDU_MAX_LEN];
    let n = client::disable(&mut pdu, ASE_ID);
    if ase_step(client, mic, notes, &pdu[..n], AseState::Disabling).await.is_ok() {
        let n = client::receiver_stop_ready(&mut pdu, ASE_ID);
        let _ = ase_step(client, mic, notes, &pdu[..n], AseState::QosConfigured).await;
    }
    if let Some(cis) = with_slot(slot, |s| s.and_then(|i| i.cis)) {
        remove_iso(slot, cis, stack).await;
    }
}

async fn remove_iso(slot: usize, cis: u16, stack: &'static BleStack) {
    let cmd = Disconnect::new(ConnHandle::new(cis), DisconnectReason::RemoteUserTerminatedConn);
    if stack.command(cmd).await.is_ok() {
        let _ = with_timeout(Duration::from_secs(2), CIS_DOWN[slot].wait()).await;
    }
    let r = raw_command(OP_LE_REMOVE_CIG, &[slot as u8]);
    if r.status != 0 {
        warn!("LE Remove CIG {=usize}: status {=u8:#x}", slot, r.status);
    }
    with_slot(slot, |s| s.as_mut().map(|i| { i.cis = None; i.stream = None }));
}
