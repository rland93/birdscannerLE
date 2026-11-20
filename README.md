# birdscannerLE

Garden microphones over Bluetooth LE Audio.

- **Mic** (`firmware/` + `netcore/`, nRF5340 DK): an I2S MEMS mic exposed as a
  BAP Unicast Server with one Source ASE, LC3 over a CIS.
- **Receiver** (`dongle/`, nRF52840 Dongle): BAP Unicast Client for up to
  three mics; forwards their LC3 frames over USB.
- **Host app** (`garden-host/`, Windows/macOS/Linux): talks to the receiver,
  decodes LC3, and serves a web UI for live listening, levels and recordings.

Everything we wrote is Rust. The BLE link layer is Nordic's SoftDevice
Controller (a precompiled library).

```
mic: app core (firmware/)              mic: net core (netcore/)
  I2S -> LC3 encode (crates/lc3)         SoftDevice Controller
  trouble-host, PACS/ASCS (crates/bap)     ^ raw HCI (crates/sdc-hci)
  hci_link  <---- shared RAM HCI (crates/hci-shm) + IPC
        |
        | BLE: connection + CIS (LC3)
        v
receiver: nRF52840 dongle (dongle/)        host (garden-host/)
  SoftDevice Controller (crates/sdc-hci)     USB serial (crates/garden-link)
  trouble-host, BAP client (crates/bap) ---> LC3 decode, web UI, WAV files
```

## Layout

| Path | What |
|------|------|
| `firmware/` | Mic app core: capture, LC3 encode, BLE host, LE Audio server |
| `netcore/` | Mic net core: SoftDevice Controller as an HCI controller over shared RAM |
| `dongle/` | Receiver: SoftDevice Controller + BLE host + BAP client + USB CDC-ACM |
| `garden-host/` | Host app: receiver link, LC3 decode, web UI (Askama + Alloy CSS), recordings |
| `crates/garden-link` | Receiver <-> host protocol (postcard + COBS over USB serial) |
| `crates/bap` | PACS/ASCS server logic and unicast-client parsers/PDU builders (no I/O, host-tested) |
| `crates/sdc-hci` | Raw HCI command dispatch and packet fetch for the SoftDevice Controller (both chips) |
| `crates/hci-shm` | HCI rings in shared RAM between the nRF5340 cores |
| `crates/lc3` | no_std port of the pure-Rust `lc3` 0.1.0 codec (bit-exact with liblc3) |
| `crates/bond-store` | Flash-backed bonds for trouble-host (mic) |
| `mic-capture/` | Debug tool: records the mic's raw-PCM RTT tap to WAV |

## Mic

### Wiring

| Mic  | DK pin      | nRF I2S |
|------|-------------|---------|
| BCLK | P1.15 (D13) | SCK (nRF is master) |
| LRCLK| P1.12 (D10) | LRCK    |
| DOUT | P1.14 (D12) | SDIN (internal pull-down) |
| SEL  | GND = left, VDD = right | auto-detected |

Clocks: audio PLL at 12.288 MHz, LRCK exactly 32 kHz, SCK 2.048 MHz, 32-bit
slots. One 10 ms DMA block is one LC3 frame.

### Flash and run

The network core only needs flashing when `netcore/` changes. Its runner uses
the full `nRF5340_xxAA` target so probe-rs can reach core 1; the app firmware
must be running (it releases the net core from reset).

```sh
cd netcore && cargo run --release     # probe-rs download to the net core
cd firmware && cargo run --release    # flash app core + defmt log
```

The app runner uses `nRF5340_xxAA_APPONLY` so probe-rs never touches (or
mass-erases) the network core.

Log: mic levels twice a second, BLE/LE Audio events (`connected`,
`ASE 1 -> Streaming`, `LC3: N SDUs, M dropped, worst encode T us`).
LED1..LED4: VU meter, block peak >= -50/-40/-30/-20 dBFS.

### LE Audio

- Advertises as `birdscannerLE` (appearance: Microphone) with ASCS and CAS
  service data. Legacy connectable advertising, name in the scan response.
- PACS: LC3, 16/24/32 kHz, 10 ms frames, 26..80 octets/frame, 1 channel,
  Front Left. Source contexts: Unspecified, Conversational, Media, Game,
  Voice Assistants, Live.
- **The LE Audio characteristics do not require encryption**, because the
  nRF52840 receiver cannot encrypt isochronous channels (Nordic: only nRF52820
  and nRF52833 can among the nRF52 Series) and a CIS on an encrypted link must
  be encrypted. Anyone in range with an LE Audio client can listen. Clients
  that pair anyway (BlueZ) get an encrypted link and CIS; those bonds are
  kept in the last 16 KiB of app flash.
- When the client enables the Source ASE: accept its CIS, set up an HCI input
  data path, start LC3 on Receiver Start Ready. Encoding 32 kHz/80 B takes
  ~3.4 ms per 10 ms frame.

## Receiver dongle

nRF52840 Dongle (PCA10059) with Nordic's Open USB DFU bootloader, which is
kept: the firmware is linked behind the MBR and flashed over USB.

```sh
# press the dongle's RESET button (red LED pulses), then:
cd dongle && cargo run --release      # builds and flashes via ./flash.sh (nrfutil)
```

Needs `nrfutil` with `nrf5sdk-tools` (`nrfutil install nrf5sdk-tools`). The
dongle enumerates as `1209:0001` "birdscannerLE receiver" (a CDC-ACM serial port,
driverless on all three OSes). It scans for ASCS adverts, connects mics on
request, configures each Source ASE (one CIG per mic) and forwards LC3 frames
with their ISO sequence numbers; lost frames are sent empty.

Details:
- trouble-host drives the controller through `crates/sdc-hci`, the same raw
  HCI layer as the mic's net core: nrf-sdc 0.4's own controller panics on
  incoming ISO data.
- LE Set CIG Parameters and LE Create CIS are issued raw: bt-hci 0.10 encodes
  both with the wrong field layout.

## Host app

```sh
cd garden-host && cargo run --release -- [--port PATH] [--listen 127.0.0.1:8080] [--data-dir DIR]
```

Finds the receiver by USB ID, then serves the UI: add mics (scan), per-mic
state, RSSI and level, Listen (browser, WebSocket + AudioWorklet), Start/Stop
stream, Record (hourly WAV files under `<data-dir>/recordings/<mic>/<date>/`),
and a Recordings page. Mics stream only while someone listens, records, or
pressed Start, to save mic power. Known mics are reconnected with backoff.

Live listening needs no HTTPS: browsers only expose AudioWorklet in a secure
context (https or localhost), so over plain `http://` on a LAN address the
page falls back to a ScriptProcessorNode with the same jitter buffer. Any
audio problem (blocked autoplay, stopped stream) is reported in a banner at
the bottom of the page.

## Testing the mic with Linux (BlueZ + PipeWire)

Without the dongle, a Linux PC can be the LE Audio client. It needs BlueZ's
ISO socket and experimental D-Bus interfaces, and PipeWire's BAP roles:

```sh
# /etc/bluetooth/main.conf
Experimental = true
KernelExperimental = 6fbaf188-05e0-496a-9885-d6ddfdb4e03e
sudo systemctl restart bluetooth

# ~/.config/wireplumber/wireplumber.conf.d/90-birdscannerLE-bap.conf
monitor.bluez.properties = {
  bluez5.roles = [ a2dp_sink a2dp_source bap_sink bap_source hsp_hs hsp_ag hfp_hf hfp_ag ]
}
systemctl --user restart pipewire wireplumber
```

Then connect (`bluetoothctl connect <addr>`); `birdscannerLE` appears as a
PipeWire source:

```sh
pw-record --target birdscannerLE --rate 32000 --channels 1 out.wav
```

The mic takes one connection at a time: disconnect the PC before using the
dongle, and vice versa.

### Raw PCM over RTT

Independently of Bluetooth, the mic firmware streams DC-blocked 16-bit mono
PCM (32 kHz) on RTT channel 1. With no other probe-rs session attached:

```sh
cd mic-capture && cargo run --release -- out.wav --seconds 10
```

## Notes

- `firmware` enables the app core I-cache and 128 MHz HCLK at boot
  (`full_speed_cpu`); embassy-nrf leaves both at reset defaults (off, 64 MHz),
  which made LC3 encoding ~5x slower than real time.
- `crates/sdc-hci/src/dispatch.rs` maps HCI opcodes to SoftDevice Controller
  command functions; it was generated from the nrfxlib headers in
  nrf-sdc-sys 0.3.0.
- trouble-host 0.8.0's `iso` feature only compiles together with `subrating`.
