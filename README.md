# nrf53bird

An I2S MEMS microphone on the nRF5340 DK that shows up as a Bluetooth LE
Audio source: connectable, pairable (LE Secure Connections, bonds kept in
flash), BAP Unicast Server with one Source ASE, LC3 over a CIS.

Everything we wrote is Rust. The BLE link layer is Nordic's SoftDevice
Controller (a precompiled library) running on the network core.

```
app core (Cortex-M33 @128 MHz, firmware/)      net core (netcore/)
  I2S mic -> LC3 encoder (crates/lc3)            SoftDevice Controller + MPSL
  trouble-host: GAP/GATT/SMP, ISO                  ^
  PACS/ASCS/CAS (crates/bap)                       | HCI (H4 records)
  hci_link: HCI transport  <---- shared RAM rings (crates/hci-shm) + IPC
```

## Layout

| Path | What |
|------|------|
| `firmware/` | App core: mic capture, LC3, BLE host, LE Audio server |
| `netcore/` | Net core: SoftDevice Controller exposed as an HCI controller |
| `crates/hci-shm` | HCI rings in shared RAM (top 16 KiB of app RAM) |
| `crates/bap` | PACS/ASCS values and the ASE state machine (no I/O, host-tested) |
| `crates/lc3` | no_std port of the pure-Rust `lc3` 0.1.0 codec (bit-exact with liblc3) |
| `mic-capture/` | Host tool: records the raw-PCM RTT tap to WAV |

## Wiring

| Mic  | DK pin      | nRF I2S |
|------|-------------|---------|
| BCLK | P1.15 (D13) | SCK (nRF is master) |
| LRCLK| P1.12 (D10) | LRCK    |
| DOUT | P1.14 (D12) | SDIN (internal pull-down) |
| SEL  | GND = left, VDD = right | auto-detected |

Clocks: audio PLL at 12.288 MHz, LRCK exactly 32 kHz, SCK 2.048 MHz, 32-bit
slots. One 10 ms DMA block is one LC3 frame.

## Flash and run

The network core only needs flashing when `netcore/` changes. Its runner uses
the full `nRF5340_xxAA` target so probe-rs can reach core 1; the app firmware
must be running (it releases the net core from reset).

```sh
cd netcore && cargo run --release     # probe-rs download to the net core
cd firmware && cargo run --release    # flash app core + defmt log
```

The app runner uses `nRF5340_xxAA_APPONLY` so probe-rs never touches (or
mass-erases) the network core.

Log: mic levels twice a second, BLE/LE Audio events (`connected`, `paired`,
`ASE 1 -> Streaming`, `LC3: N SDUs, M dropped, worst encode T us`).
LED1..LED4: VU meter, block peak >= -50/-40/-30/-20 dBFS.

## LE Audio

- Advertises as `nrf53bird` (appearance: Microphone) with ASCS and CAS
  service data. Legacy connectable advertising, name in the scan response.
- PACS: LC3, 16/24/32 kHz, 10 ms frames, 26..80 octets/frame, 1 channel,
  Front Left. Source contexts: Unspecified, Conversational, Media, Game,
  Voice Assistants, Live.
- All PACS/ASCS characteristics require encryption, so clients pair first
  (Just Works). Bonds are stored in the last 16 KiB of app flash.
- When the client enables the Source ASE: accept its CIS, set up an HCI input
  data path, start LC3 on Receiver Start Ready. Encoding 32 kHz/80 B takes
  ~3.4 ms per 10 ms frame.

### Testing with Linux (BlueZ + PipeWire)

The PC needs BlueZ's ISO socket and experimental D-Bus interfaces, and
PipeWire's BAP roles:

```sh
# /etc/bluetooth/main.conf
Experimental = true
KernelExperimental = 6fbaf188-05e0-496a-9885-d6ddfdb4e03e
sudo systemctl restart bluetooth

# ~/.config/wireplumber/wireplumber.conf.d/90-nrf53bird-bap.conf
monitor.bluez.properties = {
  bluez5.roles = [ a2dp_sink a2dp_source bap_sink bap_source hsp_hs hsp_ag hfp_hf hfp_ag ]
}
systemctl --user restart pipewire wireplumber
```

Then pair (`bluetoothctl pair <addr>`); `nrf53bird` appears as a PipeWire
source:

```sh
pw-record --target nrf53bird --rate 32000 --channels 1 out.wav
```

macOS has no user-facing LE Audio client.

### Raw PCM over RTT

Independently of Bluetooth, the firmware streams DC-blocked 16-bit mono PCM
(32 kHz) on RTT channel 1. With no other probe-rs session attached:

```sh
cd mic-capture && cargo run --release -- out.wav --seconds 10
```

## Notes

- `firmware` enables the app core I-cache and 128 MHz HCLK at boot
  (`full_speed_cpu`); embassy-nrf leaves both at reset defaults (off, 64 MHz),
  which made LC3 encoding ~5x slower than real time.
- `netcore/src/dispatch.rs` maps HCI opcodes to SoftDevice Controller command
  functions; it was generated from the nrfxlib headers in nrf-sdc-sys 0.3.0.
- trouble-host 0.8.0's `iso` feature only compiles together with `subrating`.
