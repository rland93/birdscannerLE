# nrf53bird

I2S MEMS microphone bring-up on the nRF5340 DK application core, all Rust
(embassy-nrf + a register-level I2S receiver).

## Wiring

| Mic  | DK pin      | nRF I2S |
|------|-------------|---------|
| BCLK | P1.15 (D13) | SCK (nRF is master) |
| LRCLK| P1.12 (D10) | LRCK    |
| DOUT | P1.14 (D12) | SDIN (internal pull-down) |
| SEL  | GND = left, VDD = right | auto-detected |

Clocks: MCK 2 MHz, LRCK 31 250 Hz, SCK 2 MHz, 32-bit slots (64 SCK/frame),
standard I2S format.

## Run

```sh
cd firmware
cargo run --release      # flash + defmt log (RTT channel 0)
```

The log prints per-slot RMS/peak (dBFS re 24-bit full scale), DC offset and
the measured LRCK rate about twice a second, and reports which slot the mic
is on. LED1..LED4 form a VU meter: block peak ≥ -50/-40/-30/-20 dBFS.

The target uses the `nRF5340_xxAA_APPONLY` probe-rs chip so probe-rs never
touches (or mass-erases) the network core.

## Record audio

The firmware streams DC-blocked 16-bit mono PCM on RTT channel 1 (`pcm`).
With the firmware running and no other probe-rs session attached:

```sh
cd mic-capture
cargo run --release -- out.wav --seconds 10
```

It attaches without reset, zero-fills and reports any dropped frames. The
DK's J-Link OB needs the 2 MHz SWD clock (default) to keep up with 62.5 kB/s.
