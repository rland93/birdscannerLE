//! Bond persistence: LE Secure Connections keys survive a reset, so a paired
//! client can reconnect (and re-encrypt) without pairing again.
//!
//! Bonds live in a sequential-storage map in the last 16 KiB of app-core
//! flash (excluded from the program region in `memory.x`), keyed by the
//! peer's identity address.

use core::ops::Range;

use defmt::{info, warn};
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_nrf::Peri;
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::peripherals::NVMC;
use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::map::{MapConfig, MapStorage, PostcardValue};
use serde::{Deserialize, Serialize};
use trouble_host::prelude::*;
use trouble_host::BondInformation;

/// Must match the FLASH length in `memory.x`.
const BOND_FLASH: Range<u32> = 0x000F_C000..0x0010_0000;

#[derive(Serialize, Deserialize)]
struct StoredBond(BondInformation);
impl PostcardValue<'_> for StoredBond {}

type Flash = BlockingAsync<Nvmc<'static>>;

pub struct BondStore {
    map: MapStorage<[u8; 7], Flash, Cache<Uncached, Uncached, Uncached, [u8; 7]>>,
    buf: [u8; 128],
}

impl BondStore {
    pub fn new(nvmc: Peri<'static, NVMC>) -> Self {
        let flash = BlockingAsync::new(Nvmc::new(nvmc));
        Self { map: MapStorage::new(flash, MapConfig::new(BOND_FLASH), Cache::new_uncached()), buf: [0; 128] }
    }

    /// Hand every stored bond to the host.
    pub async fn load<C: Controller, P: PacketPool>(&mut self, stack: &Stack<'_, C, P>) {
        let mut items = match self.map.fetch_all_items(&mut self.buf).await {
            Ok(items) => items,
            Err(e) => {
                warn!("bond storage unreadable: {}", e);
                return;
            }
        };
        let mut count = 0u32;
        let mut buf = [0u8; 128];
        loop {
            match items.next::<StoredBond>(&mut buf).await {
                Ok(Some((_, StoredBond(bond)))) => {
                    // Older copies of the same key come first; the host replaces them.
                    let _ = stack.remove_bond_information(bond.identity);
                    match stack.add_bond_information(bond) {
                        Ok(()) => count += 1,
                        Err(e) => warn!("bond not restored: {}", e),
                    }
                }
                Ok(None) => break,
                Err(e) => {
                    warn!("bond storage read error: {}", e);
                    break;
                }
            }
        }
        info!("restored {=u32} bond record(s)", count);
    }

    pub async fn save(&mut self, bond: &BondInformation) {
        let key = bond.identity.addr.to_bytes();
        match self.map.store_item(&mut self.buf, &key, &StoredBond(bond.clone())).await {
            Ok(()) => info!("bond with {} stored", bond.identity.addr),
            Err(e) => warn!("storing bond failed: {}", e),
        }
    }
}
