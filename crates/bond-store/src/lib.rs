//! Persistent LE Secure Connections bonds for trouble-host, kept in a
//! sequential-storage map in a reserved flash range and keyed by the peer's
//! identity address. Shared by the mic firmware (peripheral) and the receiver
//! dongle (central) so both sides survive resets without re-pairing.

#![no_std]

use core::ops::Range;

use defmt::{info, warn};
use embedded_storage_async::nor_flash::{MultiwriteNorFlash, NorFlash};
use sequential_storage::cache::{Cache, Uncached};
use sequential_storage::map::{MapConfig, MapStorage, PostcardValue};
use serde::{Deserialize, Serialize};
use trouble_host::prelude::*;
use trouble_host::{BondInformation, Identity};

#[derive(Serialize, Deserialize)]
struct StoredBond(BondInformation);
impl PostcardValue<'_> for StoredBond {}

/// Postcard-encoded bond plus key; comfortably above the ~60 bytes needed.
const BUF: usize = 128;

pub struct BondStore<S: NorFlash> {
    map: MapStorage<[u8; 7], S, Cache<Uncached, Uncached, Uncached, [u8; 7]>>,
    buf: [u8; BUF],
}

impl<S: NorFlash> BondStore<S>
where
    S::Error: defmt::Format,
{
    /// `range`: page-aligned flash range reserved for bonds (at least two pages).
    pub fn new(flash: S, range: Range<u32>) -> Self {
        Self { map: MapStorage::new(flash, MapConfig::new(range), Cache::new_uncached()), buf: [0; BUF] }
    }

    /// Hand every stored bond to the host. Returns how many were restored.
    pub async fn load<C: Controller, P: PacketPool>(&mut self, stack: &Stack<'_, C, P>) -> u32 {
        let mut items = match self.map.fetch_all_items(&mut self.buf).await {
            Ok(items) => items,
            Err(e) => {
                warn!("bond storage unreadable: {}", e);
                return 0;
            }
        };
        let mut count = 0u32;
        let mut buf = [0u8; BUF];
        loop {
            match items.next::<StoredBond>(&mut buf).await {
                Ok(Some((_, StoredBond(bond)))) => {
                    // Older copies of the same key may come first; keep the latest.
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
        count
    }

    pub async fn save(&mut self, bond: &BondInformation) {
        let key = bond.identity.addr.to_bytes();
        match self.map.store_item(&mut self.buf, &key, &StoredBond(bond.clone())).await {
            Ok(()) => info!("bond with {} stored", bond.identity.addr),
            Err(e) => warn!("storing bond failed: {}", e),
        }
    }
}

impl<S: MultiwriteNorFlash> BondStore<S>
where
    S::Error: defmt::Format,
{
    /// Delete the stored bond (if any) for `identity` and drop it from the host.
    pub async fn forget<C: Controller, P: PacketPool>(&mut self, stack: &Stack<'_, C, P>, identity: Identity) {
        let key = identity.addr.to_bytes();
        if let Err(e) = self.map.remove_item(&mut self.buf, &key).await {
            warn!("removing bond failed: {}", e);
        }
        let _ = stack.remove_bond_information(identity);
    }
}
