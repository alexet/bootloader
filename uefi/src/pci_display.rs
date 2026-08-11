//! Resolve the PCI(e) function backing a UEFI display handle (the GOP
//! handle), so the kernel can later hand the *same* physical device off to a
//! real GPU driver (see `bootloader_api::info::BootInfo::display_pci_device`).
//!
//! A `DevicePath`'s `Hardware::Pci` nodes only encode a (device, function)
//! pair per hop — the bus number of each hop is implicit in the topology, not
//! stored in the path. So this walks the hops from the root complex (bus 0)
//! down, re-deriving each bus number from PCI configuration space via
//! `PciRootBridgeIo`, the same way firmware itself would.

use bootloader_api::info::PciDeviceLocation;
use uefi::Identify;
use uefi::boot::{self, OpenProtocolAttributes, OpenProtocolParams, ScopedProtocol, SearchType};
use uefi::proto::ProtocolPointer;
use uefi::proto::device_path::{DevicePath, hardware};
use uefi::proto::pci::PciIoAddress;
use uefi::proto::pci::root_bridge::PciRootBridgeIo;

/// PCI config space offsets used below (see the PCI spec's type 0/1 header
/// layout — the same registers `bootloader-x86_64-uefi`'s upstream would read
/// if it needed to enumerate bridges).
const REG_HEADER_TYPE_DWORD: u8 = 0x0C;
const REG_BUS_NUMBERS_DWORD: u8 = 0x18;

/// Opens `P` on `handle` non-exclusively (`GetProtocol`), for read-only
/// queries. Exclusive opens are deliberately avoided here: per the UEFI spec,
/// `Exclusive`/`ByDriverExclusive` attempt to *disconnect* any driver that
/// already has the same protocol interface open `ByDriver` — and against
/// shared bus infrastructure like `PciRootBridgeIo`, or a GOP handle's own
/// `DevicePath`, that can knock out unrelated device stacks sitting on the
/// same handle. (Observed in practice: opening either exclusively reset GOP
/// into Blt-only and broke the boot disk driver living on the same PCI root
/// bridge as the display.)
///
/// # Safety
///
/// The returned protocol is only ever used here to issue read-only queries
/// (device path node iteration, PCI config-space reads) for the duration of
/// one call, with no concurrent boot-service activity expected to
/// uninstall/reinstall it — satisfying `GetProtocol`'s safety contract.
unsafe fn open_get_protocol<P: ProtocolPointer + ?Sized>(
    handle: uefi::Handle,
) -> Option<ScopedProtocol<P>> {
    unsafe {
        boot::open_protocol::<P>(
            OpenProtocolParams {
                handle,
                agent: boot::image_handle(),
                controller: None,
            },
            OpenProtocolAttributes::GetProtocol,
        )
        .ok()
    }
}

/// Resolve the PCI(e) function backing `handle`'s device path.
///
/// Returns `None` if `handle` has no device path, the path has no PCI
/// hardware nodes (the display isn't backed by a real PCI device), or no
/// root bridge's configuration space confirms a device at the resolved
/// address (firmware and OS-visible topology disagreeing, which shouldn't
/// happen but isn't worth panicking over).
pub fn resolve(handle: uefi::Handle) -> Option<PciDeviceLocation> {
    let device_path = unsafe { open_get_protocol::<DevicePath>(handle) }?;
    let (hops, hop_count) = pci_hops(&device_path)?;
    let hops = &hops[..hop_count];

    // Normally there's exactly one root bridge/segment on the targets this
    // bootloader supports (QEMU, real single-socket x86_64 desktops/laptops).
    // Try each one and keep whichever confirms a device at the resolved
    // address, rather than assuming the first is correct.
    let root_bridge_handles =
        boot::locate_handle_buffer(SearchType::ByProtocol(&PciRootBridgeIo::GUID)).ok()?;
    for &rb_handle in root_bridge_handles.iter() {
        let Some(mut root_bridge) = (unsafe { open_get_protocol::<PciRootBridgeIo>(rb_handle) })
        else {
            continue;
        };
        if let Some(addr) = resolve_on_root_bridge(&mut root_bridge, hops) {
            return Some(PciDeviceLocation {
                segment: root_bridge.segment_nr() as u16,
                bus: addr.bus,
                device: addr.dev,
                function: addr.fun,
            });
        }
    }
    None
}

/// Collect the (device, function) pair of each `Hardware::Pci` node in
/// `device_path`, in root-to-leaf order. A device directly on the root
/// complex has exactly one; a device behind one or more PCI-to-PCI bridges
/// has more. Returns the hops and how many of the fixed 8 slots are used —
/// 8 is already an implausibly deep PCI topology, so a path with more nodes
/// than that is rejected rather than silently truncated.
fn pci_hops(device_path: &DevicePath) -> Option<([(u8, u8); 8], usize)> {
    let mut hops = [(0u8, 0u8); 8];
    let mut count = 0;
    for node in device_path.node_iter() {
        if let Ok(pci) = <&hardware::Pci>::try_from(node) {
            if count == hops.len() {
                return None;
            }
            hops[count] = (pci.device(), pci.function());
            count += 1;
        }
    }
    if count == 0 {
        None
    } else {
        Some((hops, count))
    }
}

/// Walk `hops` starting from bus 0 of `root_bridge`, confirming each hop's
/// presence via its vendor ID and, for non-leaf hops, descending through its
/// secondary bus number — the same walk a PCI-to-PCI bridge enumerator does,
/// just directed by `hops` instead of exhaustively scanning every device.
fn resolve_on_root_bridge(
    root_bridge: &mut PciRootBridgeIo,
    hops: &[(u8, u8)],
) -> Option<PciIoAddress> {
    let mut bus = 0u8;
    let mut addr = PciIoAddress::new(0, 0, 0);
    for (i, &(device, function)) in hops.iter().enumerate() {
        addr = PciIoAddress::new(bus, device, function);

        let vendor_device: u32 = root_bridge.pci().read_one(addr.with_register(0)).ok()?;
        if (vendor_device & 0xFFFF) as u16 == 0xFFFF {
            return None; // nothing at this address on this root bridge
        }

        if i + 1 < hops.len() {
            // More hops follow, so this address must be a PCI-to-PCI bridge:
            // confirm the header type, then descend into its secondary bus.
            let header: u32 = root_bridge
                .pci()
                .read_one(addr.with_register(REG_HEADER_TYPE_DWORD))
                .ok()?;
            let header_type = ((header >> 16) & 0x7F) as u8;
            if header_type != 0x01 {
                return None;
            }
            let bus_numbers: u32 = root_bridge
                .pci()
                .read_one(addr.with_register(REG_BUS_NUMBERS_DWORD))
                .ok()?;
            bus = ((bus_numbers >> 8) & 0xFF) as u8;
        }
    }
    Some(addr)
}
