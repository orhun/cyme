//! System USB profilers
//!
//! Get [`system_profiler::SPUSBDataType`] struct of system USB buses and devices with extra data like configs, interfaces and endpoints
//!
//! ```no_run
//! use cyme::usb::profiler;
//!
//! let spusb = profiler::get_spusb_with_extra().unwrap();
//! // print with alternative styling (#) is using utf-8 icons
//! println!("{:#}", spusb);
//! ```
//!
//! See [`system_profiler`] docs for what can be done with returned data, such as [`system_profiler::USBFilter`]
use crate::error::Result;
use itertools::Itertools;
use std::collections::HashMap;

use crate::error::{Error, ErrorKind};
use crate::system_profiler;
#[cfg(all(target_os = "linux", any(feature = "udev", feature = "udevlib")))]
use crate::udev;
use crate::usb;

const REQUEST_GET_DESCRIPTOR: u8 = 0x06;
const REQUEST_GET_STATUS: u8 = 0x00;

/// Transfer direction
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub(crate) enum Direction {
    /// Host to device
    Out = 0,
    /// Device to host
    In = 1,
}

/// Specification defining the request.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub(crate) enum ControlType {
    /// Request defined by the USB standard.
    Standard = 0,
    /// Request defined by the standard USB class specification.
    Class = 1,
    /// Non-standard request.
    Vendor = 2,
}

/// Entity targeted by the request.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub(crate) enum Recipient {
    /// Request made to device as a whole.
    Device = 0,
    /// Request made to specific interface.
    Interface = 1,
    /// Request made to specific endpoint.
    Endpoint = 2,
    /// Other request.
    Other = 3,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) struct ControlRequest {
    pub control_type: ControlType,
    pub recipient: Recipient,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: usize,
}

pub(crate) trait UsbOperations {
    fn get_descriptor_string(&self, string_index: u8) -> Option<String>;
    fn get_control_msg(&self, control_request: &ControlRequest) -> Result<Vec<u8>>;
}

pub(crate) trait Profiler<T>
where
    T: UsbOperations,
    Self: std::fmt::Debug,
{
    fn get_report_descriptor(device: &T, index: u16, length: u16) -> Result<Vec<u8>> {
        let control_request = ControlRequest {
            control_type: ControlType::Standard,
            recipient: Recipient::Interface,
            request: REQUEST_GET_DESCRIPTOR,
            value: (u8::from(usb::DescriptorType::Report) as u16) << 8,
            index,
            length: length as usize,
        };
        device.get_control_msg(&control_request)
    }

    fn get_hub_descriptor(
        device: &T,
        protocol: u8,
        bcd: u16,
        has_ssp: bool,
    ) -> Result<usb::HubDescriptor> {
        let is_ext_status = protocol == 3 && bcd >= 0x0310 && has_ssp;
        let value = if bcd >= 0x0300 {
            (u8::from(usb::DescriptorType::SuperSpeedHub) as u16) << 8
        } else {
            (u8::from(usb::DescriptorType::Hub) as u16) << 8
        };
        let control = ControlRequest {
            control_type: ControlType::Class,
            request: REQUEST_GET_DESCRIPTOR,
            value,
            index: 0,
            recipient: Recipient::Device,
            length: 9,
        };
        let data = device.get_control_msg(&control)?;
        let mut hub = usb::HubDescriptor::try_from(data.as_slice())?;

        // get port statuses
        let mut port_statues: Vec<[u8; 8]> = Vec::with_capacity(hub.num_ports as usize);
        for p in 0..hub.num_ports {
            let control = ControlRequest {
                control_type: ControlType::Class,
                request: REQUEST_GET_STATUS,
                index: p as u16 + 1,
                value: 0x23 << 8,
                recipient: Recipient::Other,
                length: if is_ext_status { 8 } else { 4 },
            };
            match device.get_control_msg(&control) {
                Ok(mut data) => {
                    if data.len() < 8 {
                        let remaining = 8 - data.len();
                        data.extend(vec![0; remaining]);
                    }
                    port_statues.push(data.try_into().unwrap());
                }
                Err(e) => {
                    log::warn!("Failed to get port {} status: {}", p + 1, e);
                    return Ok(hub);
                }
            }
        }

        hub.port_statuses = Some(port_statues);

        Ok(hub)
    }

    fn get_device_status(device: &T) -> Result<u16> {
        let control = ControlRequest {
            control_type: ControlType::Standard,
            request: REQUEST_GET_STATUS,
            value: 0,
            index: 0,
            recipient: Recipient::Device,
            length: 2,
        };
        let data = device.get_control_msg(&control)?;
        Ok(u16::from_le_bytes([data[0], data[1]]))
    }

    fn get_debug_descriptor(device: &T) -> Result<usb::DebugDescriptor> {
        let control = ControlRequest {
            control_type: ControlType::Standard,
            request: REQUEST_GET_DESCRIPTOR,
            value: (u8::from(usb::DescriptorType::Debug) as u16) << 8,
            index: 0,
            recipient: Recipient::Device,
            length: 2,
        };
        let data = device.get_control_msg(&control)?;
        usb::DebugDescriptor::try_from(data.as_slice())
    }

    fn get_bos_descriptor(
        device: &T,
    ) -> Result<usb::descriptors::bos::BinaryObjectStoreDescriptor> {
        let mut control = ControlRequest {
            control_type: ControlType::Standard,
            request: REQUEST_GET_DESCRIPTOR,
            value: (u8::from(usb::DescriptorType::Bos) as u16) << 8,
            index: 0,
            recipient: Recipient::Device,
            length: 5,
        };
        let data = device.get_control_msg(&control)?;
        let total_length = u16::from_le_bytes([data[2], data[3]]);
        log::trace!("Attempt read BOS descriptor total length: {}", total_length);
        // now get full descriptor
        control.length = total_length as usize;
        let data = device.get_control_msg(&control)?;
        log::trace!("BOS descriptor data: {:?}", data);
        let mut bos =
            usb::descriptors::bos::BinaryObjectStoreDescriptor::try_from(data.as_slice())?;

        // get any extra descriptor data now with handle
        for c in bos.capabilities.iter_mut() {
            match c {
                usb::descriptors::bos::BosCapability::WebUsbPlatform(w) => {
                    w.url = Self::get_webusb_url(device, w.vendor_code, w.landing_page_index).ok();
                    log::trace!("WebUSB URL: {:?}", w.url);
                }
                usb::descriptors::bos::BosCapability::Billboard(ref mut b) => {
                    b.additional_info_url =
                        device.get_descriptor_string(b.additional_info_url_index);
                    for a in b.alternate_modes.iter_mut() {
                        a.alternate_mode_string =
                            device.get_descriptor_string(a.alternate_mode_string_index);
                    }
                }
                _ => (),
            }
        }

        Ok(bos)
    }

    fn get_device_qualifier(device: &T) -> Result<usb::DeviceQualifierDescriptor> {
        let control = ControlRequest {
            control_type: ControlType::Standard,
            request: REQUEST_GET_DESCRIPTOR,
            value: 0x06 << 8,
            index: 0,
            recipient: Recipient::Device,
            length: 10,
        };
        let data = device.get_control_msg(&control)?;
        log::trace!("Device Qualifier descriptor data: {:?}", data);
        usb::DeviceQualifierDescriptor::try_from(data.as_slice())
    }

    /// Gets the WebUSB URL from the device, parsed and formatted as a URL
    ///
    /// https://github.com/gregkh/usbutils/blob/master/lsusb.c#L3261
    fn get_webusb_url(device: &T, vendor_request: u8, index: u8) -> Result<String> {
        let control = ControlRequest {
            control_type: ControlType::Vendor,
            request: vendor_request,
            value: (usb::WEBUSB_GET_URL as u16) << 8,
            index: index as u16,
            recipient: Recipient::Device,
            length: 3,
        };
        let data = device.get_control_msg(&control)?;
        log::trace!("WebUSB URL descriptor data: {:?}", data);
        let len = data[0] as usize;

        if data[1] != usb::USB_DT_WEBUSB_URL {
            return Err(Error {
                kind: ErrorKind::Parsing,
                message: "Failed to parse WebUSB URL: Bad URL descriptor type".to_string(),
            });
        }

        if data.len() < len {
            return Err(Error {
                kind: ErrorKind::Parsing,
                message: "Failed to parse WebUSB URL: Data length mismatch".to_string(),
            });
        }

        let url = String::from_utf8(data[3..len].to_vec()).map_err(|e| Error {
            kind: ErrorKind::Parsing,
            message: format!("Failed to parse WebUSB URL: {}", e),
        })?;

        match data[2] {
            0x00 => Ok(format!("http://{}", url)),
            0x01 => Ok(format!("https://{}", url)),
            0xFF => Ok(url),
            _ => Err(Error {
                kind: ErrorKind::Parsing,
                message: "Failed to parse WebUSB URL: Bad URL scheme".to_string(),
            }),
        }
    }

    /// Build fully described USB device descriptor with extra bytes
    fn build_descriptor_extra<C: Into<usb::ClassCode> + Copy>(
        &self,
        device: &T,
        class_code: Option<usb::ClassCodeTriplet<C>>,
        interface_number: Option<u8>,
        extra_bytes: &[u8],
    ) -> Result<usb::Descriptor> {
        // Get any extra descriptors into a known type and add any handle data while we have it
        let mut dt = match usb::Descriptor::try_from(extra_bytes) {
            Ok(d) => d,
            Err(e) => {
                log::debug!("Failed to convert extra descriptor bytes: {}", e);
                return Err(e);
            }
        };

        // Assign class context to interface since descriptor did not know it
        if let Some(interface_desc) = class_code {
            if let Err(e) = dt.update_with_class_context(interface_desc) {
                log::debug!(
                    "Failed to update extra descriptor with class context: {}",
                    e
                );
            }
        }

        // get any strings at string indexes while we have handle
        match dt {
            usb::Descriptor::InterfaceAssociation(ref mut iad) => {
                iad.function_string = device.get_descriptor_string(iad.function_string_index);
            }
            usb::Descriptor::Device(ref mut c)
            | usb::Descriptor::Interface(ref mut c)
            | usb::Descriptor::Endpoint(ref mut c) => match c {
                usb::ClassDescriptor::Printer(ref mut p) => {
                    for pd in p.descriptors.iter_mut() {
                        pd.uuid_string = device.get_descriptor_string(pd.uuid_string_index);
                    }
                }
                usb::ClassDescriptor::Communication(ref mut cdc) => match cdc.interface {
                    usb::descriptors::cdc::CdcInterfaceDescriptor::CountrySelection(ref mut d) => {
                        d.country_code_date =
                            device.get_descriptor_string(d.country_code_date_index);
                    }
                    usb::descriptors::cdc::CdcInterfaceDescriptor::NetworkChannel(ref mut d) => {
                        d.name = device.get_descriptor_string(d.name_string_index);
                    }
                    usb::descriptors::cdc::CdcInterfaceDescriptor::EthernetNetworking(
                        ref mut d,
                    ) => {
                        d.mac_address = device.get_descriptor_string(d.mac_address_index);
                    }
                    usb::descriptors::cdc::CdcInterfaceDescriptor::CommandSet(ref mut d) => {
                        d.command_set_string =
                            device.get_descriptor_string(d.command_set_string_index);
                    }
                    _ => (),
                },
                // grab report descriptor data using usb_control_msg
                usb::ClassDescriptor::Hid(ref mut hd) => {
                    for rd in hd.descriptors.iter_mut() {
                        if let Some(index) = interface_number {
                            rd.data =
                                Self::get_report_descriptor(device, index as u16, rd.length).ok();
                        }
                    }
                }
                usb::ClassDescriptor::Midi(ref mut md, _) => match md.interface {
                    usb::descriptors::audio::MidiInterfaceDescriptor::InputJack(ref mut mh) => {
                        mh.jack_string = device.get_descriptor_string(mh.jack_string_index);
                    }
                    usb::descriptors::audio::MidiInterfaceDescriptor::OutputJack(ref mut mh) => {
                        mh.jack_string = device.get_descriptor_string(mh.jack_string_index);
                    }
                    usb::descriptors::audio::MidiInterfaceDescriptor::Element(ref mut mh) => {
                        mh.element_string = device.get_descriptor_string(mh.element_string_index);
                    }
                    _ => (),
                },
                usb::ClassDescriptor::Audio(ref mut ad, _) => match ad.interface {
                    usb::descriptors::audio::UacInterfaceDescriptor::InputTerminal1(ref mut ah) => {
                        ah.channel_names = device.get_descriptor_string(ah.channel_names_index);
                        ah.terminal = device.get_descriptor_string(ah.terminal_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::InputTerminal2(ref mut ah) => {
                        ah.channel_names = device.get_descriptor_string(ah.channel_names_index);
                        ah.terminal = device.get_descriptor_string(ah.terminal_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::OutputTerminal1(
                        ref mut ah,
                    ) => {
                        ah.terminal = device.get_descriptor_string(ah.terminal_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::OutputTerminal2(
                        ref mut ah,
                    ) => {
                        ah.terminal = device.get_descriptor_string(ah.terminal_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::StreamingInterface2(
                        ref mut ah,
                    ) => {
                        ah.channel_names = device.get_descriptor_string(ah.channel_names_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::SelectorUnit1(ref mut ah) => {
                        ah.selector = device.get_descriptor_string(ah.selector_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::SelectorUnit2(ref mut ah) => {
                        ah.selector = device.get_descriptor_string(ah.selector_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::ProcessingUnit1(
                        ref mut ah,
                    ) => {
                        ah.channel_names = device.get_descriptor_string(ah.channel_names_index);
                        ah.processing = device.get_descriptor_string(ah.processing_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::ProcessingUnit2(
                        ref mut ah,
                    ) => {
                        ah.channel_names = device.get_descriptor_string(ah.channel_names_index);
                        ah.processing = device.get_descriptor_string(ah.processing_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::EffectUnit2(ref mut ah) => {
                        ah.effect = device.get_descriptor_string(ah.effect_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::FeatureUnit1(ref mut ah) => {
                        ah.feature = device.get_descriptor_string(ah.feature_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::FeatureUnit2(ref mut ah) => {
                        ah.feature = device.get_descriptor_string(ah.feature_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::ExtensionUnit1(ref mut ah) => {
                        ah.channel_names = device.get_descriptor_string(ah.channel_names_index);
                        ah.extension = device.get_descriptor_string(ah.extension_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::ExtensionUnit2(ref mut ah) => {
                        ah.channel_names = device.get_descriptor_string(ah.channel_names_index);
                        ah.extension = device.get_descriptor_string(ah.extension_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::ClockSource2(ref mut ah) => {
                        ah.clock_source = device.get_descriptor_string(ah.clock_source_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::ClockSelector2(ref mut ah) => {
                        ah.clock_selector = device.get_descriptor_string(ah.clock_selector_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::ClockMultiplier2(
                        ref mut ah,
                    ) => {
                        ah.clock_multiplier =
                            device.get_descriptor_string(ah.clock_multiplier_index);
                    }
                    usb::descriptors::audio::UacInterfaceDescriptor::SampleRateConverter2(
                        ref mut ah,
                    ) => {
                        ah.src = device.get_descriptor_string(ah.src_index);
                    }
                    _ => (),
                },
                usb::ClassDescriptor::Video(ref mut vd, _) => match vd.interface {
                    usb::descriptors::video::UvcInterfaceDescriptor::InputTerminal(ref mut vh) => {
                        vh.terminal = device.get_descriptor_string(vh.terminal_index);
                    }
                    usb::descriptors::video::UvcInterfaceDescriptor::OutputTerminal(ref mut vh) => {
                        vh.terminal = device.get_descriptor_string(vh.terminal_index);
                    }
                    usb::descriptors::video::UvcInterfaceDescriptor::SelectorUnit(ref mut vh) => {
                        vh.selector = device.get_descriptor_string(vh.selector_index);
                    }
                    usb::descriptors::video::UvcInterfaceDescriptor::ProcessingUnit(ref mut vh) => {
                        vh.processing = device.get_descriptor_string(vh.processing_index);
                    }
                    usb::descriptors::video::UvcInterfaceDescriptor::ExtensionUnit(ref mut vh) => {
                        vh.extension = device.get_descriptor_string(vh.extension_index);
                    }
                    usb::descriptors::video::UvcInterfaceDescriptor::EncodingUnit(ref mut vh) => {
                        vh.encoding = device.get_descriptor_string(vh.encoding_index);
                    }
                    _ => (),
                },
                _ => (),
            },
            _ => (),
        }

        Ok(dt)
    }

    fn build_config_descriptor_extra(
        &self,
        device: &T,
        mut raw: Vec<u8>,
    ) -> Result<Vec<usb::Descriptor>> {
        let extra_len = raw.len();
        let mut taken = 0;
        let mut ret = Vec::new();

        // Iterate on chunks of the header length
        while taken < extra_len && extra_len >= 2 {
            let dt_len = raw[0] as usize;
            let dt = self.build_descriptor_extra::<u8>(
                device,
                None,
                None,
                &raw.drain(..dt_len).collect::<Vec<u8>>(),
            )?;
            log::trace!("Config descriptor extra: {:?}", dt);
            ret.push(dt);
            taken += dt_len;
        }

        Ok(ret)
    }

    fn build_interface_descriptor_extra<C: Into<usb::ClassCode> + Copy>(
        &self,
        device: &T,
        class_code: usb::ClassCodeTriplet<C>,
        interface_number: u8,
        mut raw: Vec<u8>,
    ) -> Result<Vec<usb::Descriptor>> {
        let extra_len = raw.len();
        let mut taken = 0;
        let mut ret = Vec::new();

        // Iterate on chunks of the header length
        while taken < extra_len && extra_len >= 2 {
            let dt_len = raw[0] as usize;
            if let Some(b) = raw.get_mut(1) {
                // Mask request type LIBUSB_REQUEST_TYPE_CLASS
                *b &= !(0x01 << 5);
                // if not Device or Interface, force it to Interface
                if *b != 0x01 || *b != 0x04 {
                    *b = 0x04;
                }
            }

            let dt = self.build_descriptor_extra(
                device,
                Some(class_code),
                Some(interface_number),
                &raw.drain(..dt_len).collect::<Vec<u8>>(),
            )?;

            log::trace!("Interface descriptor extra: {:?}", dt);
            ret.push(dt);
            taken += dt_len;
        }

        Ok(ret)
    }

    fn build_endpoint_descriptor_extra<C: Into<usb::ClassCode> + Copy>(
        &self,
        device: &T,
        class_code: usb::ClassCodeTriplet<C>,
        interface_number: u8,
        mut raw: Vec<u8>,
    ) -> Result<Option<Vec<usb::Descriptor>>> {
        let extra_len = raw.len();
        let mut taken = 0;
        let mut ret = Vec::new();

        // Iterate on chunks of the header length
        while taken < extra_len && extra_len >= 2 {
            let dt_len = raw[0] as usize;
            if let Some(b) = raw.get_mut(1) {
                // Mask request type LIBUSB_REQUEST_TYPE_CLASS for Endpoint: 0x25
                if *b == 0x25 {
                    *b &= !(0x01 << 5);
                }
            };

            let dt = self.build_descriptor_extra(
                device,
                Some(class_code),
                Some(interface_number),
                &raw.drain(..dt_len).collect::<Vec<u8>>(),
            )?;

            log::trace!("Endpoint descriptor extra: {:?}", dt);
            ret.push(dt);
            taken += dt_len;
        }

        Ok(Some(ret))
    }

    fn profile_devices(
        &self,
        devices: &mut Vec<system_profiler::USBDevice>,
        root_hubs: &mut HashMap<u8, system_profiler::USBDevice>,
        with_extra: bool,
    ) -> Result<()>;

    fn get_spusb(&self, with_extra: bool) -> Result<system_profiler::SPUSBDataType> {
        let mut spusb = system_profiler::SPUSBDataType { buses: Vec::new() };
        // temporary store of devices created when iterating through DeviceList
        let mut cache: Vec<system_profiler::USBDevice> = Vec::new();
        // lookup for root hubs to assign info to bus on linux
        let mut root_hubs: HashMap<u8, system_profiler::USBDevice> = HashMap::new();

        log::info!("Building SPUSBDataType with {:?}", self);

        self.profile_devices(&mut cache, &mut root_hubs, with_extra)?;

        cache.sort_by_key(|d| d.location_id.bus);
        log::trace!("Sorted devices {:#?}", cache);

        // group by bus number and then stick them into a bus in the returned SPUSBDataType
        for (key, group) in &cache.into_iter().group_by(|d| d.location_id.bus) {
            let root = if !cfg!(target_os = "macos") {
                root_hubs.get(&key)
            } else {
                None
            };

            // create the bus, we'll add devices at next step
            let mut new_bus = system_profiler::USBBus {
                name: "Unknown".into(),
                host_controller: "Unknown".into(),
                usb_bus_number: Some(key),
                ..Default::default()
            };

            if let Some(root_hub) = root {
                root_hub.name.clone_into(&mut new_bus.name);
                root_hub
                    .manufacturer
                    .as_ref()
                    .unwrap_or(&String::new())
                    .clone_into(&mut new_bus.host_controller);
                new_bus.pci_vendor = root_hub.vendor_id;
                new_bus.pci_device = root_hub.product_id;
            }

            // group into parent groups with parent path as key or trunk devices so they end up in same place
            let parent_groups = group.group_by(|d| d.parent_path().unwrap_or(d.trunk_path()));

            // now go through parent paths inserting devices owned by that parent
            // this is not perfect...if the sort of devices does not result in order of depth, it will panic because the parent of a device will not exist. But that won't happen, right...
            // sort key - ends_with to ensure root_hubs, which will have same str length as trunk devices will still be ahead
            for (parent_path, children) in parent_groups
                .into_iter()
                .sorted_by_key(|x| x.0.len() - x.0.ends_with("-0") as usize)
            {
                log::debug!("Adding devices to parent {}", parent_path);
                // if root devices, add them to bus
                if parent_path.ends_with("-0") {
                    // if parent_path == "-" {
                    let devices = std::mem::take(&mut new_bus.devices);
                    if let Some(mut d) = devices {
                        for new_device in children {
                            d.push(new_device);
                        }
                        new_bus.devices = Some(d);
                    } else {
                        new_bus.devices = Some(children.collect());
                    }
                    log::trace!("Updated bus devices {:?}", new_bus.devices);
                    // else find and add parent - this should work because we are sorted to accend the tree so parents should be created before their children
                } else {
                    let parent_node = new_bus
                        .get_node_mut(&parent_path)
                        .expect("Parent node does not exist in new bus!");
                    let devices = std::mem::take(&mut parent_node.devices);
                    if let Some(mut d) = devices {
                        for new_device in children {
                            d.push(new_device);
                        }
                        parent_node.devices = Some(d);
                    } else {
                        parent_node.devices = Some(children.collect());
                    }
                    log::trace!("Updated parent devices {:?}", parent_node.devices);
                }
            }

            spusb.buses.push(new_bus);
        }

        Ok(spusb)
    }

    /// Fills a passed mutable `spusb` reference to fill using `get_spusb`. Will replace existing [`system_profiler::USBDevice`]s found in the Profiler tree but leave others and the buses.
    ///
    /// The main use case for this is to merge with macOS `system_profiler` data, so that [`usb::USBDeviceExtra`] can be obtained but internal buses kept. One could also use it to update a static .json dump.
    fn fill_spusb(&self, spusb: &mut system_profiler::SPUSBDataType) -> Result<()> {
        let libusb_spusb = self.get_spusb(true)?;

        // merge if passed has any buses
        if !spusb.buses.is_empty() {
            for mut bus in libusb_spusb.buses {
                if let Some(existing) = spusb
                    .buses
                    .iter_mut()
                    .find(|b| b.get_bus_number() == bus.get_bus_number())
                {
                    // just take the devices and put them in since nusb/libusb will be more verbose
                    existing.devices = std::mem::take(&mut bus.devices);
                }
            }
        }

        Ok(())
    }
}

/// Attempt to retrieve the current bConfigurationValue and iConfiguration for a device
/// This will only return the current configuration, not all possible configurations
/// If there are any failures in retrieving the data, None is returned
#[allow(unused_variables)]
fn get_sysfs_configuration_string(sysfs_name: &str) -> Option<(u8, String)> {
    #[cfg(target_os = "linux")]
    // Determine bConfigurationValue value on linux
    match get_sysfs_string(sysfs_name, "bConfigurationValue") {
        Some(s) => match s.parse::<u8>() {
            Ok(v) => {
                // Determine iConfiguration
                get_sysfs_string(sysfs_name, "configuration").map(|s| (v, s))
            }
            Err(_) => None,
        },
        None => None,
    }

    #[cfg(not(target_os = "linux"))]
    None
}

#[allow(unused_variables)]
fn get_sysfs_string(sysfs_name: &str, name: &str) -> Option<String> {
    #[cfg(target_os = "linux")]
    match std::fs::read_to_string(format!("/sys/bus/usb/devices/{}/{}", sysfs_name, name)) {
        Ok(s) => Some(s.trim().to_string()),
        Err(_) => None,
    }

    #[cfg(not(target_os = "linux"))]
    None
}

#[allow(unused_variables)]
fn get_udev_driver_name(port_path: &str) -> Result<Option<String>> {
    #[cfg(all(target_os = "linux", any(feature = "udev", feature = "udevlib")))]
    return udev::get_udev_driver_name(port_path);
    #[cfg(not(all(target_os = "linux", any(feature = "udev", feature = "udevlib"))))]
    return Ok(None);
}

#[allow(unused_variables)]
fn get_udev_syspath(port_path: &str) -> Result<Option<String>> {
    #[cfg(all(target_os = "linux", any(feature = "udev", feature = "udevlib")))]
    return udev::get_udev_syspath(port_path);
    #[cfg(not(all(target_os = "linux", any(feature = "udev", feature = "udevlib"))))]
    return Ok(None);
}

#[allow(unused_variables)]
fn get_syspath(port_path: &str) -> Option<String> {
    #[cfg(target_os = "linux")]
    return Some(format!("/sys/bus/usb/devices/{}", port_path));
    #[cfg(not(target_os = "linux"))]
    return None;
}

#[cfg(feature = "libusb")]
pub mod libusb {
    //! Uses rusb (upto date libusb fork) to get system USB information - same lib as lsusb. Requires 'libusb' feature. Uses [`crate::system_profiler`] types to hold data so that it is cross-compatible with macOS system_profiler command.
    use super::*;
    use crate::error::{Error, ErrorKind};
    use crate::lsusb::names;
    use crate::usb::{self, NumericalUnit};
    use rusb as libusb;
    use usb_ids::{self, FromId};

    #[derive(Debug)]
    pub(crate) struct LibUsbProfiler;

    pub(crate) struct UsbDevice<T: libusb::UsbContext> {
        handle: libusb::DeviceHandle<T>,
        language: libusb::Language,
        timeout: std::time::Duration,
    }

    impl ControlRequest {
        fn get_request_type_in(&self) -> u8 {
            libusb::request_type(
                libusb::Direction::In,
                self.control_type.into(),
                self.recipient.into(),
            )
        }
    }

    impl From<ControlType> for libusb::RequestType {
        fn from(ct: ControlType) -> Self {
            match ct {
                ControlType::Standard => libusb::RequestType::Standard,
                ControlType::Class => libusb::RequestType::Class,
                ControlType::Vendor => libusb::RequestType::Vendor,
            }
        }
    }

    impl From<Recipient> for libusb::Recipient {
        fn from(r: Recipient) -> Self {
            match r {
                Recipient::Device => libusb::Recipient::Device,
                Recipient::Interface => libusb::Recipient::Interface,
                Recipient::Endpoint => libusb::Recipient::Endpoint,
                Recipient::Other => libusb::Recipient::Other,
            }
        }
    }

    impl<T: libusb::UsbContext> UsbOperations for UsbDevice<T> {
        /// Get string descriptor from device
        ///
        /// Returns None if string_index is 0 - reserved for language codes
        fn get_descriptor_string(&self, string_index: u8) -> Option<String> {
            if string_index == 0 {
                return None;
            }
            self.handle
                .read_string_descriptor(self.language, string_index, self.timeout)
                .map(|s| s.trim().trim_end_matches('\0').to_string())
                .ok()
        }

        /// Get control message from device, ensuring message of [`ControlRequest`] length is read
        fn get_control_msg(&self, control_request: &ControlRequest) -> Result<Vec<u8>> {
            let mut buf = vec![0; control_request.length];
            let n = self
                .handle
                .read_control(
                    control_request.get_request_type_in(),
                    control_request.request,
                    control_request.value,
                    control_request.index,
                    &mut buf,
                    self.timeout,
                )
                .map_err(|e| Error {
                    kind: ErrorKind::LibUSB,
                    message: format!("Failed to get control message: {}", e),
                })?;
            if n < control_request.length {
                log::warn!(
                    "Failed to read full control message for {}: {} < {}",
                    control_request.request,
                    n,
                    control_request.length
                );
                Err(Error {
                    kind: ErrorKind::LibUSB,
                    message: "Control message too short".to_string(),
                })
            } else {
                Ok(buf)
            }
        }
    }

    impl LibUsbProfiler {
        fn build_endpoints<T: libusb::UsbContext>(
            &self,
            handle: &UsbDevice<T>,
            interface_desc: &libusb::InterfaceDescriptor,
        ) -> Vec<usb::USBEndpoint> {
            let mut ret: Vec<usb::USBEndpoint> = Vec::new();

            for endpoint_desc in interface_desc.endpoint_descriptors() {
                let extra_desc = if let Some(extra) = endpoint_desc.extra() {
                    self.build_endpoint_descriptor_extra(
                        handle,
                        (interface_desc.class_code(), interface_desc.sub_class_code(), interface_desc.protocol_code()),
                        interface_desc.interface_number(),
                        extra.to_vec(),
                    ).ok().flatten()
                } else {
                    None
                };

                ret.push(usb::USBEndpoint {
                    address: usb::EndpointAddress {
                        address: endpoint_desc.address(),
                        number: endpoint_desc.number(),
                        direction: usb::Direction::from(endpoint_desc.direction()),
                    },
                    transfer_type: usb::TransferType::from(endpoint_desc.transfer_type()),
                    sync_type: usb::SyncType::from(endpoint_desc.sync_type()),
                    usage_type: usb::UsageType::from(endpoint_desc.usage_type()),
                    max_packet_size: endpoint_desc.max_packet_size(),
                    interval: endpoint_desc.interval(),
                    length: endpoint_desc.length(),
                    extra: extra_desc,
                });
            }

            ret
        }

        fn build_interfaces<T: libusb::UsbContext>(
            &self,
            device: &libusb::Device<T>,
            handle: &UsbDevice<T>,
            config_desc: &libusb::ConfigDescriptor,
            with_udev: bool,
        ) -> Result<Vec<usb::USBInterface>> {
            let mut ret: Vec<usb::USBInterface> = Vec::new();

            for interface in config_desc.interfaces() {
                for interface_desc in interface.descriptors() {
                    let path = usb::get_interface_path(
                        device.bus_number(),
                        &device.port_numbers()?,
                        config_desc.number(),
                        interface_desc.interface_number(),
                    );

                    let mut interface = usb::USBInterface {
                        name: get_sysfs_string(&path, "interface")
                            .or(interface_desc
                                .description_string_index().and_then(|i| handle.get_descriptor_string(i)))
                            .unwrap_or_default(),
                        string_index: interface_desc.description_string_index().unwrap_or(0),
                        number: interface_desc.interface_number(),
                        path,
                        class: usb::ClassCode::from(interface_desc.class_code()),
                        sub_class: interface_desc.sub_class_code(),
                        protocol: interface_desc.protocol_code(),
                        alt_setting: interface_desc.setting_number(),
                        driver: None,
                        syspath: None,
                        length: interface_desc.length(),
                        endpoints: self.build_endpoints(handle, &interface_desc),
                        extra: self
                            .build_interface_descriptor_extra(handle, (interface_desc.class_code(), interface_desc.sub_class_code(), interface_desc.protocol_code()), interface_desc.interface_number(), interface_desc.extra().to_vec())
                            .ok(),
                    };

                    // flag allows us to try again without udev if it raises an error
                    // but record the error for printing
                    if with_udev {
                        interface.driver = get_udev_driver_name(&interface.path)?;
                        interface.syspath = get_udev_syspath(&interface.path)?;
                    };

                    ret.push(interface);
                }
            }

            Ok(ret)
        }

        fn build_configurations<T: libusb::UsbContext>(
            &self,
            device: &libusb::Device<T>,
            handle: &UsbDevice<T>,
            device_desc: &libusb::DeviceDescriptor,
            sp_device: &system_profiler::USBDevice,
            with_udev: bool,
        ) -> Result<Vec<usb::USBConfiguration>> {
            // Retrieve the current configuration (if available)
            let cur_config = get_sysfs_configuration_string(&sp_device.sysfs_name());
            let mut ret: Vec<usb::USBConfiguration> = Vec::new();

            for n in 0..device_desc.num_configurations() {
                let config_desc = match device.config_descriptor(n) {
                    Ok(c) => c,
                    Err(_) => continue,
                };

                let mut attributes = Vec::new();
                if config_desc.remote_wakeup() {
                    attributes.push(usb::ConfigAttributes::RemoteWakeup);
                }

                if config_desc.self_powered() {
                    attributes.push(usb::ConfigAttributes::SelfPowered);
                }

                // Check if we have a cached iConfiguration string
                let config_name = if let Some((config_num, ref config_name)) = cur_config {
                    // Configs start from 1, not 0
                    if config_num - 1 == n {
                        Some(config_name.clone())
                    } else {
                        None
                    }
                } else {
                    None
                };

                ret.push(usb::USBConfiguration {
                    name: config_desc
                        .description_string_index().and_then(|i| handle.get_descriptor_string(i))
                        .or(config_name)
                        .unwrap_or(String::new()),
                    string_index: config_desc.description_string_index().unwrap_or(0),
                    number: config_desc.number(),
                    attributes,
                    max_power: NumericalUnit {
                        value: config_desc.max_power() as u32,
                        unit: String::from("mA"),
                        description: None,
                    },
                    length: config_desc.length(),
                    total_length: config_desc.total_length(),
                    interfaces: self.build_interfaces(device, handle, &config_desc, with_udev)?,
                    extra: self
                        .build_config_descriptor_extra(handle, config_desc.extra().to_vec())
                        .ok(),
                });
            }

            Ok(ret)
        }

        #[allow(unused_variables)]
        fn build_spdevice_extra<T: libusb::UsbContext>(
            &self,
            device: &libusb::Device<T>,
            handle: &UsbDevice<T>,
            device_desc: &libusb::DeviceDescriptor,
            sp_device: &system_profiler::USBDevice,
            with_udev: bool,
        ) -> Result<usb::USBDeviceExtra> {
            let mut extra = usb::USBDeviceExtra {
                max_packet_size: device_desc.max_packet_size(),
                string_indexes: (
                    device_desc.product_string_index().unwrap_or(0),
                    device_desc.manufacturer_string_index().unwrap_or(0),
                    device_desc.serial_number_string_index().unwrap_or(0),
                ),
                driver: None,
                syspath: None,
                // These are idProduct, idVendor in lsusb - from udev_hwdb/usb-ids
                vendor: names::vendor(device_desc.vendor_id())
                    .or(usb_ids::Vendor::from_id(device_desc.vendor_id())
                        .map(|v| v.name().to_owned())),
                product_name: names::product(device_desc.vendor_id(), device_desc.product_id()).or(
                    usb_ids::Device::from_vid_pid(
                        device_desc.vendor_id(),
                        device_desc.product_id(),
                    )
                    .map(|v| v.name().to_owned()),
                ),
                configurations: self.build_configurations(
                    device,
                    handle,
                    device_desc,
                    sp_device,
                    with_udev,
                )?,
                status: Self::get_device_status(handle).ok(),
                debug: Self::get_debug_descriptor(handle).ok(),
                binary_object_store: None,
                qualifier: None,
                hub: None,
            };

            // flag allows us to try again without udev if it raises an nting
            // but record the error for printing
            if with_udev {
                let sysfs_name = sp_device.sysfs_name();
                extra.driver = get_udev_driver_name(&sysfs_name)?;
                extra.syspath = get_udev_syspath(&sysfs_name)?;
            }

            // Get device specific stuff: bos, hub, dualspeed, debug and status
            if device_desc.usb_version() >= rusb::Version::from_bcd(0x0201) {
                extra.binary_object_store = Self::get_bos_descriptor(handle).ok();
            }
            if device_desc.usb_version() >= rusb::Version::from_bcd(0x0200) {
                extra.qualifier = Self::get_device_qualifier(handle).ok();
            }
            if device_desc.class_code() == usb::ClassCode::Hub as u8 {
                let has_ssp = if let Some(bos) = &extra.binary_object_store {
                    bos.capabilities.iter().any(|c| {
                        matches!(c, usb::descriptors::bos::BosCapability::SuperSpeedPlus(_))
                    })
                } else {
                    false
                };
                let bcd = sp_device.bcd_usb.map_or(0x0100, |v| v.into());
                extra.hub =
                    Self::get_hub_descriptor(handle, device_desc.protocol_code(), bcd, has_ssp)
                        .ok();
            }

            Ok(extra)
        }

        /// Builds a [`system_profiler::USBDevice`] from a [`libusb::Device`] by using `device_descriptor()` and intrograting for configuration strings. Optionally with `with_extra` will gather full device information, including from udev if feature is present.
        ///
        /// [`system_profiler::USBDevice.profiler_error`] `Option<String>` will contain any non-critical error during gather of `with_extra` data - normally due to permissions preventing open of device descriptors.
        fn build_spdevice<T: libusb::UsbContext>(
            &self,
            device: &libusb::Device<T>,
            with_extra: bool,
        ) -> Result<system_profiler::USBDevice> {
            let timeout = std::time::Duration::from_secs(1);
            let speed = match usb::Speed::from(device.speed()) {
                usb::Speed::Unknown => None,
                v => Some(system_profiler::DeviceSpeed::SpeedValue(v)),
            };

            let mut error_str = None;
            let device_desc = device.device_descriptor()?;

            // try to get open device for strings but allowed to continue if this fails - get string functions will return empty
            let mut usb_device = {
                match device.open() {
                    Ok(h) => match h.read_languages(timeout) {
                        Ok(l) => {
                            if !l.is_empty() {
                                Some(UsbDevice {
                                    handle: h,
                                    language: l[0],
                                    timeout,
                                })
                            } else {
                                None
                            }
                        }
                        Err(e) => {
                            error_str = Some(format!(
                                "Could not read languages for {:?}, will be unable to obtain all data: {}",
                                device, e
                            ));
                            None
                        }
                    },
                    Err(e) => {
                        error_str = Some(format!(
                            "Failed to open {:?}, will be unable to obtain all data: {}",
                            device, e
                        ));
                        None
                    }
                }
            };

            let mut sp_device = system_profiler::USBDevice {
                vendor_id: Some(device_desc.vendor_id()),
                product_id: Some(device_desc.product_id()),
                device_speed: speed,
                location_id: system_profiler::DeviceLocation {
                    bus: device.bus_number(),
                    number: device.address(),
                    tree_positions: device.port_numbers()?,
                },
                bcd_device: Some(device_desc.device_version().into()),
                bcd_usb: Some(device_desc.usb_version().into()),
                class: Some(usb::ClassCode::from(device_desc.class_code())),
                sub_class: Some(device_desc.sub_class_code()),
                protocol: Some(device_desc.protocol_code()),
                ..Default::default()
            };

            if let Some(usb_device) = &mut usb_device {
                sp_device.manufacturer = device_desc
                    .manufacturer_string_index().and_then(|i| usb_device.get_descriptor_string(i));

                if let Some(name) = device_desc
                    .product_string_index().and_then(|i| usb_device.get_descriptor_string(i))
                {
                    sp_device.name = name;
                }

                sp_device.serial_num = device_desc
                    .serial_number_string_index().and_then(|i| usb_device.get_descriptor_string(i));

                let extra_error_str = if with_extra {
                    match self.build_spdevice_extra(
                        device,
                        usb_device,
                        &device_desc,
                        &sp_device,
                        true,
                    ) {
                        Ok(extra) => {
                            sp_device.extra = Some(extra);
                            None
                        }
                        Err(e) => {
                            // try again without udev if we have that feature but return message so device still added
                            if cfg!(feature = "udev") && e.kind() == ErrorKind::Udev {
                                sp_device.extra = Some(self.build_spdevice_extra(
                                    device,
                                    usb_device,
                                    &device_desc,
                                    &sp_device,
                                    false,
                                )?);
                                Some(format!(
                                    "Failed to get udev data for {}, probably requires elevated permissions",
                                    sp_device
                                ))
                            } else {
                                Some(format!(
                                    "Failed to get some extra data for {}, probably requires elevated permissions: {}",
                                    sp_device, e
                                ))
                            }
                        }
                    }
                } else {
                    None
                };

                if error_str.is_none() {
                    error_str = extra_error_str;
                }
            }

            if sp_device.manufacturer.is_none() {
                // sysfs cache
                sp_device.manufacturer = get_sysfs_string(&sp_device.sysfs_name(), "manufacturer")
                    // udev-hwdb
                    .or(names::vendor(device_desc.vendor_id())) // udev, usb-ids if error
                    // usb-ids
                    .or(usb_ids::Vendor::from_id(device_desc.vendor_id())
                        .map(|vendor| vendor.name().to_owned()));
            }

            // if could not get from descriptor
            if sp_device.name.is_empty() {
                // sysfs cache
                sp_device.name = get_sysfs_string(&sp_device.sysfs_name(), "product")
                    // udev-hwdb
                    .or(names::product(
                        device_desc.vendor_id(),
                        device_desc.product_id(),
                    ))
                    // usb-ids
                    .or(usb_ids::Device::from_vid_pid(
                        device_desc.vendor_id(),
                        device_desc.product_id(),
                    )
                    .map(|device| device.name().to_owned()))
                    // empty
                    .unwrap_or_default();
            }

            if sp_device.serial_num.is_none() {
                sp_device.serial_num = get_sysfs_string(&sp_device.sysfs_name(), "serial");
            }

            sp_device.profiler_error = error_str;
            Ok(sp_device)
        }
    }

    impl<C: libusb::UsbContext> Profiler<UsbDevice<C>> for LibUsbProfiler {
        fn profile_devices(
            &self,
            devices: &mut Vec<system_profiler::USBDevice>,
            root_hubs: &mut HashMap<u8, system_profiler::USBDevice>,
            with_extra: bool,
        ) -> Result<()> {
            // run through devices building USBDevice types
            for device in libusb::DeviceList::new()?.iter() {
                match self.build_spdevice(&device, with_extra) {
                    Ok(sp_device) => {
                        devices.push(sp_device.to_owned());
                        let print_stderr =
                            std::env::var_os("CYME_PRINT_NON_CRITICAL_PROFILER_STDERR").is_some();

                        // print any non-critical error during extra capture
                        sp_device.profiler_error.iter().for_each(|e| {
                            if print_stderr {
                                eprintln!("{}", e);
                            } else {
                                log::warn!("Non-critical error during profile: {}", e);
                            }
                        });

                        // save it if it's a root_hub for assigning to bus data
                        if !cfg!(target_os = "macos") && sp_device.is_root_hub() {
                            root_hubs.insert(sp_device.location_id.bus, sp_device);
                        }
                    }
                    Err(e) => eprintln!("Failed to get data for {:?}: {}", device, e),
                }
            }

            Ok(())
        }
    }

    impl From<libusb::Error> for Error {
        fn from(error: libusb::Error) -> Self {
            Error {
                kind: ErrorKind::LibUSB,
                message: format!(
                    "Failed to gather system USB data from libusb: Error({})",
                    &error.to_string()
                ),
            }
        }
    }

    /// Set log level for rusb
    pub fn set_log_level(debug: u8) {
        let log_level = match debug {
            0 => rusb::LogLevel::None,
            1 => rusb::LogLevel::Warning,
            2 => rusb::LogLevel::Info,
            _ => rusb::LogLevel::Debug,
        };

        rusb::set_log_level(log_level);
    }

    /// Covert to our crate speed
    impl From<libusb::Speed> for usb::Speed {
        fn from(libusb: libusb::Speed) -> Self {
            match libusb {
                libusb::Speed::SuperPlus => usb::Speed::SuperSpeedPlus,
                libusb::Speed::Super => usb::Speed::SuperSpeed,
                libusb::Speed::High => usb::Speed::HighSpeed,
                libusb::Speed::Full => usb::Speed::FullSpeed,
                libusb::Speed::Low => usb::Speed::LowSpeed,
                _ => usb::Speed::Unknown,
            }
        }
    }

    impl From<libusb::Direction> for usb::Direction {
        fn from(libusb: libusb::Direction) -> Self {
            match libusb {
                libusb::Direction::Out => usb::Direction::Out,
                libusb::Direction::In => usb::Direction::In,
            }
        }
    }

    impl From<libusb::TransferType> for usb::TransferType {
        fn from(libusb: libusb::TransferType) -> Self {
            match libusb {
                libusb::TransferType::Control => usb::TransferType::Control,
                libusb::TransferType::Isochronous => usb::TransferType::Isochronous,
                libusb::TransferType::Bulk => usb::TransferType::Bulk,
                libusb::TransferType::Interrupt => usb::TransferType::Interrupt,
            }
        }
    }

    impl From<libusb::UsageType> for usb::UsageType {
        fn from(libusb: libusb::UsageType) -> Self {
            match libusb {
                libusb::UsageType::Data => usb::UsageType::Data,
                libusb::UsageType::Feedback => usb::UsageType::Feedback,
                libusb::UsageType::FeedbackData => usb::UsageType::FeedbackData,
                libusb::UsageType::Reserved => usb::UsageType::Reserved,
            }
        }
    }

    impl From<libusb::SyncType> for usb::SyncType {
        fn from(libusb: libusb::SyncType) -> Self {
            match libusb {
                libusb::SyncType::NoSync => usb::SyncType::None,
                libusb::SyncType::Asynchronous => usb::SyncType::Asynchronous,
                libusb::SyncType::Adaptive => usb::SyncType::Adaptive,
                libusb::SyncType::Synchronous => usb::SyncType::Synchronous,
            }
        }
    }

    impl From<libusb::Version> for usb::Version {
        fn from(libusb: libusb::Version) -> Self {
            usb::Version(libusb.major(), libusb.minor(), libusb.sub_minor())
        }
    }
}

#[cfg(feature = "nusb")]
pub mod nusb {
    //! Uses nusb (pure Rust) to get system USB information. Requires 'nusb' feature. Uses [`crate::system_profiler`] types to hold data so that it is cross-compatible with macOS system_profiler command.
    use super::*;
    use crate::error::{Error, ErrorKind};
    use crate::lsusb::names;
    use crate::usb::{self, NumericalUnit};
    use ::nusb;
    use usb_ids::{self, FromId};

    #[derive(Debug)]
    pub(crate) struct NusbProfiler;

    pub(crate) struct UsbDevice {
        handle: nusb::Device,
        language: u16,
        vidpid: (u16, u16),
        location: system_profiler::DeviceLocation,
        timeout: std::time::Duration,
    }

    impl From<ControlRequest> for nusb::transfer::Control {
        fn from(request: ControlRequest) -> Self {
            nusb::transfer::Control {
                control_type: request.control_type.into(),
                request: request.request,
                value: request.value,
                index: request.index,
                recipient: request.recipient.into(),
            }
        }
    }

    impl From<ControlType> for nusb::transfer::ControlType {
        fn from(control: ControlType) -> Self {
            match control {
                ControlType::Standard => nusb::transfer::ControlType::Standard,
                ControlType::Class => nusb::transfer::ControlType::Class,
                ControlType::Vendor => nusb::transfer::ControlType::Vendor,
            }
        }
    }

    impl From<Recipient> for nusb::transfer::Recipient {
        fn from(recipient: Recipient) -> Self {
            match recipient {
                Recipient::Device => nusb::transfer::Recipient::Device,
                Recipient::Interface => nusb::transfer::Recipient::Interface,
                Recipient::Endpoint => nusb::transfer::Recipient::Endpoint,
                Recipient::Other => nusb::transfer::Recipient::Other,
            }
        }
    }

    /// Covert to our crate speed
    impl From<nusb::Speed> for usb::Speed {
        fn from(nusb: nusb::Speed) -> Self {
            match nusb {
                nusb::Speed::SuperPlus => usb::Speed::SuperSpeedPlus,
                nusb::Speed::Super => usb::Speed::SuperSpeed,
                nusb::Speed::High => usb::Speed::HighSpeed,
                nusb::Speed::Full => usb::Speed::FullSpeed,
                nusb::Speed::Low => usb::Speed::LowSpeed,
                _ => usb::Speed::Unknown,
            }
        }
    }

    impl UsbOperations for UsbDevice {
        fn get_descriptor_string(&self, string_index: u8) -> Option<String> {
            if string_index == 0 {
                return None;
            }
            self.handle
                .get_string_descriptor(string_index, self.language, self.timeout)
                .map(|s| s.to_string())
                .ok()
        }

        #[cfg(any(target_os = "linux", target_os = "macos"))]
        fn get_control_msg(&self, control_request: &ControlRequest) -> Result<Vec<u8>> {
            let mut data = vec![0; control_request.length];
            let nusb_control: nusb::transfer::Control = (*control_request).into();
            let n = self
                .handle
                .control_in_blocking(nusb_control, data.as_mut_slice(), self.timeout)
                .map_err(|e| Error {
                    kind: ErrorKind::Nusb,
                    message: format!("Failed to get control message: {}", e),
                })?;

            if n < control_request.length {
                log::debug!(
                    "Failed to get full control message, only read {} of {}",
                    n,
                    control_request.length
                );
                return Err(Error {
                    kind: ErrorKind::Nusb,
                    message: format!(
                        "Failed to get full control message, only read {} of {}",
                        n, control_request.length
                    ),
                });
            }

            Ok(data)
        }

        #[cfg(target_os = "windows")]
        fn get_control_msg(&self, control_request: &ControlRequest) -> Result<Vec<u8>> {
            let mut data = vec![0; control_request.length];
            let nusb_control: nusb::transfer::Control = control_request.clone().into();
            // TODO this should probably be dependant on the interface being called?
            let interface = self.handle.claim_interface(0)?;
            let n = interface
                .control_in_blocking(nusb_control, data.as_mut_slice(), self.timeout)
                .map_err(|e| Error {
                    kind: ErrorKind::Nusb,
                    message: format!("Failed to get control message: {}", e),
                })?;

            if n < control_request.length {
                log::debug!(
                    "Failed to get full control message, only read {} of {}",
                    n,
                    control_request.length
                );
                return Err(Error {
                    kind: ErrorKind::Nusb,
                    message: format!(
                        "Failed to get full control message, only read {} of {}",
                        n, control_request.length
                    ),
                });
            }

            Ok(data)
        }
    }

    impl NusbProfiler {
        fn build_endpoints(
            &self,
            device: &UsbDevice,
            interface_desc: &nusb::descriptors::InterfaceAltSetting,
        ) -> Vec<usb::USBEndpoint> {
            let mut ret: Vec<usb::USBEndpoint> = Vec::new();

            for endpoint in interface_desc.endpoints() {
                let endpoint_desc = endpoint.descriptors().next().unwrap();
                let endpoint_extra = endpoint
                    .descriptors()
                    .skip(1)
                    .filter(|d| d.descriptor_type() == 0x05 || d.descriptor_type() == 0x25)
                    .flat_map(|d| d.to_vec())
                    .collect::<Vec<u8>>();

                ret.push(usb::USBEndpoint {
                    address: usb::EndpointAddress::from(endpoint.address()),
                    transfer_type: usb::TransferType::from(endpoint.transfer_type() as u8),
                    sync_type: usb::SyncType::from(endpoint.transfer_type() as u8),
                    usage_type: usb::UsageType::from(endpoint.transfer_type() as u8),
                    max_packet_size: endpoint.max_packet_size() as u16,
                    interval: endpoint.interval(),
                    length: endpoint_desc[0],
                    extra: self
                        .build_endpoint_descriptor_extra(device, (interface_desc.class(), interface_desc.subclass(), interface_desc.protocol()), interface_desc.interface_number(), endpoint_extra)
                        .ok()
                        .flatten(),
                });
            }

            ret
        }

        fn build_interfaces(
            &self,
            device: &UsbDevice,
            config: &nusb::descriptors::Configuration,
            with_udev: bool,
        ) -> Result<Vec<usb::USBInterface>> {
            let mut ret: Vec<usb::USBInterface> = Vec::new();

            for interface in config.interfaces() {
                for interface_alt in interface.alt_settings() {
                    let path = usb::get_interface_path(
                        device.location.bus,
                        &device.location.tree_positions,
                        config.configuration_value(),
                        interface_alt.interface_number(),
                    );

                    let interface_desc = interface_alt.descriptors().next().unwrap();
                    let interface_extra = interface_alt
                        .descriptors()
                        .skip(1)
                        // only want device and interface descriptors - nusb everything trailing
                        .filter(|d| {
                            (d.descriptor_type() & 0x0F) == 0x04
                                || (d.descriptor_type() & 0x0F) == 0x01
                        })
                        .flat_map(|d| d.to_vec())
                        .collect::<Vec<u8>>();

                    let mut interface = usb::USBInterface {
                        name: get_sysfs_string(&path, "interface")
                            .or(interface_alt
                                .string_index().and_then(|i| device.get_descriptor_string(i)))
                            .unwrap_or_default(),
                        string_index: interface_alt.string_index().unwrap_or(0),
                        number: interface_alt.interface_number(),
                        path,
                        class: usb::ClassCode::from(interface_alt.class()),
                        sub_class: interface_alt.subclass(),
                        protocol: interface_alt.subclass(),
                        alt_setting: interface_alt.alternate_setting(),
                        driver: None,
                        syspath: None,
                        length: interface_desc[0],
                        endpoints: self.build_endpoints(device, &interface_alt),
                        extra: self
                            .build_interface_descriptor_extra(
                                device,
                                (interface_alt.class(), interface_alt.subclass(), interface_alt.protocol()),
                                interface_alt.interface_number(),
                                interface_extra,
                            )
                            .ok(),
                    };

                    // flag allows us to try again without udev if it raises an error
                    // but record the error for printing
                    if with_udev {
                        interface.driver = get_udev_driver_name(&interface.path)?;
                        interface.syspath = get_udev_syspath(&interface.path)?;
                    };

                    ret.push(interface);
                }
            }

            Ok(ret)
        }

        fn build_configurations(
            &self,
            device: &UsbDevice,
            with_udev: bool,
        ) -> Result<Vec<usb::USBConfiguration>> {
            let mut ret: Vec<usb::USBConfiguration> = Vec::new();

            for c in device.handle.configurations() {
                let mut attributes = Vec::new();
                if c.attributes() & 0x10 != 0 {
                    attributes.push(usb::ConfigAttributes::BatteryPowered);
                }
                if c.attributes() & 0x20 != 0 {
                    attributes.push(usb::ConfigAttributes::RemoteWakeup);
                }
                if c.attributes() & 0x40 != 0 {
                    attributes.push(usb::ConfigAttributes::SelfPowered);
                }

                let config_desc = c.descriptors().next().unwrap();
                let config_extra = c
                    .descriptors()
                    .skip(1)
                    // only config descriptors - nusb everything trailing
                    .filter(|d| d.descriptor_type() == 0x02)
                    .flat_map(|d| d.to_vec())
                    .collect::<Vec<u8>>();
                let total_length = u16::from_le_bytes(config_desc[2..4].try_into().unwrap());

                ret.push(usb::USBConfiguration {
                    name: c
                        .string_index().and_then(|i| device.get_descriptor_string(i))
                        .unwrap_or_default(),
                    string_index: c.string_index().unwrap_or(0),
                    number: c.configuration_value(),
                    attributes,
                    max_power: NumericalUnit {
                        value: c.max_power() as u32,
                        unit: String::from("mA"),
                        description: None,
                    },
                    length: config_desc.len() as u8,
                    total_length,
                    interfaces: self.build_interfaces(device, &c, with_udev)?,
                    extra: self
                        .build_config_descriptor_extra(device, config_extra)
                        .ok(),
                });
            }

            Ok(ret)
        }

        fn build_spdevice_extra(
            &self,
            device: &UsbDevice,
            sp_device: &mut system_profiler::USBDevice,
            with_udev: bool,
        ) -> Result<usb::USBDeviceExtra> {
            // get the Device Descriptor since not all data is cached
            let device_desc_raw = device.handle.get_descriptor(
                0x01,
                0x00,
                0x00,
                device.timeout,
            )?;
            let device_desc: usb::DeviceDescriptor =
                usb::DeviceDescriptor::try_from(device_desc_raw.as_slice())?;
            sp_device.bcd_usb = Some(device_desc.usb_version);

            // try to get strings from device descriptors
            if let Ok(name) = device
                .handle
                .get_string_descriptor(device_desc.product_string_index, 0, device.timeout)
                .map(|s| s.to_string())
            {
                sp_device.name = name;
            }

            if let Ok(manufacturer) = device
                .handle
                .get_string_descriptor(device_desc.manufacturer_string_index, 0, device.timeout)
                .map(|s| s.to_string())
            {
                sp_device.manufacturer = Some(manufacturer);
            }

            if let Ok(serial) = device
                .handle
                .get_string_descriptor(device_desc.serial_number_string_index, 0, device.timeout)
                .map(|s| s.to_string())
            {
                sp_device.serial_num = Some(serial);
            }

            let mut extra = usb::USBDeviceExtra {
                max_packet_size: device_desc.max_packet_size,
                string_indexes: (
                    device_desc.product_string_index,
                    device_desc.manufacturer_string_index,
                    device_desc.serial_number_string_index,
                ),
                driver: None,
                syspath: get_syspath(&sp_device.sysfs_name()),
                // These are idProduct, idVendor in lsusb - from udev_hwdb/usb-ids - not device descriptor
                vendor: names::vendor(device_desc.vendor_id)
                    .or(usb_ids::Vendor::from_id(device_desc.vendor_id)
                        .map(|v| v.name().to_owned())),
                product_name: names::product(device_desc.vendor_id, device_desc.product_id).or(
                    usb_ids::Device::from_vid_pid(device_desc.vendor_id, device_desc.product_id)
                        .map(|v| v.name().to_owned()),
                ),
                configurations: self.build_configurations(device, with_udev)?,
                status: Self::get_device_status(device).ok(),
                debug: Self::get_debug_descriptor(device).ok(),
                binary_object_store: None,
                qualifier: None,
                hub: None,
            };

            // flag allows us to try again without udev if it raises an nting
            // but record the error for printing
            if with_udev {
                let sysfs_name = sp_device.sysfs_name();
                extra.driver = get_udev_driver_name(&sysfs_name)?;
                extra.syspath = get_udev_syspath(&sysfs_name)?;
            }

            // Get device specific stuff: bos, hub, dualspeed, debug and status
            if device_desc.usb_version >= usb::Version::from_bcd(0x0201) {
                extra.binary_object_store = Self::get_bos_descriptor(device).ok();
            }
            if device_desc.usb_version >= usb::Version::from_bcd(0x0200) {
                extra.qualifier = Self::get_device_qualifier(device).ok();
            }

            if device_desc.device_class == usb::ClassCode::Hub as u8 {
                let has_ssp = if let Some(bos) = &extra.binary_object_store {
                    bos.capabilities.iter().any(|c| {
                        matches!(c, usb::descriptors::bos::BosCapability::SuperSpeedPlus(_))
                    })
                } else {
                    false
                };
                let bcd = sp_device.bcd_usb.map_or(0x0100, |v| v.into());
                extra.hub =
                    Self::get_hub_descriptor(device, device_desc.device_protocol, bcd, has_ssp)
                        .ok();
            }

            Ok(extra)
        }

        fn build_spdevice(
            &self,
            device_info: &nusb::DeviceInfo,
            with_extra: bool,
        ) -> Result<system_profiler::USBDevice> {
            let speed = device_info.speed().map(|s| {
                let s = usb::Speed::from(s);
                system_profiler::DeviceSpeed::SpeedValue(s)
            });

            let mut sp_device = system_profiler::USBDevice {
                vendor_id: Some(device_info.vendor_id()),
                product_id: Some(device_info.product_id()),
                device_speed: speed,
                location_id: system_profiler::DeviceLocation {
                    // nusb bus_id is a string; busnum on Linux (number)
                    bus: device_info.bus_id().parse::<u8>().unwrap_or(0),
                    number: device_info.device_address(),
                    tree_positions: device_info.port_chain().to_vec(),
                },
                bcd_device: Some(usb::Version::from_bcd(device_info.device_version())),
                // gets added on the extra read
                bcd_usb: None,
                class: Some(usb::ClassCode::from(device_info.class())),
                sub_class: Some(device_info.subclass()),
                protocol: Some(device_info.protocol()),
                ..Default::default()
            };

            // tree positions in relative to bus so remove bus number and if it's a bus (port 0), clear the vec
            // (legacy to libusb code)
            if sp_device.location_id.tree_positions.get(1) == Some(&0) {
                sp_device.location_id.tree_positions = vec![];
            } else {
                sp_device.location_id.tree_positions = sp_device
                    .location_id
                    .tree_positions
                    .into_iter()
                    .skip(1)
                    .collect();
            }

            sp_device.manufacturer =
                device_info
                    .manufacturer_string()
                    .map(|s| s.to_string())
                    .or(get_sysfs_string(&sp_device.sysfs_name(), "manufacturer"))
                    .or(names::vendor(device_info.vendor_id()))
                    .or(usb_ids::Vendor::from_id(device_info.vendor_id())
                        .map(|v| v.name().to_string()));
            sp_device.name = device_info
                .product_string()
                .map(|s| s.to_string())
                .or(get_sysfs_string(&sp_device.sysfs_name(), "product"))
                .or(names::product(
                    device_info.vendor_id(),
                    device_info.product_id(),
                ))
                .or(usb_ids::Device::from_vid_pid(
                    device_info.vendor_id(),
                    device_info.product_id(),
                )
                .map(|d| d.name().to_string()))
                .unwrap_or_default();
            sp_device.serial_num = device_info
                .serial_number()
                .map(|s| s.to_string())
                .or(get_sysfs_string(&sp_device.sysfs_name(), "serial"));

            if let Ok(device) = device_info.open() {
                let mut error_str = None;

                // get the first language - proably US English
                let languages: Vec<u16> = device
                    .get_string_descriptor_supported_languages(std::time::Duration::from_secs(1))
                    .map(|i| i.collect())
                    .unwrap_or_default();
                let language = languages
                    .first()
                    .copied()
                    .unwrap_or(nusb::descriptors::language_id::US_ENGLISH);

                let extra_error_str = if with_extra {
                    let usb_device = UsbDevice {
                        handle: device,
                        language,
                        vidpid: (device_info.vendor_id(), device_info.product_id()),
                        location: sp_device.location_id.clone(),
                        timeout: std::time::Duration::from_secs(1),
                    };

                    match self.build_spdevice_extra(&usb_device, &mut sp_device, true) {
                        Ok(extra) => {
                            sp_device.extra = Some(extra);
                            None
                        }
                        Err(e) => {
                            // try again without udev if we have that feature but return message so device still added
                            if cfg!(feature = "udev") && e.kind() == ErrorKind::Udev {
                                sp_device.extra = Some(self.build_spdevice_extra(
                                    &usb_device,
                                    &mut sp_device,
                                    false,
                                )?);
                                Some(format!(
                                        "Failed to get udev data for {}, probably requires elevated permissions",
                                        sp_device
                                ))
                            } else {
                                Some(format!("Failed to get some extra data for {}, probably requires elevated permissions: {}", sp_device, e))
                            }
                        }
                    }
                } else {
                    None
                };

                if error_str.is_none() {
                    error_str = extra_error_str;
                }

                sp_device.profiler_error = error_str;
            } else {
                log::warn!("Failed to open device for extra data: {:04x}:{:04x}. Ensure user has USB access permissions: https://docs.rs/nusb/latest/nusb/#linux", device_info.vendor_id(), device_info.product_id());
                sp_device.profiler_error =
                    Some("Failed to open device, extra data incomplete and possibly inaccurate".to_string());
                sp_device.extra = Some(usb::USBDeviceExtra {
                    max_packet_size: device_info.max_packet_size_0(),
                    // nusb doesn't have these cached
                    string_indexes: (0, 0, 0),
                    driver: None,
                    syspath: get_syspath(&sp_device.sysfs_name()),
                    vendor: names::vendor(device_info.vendor_id())
                        .or(usb_ids::Vendor::from_id(device_info.vendor_id())
                            .map(|v| v.name().to_owned())),
                    product_name: names::product(device_info.vendor_id(), device_info.product_id()).or(
                        usb_ids::Device::from_vid_pid(device_info.vendor_id(), device_info.product_id())
                            .map(|v| v.name().to_owned()),
                    ),
                    configurations: vec![],
                    status: None,
                    debug: None,
                    binary_object_store: None,
                    qualifier: None,
                    hub: None,
                });
            }

            Ok(sp_device)
        }
    }

    impl Profiler<UsbDevice> for NusbProfiler {
        fn profile_devices(
            &self,
            devices: &mut Vec<system_profiler::USBDevice>,
            root_hubs: &mut HashMap<u8, system_profiler::USBDevice>,
            with_extra: bool,
        ) -> Result<()> {
            for device in nusb::list_devices()? {
                match self.build_spdevice(&device, with_extra) {
                    Ok(sp_device) => {
                        devices.push(sp_device.to_owned());
                        let print_stderr =
                            std::env::var_os("CYME_PRINT_NON_CRITICAL_PROFILER_STDERR").is_some();

                        // print any non-critical error during extra capture
                        sp_device.profiler_error.iter().for_each(|e| {
                            if print_stderr {
                                eprintln!("{}", e);
                            } else {
                                log::warn!("Non-critical error during profile: {}", e);
                            }
                        });

                        // save it if it's a root_hub for assigning to bus data
                        if !cfg!(target_os = "macos") && sp_device.is_root_hub() {
                            root_hubs.insert(sp_device.location_id.bus, sp_device);
                        }
                    }
                    Err(e) => eprintln!("Failed to get data for {:?}: {}", device, e),
                }
            }

            Ok(())
        }
    }
}

/// Get [`system_profiler::SPUSBDataType`] using `libusb`. Does not source [`usb::USBDeviceExtra`] - use [`get_spusb_with_extra`] for that; the extra operation is mostly moving data around so the only hit is to stack.
///
/// Runs through `libusb::DeviceList` creating a cache of [`system_profiler::USBDevice`]. Then sorts into parent groups, accending in depth to build the [`system_profiler::USBBus`] tree.
///
/// Building the [`system_profiler::SPUSBDataType`] depends on system; on Linux, the root devices are at buses where as macOS the buses are not listed
#[cfg(all(feature = "libusb", not(feature = "nusb")))]
pub fn get_spusb() -> Result<system_profiler::SPUSBDataType> {
    let profiler = libusb::LibUsbProfiler;
    profiler.get_spusb(true)
}

/// Get [`system_profiler::SPUSBDataType`] using `nusb`. Does not source [`usb::USBDeviceExtra`] - use [`get_spusb_with_extra`] for that; the extra operation is mostly moving data around so the only hit is to stack.
///
/// Runs through `libusb::DeviceList` creating a cache of [`system_profiler::USBDevice`]. Then sorts into parent groups, accending in depth to build the [`system_profiler::USBBus`] tree.
///
/// Building the [`system_profiler::SPUSBDataType`] depends on system; on Linux, the root devices are at buses where as macOS the buses are not listed
#[cfg(feature = "nusb")]
pub fn get_spusb() -> Result<system_profiler::SPUSBDataType> {
    let profiler = nusb::NusbProfiler;
    profiler.get_spusb(true)
}

/// Abort with exit code before trying to call libusb feature if not present
#[cfg(all(not(feature = "libusb"), not(feature = "nusb")))]
pub fn get_spusb() -> Result<system_profiler::SPUSBDataType> {
    Err(crate::error::Error::new(
        crate::error::ErrorKind::Unsupported,
        "nusb or libusb feature is required to do this, install with `cargo install --features nusb/libusb`",
    ))
}

/// Get [`system_profiler::SPUSBDataType`] using `libusb` including [`usb::USBDeviceExtra`] - the main function to use for most use cases unless one does not want verbose data.
///
/// Like `get_spusb`, runs through `libusb::DeviceList` creating a cache of [`system_profiler::USBDevice`]. On Linux and with the 'udev' feature enabled, the syspath and driver will attempt to be obtained.
#[cfg(all(feature = "libusb", not(feature = "nusb")))]
pub fn get_spusb_with_extra() -> Result<system_profiler::SPUSBDataType> {
    let profiler = libusb::LibUsbProfiler;
    profiler.get_spusb(true)
}

/// Get [`system_profiler::SPUSBDataType`] using `nusb` including [`usb::USBDeviceExtra`] - the main function to use for most use cases unless one does not want verbose data.
///
/// Like `get_spusb`, runs through `libusb::DeviceList` creating a cache of [`system_profiler::USBDevice`]. On Linux and with the 'udev' feature enabled, the syspath and driver will attempt to be obtained.
#[cfg(feature = "nusb")]
pub fn get_spusb_with_extra() -> Result<system_profiler::SPUSBDataType> {
    let profiler = nusb::NusbProfiler;
    profiler.get_spusb(true)
}

/// Abort with exit code before trying to call libusb feature if not present
#[cfg(all(not(feature = "libusb"), not(feature = "nusb")))]
pub fn get_spusb_with_extra() -> Result<system_profiler::SPUSBDataType> {
    Err(crate::error::Error::new(
        crate::error::ErrorKind::Unsupported,
        "nusb or libusb feature is required to do this, install with `cargo install --features nusb/libusb`",
    ))
}
