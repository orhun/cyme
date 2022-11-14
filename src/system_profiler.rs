/// Parser for macOS `system_profiler` command -json output with SPUSBDataType.
///
/// USBBus and USBDevice structs are used as deserializers for serde. The JSON output with the -json flag is not really JSON; all values are String regardless of contained data so it requires some extra work. Additionally, some values differ slightly from the non json output such as the speed - it is a description rather than numerical.
///
/// J.Whittington - 2022
use std::fmt;
use std::io;
use std::str::FromStr;

use colored::*;
use serde::{Deserialize, Deserializer, Serialize};
use serde::de::{self, Visitor};
use std::process::Command;

/// borrowed from https://github.com/vityafx/serde-aux/blob/master/src/field_attributes.rs with addition of base16 encoding
/// Deserializes an option number from string or a number.
/// Only really used for vendor id and product id so TODO make struct for these
fn deserialize_option_number_from_string<'de, T, D>(deserializer: D) -> Result<Option<T>, D::Error>
where
    D: Deserializer<'de>,
    T: FromStr + serde::Deserialize<'de>,
    <T as FromStr>::Err: fmt::Display,
{
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum NumericOrNull<'a, T> {
        Str(&'a str),
        FromStr(T),
        Null,
    }

    match NumericOrNull::<T>::deserialize(deserializer)? {
        NumericOrNull::Str(mut s) => match s {
            "" => Ok(None),
            _ => {
                // -json returns apple_vendor_id in vendor_id for some reason not base16 like normal
                if s.contains("apple_vendor_id") {
                    s = "0x05ac";
                }
                // the vendor_id can be appended with manufacturer name for some reason...split with space to get just base16 encoding
                let vendor_vec: Vec<&str> = s.split(" ").collect();
                let removed_0x = vendor_vec[0].trim_start_matches("0x");

                if removed_0x != vendor_vec[0] {
                    let base16_num = u64::from_str_radix(removed_0x.trim(), 16);
                    let result = match base16_num {
                        Ok(num) => T::from_str(num.to_string().as_str()),
                        Err(e) => return Err(serde::de::Error::custom(e)),
                    };
                    result.map(Some).map_err(serde::de::Error::custom)
                } else {
                    T::from_str(s.trim())
                        .map(Some)
                        .map_err(serde::de::Error::custom)
                }
            }
        },
        NumericOrNull::FromStr(i) => Ok(Some(i)),
        NumericOrNull::Null => Ok(None),
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct SPUSBDataType {
    #[serde(rename(deserialize = "SPUSBDataType"))]
    pub buses: Vec<USBBus>,
}

impl SPUSBDataType {
    /// Returns a flattened Vec of all the USBDevices returned from system_profiler as a reference
    pub fn flatten_devices<'a>(&'a self) -> Vec<&'a USBDevice> {
        let mut ret = Vec::new();
        for bus in &self.buses {
            ret.append(&mut bus.flatten_devices());
        }

        ret
    }
}

impl fmt::Display for SPUSBDataType {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        for v in &self.buses {
            if f.alternate() {
                if f.sign_plus() {
                    writeln!(f, "{:+#}", v)?;
                } else {
                    writeln!(f, "{:#}", v)?;
                }
            } else if f.sign_plus() {
                write!(f, "{:+}", v)?;
            } else {
                write!(f, "{:}", v)?;
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct USBBus {
    #[serde(rename(deserialize = "_name"))]
    name: String,
    host_controller: String,
    #[serde(default, deserialize_with = "deserialize_option_number_from_string")]
    pci_device: Option<u16>,
    #[serde(default, deserialize_with = "deserialize_option_number_from_string")]
    pci_revision: Option<u16>,
    #[serde(default, deserialize_with = "deserialize_option_number_from_string")]
    pci_vendor: Option<u16>,
    #[serde(default, deserialize_with = "deserialize_option_number_from_string")]
    usb_bus_number: Option<u8>,
    // devices are normally hubs
    #[serde(rename(deserialize = "_items"))]
    devices: Option<Vec<USBDevice>>,
}

/// Returns of Vec of devices in the USBBus as a reference
impl USBBus {
    pub fn flatten_devices<'a>(&'a self) -> Vec<&'a USBDevice> {
        if let Some(devices) = &self.devices {
            get_all_devices(&devices)
        } else {
            Vec::new()
        }
    }
}

/// Recursively gets reference to all devices in a `USBDevice`
pub fn get_all_devices(devices: &Vec<USBDevice>) -> Vec<&USBDevice> {
    let mut ret: Vec<&USBDevice> = Vec::new();
    for device in devices {
        // push each device into pointer array
        ret.push(device);
        // and run recursively for the device if it has some
        if let Some(d) = &device.devices {
            ret.append(&mut get_all_devices(&d))
        }
    }

    return ret;
}

pub fn write_devices_recursive(f: &mut fmt::Formatter, devices: &Vec<USBDevice>) -> fmt::Result {
    for device in devices {
        // print the device details
        if f.alternate() {
            if f.sign_plus() {
                writeln!(f, "{:+#}", device)?;
            } else {
                writeln!(f, "{:#}", device)?;
            }
        } else if f.sign_plus() {
            writeln!(f, "{:+}", device)?;
        } else {
            writeln!(f, "{}", device)?;
        }
        // print all devices with this device - if hub for example
        device
            .devices
            .as_ref()
            .map(|d| write_devices_recursive(f, d));
    }
    Ok(())
}

impl fmt::Display for USBBus {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        // use plus formatter to add tree
        let tree: &str = if !f.sign_plus() {
            ""
        } else {
            if f.alternate() {
                if self.devices.is_some() {
                    "╓ "
                } else {
                    "- "
                }
            // lsusb tree
            } else {
                "/: "
            }
        };

        // write the bus details - alternative for coloured and apple info style
        if f.alternate() {
            writeln!(
                f,
                "{:}{:} {:} {:}:{:} Revision: 0x{:04x}",
                tree.bright_black().bold(),
                self.name.blue(),
                self.host_controller.green(),
                format!("0x{:04x}", self.pci_vendor.unwrap_or(0xffff))
                    .yellow()
                    .bold(),
                format!("0x{:04x}", self.pci_device.unwrap_or(0xffff)).yellow(),
                self.pci_revision.unwrap_or(0xffff),
            )?;
        // lsusb style but not really accurate...
        } else {
            writeln!(
                f,
                "{:}Bus {:03} Device 000: ID {:04x}:{:04x} {:} {:}",
                tree,
                // bus number is not always provided in host json so try to extract from first device
                self.usb_bus_number.unwrap_or(self.devices.as_ref().map_or(None, |d| d.first().map(|dd| dd.location_id.bus)).unwrap_or(0)),
                self.pci_vendor.unwrap_or(0xffff),
                self.pci_device.unwrap_or(0xffff),
                self.name,
                self.host_controller,
            )?;
        }
        // followed by devices if there are some
        self.devices.as_ref().map(|d| write_devices_recursive(f, d));
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
/// location_id String from system_profiler is "LocationReg / Port"
/// The LocationReg has the tree structure (0xbbdddddd):
///   0x  -- always
///   bb  -- bus number in hexadecimal
///   dddddd -- up to six levels for the tree, each digit represents its
///             position on that level
struct DeviceLocation {
    bus: u8,
    tree_positions: Vec<u8>,
    port: Option<u8>,
}

impl FromStr for DeviceLocation {
    type Err = io::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let location_split: Vec<&str> = s.split("/").collect();
        let reg = location_split
            .first()
            .unwrap()
            .trim()
            .trim_start_matches("0x");

        // get position in tree based on number of non-zero chars or just 0 if not using tree
        let tree_positions = reg.get(2..).unwrap_or("0").trim_end_matches("0")
            .chars().map(|v| v.to_digit(10).unwrap_or(0) as u8).collect();
        // bus no is msb
        let bus = (u32::from_str_radix(&reg, 16).map_err(|v| io::Error::new(io::ErrorKind::Other, v)).unwrap() >> 24) as u8;
        // port is after / but not always present
        let port = match location_split
            .last()
            .unwrap()
            .trim()
            .parse::<u8>() {
                Ok(v) => Some(v),
                Err(_) => None,
            };

        Ok(DeviceLocation { bus, tree_positions, port })
    }
}

impl<'de> Deserialize<'de> for DeviceLocation {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct DeviceLocationVisitor;

        impl<'de> Visitor<'de> for DeviceLocationVisitor {
            type Value = DeviceLocation;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("a string representation of speed")
            }

            fn visit_string<E>(self, value: String) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                DeviceLocation::from_str(value.as_str()).map_err(serde::de::Error::custom)
            }

            fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                DeviceLocation::from_str(value).map_err(serde::de::Error::custom)
            }
        }

        deserializer.deserialize_any(DeviceLocationVisitor)
    }
}


#[derive(Debug, Clone, PartialEq, Serialize)]
struct DeviceNumericalUnit<T> {
    value: T,
    unit: String,
    description: Option<String>,
}

impl fmt::Display for DeviceNumericalUnit<u32> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{:} {:}", self.value, self.unit)
    }
}

impl fmt::Display for DeviceNumericalUnit<f32> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        if let Some(precision) = f.precision() {
            // If we received a precision, we use it.
            write!(f, "{1:.*} {2}", precision, self.value, self.unit)
        } else {
            // Otherwise we default to 2.
            write!(f, "{:.2} {}", self.value, self.unit)
        }
    }
}

impl FromStr for DeviceNumericalUnit<u32> {
    type Err = io::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let value_split: Vec<&str> = s.trim().split(' ').collect();
        if value_split.len() >= 2 {
            Ok(DeviceNumericalUnit { 
                value: value_split[0].trim().parse::<u32>().map_err(|e| io::Error::new(io::ErrorKind::Other, e))?, 
                unit: value_split[1].trim().to_string(), 
                description: None,
            })
        } else {
            Err(io::Error::new(io::ErrorKind::Other, "string split does not contain [u32] [unit]"))
        }
    }
}

impl FromStr for DeviceNumericalUnit<f32> {
    type Err = io::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let value_split: Vec<&str> = s.trim().split(' ').collect();
        if value_split.len() >= 2 {
            Ok(DeviceNumericalUnit { 
                value: value_split[0].trim().parse::<f32>().map_err(|e| io::Error::new(io::ErrorKind::Other, e))?, 
                unit: value_split[1].trim().to_string(), 
                description: None,
            })
        } else {
            Err(io::Error::new(io::ErrorKind::Other, "string split does not contain [f32] [unit]"))
        }
    }
}


//TODO these should be generic but not sure how to pass generic to visitor?
impl<'de> Deserialize<'de> for DeviceNumericalUnit<u32> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {

        struct DeviceNumericalUnitU32Visitor;

        impl<'de> Visitor<'de> for DeviceNumericalUnitU32Visitor {
            type Value = DeviceNumericalUnit<u32>;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("a string with format '[int] [unit]'")
            }

            fn visit_string<E>(self, value: String) -> Result<Self::Value, E>
                where
                    E: de::Error,
            {
                Ok(DeviceNumericalUnit::from_str(value.as_str()).map_err(E::custom)?)
            }

            fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
                where
                    E: de::Error,
            {
                Ok(DeviceNumericalUnit::from_str(value).map_err(E::custom)?)
            }
        }

        deserializer.deserialize_str(DeviceNumericalUnitU32Visitor)
    }
}

impl<'de> Deserialize<'de> for DeviceNumericalUnit<f32> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {

        struct DeviceNumericalUnitF32Visitor;

        impl<'de> Visitor<'de> for DeviceNumericalUnitF32Visitor {
            type Value = DeviceNumericalUnit<f32>;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("a string with format '[float] [unit]'")
            }

            fn visit_string<E>(self, value: String) -> Result<Self::Value, E>
                where
                    E: de::Error,
            {
                Ok(DeviceNumericalUnit::from_str(value.as_str()).map_err(E::custom)?)
            }

            fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
                where
                    E: de::Error,
            {
                Ok(DeviceNumericalUnit::from_str(value).map_err(E::custom)?)
            }
        }

        deserializer.deserialize_str(DeviceNumericalUnitF32Visitor)
    }
}

// TODO this could probably convert to Enun of SuperSpeedPlus, FullSpeed etc so that serde auto deserialize enum then use TryInto DeviceNumericalUnit for enum value
#[derive(Debug, Clone, PartialEq, Serialize)]
enum DeviceSpeed {
    NumericalUnit(DeviceNumericalUnit<f32>),
    Description(String),
}

impl fmt::Display for DeviceSpeed {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            DeviceSpeed::NumericalUnit(v) => write!(f, "{:.1}", v),
            DeviceSpeed::Description(v) => write!(f, "{}", v),
        }
    }
}

impl<'de> Deserialize<'de> for DeviceSpeed {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        struct DeviceSpeedVisitor;

        impl<'de> Visitor<'de> for DeviceSpeedVisitor {
            type Value = DeviceSpeed;

            fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
                formatter.write_str("a string representation of speed")
            }

            fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
            where
                E: de::Error,
            {
                // TODO
                // value.try_into(DeviceNumericalUnit);

                match value.trim() {
                    "super_speed_plus" => Ok(DeviceSpeed::NumericalUnit(DeviceNumericalUnit{value: 20.0, unit: String::from("Gb/s"), description: Some(value.to_owned())})),
                    "super_speed" => Ok(DeviceSpeed::NumericalUnit(DeviceNumericalUnit{value: 5.0, unit: String::from("Gb/s"), description: Some(value.to_owned())})),
                    "high_speed"|"high_bandwidth" => Ok(DeviceSpeed::NumericalUnit(DeviceNumericalUnit{value: 480.0, unit: String::from("Mb/s"), description: Some(value.to_owned())})),
                    "full_speed" => Ok(DeviceSpeed::NumericalUnit(DeviceNumericalUnit{value: 12.0, unit: String::from("Mb/s"), description: Some(value.to_owned())})),
                    "low_speed" => Ok(DeviceSpeed::NumericalUnit(DeviceNumericalUnit{value: 1.5, unit: String::from("Mb/s"), description: Some(value.to_owned())})),
                    v => Ok(DeviceSpeed::Description(v.to_string())),
                }
            }
        }

        deserializer.deserialize_any(DeviceSpeedVisitor)
    }
}


#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct USBDevice {
    #[serde(rename(deserialize = "_name"))]
    name: String,
    #[serde(default, deserialize_with = "deserialize_option_number_from_string")]
    vendor_id: Option<u16>,
    #[serde(default, deserialize_with = "deserialize_option_number_from_string")]
    product_id: Option<u16>,
    location_id: DeviceLocation,
    serial_num: Option<String>,
    manufacturer: Option<String>,
    #[serde(default, deserialize_with = "deserialize_option_number_from_string")]
    bcd_device: Option<f32>,
    #[serde(default, deserialize_with = "deserialize_option_number_from_string")]
    bus_power: Option<u16>,
    #[serde(default, deserialize_with = "deserialize_option_number_from_string")]
    bus_power_used: Option<u16>,
    device_speed: Option<DeviceSpeed>,
    #[serde(default, deserialize_with = "deserialize_option_number_from_string")]
    extra_current_used: Option<u8>,
    // devices can be hub and have devices attached
    #[serde(rename(deserialize = "_items"))]
    devices: Option<Vec<USBDevice>>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct USBFilter {
    pub vid: Option<u16>,
    pub pid: Option<u16>,
    pub bus: Option<u8>,
    pub port: Option<u8>,
}

impl USBFilter {
    // pub fn retain_buses(self, buses: &mut Vec<USBBus>) -> () {
    //     buses.retain(|b| b.usb_bus_number == self.bus || self.bus.is_none() || b.usb_bus_number.is_none());

    //     for bus in buses {
    //         bus.devices.as_mut().map_or((), |d| self.retain_devices(d));
    //     }
    // }

    // pub fn retain_devices(self, devices: &mut Vec<USBDevice>) -> () {
    //     *devices = devices
    //         .iter()
    //         .filter(|d| Some(d.location_id.bus) == self.bus || self.bus.is_none())
    //         .filter(|d| d.location_id.port == self.port || self.port.is_none())
    //         .filter(|d| d.vendor_id == self.vid || self.vid.is_none())
    //         .filter(|d| d.product_id == self.pid || self.pid.is_none())
    //         .map(|d| d.to_owned())
    //         .collect::<Vec<USBDevice>>();

    //     for d in devices {
    //         d.devices.as_mut().map_or((), |d| self.retain_devices(d));
    //     }
    // }

    pub fn filter_devices_ref(self, devices: Vec<&USBDevice>) -> Vec<&USBDevice> {
        let new: Vec<&USBDevice> = devices
            .into_iter()
            .filter(|d| Some(d.location_id.bus) == self.bus || self.bus.is_none())
            .filter(|d| d.location_id.port == self.port || self.port.is_none())
            .filter(|d| d.vendor_id == self.vid || self.vid.is_none())
            .filter(|d| d.product_id == self.pid || self.pid.is_none())
            .collect();

        new
    }
}

impl fmt::Display for USBDevice {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let mut spaces = if f.sign_plus() {
            self.location_id.tree_positions.len() * 4
        } else {
            0
        };

        // map speed from text back to data rate if tree
        let speed = match &self.device_speed {
            Some(v) => v.to_string(),
            None => String::from(""),
        };

        // tree chars to prepend if plus formatted
        let tree: &str = if !f.sign_plus() {
            ""
        } else {
            // TODO use "╟─ " unless last
            if f.alternate() {
                "╙── "
            } else {
                "|__ "
            }
        };

        // alternate for coloured, slightly different format to lsusb
        if f.alternate() {
            write!(
                f,
                "{:>spaces$}{}/{} {}:{} {} {} {}",
                tree.bright_black(),
                format!("{:03}", self.location_id.bus).cyan(),
                format!("{:03}", self.location_id.port.unwrap_or(0)).magenta(),
                format!("0x{:04x}", self.vendor_id.unwrap()).yellow().bold(),
                format!("0x{:04x}", self.product_id.unwrap()).yellow(),
                self.name.trim().bold().blue(),
                self.serial_num
                    .as_ref()
                    .unwrap_or(&String::from("None"))
                    .trim()
                    .green(),
                speed.purple()
            )
        // not same data as lsusb when tree (show port, class, driver etc.)
        } else {
            // add 3 because lsusb is like this
            if spaces > 0 {
                spaces += 3;
            }
            write!(
                f,
                "{:>spaces$}Bus {:03} Device {:03}: ID {:04x}:{:04x} {}",
                tree,
                self.location_id.bus,
                self.location_id.port.unwrap_or(0),
                self.vendor_id.unwrap_or(0xffff),
                self.product_id.unwrap_or(0xffff),
                self.name.trim(),
            )
        }
    }
}

pub fn get_spusb() -> Result<SPUSBDataType, io::Error> {
    let output = if cfg!(target_os = "macos") {
        Command::new("system_profiler")
            .args(["-json", "SPUSBDataType"])
            .output()?
    } else {
        return Err(io::Error::new(
            io::ErrorKind::Unsupported,
            "system_profiler is only supported on macOS",
        ));
    };

    serde_json::from_str(String::from_utf8(output.stdout).unwrap().as_str())
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_deserialize_device() {
        let device_json = "{
              \"_name\" : \"Arduino Zero\",
              \"bcd_device\" : \"1.00\",
              \"bus_power\" : \"500\",
              \"bus_power_used\" : \"500\",
              \"device_speed\" : \"full_speed\",
              \"extra_current_used\" : \"0\",
              \"location_id\" : \"0x02110000 / 3\",
              \"manufacturer\" : \"Arduino LLC\",
              \"product_id\" : \"0x804d\",
              \"serial_num\" : \"6DC00ADC5053574C342E3120FF122422\",
              \"vendor_id\" : \"0x2341\"
            }";

        let device: USBDevice = serde_json::from_str(device_json).unwrap();

        assert_eq!(device.name, "Arduino Zero");
        assert_eq!(device.bcd_device, Some(1.00));
        assert_eq!(device.bus_power, Some(500));
        assert_eq!(device.bus_power_used, Some(500));
        assert_eq!(device.device_speed, Some(DeviceSpeed::NumericalUnit(DeviceNumericalUnit { value: 12.0, unit: String::from("Mb/s"), description: Some(String::from("full_speed")) })));
        assert_eq!(device.extra_current_used, Some(0));
        assert_eq!(device.location_id, DeviceLocation{ bus: 2, tree_positions: vec![1, 1], port: Some(3)} );
        assert_eq!(device.manufacturer, Some("Arduino LLC".to_string()));
        assert_eq!(device.product_id, Some(0x804d));
        assert_eq!(device.vendor_id, Some(0x2341));
    }

    #[test]
    fn test_deserialize_bus() {
        let device_json = "{
            \"_name\" : \"USB31Bus\",
            \"host_controller\" : \"AppleUSBXHCITR\",
            \"pci_device\" : \"0x15f0 \",
            \"pci_revision\" : \"0x0006 \",
            \"pci_vendor\" : \"0x8086 \",
            \"usb_bus_number\" : \"0x00 \"
        }";

        let device: USBBus = serde_json::from_str(device_json).unwrap();

        assert_eq!(device.name, "USB31Bus");
        assert_eq!(device.host_controller, "AppleUSBXHCITR");
        assert_eq!(device.pci_device, Some(0x15f0));
        assert_eq!(device.pci_revision, Some(0x0006));
        assert_eq!(device.pci_vendor, Some(0x8086));
        assert_eq!(device.usb_bus_number, Some(0x00));
    }
}
