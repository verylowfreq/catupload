use std::fs;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use clap::Parser;
use hidapi::{HidApi, HidDevice};

#[repr(u8)]
#[derive(Copy, Clone, Debug)]
enum Command {
    Nop = 0,
    Ident = 1,
    Erase = 2,
    ProgramStart = 3,
    ProgramAppend = 4,
    Flush = 5,
    Read = 6,
    Reset = 7,
    Crc = 8,
}

const TIMEOUT: Duration = Duration::from_millis(100);
const READ_BLOCK_SIZE: usize = 62;
const WRITE_BLOCK_SIZE: usize = 54;
const FLUSH_PAGE_SIZE: usize = 4096;
const PACKET_SIZE: usize = 64;

#[derive(Debug, Parser)]
#[command(author, version, about = "Bootloader uploader")]
struct Cli {
    /// Path to binary firmware
    #[arg(long, value_name = "PATH")]
    bin: PathBuf,
    /// Base address (e.g. 0x08000000)
    #[arg(long, value_parser = parse_u32, value_name = "ADDR")]
    address: u32,
    /// Offset added to base address
    #[arg(long, default_value = "0", value_parser = parse_u32, value_name = "OFFSET")]
    offset: u32,
    /// VID (default 0xf055)
    #[arg(long, default_value = "0xf055", value_parser = parse_u16, value_name = "VID")]
    vid: u16,
    /// PID (default 0x6585)
    #[arg(long, default_value = "0x6585", value_parser = parse_u16, value_name = "PID")]
    pid: u16,
    /// Product string to match for HID (use empty to skip)
    #[arg(long, default_value = "HID Bootloader", value_name = "NAME")]
    product: String,
    /// Erase before programming
    #[arg(long, default_value_t = false)]
    erase: bool,
    /// Verify CRC16 after programming
    #[arg(long, default_value_t = false)]
    verify: bool,
}

trait BootTransport {
    fn send(
        &self,
        command: Command,
        param1: u32,
        param2: u32,
        data: &[u8],
        timeout: Duration,
    ) -> Result<Vec<u8>>;
    fn send_without_response(
        &self,
        command: Command,
        param1: u32,
        param2: u32,
        data: &[u8],
    ) -> Result<()>;
    fn kind(&self) -> &'static str;
}

struct Bootloader {
    transport: Box<dyn BootTransport>,
}

impl Bootloader {
    fn new(transport: Box<dyn BootTransport>) -> Self {
        Self { transport }
    }

    fn transport_kind(&self) -> &'static str {
        self.transport.kind()
    }

    fn get_ident(&self) -> Result<String> {
        let resp = self.send(Command::Ident, 0, 0, &[], TIMEOUT)?;
        let len = *resp.get(1).unwrap_or(&0) as usize;
        let end = len.saturating_add(2).min(resp.len());
        let text = std::str::from_utf8(&resp[2..end]).context("IDENT returned invalid UTF-8")?;
        Ok(text.to_string())
    }

    fn read(&self, start_address: u32, size: usize) -> Result<Vec<u8>> {
        let mut remaining = size;
        let mut address = start_address;
        let mut data = Vec::with_capacity(size);

        while remaining > 0 {
            let read_len = remaining.min(READ_BLOCK_SIZE) as u32;
            let resp = self.send(Command::Read, address, read_len, &[], TIMEOUT)?;
            let payload_len = usize::min(
                *resp.get(1).unwrap_or(&0) as usize,
                resp.len().saturating_sub(2),
            );
            data.extend_from_slice(&resp[2..2 + payload_len]);
            address = address.wrapping_add(read_len);
            remaining -= read_len as usize;
        }

        Ok(data)
    }

    fn write(&self, start_address: u32, data: &[u8]) -> Result<()> {
        if data.is_empty() {
            return Ok(());
        }

        let mut remaining = data;
        let mut address = start_address;
        let mut pos_in_page: usize = 0;

        println!("Program start");

        while !remaining.is_empty() {
            if pos_in_page == 0 {
                self.send_without_response(Command::ProgramStart, address, 0, &[])?;
            }

            let mut write_len = remaining.len().min(WRITE_BLOCK_SIZE);
            if pos_in_page + write_len > FLUSH_PAGE_SIZE {
                write_len = FLUSH_PAGE_SIZE - pos_in_page;
            }

            let chunk = &remaining[..write_len];
            self.send_without_response(Command::ProgramAppend, address, write_len as u32, chunk)?;
            // std::thread::sleep(Duration::from_millis(1));

            remaining = &remaining[write_len..];
            address = address.wrapping_add(write_len as u32);
            pos_in_page += write_len;

            if pos_in_page == FLUSH_PAGE_SIZE || remaining.is_empty() {
                println!("Program address: 0x{address:08x}");
                self.send(Command::Flush, 0, 0, &[], TIMEOUT.mul_f64(64.0))?;
                pos_in_page = 0;
            }
        }

        self.send(Command::Flush, 0, 0, &[], TIMEOUT.mul_f64(32.0))?;

        println!("OK.");
        Ok(())
    }

    fn erase(&self, start_address: u32, size: usize) -> Result<()> {
        if start_address % FLUSH_PAGE_SIZE as u32 != 0 || size % FLUSH_PAGE_SIZE != 0 {
            bail!("erase requires address and size to be 4096-byte aligned");
        }

        let mut address = start_address;
        let mut remaining = size;
        while remaining > 0 {
            let resp = self.send(
                Command::Erase,
                address,
                FLUSH_PAGE_SIZE as u32,
                &[],
                TIMEOUT.mul_f64((size / 1024) as f64),
            )?;
            if resp.get(0).copied().unwrap_or(0) != 0x01 {
                bail!("Erase failed (addr=0x{address:08x})");
            }
            address = address.wrapping_add(FLUSH_PAGE_SIZE as u32);
            remaining -= FLUSH_PAGE_SIZE;
        }
        Ok(())
    }

    fn verify(&self, start_address: u32, data: &[u8]) -> Result<bool> {
        let expected = crc16_ccitt(data);
        let timeout = TIMEOUT.mul_f64((data.len() as f64 / 1024.0).max(1.0));
        let resp = self.send(Command::Crc, start_address, data.len() as u32, &[], timeout)?;
        let actual = u16::from_le_bytes([
            resp.get(2).copied().unwrap_or(0),
            resp.get(3).copied().unwrap_or(0),
        ]);
        Ok(expected == actual)
    }

    fn reset(&self) {
        let _resp = self.send(Command::Reset, 0, 0, &[], TIMEOUT);
    }

    fn send(
        &self,
        command: Command,
        param1: u32,
        param2: u32,
        data: &[u8],
        timeout: Duration,
    ) -> Result<Vec<u8>> {
        self.transport.send(command, param1, param2, data, timeout)
    }

    fn send_without_response(&self, command: Command, param1: u32, param2: u32, data: &[u8]) -> Result<()> {
        self.transport
            .send_without_response(command, param1, param2, data)
    }
}

struct HidTransport {
    device: HidDevice,
}

impl HidTransport {
    fn new(device: HidDevice) -> Self {
        Self { device }
    }
}

impl BootTransport for HidTransport {
    fn send(
        &self,
        command: Command,
        param1: u32,
        param2: u32,
        data: &[u8],
        timeout: Duration,
    ) -> Result<Vec<u8>> {
        let payload = build_payload(command, param1, param2, data);

        let mut packet = Vec::with_capacity(PACKET_SIZE + 1);
        packet.push(0);
        packet.extend_from_slice(&payload);
        self.device
            .write(&packet)
            .context("HID write failed")?;

        let deadline = Instant::now() + timeout;
        let mut resp = vec![0u8; PACKET_SIZE];

        while Instant::now() < deadline {
            let remaining = deadline.saturating_duration_since(Instant::now());
            let timeout_ms = remaining.as_millis().clamp(1, i64::from(i32::MAX) as u128) as i32;
            let read_len = self
                .device
                .read_timeout(&mut resp, timeout_ms)
                .context("HID read failed")?;
            if read_len == PACKET_SIZE {
                return Ok(resp);
            }
        }

        bail!("HID response timeout");
    }

    fn send_without_response(
        &self,
        command: Command,
        param1: u32,
        param2: u32,
        data: &[u8],
    ) -> Result<()> {
        let payload = build_payload(command, param1, param2, data);
        let mut packet = Vec::with_capacity(PACKET_SIZE + 1);
        packet.push(0);
        packet.extend_from_slice(&payload);
        self.device
            .write(&packet)
            .context("HID write failed")?;
        Ok(())
    }

    fn kind(&self) -> &'static str {
        "HID"
    }
}

mod nusb_transport {
    use super::*;
    use anyhow::Error;
    use futures_lite::future::block_on;
    use nusb::transfer::{Direction, EndpointType, RequestBuffer};

    pub struct NusbTransport {
        interface: nusb::Interface,
        endpoint_in: u8,
        endpoint_out: u8,
    }

    impl NusbTransport {
        pub fn open(vid: u16, pid: u16) -> Result<Self> {
            let devices: Vec<nusb::DeviceInfo> = nusb::list_devices()
                .context("NUSB list_devices failed")?
                .filter(|dev| dev.vendor_id() == vid && dev.product_id() == pid)
                .collect();

            if devices.is_empty() {
                bail!("NUSB device not found");
            }

            log_device_candidates(&devices);

            let mut preferred = Vec::with_capacity(devices.len());
            // let mut fallback = Vec::new();
            preferred.extend(devices);
            // for dev in devices {
            //     if is_winusb_driver(&dev) {
            //         preferred.push(dev);
            //     } else {
            //         fallback.push(dev);
            //     }
            // }
            // preferred.extend(fallback);

            let mut last_err = None;
            for device_info in preferred {
                log_device_attempt(&device_info);
                let result = try_open_device(&device_info);
                match result {
                    Ok(transport) => return Ok(transport),
                    Err(err) => last_err = Some(err),
                }
            }

            Err(last_err.unwrap_or_else(|| anyhow::anyhow!("NUSB open failed")))
        }
    }

    impl BootTransport for NusbTransport {
        fn send(
            &self,
            command: Command,
            param1: u32,
            param2: u32,
            data: &[u8],
            _timeout: Duration,
        ) -> Result<Vec<u8>> {
            let payload = build_payload(command, param1, param2, data);

            let completion = block_on(self.interface.bulk_out(self.endpoint_out, payload))
                .into_result()
                .context("NUSB write failed")?;
            if completion.actual_length() != PACKET_SIZE {
                bail!("NUSB write size mismatch ({} bytes)", completion.actual_length());
            }

            for _ in 0..8 {
                let resp =
                    block_on(self.interface.bulk_in(self.endpoint_in, RequestBuffer::new(PACKET_SIZE)))
                        .into_result()
                        .context("NUSB read failed")?;
                if resp.len() != PACKET_SIZE {
                    std::thread::sleep(Duration::from_millis(2));
                    continue;
                } else {
                    return Ok(resp);
                }
            }
            Err(Error::msg("no response"))
            // Ok(resp)
        }

        fn send_without_response(
            &self,
            command: Command,
            param1: u32,
            param2: u32,
            data: &[u8],
        ) -> Result<()> {
            let payload = build_payload(command, param1, param2, data);
            let completion = block_on(self.interface.bulk_out(self.endpoint_out, payload))
                .into_result()
                .context("NUSB write failed")?;
            if completion.actual_length() != PACKET_SIZE {
                bail!("NUSB write size mismatch ({} bytes)", completion.actual_length());
            }
            Ok(())
        }

        fn kind(&self) -> &'static str {
            "NUSB"
        }
    }

    fn try_open_device(device_info: &nusb::DeviceInfo) -> Result<NusbTransport> {
        let device = device_info.open().context("NUSB open failed")?;
        let config = device
            .active_configuration()
            .context("NUSB active configuration failed")?;
        let (interface_number, alt_setting, endpoint_in, endpoint_out) =
            find_vendor_bulk_interface(&config)
                .context("Vendor bulk endpoints not found")?;
        eprintln!(
            "NUSB select interface={} alt={} in=0x{:02x} out=0x{:02x}",
            interface_number, alt_setting, endpoint_in, endpoint_out
        );

        let interface = device
            .claim_interface(interface_number)
            .with_context(|| format!("NUSB claim interface {interface_number} failed"))?;
        if alt_setting != 0 {
            interface
                .set_alt_setting(alt_setting)
                .with_context(|| format!("NUSB set alt setting {alt_setting} failed"))?;
        }

        Ok(NusbTransport {
            interface,
            endpoint_in,
            endpoint_out,
        })
    }

    fn is_winusb_driver(dev: &nusb::DeviceInfo) -> bool {
        #[cfg(target_os = "windows")]
        {
            if let Some(driver) = dev.driver() {
                return driver.eq_ignore_ascii_case("winusb");
            }
        }
        false
    }

    fn find_vendor_bulk_interface(
        config: &nusb::descriptors::Configuration<'_>,
    ) -> Option<(u8, u8, u8, u8)> {
        let mut best = None;

        for interface in config.interfaces() {
            for alt in interface.alt_settings() {
                let mut endpoint_in = None;
                let mut endpoint_out = None;
                for endpoint in alt.endpoints() {
                    if endpoint.transfer_type() != EndpointType::Bulk {
                        continue;
                    }
                    match endpoint.direction() {
                        Direction::In => {
                            if endpoint_in.is_none() {
                                endpoint_in = Some(endpoint.address());
                            }
                        }
                        Direction::Out => {
                            if endpoint_out.is_none() {
                                endpoint_out = Some(endpoint.address());
                            }
                        }
                    }
                }

                if let (Some(in_ep), Some(out_ep)) = (endpoint_in, endpoint_out) {
                    let candidate = (
                        interface.interface_number(),
                        alt.alternate_setting(),
                        in_ep,
                        out_ep,
                    );
                    if alt.class() == 0xFF {
                        return Some(candidate);
                    }
                    if best.is_none() {
                        best = Some(candidate);
                    }
                }
            }
        }
        best
    }

    fn log_device_candidates(devices: &[nusb::DeviceInfo]) {
        eprintln!("NUSB candidates: {}", devices.len());
        for (idx, dev) in devices.iter().enumerate() {
            #[cfg(target_os = "windows")]
            {
                eprintln!(
                    "  [{}] driver={:?} instance_id={:?}",
                    idx,
                    dev.driver(),
                    dev.instance_id()
                );
            }
            #[cfg(not(target_os = "windows"))]
            {
                eprintln!("  [{}] device", idx);
            }

            let mut interfaces = dev.interfaces().peekable();
            if interfaces.peek().is_none() {
                eprintln!("      interfaces: <none>");
            } else {
                for iface in interfaces {
                    eprintln!(
                        "      interface {} class=0x{:02x} subclass=0x{:02x} protocol=0x{:02x}",
                        iface.interface_number(),
                        iface.class(),
                        iface.subclass(),
                        iface.protocol()
                    );
                }
            }
        }
    }

    fn log_device_attempt(dev: &nusb::DeviceInfo) {
        #[cfg(target_os = "windows")]
        {
            eprintln!(
                "NUSB try: driver={:?} instance_id={:?}",
                dev.driver(),
                dev.instance_id()
            );
        }
        #[cfg(not(target_os = "windows"))]
        {
            eprintln!("NUSB try device");
        }
    }
}

fn build_payload(command: Command, param1: u32, param2: u32, data: &[u8]) -> Vec<u8> {
    let mut payload = Vec::with_capacity(PACKET_SIZE);
    payload.push(command as u8);
    payload.extend_from_slice(&param1.to_le_bytes());
    payload.extend_from_slice(&param2.to_le_bytes());
    payload.extend_from_slice(data);
    payload.resize(PACKET_SIZE, 0);
    payload
}

fn parse_u64(input: &str) -> Result<u64, String> {
    let (radix, digits) = if let Some(rest) = input.strip_prefix("0x").or_else(|| input.strip_prefix("0X")) {
        (16, rest)
    } else if let Some(rest) = input.strip_prefix("0b").or_else(|| input.strip_prefix("0B")) {
        (2, rest)
    } else if let Some(rest) = input.strip_prefix("0o").or_else(|| input.strip_prefix("0O")) {
        (8, rest)
    } else {
        (10, input)
    };

    u64::from_str_radix(digits, radix)
        .map_err(|e| format!("invalid number ({input}): {e}"))
}

fn parse_u32(input: &str) -> Result<u32, String> {
    parse_u64(input).and_then(|v| u32::try_from(v).map_err(|_| format!("number too large ({input})")))
}

fn parse_u16(input: &str) -> Result<u16, String> {
    parse_u64(input).and_then(|v| u16::try_from(v).map_err(|_| format!("number too large ({input})")))
}

fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    let poly: u16 = 0x1021;

    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }

    crc
}

fn open_hid_transport(cli: &Cli) -> Result<HidTransport> {
    let api = HidApi::new().context("HID API init failed")?;
    let device = api
        .open(cli.vid, cli.pid)
        .context("HID device open failed")?;

    if !cli.product.is_empty() {
        let product_string = device
            .get_product_string()
            .context("HID product string read failed")?;
        if let Some(name) = product_string {
            if name != cli.product {
                bail!("Product name mismatch: device=\"{name}\" expected=\"{}\"", cli.product);
            }
        } else {
            bail!("HID product string missing");
        }
    }

    Ok(HidTransport::new(device))
}

fn open_transport(cli: &Cli) -> Result<Box<dyn BootTransport>> {
    match nusb_transport::NusbTransport::open(cli.vid, cli.pid) {
        Ok(transport) => Ok(Box::new(transport)),
        Err(err) => {
            eprintln!("NUSB open failed, retrying with HID: {err:?}");
            Ok(Box::new(open_hid_transport(cli)?))
        }
    }
}

fn run() -> Result<()> {
    let cli = Cli::parse();
    let start_address = cli
        .address
        .checked_add(cli.offset)
        .context("address + offset overflow")?;

    let firmware = fs::read(&cli.bin)
        .with_context(|| format!("failed to read firmware: {}", cli.bin.display()))?;

    if firmware.is_empty() {
        bail!("firmware is empty: {}", cli.bin.display());
    }

    println!(
        "Device: vid=0x{vid:04x} pid=0x{pid:04x}",
        vid = cli.vid,
        pid = cli.pid
    );

    let transport = open_transport(&cli)?;
    let mut boot = Bootloader::new(transport);
    println!("Transport: {}", boot.transport_kind());
    let ident = match boot.get_ident() {
        Ok(ident) => ident,
        Err(err) => {
            if boot.transport_kind() == "NUSB" {
                eprintln!("IDENT failed over NUSB, retrying with HID: {err:?}");
                boot = Bootloader::new(Box::new(open_hid_transport(&cli)?));
                println!("Transport: {}", boot.transport_kind());
                boot.get_ident()?
            } else {
                return Err(err);
            }
        }
    };
    println!("Ident: {ident}");

    if cli.erase {
        let erase_size = ((firmware.len() + FLUSH_PAGE_SIZE - 1) / FLUSH_PAGE_SIZE) * FLUSH_PAGE_SIZE;
        println!(
            "Erase: addr=0x{start:08x} size={size} ({} KB)",
            erase_size / 1024,
            start = start_address,
            size = erase_size
        );
        boot.erase(start_address, erase_size)?;
    }

    println!(
        "Program: addr=0x{start:08x} size={} bytes",
        firmware.len(),
        start = start_address
    );
    boot.write(start_address, &firmware)?;
    println!("Write done.");

    if cli.verify {
        println!("CRC16 verify...");
        let ok = boot.verify(start_address, &firmware)?;
        if ok {
            println!("Verify OK");
        } else {
            bail!("Verify NG (CRC mismatch)");
        }
    }

    println!("Resetting device...");
    boot.reset();

    Ok(())
}

fn main() {
    if let Err(err) = run() {
        eprintln!("Error: {err:?}");
        std::process::exit(1);
    }
}
