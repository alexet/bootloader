use core::fmt;
use uart_16550::backend::PioBackend;

// TODO this type can be replaced with Uart16550Tty but using it currently panics
// in the new constructor.
pub struct SerialPort {
    port: uart_16550::Uart16550<PioBackend>,
}

impl SerialPort {
    /// Attempts to bring up the legacy COM1 UART at I/O port `0x3F8`.
    ///
    /// Real hardware commonly has no working 16550 at this port (no physical
    /// serial port, or it's not decoded/enabled in firmware), which fails the
    /// scratch-register loopback test `Uart16550::init` performs to detect the
    /// device. That's a normal, expected outcome on such machines — not a bug —
    /// so this returns `None` instead of panicking, leaving the caller to fall
    /// back to other logging (e.g. the framebuffer).
    ///
    /// # Safety
    ///
    /// unsafe because this function must only be called once
    pub unsafe fn init() -> Option<Self> {
        let mut port =
            unsafe { uart_16550::Uart16550::new_port(0x3F8) }.expect("should be valid port");
        port.init(uart_16550::Config::default()).ok()?;
        Some(Self { port })
    }
}

impl fmt::Write for SerialPort {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for char in s.bytes() {
            match char {
                b'\n' => self.port.send_bytes_exact(b"\r\n"),
                byte => self.port.send_bytes_exact(&[byte]),
            }
        }
        Ok(())
    }
}
