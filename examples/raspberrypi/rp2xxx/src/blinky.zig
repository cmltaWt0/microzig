const std = @import("std");
const microzig = @import("microzig");

const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const usb = microzig.core.usb;

// Port-specific USB device implementation used by the USB core.
const USB_Device = rp2xxx.usb.Polled(.{});
// CDC (serial) class driver — this is what shows up as /dev/ttyACM0 on the host.
const USB_Serial = usb.drivers.CDC;

var usb_device: USB_Device = undefined;

// Device controller with a CDC serial driver and picotool-controlled reset.
var usb_controller: usb.DeviceController(.{
    .bcd_usb = USB_Device.max_supported_bcd_usb,
    .device_triple = .unspecified,
    .vendor = USB_Device.default_vendor_id,
    .product = USB_Device.default_product_id,
    .bcd_device = .v1_00,
    .serial = "blinky-cdc",
    .max_supported_packet_size = USB_Device.max_supported_packet_size,
    .configurations = &.{.{
        .attributes = .{ .self_powered = false },
        .max_current_ma = 50,
        .Drivers = struct { serial: USB_Serial, reset: rp2xxx.usb.ResetDriver(null, 0) },
    }},
}, .{.{
    .serial = .{ .itf_notifi = "Board CDC", .itf_data = "Board CDC Data" },
    .reset = "",
}}) = .init;

// We send our output explicitly over USB CDC below, but std_options must still
// provide a freestanding logFn — otherwise std pulls in its stderr-based default
// logger, which doesn't exist on a microcontroller.
pub const std_options = microzig.std_options(.{
    .log_level = .info,
    .logFn = rp2xxx.uart.log,
});

comptime {
    _ = microzig.export_startup();
}

pub fn panic(message: []const u8, _: ?*std.builtin.StackTrace, _: ?usize) noreturn {
    _ = message;
    @breakpoint();
    while (true) {}
}

const pin_config: rp2xxx.pins.GlobalConfiguration = .{
    .GPIO25 = .{ .name = "led", .direction = .out },
};

// BME280 / BMP280: the "Who Am I" chip-ID register.
const BME280_ADDR: rp2xxx.i2c.Address = @enumFromInt(0x76);
const REG_CHIP_ID: u8 = 0xD0;

pub fn main() !void {
    const pins = pin_config.apply();

    // Configure I2C0 on GPIO4 (SDA) / GPIO5 (SCL). Pins are set up separately
    // from i2c.apply() — they are not part of the I2C Config.
    const i2c0 = rp2xxx.i2c.instance.num(0);
    const sda_pin = rp2xxx.gpio.num(4);
    const scl_pin = rp2xxx.gpio.num(5);
    inline for (&.{ scl_pin, sda_pin }) |pin| {
        pin.set_slew_rate(.slow);
        pin.set_schmitt_trigger_enabled(true);
        pin.set_function(.i2c);
    }
    i2c0.apply(.{ .clock_config = rp2xxx.clock_config });

    // Read the chip-ID register. Don't `try` here: a missing/miswired sensor
    // would otherwise fault the chip before USB enumerates, leaving no output.
    // Capture the result and report it over USB instead.
    var id_buffer: [1]u8 = .{0};
    const id_result = i2c0.write_then_read_blocking(BME280_ADDR, &.{REG_CHIP_ID}, &id_buffer, null);

    // Initialize the USB device. The host will enumerate it as a serial port.
    usb_device = .init();

    var last: u64 = time.get_time_since_boot().to_us();
    var i: u32 = 0;

    while (true) {
        // Must be polled frequently to service USB events.
        usb_device.poll(&usb_controller);

        // drivers() is non-null only once the host has finished enumeration.
        if (usb_controller.drivers()) |drivers| {
            const now = time.get_time_since_boot().to_us();
            if (now - last > 1_000_000) {
                last = now;
                pins.led.toggle();
                i += 1;
                usb_cdc_write(&drivers.serial, "LED is ON ({})\r\n", .{i});

                if (id_result) |_| {
                    const id = id_buffer[0];
                    const name = switch (id) {
                        0x60 => "BME280-60",
                        0x58 => "BMP280-58",
                        else => "Unknown",
                    };
                    usb_cdc_write(&drivers.serial, "{s} chip ID: 0x{X:0>2}\r\n", .{ name, id });
                } else |err| {
                    usb_cdc_write(&drivers.serial, "BME280 read failed: {s}\r\n", .{@errorName(err)});
                }
            }
        }
    }
}

var usb_tx_buff: [256]u8 = undefined;

/// Transfer data to host. After each chunk we poll so the bus TX events get handled.
pub fn usb_cdc_write(serial: *USB_Serial, comptime fmt: []const u8, args: anytype) void {
    var tx = std.fmt.bufPrint(&usb_tx_buff, fmt, args) catch &.{};

    while (tx.len > 0) {
        tx = tx[serial.write(tx)..];
        usb_device.poll(&usb_controller);
    }
    // Short messages buffer up, so force a flush.
    while (!serial.flush())
        usb_device.poll(&usb_controller);
}
