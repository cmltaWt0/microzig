const std = @import("std");
const microzig = @import("microzig");
const bme280 = @import("bme280.zig");
const font8x8 = @import("font8x8");

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

    time.sleep_ms(250); // let the OLED + MBE280 finish their power-on before I2C init

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
    // var id_buffer: [1]u8 = .{0};
    // const id_result = i2c0.write_then_read_blocking(BME280_ADDR, &.{REG_CHIP_ID}, &id_buffer, null);
    var sensor = bme280.BME280.init(i2c0, BME280_ADDR) catch null;

    const ssd1306 = microzig.drivers.display.ssd1306;
    const oled_dd = rp2xxx.drivers.I2C_Datagram_Device.init(
        i2c0, @enumFromInt(0x3C), null
    );
    const lcd = ssd1306.init(
        .i2c, oled_dd, null
    ) catch null; // ?Display — keeps USB alive if absent

    if (lcd) |l| {
        var screen: [16 * 8]u8 = @splat(' ');
        _ = std.fmt.bufPrint(screen[16..32], "     Hello!", .{}) catch {};
        _ = std.fmt.bufPrint(screen[48..64], "     Weather", .{}) catch {};
        var glyphs: [screen.len * 8]u8 = undefined;
        _ = font8x8.Fonts.draw(&glyphs, &screen); // fills all 1024 bytes of glyphs with font data
        l.write_full_display(&glyphs) catch {}; // sets addressing window + write full frame
    }
    // Initialize the USB device. The host will enumerate it as a serial port.
    usb_device = .init();

    var last: u64 = time.get_time_since_boot().to_us();
    var i: u32 = 0;

    while (true) {
        // Must be polled frequently to service USB events. REQUIRED even if we
        // don't use CDC: picotool's `-f` reboot talks to the ResetDriver, which
        // only responds while this is being polled. poll() is non-blocking.
        usb_device.poll(&usb_controller);

        const now = time.get_time_since_boot().to_us();

        if (now - last > 1_000_000) {
            last = now;
            pins.led.toggle();
            i += 1;

            // Time for Display
            const secs = now / 1_000_000;
            const hh = secs / 3600;
            const mm = (secs % 3600) / 60;
            const ss = secs % 60;

            if (sensor) |*s| {
                if (s.measure()) |m| {
                    var screen: [16 * 8]u8 = @splat(' ');       // 4 rows, all spaces
                    _ = std.fmt.bufPrint(screen[0..16], "T:{d:.1}C", .{ m.temperature_c }) catch {};
                    _ = std.fmt.bufPrint(screen[32..48], "P:{d:.0}hPa", .{ m.pressure_pa / 100.0}) catch {};
                    _ = std.fmt.bufPrint(screen[64..80], "H:{d:.1}%", .{ m.humidity_rh}) catch {};
                    _ = std.fmt.bufPrint(screen[96..112], "U:{d:0>2}:{d:0>2}:{d:0>2}", .{ hh, mm, ss}) catch {};

                    var glyphs: [screen.len * 8]u8 = undefined;  // 8 bytes per char
                    if (lcd) |l| {
                        _ = font8x8.Fonts.draw(&glyphs, &screen); // fills all 1024 bytes of glyphs with font data
                        l.write_full_display(&glyphs) catch {}; // sets addressing window + write full frame
                        if (usb_controller.drivers()) |drivers| {
                            usb_cdc_write(&drivers.serial, "T: {d:.2} C  P: {d:.2} hPa  H: {d:.2} %\r\n",
                                .{ m.temperature_c, m.pressure_pa / 100.0, m.humidity_rh });
                        }
                    }
                    // } else |err|{
                    //     std.log.debug("Err: {s}", .{ @errorName(err) });
                    //     // usb_cdc_write(&drivers.serial, "measure failed: {s}\r\n", .{@errorName(err)});
                    // }
                } else |err|{
                    std.log.debug("Err: {s}", .{ @errorName(err) });
                    // usb_cdc_write(&drivers.serial, "BME280 not found\r\n", .{});
                }
            }
        }
    }
}

var usb_tx_buff: [256]u8 = undefined;

/// Transfer data to host. After each chunk we poll so the bus TX events get handled.
pub fn usb_cdc_write(serial: *USB_Serial, comptime fmt: []const u8, args: anytype) void {
    var tx = std.fmt.bufPrint(&usb_tx_buff, fmt, args) catch &.{};

    var stalls: u8 = 8;
    while (tx.len > 0 and stalls < 16) {
        const n = serial.write(tx);
        if (n == 0) stalls += 1 else stalls = 0; // count only no-progress polls
        tx = tx[n..];
        usb_device.poll(&usb_controller);
    }

    stalls = 0;
    while (!serial.flush() and stalls < 16) : (stalls += 1)
        usb_device.poll(&usb_controller);
}
