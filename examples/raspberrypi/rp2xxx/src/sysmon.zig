const std = @import("std");
const microzig = @import("microzig");
const font8x8 = @import("font8x8");

const rp2xxx = microzig.hal;
const time = rp2xxx.time;
const usb = microzig.core.usb;

const Stats = struct {
    cpu: u32 = 0,
    mem: u32 = 0,
    temp: i32 = 0, // can be negative in theory :)
    net_down: u32 = 0,
    net_up: u32 = 0,
};

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
    .serial = "sysmon-cdc",
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

fn render(lcd: anytype, stats: Stats) void {
    if (lcd) |l| {
        var screen: [16 * 8]u8 = @splat(' '); // 16 cols * 8 rows of ASCII
        _ = std.fmt.bufPrint(screen[0..16],   "CPU   {d}%", .{stats.cpu}) catch {};
        _ = std.fmt.bufPrint(screen[32..48],  "MEM   {d}%", .{stats.mem}) catch {};
        _ = std.fmt.bufPrint(screen[64..80],  "TEMP  {d}C", .{stats.temp}) catch {};
        _ = std.fmt.bufPrint(screen[96..112], "NET   {d}KBs/{d}KBs", .{ stats.net_down, stats.net_up }) catch {};

        var glyphs: [screen.len * 8]u8 = undefined;
        _ = font8x8.Fonts.draw(&glyphs, &screen);
        l.write_full_display(&glyphs) catch {};
    }
}

fn parse_line(text: []const u8, stats: *Stats) void {
    var it = std.mem.tokenizeScalar(u8, text, ' '); // split on spaces
    while (it.next()) |tok| {
        const colon = std.mem.indexOfScalar(u8, tok, ':') orelse continue;
        const key = tok[0..colon];
        const val = tok[colon + 1 ..];

        if (std.mem.eql(u8, key, "C")) {
            stats.cpu = std.fmt.parseInt(u32, val, 10) catch stats.cpu;
        } else if (std.mem.eql(u8, key, "M")) {
            stats.mem = std.fmt.parseInt(u32, val, 10) catch stats.mem;
        } else if (std.mem.eql(u8, key, "T")) {
            stats.temp = std.fmt.parseInt(i32, val, 10) catch stats.temp;
        } else if (std.mem.eql(u8, key, "N")) {
            const slash = std.mem.indexOfScalar(u8, val, '/') orelse continue;
            stats.net_down = std.fmt.parseInt(u32, val[0..slash], 10) catch stats.net_down;
            stats.net_up = std.fmt.parseInt(u32, val[slash + 1 ..], 10) catch stats.net_up;
        }
        // unknow keys are ignored -> protocol cn grow later w/o breaking
    }
}

pub fn main() !void {
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

    const ssd1306 = microzig.drivers.display.ssd1306;
    const oled_dd = rp2xxx.drivers.I2C_Datagram_Device.init(
        i2c0, @enumFromInt(0x3C), null
    );
    const lcd = ssd1306.init(
        .i2c, oled_dd, null
    ) catch null; // ?Display — keeps USB alive if absent

    if (lcd) |l| {
        var screen: [16 * 8]u8 = @splat(' ');
        _ = std.fmt.bufPrint(screen[16..32], "    Hardware", .{}) catch {};
        _ = std.fmt.bufPrint(screen[48..64], "   Monitoring", .{}) catch {};
        var glyphs: [screen.len * 8]u8 = undefined;
        _ = font8x8.Fonts.draw(&glyphs, &screen); // fills all 1024 bytes of glyphs with font data
        l.write_full_display(&glyphs) catch {}; // sets addressing window + write full frame
    }
    // Initialize the USB device. The host will enumerate it as a serial port.
    usb_device = .init();

    // buffer that accumulates incomming bytes until we hit a new line.
    var line: [64]u8 = undefined;
    var line_len: usize = 0;
    var stats: Stats = .{};

    while (true) {
        // Must be polled frequently to service USB events. REQUIRED even if we
        // don't use CDC: picotool's `-f` reboot talks to the ResetDriver, which
        // only responds while this is being polled. poll() is non-blocking.
        usb_device.poll(&usb_controller);

        // drivers() is null until the host has finished enumerating us.
        if (usb_controller.drivers()) |drivers| {
            var chunk: [64]u8 = undefined;
            while (true) {
                const n = drivers.serial.read(&chunk);  // bytes the host sent us
                if (n == 0) break; // nothing more buffered

                for (chunk[0..n]) |b| {
                    if (b == '\n' or b == '\r') {
                        if (line_len > 0) {
                            parse_line(line[0..line_len], &stats);
                            render(lcd, stats);
                            line_len = 0;
                        }
                        line_len = 0;
                    } else if (line_len < line.len) {
                        line[line_len] = b;
                        line_len += 1;
                    } else {
                        line_len = 0; // overflow w/o newline -> drop and resync
                    }
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
