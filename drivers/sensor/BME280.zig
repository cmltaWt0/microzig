//! Generic driver for the Bosch BME280 humidity / pressure / temperature sensor.
//!
//! Datasheet (BST-BME280-DS002):
//!   https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
//! Compensation ports Bosch's fixed-point integer reference (§4.2.3 / §8).

const std = @import("std");
const mdf = @import("../framework.zig");

const chip_id: u8 = 0x60;

const Register = enum(u8) {
    id = 0xD0,
    reset = 0xE0,
    ctrl_hum = 0xF2,
    status = 0xF3,
    ctrl_meas = 0xF4,
    config = 0xF5,
    data = 0xF7,
    calib_tp = 0x88,
    calib_h = 0xE1,

    fn v(self: Register) u8 {
        return @intFromEnum(self);
    }
};

pub const BME280_Config = struct {
    i2c: mdf.base.I2C_Device,
    address: mdf.base.I2C_Device.Address,
    clock: mdf.base.Clock_Device,

    temperature_oversampling: Oversampling = .x1,
    pressure_oversampling: Oversampling = .x1,
    humidity_oversampling: Oversampling = .x1,

    filter: Filter = .off,
};

pub const BME280 = struct {
    const Self = @This();

    const CtrlMeas = packed struct(u8) {
        mode: Mode, // bits 0–1
        osrs_p: Oversampling, // bits 2–4
        osrs_t: Oversampling, // bits 5–7
    };

    const CtrlHum = packed struct(u8) {
        osrs_h: Oversampling, // bits 0–2  (3 bits)
        reserved: u5 = 0, // bits 3–7  (5 bits, always 0)
    };

    const ConfigReg = packed struct(u8) {
        spi3w_en: u1 = 0, // bit 0
        reserved: u1 = 0, // bit 1
        filter: Filter, // bits 2–4
        standby: u3 = 0, // bits 5–7 (raw for now; Stage 5 makes it an enum)
    };

    pub const Error = mdf.base.I2C_Device.InterfaceError || error{UnexpectedChipId};
    pub const Measurement = struct {
        temperature: i32, // °C × 100 (2534 → 25.34 °C)
        pressure: u32, // Pa, Q24.8   (÷ 256)
        humidity: u32, // %RH, Q22.10 (÷ 1024)

        pub fn temperature_celsius(self: Measurement) f32 {
            return @as(f32, @floatFromInt(self.temperature)) / 100.0;
        }
        pub fn pressure_pascal(self: Measurement) f32 {
            return @as(f32, @floatFromInt(self.pressure)) / 256.0;
        }
        pub fn humidity_percent(self: Measurement) f32 {
            return @as(f32, @floatFromInt(self.humidity)) / 1024.0;
        }
    };

    i2c: mdf.base.I2C_Device,
    address: mdf.base.I2C_Device.Address,
    clock: mdf.base.Clock_Device,
    cal: Calibration,

    temperature_oversampling: Oversampling,
    pressure_oversampling: Oversampling,
    humidity_oversampling: Oversampling,

    /// Read `dst.len` bytes starting at `reg` (BME280 auto-increments on multi-byte reads).
    fn read_registers(self: *const Self, reg: Register, dst: []u8) mdf.base.I2C_Device.InterfaceError!void {
        try self.i2c.write_then_read(self.address, &.{reg.v()}, dst);
    }

    fn write_register(self: *const Self, reg: Register, value: u8) mdf.base.I2C_Device.InterfaceError!void {
        try self.i2c.write(self.address, &.{ reg.v(), value });
    }

    fn read_calibration(self: *const Self) mdf.base.I2C_Device.InterfaceError!Calibration {
        var tp: [26]u8 = undefined;
        try self.read_registers(.calib_tp, &tp);
        var h: [7]u8 = undefined;
        try self.read_registers(.calib_h, &h);

        return .{
            .t1 = std.mem.readInt(u16, tp[0..2], .little),
            .t2 = std.mem.readInt(i16, tp[2..4], .little),
            .t3 = std.mem.readInt(i16, tp[4..6], .little),
            .p1 = std.mem.readInt(u16, tp[6..8], .little),
            .p2 = std.mem.readInt(i16, tp[8..10], .little),
            .p3 = std.mem.readInt(i16, tp[10..12], .little),
            .p4 = std.mem.readInt(i16, tp[12..14], .little),
            .p5 = std.mem.readInt(i16, tp[14..16], .little),
            .p6 = std.mem.readInt(i16, tp[16..18], .little),
            .p7 = std.mem.readInt(i16, tp[18..20], .little),
            .p8 = std.mem.readInt(i16, tp[20..22], .little),
            .p9 = std.mem.readInt(i16, tp[22..24], .little),
            .h1 = tp[25], // tp[24] is 0xA0 (reserved)
            .h2 = std.mem.readInt(i16, h[0..2], .little),
            .h3 = h[2],
            .h4 = (@as(i16, @as(i8, @bitCast(h[3]))) * 16) | @as(i16, h[4] & 0x0F),
            .h5 = (@as(i16, @as(i8, @bitCast(h[5]))) * 16) | @as(i16, h[4] >> 4),
            .h6 = @bitCast(h[6]),
        };
    }

    fn measurement_time_us(self: *const Self) u32 {
        var us: u32 = 1250; // 1.25 ms base
        us += 2300 * self.temperature_oversampling.factor();
        if (self.pressure_oversampling != .skip) us += 2300 * self.pressure_oversampling.factor() + 575;
        if (self.humidity_oversampling != .skip) us += 2300 * self.humidity_oversampling.factor() + 575;
        return us;
    }

    pub fn init(config: BME280_Config) Error!Self {
        var self = Self{
            .i2c = config.i2c,
            .address = config.address,
            .clock = config.clock,
            .cal = undefined, // filled below, before init returns or anything reads it
            .temperature_oversampling = config.temperature_oversampling,
            .pressure_oversampling = config.pressure_oversampling,
            .humidity_oversampling = config.humidity_oversampling,
        };

        var id_buf: [1]u8 = undefined;
        try self.read_registers(.id, &id_buf);
        if (id_buf[0] != chip_id) return error.UnexpectedChipId;

        self.cal = try self.read_calibration();
        // write config
        try self.write_register(.config, @bitCast(ConfigReg{ .filter = config.filter }));

        return self;
    }

    pub fn measure(self: *const Self) Error!Measurement {
        try self.write_register(.ctrl_hum, @bitCast(CtrlHum{ .osrs_h = self.humidity_oversampling }));
        try self.write_register(.ctrl_meas, @bitCast(CtrlMeas{
            .mode = .forced,
            .osrs_p = self.pressure_oversampling,
            .osrs_t = self.temperature_oversampling,
        }));

        self.clock.sleep_us(self.measurement_time_us());

        var buf: [8]u8 = undefined;
        try self.read_registers(.data, &buf);

        // MSB-first ADC packings
        const adc_p: i32 = (@as(i32, buf[0]) << 12) | (@as(i32, buf[1]) << 4) | (@as(i32, buf[2]) >> 4);
        const adc_t: i32 = (@as(i32, buf[3]) << 12) | (@as(i32, buf[4]) << 4) | (@as(i32, buf[5]) >> 4);
        const adc_h: i32 = (@as(i32, buf[6]) << 8) | @as(i32, buf[7]);

        const t = compensate_temperature(self.cal, adc_t);
        return Measurement{
            .temperature = t.hundredths_c,
            .pressure = compensate_pressure(self.cal, t.t_fine, adc_p),
            .humidity = compensate_humidity(self.cal, t.t_fine, adc_h),
        };
    }
};

pub const Calibration = struct {
    t1: u16,
    t2: i16,
    t3: i16,
    p1: u16,
    p2: i16,
    p3: i16,
    p4: i16,
    p5: i16,
    p6: i16,
    p7: i16,
    p8: i16,
    p9: i16,
    h1: u8,
    h2: i16,
    h3: u8,
    h4: i16,
    h5: i16,
    h6: i8,
};

pub const Oversampling = enum(u3) {
    skip = 0,
    x1 = 1,
    x2 = 2,
    x4 = 3,
    x8 = 4,
    x16 = 5,

    fn factor(self: Oversampling) u32 { // the multiplier, for the conversion-time formula
        return switch (self) {
            .skip => 0,
            .x1 => 1,
            .x2 => 2,
            .x4 => 4,
            .x8 => 8,
            .x16 => 16,
        };
    }
};
pub const Filter = enum(u3) { off = 0, x2 = 1, x4 = 2, x8 = 3, x16 = 4 };
pub const Mode = enum(u2) { sleep = 0, forced = 1, normal = 3 }; // 2 is also "forced"; we use 1

const Temperature = struct { t_fine: i32, hundredths_c: i32 };

fn compensate_temperature(cal: Calibration, adc_t: i32) Temperature {
    const dig_t1: i32 = cal.t1;
    const dig_t2: i32 = cal.t2;
    const dig_t3: i32 = cal.t3;

    const var1 = (((adc_t >> 3) - (dig_t1 << 1)) * dig_t2) >> 11;
    const d = (adc_t >> 4) - dig_t1;
    const var2 = (((d * d) >> 12) * dig_t3) >> 14;
    const t_fine = var1 + var2;
    const hundredths_c = (t_fine * 5 + 128) >> 8;
    return .{ .t_fine = t_fine, .hundredths_c = hundredths_c };
}

fn compensate_pressure(cal: Calibration, t_fine: i32, adc_p: i32) u32 {
    // Widen everything to i64 once via typed consts (lossless coercion).
    // Bosch uses int64_t throughout - pressure needs the headroom.
    const tf: i64 = t_fine;
    const adc: i64 = adc_p;
    const dig_p1: i64 = cal.p1;
    const dig_p2: i64 = cal.p2;
    const dig_p3: i64 = cal.p3;
    const dig_p4: i64 = cal.p4;
    const dig_p5: i64 = cal.p5;
    const dig_p6: i64 = cal.p6;
    const dig_p7: i64 = cal.p7;
    const dig_p8: i64 = cal.p8;
    const dig_p9: i64 = cal.p9;

    var var1: i64 = tf - 128000;
    var var2: i64 = var1 * var1 * dig_p6;
    var2 = var2 + ((var1 * dig_p5) << 17);
    var2 = var2 + (dig_p4 << 35);
    var1 = ((var1 * var1 * dig_p3) >> 8) + ((var1 * dig_p2) << 12);
    var1 = (((@as(i64, 1) << 47) + var1) * dig_p1) >> 33;

    if (var1 == 0) return 0; // avoid divide-by-zero (var1 scales the divisor below)

    var p: i64 = 1048576 - adc;
    p = @divTrunc(((p << 31) - var2) * 3125, var1);
    var1 = (dig_p9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = (dig_p8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (dig_p7 << 4);

    return @intCast(p); // Q24.8 Pa (divide by 256.0 for Pa)
}

fn compensate_humidity(cal: Calibration, t_fine: i32, adc_h: i32) u32 {
    const dig_h1: i32 = cal.h1;
    const dig_h2: i32 = cal.h2;
    const dig_h3: i32 = cal.h3;
    const dig_h4: i32 = cal.h4;
    const dig_h5: i32 = cal.h5;
    const dig_h6: i32 = cal.h6;

    var v: i32 = t_fine - 76800;
    v = (((((adc_h << 14) - ((dig_h4) << 20) - ((dig_h5) * v)) + 16384) >> 15) *
        (((((((v * dig_h6) >> 10) * (((v * dig_h3) >> 11) + 32768)) >> 10) +
            2097152) * dig_h2 + 8192) >> 14));
    v = v - (((((v >> 15) * (v >> 15)) >> 7) * dig_h1) >> 4);
    v = std.math.clamp(v, 0, 419430400);

    return @intCast(v >> 12);
}

test "BME280 init verifies chip id" {
    const Test_I2C = mdf.base.I2C_Device.Test_Device;
    const Test_Clock = mdf.base.Clock_Device.Test_Device;

    // Canned read responses, consumed in order. One read → the chip id.
    const reads = [_][]const u8{ &.{0x60}, &[_]u8{0} ** 26, &[_]u8{0} ** 7 };
    var td = Test_I2C.init(&reads, true);
    defer td.deinit();
    var tc = Test_Clock.init();

    _ = try BME280.init(.{
        .i2c = td.i2c_device(),
        .address = @enumFromInt(0x76),
        .clock = tc.clock_device(),
    });

    // init must have written the id register address (0xD0) to read it back.
    try td.expect_sent(&.{
        &.{0xD0},
        &.{0x88},
        &.{0xE1},
        &.{ 0xF5, 0x00 },
    });
}

test "BME280 init rejects wrong chip id" {
    const Test_I2C = mdf.base.I2C_Device.Test_Device;
    const Test_Clock = mdf.base.Clock_Device.Test_Device;

    // Canned read responses, consumed in order.
    // One read → BMP280's 0x58, not a BME280.
    const reads = [_][]const u8{&.{0x58}};
    var td = Test_I2C.init(&reads, true);
    defer td.deinit();
    var tc = Test_Clock.init();

    // Unexpected chip id should return an error.
    try std.testing.expectError(
        error.UnexpectedChipId,
        BME280.init(.{
            .i2c = td.i2c_device(),
            .address = @enumFromInt(0x76),
            .clock = tc.clock_device(),
        }),
    );
}

test "BME280 fixed-point compensation matches f64 reference across the range" {
    const cal = test_calibration;

    const Vector = struct { adc_t: i32, adc_p: i32, adc_h: i32 };
    const vectors = [_]Vector{
        .{ .adc_t = 415000, .adc_p = 350000, .adc_h = 25975 }, // cool / low pressure / drier
        .{ .adc_t = 519888, .adc_p = 415148, .adc_h = 33000 }, // mid
        .{ .adc_t = 620000, .adc_p = 480000, .adc_h = 42000 }, // warm / high pressure / humid
    };

    for (vectors) |vec| {
        const t = compensate_temperature(cal, vec.adc_t);
        const p = compensate_pressure(cal, t.t_fine, vec.adc_p);
        const h = compensate_humidity(cal, t.t_fine, vec.adc_h);

        const rt = ref.temperature(cal, vec.adc_t); // independent f64 t_fine per vector
        try std.testing.expectApproxEqAbs(rt.celsius, ref.f(t.hundredths_c) / 100.0, 0.05);
        try std.testing.expectApproxEqAbs(ref.pressure_pa(cal, rt.t_fine, vec.adc_p), ref.f(p) / 256.0, 3.0);
        try std.testing.expectApproxEqAbs(ref.humidity_rh(cal, rt.t_fine, vec.adc_h), ref.f(h) / 1024.0, 0.2);
    }
}

test "BME280 humidity compensation clamps to 0..100 %RH" {
    const cal = test_calibration;
    const t_fine: i32 = 100000; // ~19.5 °C, a valid fine-temperature

    // Very dry: the uncompensated value goes negative. Without the lower clamp
    // this would make `v >> 12` negative and PANIC the `@intCast`. Expect exactly 0.
    try std.testing.expectEqual(@as(u32, 0), compensate_humidity(cal, t_fine, 0));

    // Saturated: a max raw reading must never report above 100 %RH.
    // 100 %RH in Q22.10 == 100 * 1024 == 102400.
    try std.testing.expect(compensate_humidity(cal, t_fine, 65535) <= 100 * 1024);
}

test "BME280 init parses calibration and measure() reads + compensates" {
    const Test_I2C = mdf.base.I2C_Device.Test_Device;
    const Test_Clock = mdf.base.Clock_Device.Test_Device;

    // test_calibration, encoded LE: T1,T2,T3, P1..P9, 0xA0 reserved, H1
    const calib_tp = [_]u8{
        0x45, 0x6F, 0x6F, 0x68, 0x32, 0x00,
        0x82, 0x8F, 0x75, 0xD6, 0xD0, 0x0B,
        0x3C, 0x1C, 0x66, 0xFF, 0xF9, 0xFF,
        0xAC, 0x26, 0x0A, 0xD8, 0xBD, 0x10,
        0x00, 0x4B,
    };
    // H2, H3, then H4/H5 packed (0xE4,0xE5,0xE6), H6 → H4=308, H5=0
    const calib_h = [_]u8{ 0x6A, 0x01, 0x00, 0x13, 0x04, 0x00, 0x1E };
    // data block 0xF7..0xFE → adc_p=0x523AC, adc_t=0x80000, adc_h=0x66CC
    const data = [_]u8{ 0x52, 0x3A, 0xC0, 0x80, 0x00, 0x00, 0x66, 0xCC };

    const reads = [_][]const u8{ &.{0x60}, &calib_tp, &calib_h, &data };
    var td = Test_I2C.init(&reads, true);
    defer td.deinit();
    var tc = Test_Clock.init();

    const dev = try BME280.init(.{
        .i2c = td.i2c_device(),
        .address = @enumFromInt(0x76),
        .clock = tc.clock_device(),
    });

    try std.testing.expectEqual(test_calibration, dev.cal); // parsing (incl. H4/H5) is correct

    const m = try dev.measure();
    const t = compensate_temperature(dev.cal, 0x80000);
    try std.testing.expectEqual(t.hundredths_c, m.temperature);
    try std.testing.expectEqual(compensate_pressure(dev.cal, t.t_fine, 0x523AC), m.pressure);
    try std.testing.expectEqual(compensate_humidity(dev.cal, t.t_fine, 0x66CC), m.humidity);

    try td.expect_sent(&.{
        &.{0xD0}, &.{0x88}, &.{0xE1}, // id, calib_tp, calib_h (init)
        &.{ 0xF5, 0x00 }, // config register (init): filter = off → 0x00   ← NEW
        &.{ 0xF2, 0x01 }, &.{ 0xF4, 0x25 }, // ctrl_hum, ctrl_meas (measure)
        &.{0xF7}, // data burst (measure)
    });
}

test "BME280 oversampling and filter settings produce the right register bytes" {
    const Test_I2C = mdf.base.I2C_Device.Test_Device;
    const Test_Clock = mdf.base.Clock_Device.Test_Device;

    // id + zeroed calib blocks + zeroed data — we only assert the writes here.
    const reads = [_][]const u8{ &.{0x60}, &[_]u8{0} ** 26, &[_]u8{0} ** 7, &[_]u8{0} ** 8 };
    var td = Test_I2C.init(&reads, true);
    defer td.deinit();
    var tc = Test_Clock.init();

    const dev = try BME280.init(.{
        .i2c = td.i2c_device(),
        .address = @enumFromInt(0x76),
        .clock = tc.clock_device(),
        .temperature_oversampling = .x16,
        .pressure_oversampling = .x4,
        .humidity_oversampling = .x2,
        .filter = .x16,
    });
    _ = try dev.measure();

    try td.expect_sent(&.{
        &.{0xD0}, &.{0x88}, &.{0xE1}, // init reads
        &.{ 0xF5, 0x10 }, // config:    filter ×16 → 4<<2          = 0x10
        &.{ 0xF2, 0x02 }, // ctrl_hum:  osrs_h ×2                   = 0x02
        &.{ 0xF4, 0xAD }, // ctrl_meas: osrs_t ×16<<5 | osrs_p ×4<<2 | forced
        &.{0xF7}, // data burst
    });
}

// Float reference implementation, used only to cross-check the integer math in tests.
const ref = struct {
    fn f(v: anytype) f64 {
        return @floatFromInt(v);
    }

    const Temp = struct { t_fine: f64, celsius: f64 };

    fn temperature(cal: Calibration, adc_t: i32) Temp {
        const t = f(adc_t);
        const var1 = (t / 16384.0 - f(cal.t1) / 1024.0) * f(cal.t2);
        const a = t / 131072.0 - f(cal.t1) / 8192.0;
        const var2 = a * a * f(cal.t3);
        const t_fine = var1 + var2;
        return .{ .t_fine = t_fine, .celsius = t_fine / 5120.0 };
    }

    fn pressure_pa(cal: Calibration, t_fine: f64, adc_p: i32) f64 {
        var var1 = t_fine / 2.0 - 64000.0;
        var var2 = var1 * var1 * f(cal.p6) / 32768.0;
        var2 = var2 + var1 * f(cal.p5) * 2.0;
        var2 = var2 / 4.0 + f(cal.p4) * 65536.0;
        var1 = (f(cal.p3) * var1 * var1 / 524288.0 + f(cal.p2) * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * f(cal.p1);
        if (var1 == 0.0) return 0.0; // avoid div-by-zero
        var p = 1048576.0 - f(adc_p);
        p = (p - var2 / 4096.0) * 6250.0 / var1;
        var1 = f(cal.p9) * p * p / 2147483648.0;
        var2 = p * f(cal.p8) / 32768.0;
        return p + (var1 + var2 + f(cal.p7)) / 16.0; // Pa
    }

    fn humidity_rh(cal: Calibration, t_fine: f64, adc_h: i32) f64 {
        var h = t_fine - 76800.0;
        h = (f(adc_h) - (f(cal.h4) * 64.0 + f(cal.h5) / 16384.0 * h)) *
            (f(cal.h2) / 65536.0 * (1.0 + f(cal.h6) / 67108864.0 * h * (1.0 + f(cal.h3) / 67108864.0 * h)));
        h = h * (1.0 - f(cal.h1) * h / 524288.0);
        return std.math.clamp(h, 0.0, 100.0); // %RH
    }
};

const test_calibration = Calibration{
    .t1 = 28485,
    .t2 = 26735,
    .t3 = 50,
    .p1 = 36738,
    .p2 = -10635,
    .p3 = 3024,
    .p4 = 7228,
    .p5 = -154,
    .p6 = -7,
    .p7 = 9900,
    .p8 = -10230,
    .p9 = 4285,
    .h1 = 75,
    .h2 = 362,
    .h3 = 0,
    .h4 = 308,
    .h5 = 0,
    .h6 = 30,
};
