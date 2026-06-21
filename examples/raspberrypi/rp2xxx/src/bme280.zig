const std = @import("std");
const microzig = @import("microzig");
const rp2xxx = microzig.hal;

const I2C = rp2xxx.i2c.I2C;
const Address = rp2xxx.i2c.Address;
const sleep_ms = rp2xxx.time.sleep_ms;

pub const Measurement = struct {
    temperature_c: f64,
    pressure_pa: f64,
    humidity_rh: f64,
};

pub const BME280 = struct {
    i2c: I2C,
    addr: Address,

    // Factory calibration (read once in init).
    t1: u16, t2: i16, t3: i16,
    p1: u16, p2: i16, p3: i16, p4: i16, p5: i16, p6: i16, p7: i16, p8: i16, p9: i16,
    h1: u8, h2: i16, h3: u8, h4: i16, h5: i16, h6: i8,
    t_fine: f64 = 0,

    fn write_reg(self: BME280, reg: u8, val: u8) !void {
        try self.i2c.write_blocking(self.addr, &.{ reg, val }, null);
    }

    fn read_regs(self: BME280, reg: u8, dst: []u8) !void {
        try self.i2c.write_then_read_blocking(self.addr, &.{reg}, dst, null);
    }

    /// Reads calibration and prepares the sensor. Call once.
    pub fn init(i2c: I2C, addr: Address) !BME280 {
        var self: BME280 = undefined;
        self.i2c = i2c;
        self.addr = addr;
        self.t_fine = 0;

        // Calibration block 1: 0x88..0xA1 (26 bytes)
        var c1: [26]u8 = undefined;
        try self.read_regs(0x88, &c1);
        self.t1 = std.mem.readInt(u16, c1[0..2], .little);
        self.t2 = std.mem.readInt(i16, c1[2..4], .little);
        self.t3 = std.mem.readInt(i16, c1[4..6], .little);
        self.p1 = std.mem.readInt(u16, c1[6..8], .little);
        self.p2 = std.mem.readInt(i16, c1[8..10], .little);
        self.p3 = std.mem.readInt(i16, c1[10..12], .little);
        self.p4 = std.mem.readInt(i16, c1[12..14], .little);
        self.p5 = std.mem.readInt(i16, c1[14..16], .little);
        self.p6 = std.mem.readInt(i16, c1[16..18], .little);
        self.p7 = std.mem.readInt(i16, c1[18..20], .little);
        self.p8 = std.mem.readInt(i16, c1[20..22], .little);
        self.p9 = std.mem.readInt(i16, c1[22..24], .little);
        self.h1 = c1[25]; // 0xA1

        // Calibration block 1: 0xE1..0xE7 (7 bytes)
        var c2: [7]u8 = undefined;
        try self.read_regs(0xE1, &c2);
        self.h2 = std.mem.readInt(i16, c2[0..2], .little);
        self.h3 = c2[2];
        // h4/h5 are packed 12-bit signed values sharing byte 0xE5 (c2[5])
        self.h4 = (@as(i16, @as(i8, @bitCast(c2[3]))) * 16) | @as(i16, c2[4] & 0x0F);
        self.h5 = (@as(i16, @as(i8, @bitCast(c2[5]))) * 16) | @as(i16, c2[4] >> 4);
        self.h6 = @bitCast(c2[6]);

        return self;
    }

    // Trigger one forced-mode convension and return compensated values.
    pub fn measure(self: *BME280) !Measurement {
        // ctrl_num must be written before ctrl_meas to take effect.
        try self.write_reg(0xF2, 0x01); // humidity oversampling x1
        // ctrl_meas: osrs_t=x1(001), osrs_p=x1(001), mode=forced(01) => 0x25
        try self.write_reg(0xF4, 0x25);
        sleep_ms(10); // x1 oversampling completes in <10ms; raise for higher osrs

        var raw: [8]u8 = undefined; // 0xF7..0xFE
        try self.read_regs(0xF7, &raw);

        const adc_p: i32 = @intCast((@as(u32, raw[0]) << 12) | (@as(u32, raw[1]) << 4) | (raw[2] >> 4));
        const adc_t: i32 = @intCast((@as(u32, raw[3]) << 12) | (@as(u32, raw[4]) << 4) | (raw[5] >> 4));
        const adc_h: i32 = @intCast((@as(u32, raw[6]) << 8) | raw[7]);

        const temperature_c = self.compensate_temperature(adc_t); // sets t_fine
        return .{
            .temperature_c = temperature_c,
            .pressure_pa = self.compensate_pressure(adc_p),
            .humidity_rh = self.compensate_humidity(adc_h),
        };
    }

    fn f(v: anytype) f64 {
        return @floatFromInt(v);
    }

    fn compensate_temperature(self: *BME280, adc_t: i32) f64 {
        const t = f(adc_t);
        const var1 = (t / 16384.0 - f(self.t1) / 1024.0) * f(self.t2);
        const a = t / 131072.0 - f(self.t1) / 8192.0;
        const var2 = a * a * f(self.t3);
        self.t_fine = var1 + var2;
        return (var1 + var2) / 5120.0;
    }

    fn compensate_pressure(self: *BME280, adc_p: i32) f64 {
        var var1 = self.t_fine / 2.0 - 64000.0;
        var var2 = var1 * var1 * f(self.p6) / 32768.0;
        var2 = var2 + var1 * f(self.p5) * 2.0;
        var2 = var2 / 4.0 + f(self.p4) * 65536.0;
        var1 = (f(self.p3) * var1 * var1 / 524288.0 + f(self.p2) * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * f(self.p1);
        if (var1 == 0.0) return 0.0; // avoid div-by-zero
        var p = 1048576.0 - f(adc_p);
        p = (p - var2 / 4096.0) * 6250.0 / var1;
        var1 = f(self.p9) * p * p / 2147483648.0;
        var2 = p * f(self.p8) / 32768.0;
        return p + (var1 + var2 + f(self.p7)) / 16.0; // Pa
    }

    fn compensate_humidity(self: *BME280, adc_h: i32) f64 {
        var h = self.t_fine - 76800.0;
        h = (f(adc_h) - (f(self.h4) * 64.0 + f(self.h5) / 16384.0 * h)) *
            (f(self.h2) / 65536.0 * (1.0 + f(self.h6) / 67108864.0 * h * (1.0 + f(self.h3) / 67108864.0 * h)));
        h = h * (1.0 - f(self.h1) * h / 524288.0);
        return std.math.clamp(h, 0.0, 100.0); // %RH
    }

};
