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
};

pub const BME280 = struct {
    const Self = @This();

    pub const Error = mdf.base.I2C_Device.InterfaceError || error{UnexpectedChipId};

    i2c: mdf.base.I2C_Device,
    address: mdf.base.I2C_Device.Address,
    clock: mdf.base.Clock_Device,

    /// Read `dst.len` bytes starting at `reg` (BME280 auto-increments on multi-byte reads).
    fn read_registers(self: Self, reg: Register, dst: []u8) mdf.base.I2C_Device.InterfaceError!void {
        try self.i2c.write_then_read(self.address, &.{reg.v()}, dst);
    }

    pub fn init(config: BME280_Config) Error!Self {
        const self = Self{
            .i2c = config.i2c,
            .address = config.address,
            .clock = config.clock,
        };

        var id_buf: [1]u8 = undefined;

        try self.read_registers(.id, &id_buf);
        if (id_buf[0] != chip_id) return error.UnexpectedChipId;

        return self;
    }
};

test "BME280 init verifies chip id" {
    const Test_I2C = mdf.base.I2C_Device.Test_Device;
    const Test_Clock = mdf.base.Clock_Device.Test_Device;

    // Canned read responses, consumed in order. One read → the chip id.
    const reads = [_][]const u8{&.{0x60}};
    var td = Test_I2C.init(&reads, true);
    defer td.deinit();
    var tc = Test_Clock.init();

    _ = try BME280.init(.{
        .i2c = td.i2c_device(),
        .address = @enumFromInt(0x76),
        .clock = tc.clock_device(),
    });

    // init must have written the id register address (0xD0) to read it back.
    try td.expect_sent(&.{&.{0xD0}});
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
