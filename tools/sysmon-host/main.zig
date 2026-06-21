const std = @import("std");

fn read_file(path: []const u8, buf: []u8) ![]const u8 {
    var file = try std.fs.cwd().openFile(path, .{});
    defer file.close();
    const n = try file.readAll(buf);
    return buf[0..n];
}

const CpuSample = struct { idle: u64, total: u64 };

fn read_cpu(buf: []u8) !CpuSample {
    const data = try read_file("/proc/stat", buf);
    var it = std.mem.tokenizeAny(u8, data, " \n");
    _ = it.next(); // skip the "cpu" label
    var total: u64 = 0;
    var idle: u64 = 0;
    var i: usize = 0;
    while (i < 8) : (i += 1) {
        const tok = it.next() orelse break;
        const v = std.fmt.parseInt(u64, tok, 10) catch break;
        total += v;
        if (i == 3 or i == 4) idle += v; // 4th field is idle time
    }
    return .{ .idle = idle, .total = total };
}

fn parse_kb(line: []const u8) u64 {
    var it = std.mem.tokenizeAny(u8, line, " ");
    _ = it.next(); // label , e.g. "MemTotal:"
    return std.fmt.parseInt(u64, it.next() orelse return 0, 10) catch 0;
}

fn read_mem(buf: []u8) !u32 {
    const data = try read_file("/proc/meminfo", buf);
    var total: u64 = 0;
    var avail: u64 = 0;
    var lines = std.mem.tokenizeScalar(u8, data, '\n');
    while (lines.next()) |line| {
        if (std.mem.startsWith(u8, line, "MemTotal:")) total = parse_kb(line);
        if (std.mem.startsWith(u8, line, "MemAvailable:")) avail = parse_kb(line);
    }
    if (total == 0) return 0;
    return @intCast((total-avail) * 100 / total);
}

fn read_temp() i32 {
    var dir = std.fs.openDirAbsolute(
        "/sys/class/hwmon", .{ .iterate = true }
    ) catch return 0;
    defer dir.close();
    var fallback: i32 = 0;
    var it = dir.iterate();
    while (it.next() catch null) |entry| {
        var pbuf: [128]u8 = undefined;
        var fbuf: [64]u8 = undefined;

        const name_path = std.fmt.bufPrint(
            &pbuf,
            "/sys/class/hwmon/{s}/name",
            .{entry.name}
        ) catch continue;
        const name_raw = read_file(name_path, &fbuf) catch continue;
        const name = std.mem.trim(u8, name_raw, " \n");
        const is_cpu = std.mem.eql(u8, name, "coretemp") or std.mem.eql(u8, name, "k10temp");

        const temp_path = std.fmt.bufPrint(
            &pbuf,
            "/sys/class/hwmon/{s}/temp1_input",
            .{entry.name}
        ) catch continue;
        const temp_raw = read_file(temp_path, &fbuf) catch continue;
        const milli = std.fmt.parseInt(
            i32,
            std.mem.trim(u8, temp_raw, " \n"), 10
        ) catch continue;
        const c = @divTrunc(milli, 1000);

        if (is_cpu) return c;      // prefer the real CPU sensor
        if (fallback == 0) fallback = c;
    }
    return fallback;
}

const NetSample = struct { rx: u64, tx: u64 };
const NET_IFACE = "wlan0"; // the one interface to monitor\

fn read_net(buf: []u8) !NetSample {
    const data = try read_file("/proc/net/dev", buf);
    var rx: u64 = 0;
    var tx: u64 = 0;
    var lines = std.mem.tokenizeScalar(u8, data, '\n');
    while (lines.next()) |line| {
        const colon = std.mem.indexOfScalar(u8, line, ':') orelse continue; // skip header lines
        const iface = std.mem.trim(u8, line[0..colon], " ");
        if (!std.mem.eql(u8, iface, NET_IFACE)) continue; // ignire loopback
        var n = std.mem.tokenizeAny(u8, line[colon + 1 ..], " ");
        rx += std.fmt.parseInt(u64, n.next() orelse "0", 10) catch 0; // field 0 = rx bytes
        var i: usize = 0;
        while (i < 8) : (i += 1) {
            const t = n.next() orelse break;
            if (i == 7) tx += std.fmt.parseInt(u64, t, 10) catch 10; // field 8 = tx bytes
        }
    }
    return .{ .rx = rx, .tx = tx };
}

pub fn main() !void {
    var args = std.process.args();
    _ = args.next();
    const dev = args.next() orelse "/dev/ttyACM0";

    var port = try std.fs.cwd().openFile(dev, .{ .mode = .write_only });
    defer port.close();

    var buf: [8192]u8 = undefined;
    var prev_cpu = try read_cpu(&buf);
    var prev_net = try read_net(&buf);

    while (true) {
        std.Thread.sleep(std.time.ns_per_s); // 1 second tick

        const cpu = try read_cpu(&buf);
        const dt = cpu.total - prev_cpu.total;
        const di = cpu.idle - prev_cpu.idle;
        const cpu_pct: u32 = if (dt == 0) 0 else @intCast((dt - di) * 100 / dt);
        prev_cpu = cpu;

        const mem_pct = try read_mem(&buf);
        const temp = read_temp();

        const net = try read_net(&buf);
        const down: u32 = @intCast((net.rx -| prev_net.rx) / 1024); // KB/s
        const up: u32 = @intCast((net.tx -| prev_net.tx) / 1024);
        prev_net = net;

        var lbuf: [64]u8 = undefined;
        const line = try std.fmt.bufPrint(
            &lbuf,
            "C:{d} M:{d} T:{d} N:{d}/{d}\n",
            .{ cpu_pct, mem_pct, temp, down, up }
        );
        try port.writeAll(line);

        // std.debug.print(
        //     "CPU: {d}% MEM: {d}% TEMP: {d}C NET: {d}/{d}\n",
        //     .{ cpu_pct, mem_pct, temp, down, up }
        // );
    }
}
