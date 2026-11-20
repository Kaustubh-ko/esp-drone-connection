"""
ESP-Drone bench tool (no phone app needed). PROPS OFF for every motor test!

1. Join the drone's Wi-Fi (ESP-DRONE_xxxx, password 12345678) from this PC.
2. Run from a normal terminal:

   python crtp_test.py --status
       Link check + read the settings that can block the motors
       (system.forceArm, stabilizer.stop, ...).
   python crtp_test.py --arm
       Set system.forceArm=1 and clear a latched emergency stop (stabilizer.stop=0).
   python crtp_test.py --motors
       Drive all four motors directly through the parameter system
       (motorPowerSet), bypassing the commander and PID controller.
   python crtp_test.py --arm --spin
       Send a normal thrust setpoint (what the phone app sends).

   Options:  --ip 192.168.43.42   --power 20000 (motorPowerSet value, 0-65535)
"""
import socket
import struct
import sys
import time

IP = "192.168.43.42"
PORT = 2390
POWER = 20000

# Parameter type bytes used by the firmware
T_INT8, T_UINT8, T_UINT16 = 0x00, 0x08, 0x09
TYPES = {0x00: ("<b", 1), 0x08: ("<B", 1), 0x01: ("<h", 2), 0x09: ("<H", 2),
         0x02: ("<i", 4), 0x0A: ("<I", 4), 0x06: ("<f", 4)}


def arg(name, default, cast=str):
    if name in sys.argv:
        return cast(sys.argv[sys.argv.index(name) + 1])
    return default


def cksum(data: bytes) -> int:
    return sum(data) & 0xFF


class Link:
    def __init__(self, ip, port):
        self.addr = (ip, port)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.settimeout(0.4)

    def send(self, pkt: bytes):
        self.s.sendto(pkt + bytes([cksum(pkt)]), self.addr)

    def request(self, pkt: bytes, match, tries=6):
        """Send pkt, return the first reply (without checksum) accepted by match()."""
        for _ in range(tries):
            self.send(pkt)
            end = time.time() + 0.6
            while time.time() < end:
                try:
                    data, _ = self.s.recvfrom(256)
                except socket.timeout:
                    break
                body = data[:-1]
                if len(body) >= 1 and match(body):
                    return body
        return None


def toc_info(link):
    r = link.request(bytes([0x2C, 0x01]), lambda b: (b[0] & 0xF0) == 0x20 and len(b) > 2 and b[1] == 0x01)
    return None if r is None else r[2]          # number of parameters


def read_toc(link, count):
    items = {}
    for i in range(count):
        r = link.request(bytes([0x2C, 0x00, i]),
                         lambda b, i=i: (b[0] & 0xF0) == 0x20 and (b[0] & 3) == 0
                         and len(b) > 4 and b[1] == 0x00 and b[2] == i)
        if r is None:
            continue
        ptype = r[3]
        group, name = r[4:].split(b"\0")[:2]
        items[(group.decode(), name.decode())] = (i, ptype)
    return items


def read_param(link, ident, ptype):
    fmt, size = TYPES.get(ptype & 0x3F, ("<B", 1))
    r = link.request(bytes([0x2D, ident]),
                     lambda b: (b[0] & 0xF0) == 0x20 and (b[0] & 3) == 1 and len(b) >= 2 and b[1] == ident)
    if r is None or len(r) < 2 + size:
        return None
    return struct.unpack(fmt, r[2:2 + size])[0]


ERR = {0: "OK", 2: "NOT FOUND (no such parameter)", 22: "WRONG TYPE", 13: "READ-ONLY"}


def write_param(link, group, name, ptype, value):
    fmt, _ = TYPES[ptype]
    pkt = bytes([0x2F, 0x00]) + group.encode() + b"\0" + name.encode() + b"\0" + bytes([ptype]) + struct.pack(fmt, value)
    r = link.request(pkt, lambda b: (b[0] & 0xF0) == 0x20 and (b[0] & 3) == 3 and len(b) > 3 and b[1] == 0x00)
    if r is None:
        print(f"  {group}.{name} = {value}: NO REPLY")
        return False
    code = r[-1]
    print(f"  {group}.{name} = {value}: {ERR.get(code, 'error ' + str(code))}")
    return code == 0


def main():
    ip = arg("--ip", IP)
    power = arg("--power", POWER, int)
    link = Link(ip, PORT)

    print(f"Asking {ip}:{PORT} for its parameter table...")
    count = toc_info(link)
    if count is None:
        print("NO REPLY: packets are not getting through (join the drone's Wi-Fi, check IP/firewall).")
        return
    print(f"Link OK: the drone answered ({count} parameters).")

    want = ("--status" in sys.argv or "--arm" in sys.argv or "--motors" in sys.argv)
    toc = read_toc(link, count) if want else {}
    if want and not toc:
        print("Could not read the parameter list.")
        return

    if "--status" in sys.argv:
        print("Current values:")
        for key in [("system", "selftestPassed"), ("system", "forceArm"), ("stabilizer", "stop"),
                    ("motorPowerSet", "enable"), ("controller", "tiltComp")]:
            if key in toc:
                v = read_param(link, *toc[key])
                print(f"  {key[0]}.{key[1]} = {v}")
            else:
                print(f"  {key[0]}.{key[1]}: not present in this firmware")
        print("(stabilizer.stop = 1 means an emergency stop is latched: motors stay off until cleared.)")

    if "--arm" in sys.argv:
        print("Arming:")
        write_param(link, "system", "forceArm", T_INT8, 1)
        write_param(link, "stabilizer", "stop", T_UINT8, 0)

    if "--motors" in sys.argv:
        print(f"Direct motor drive, power {power}/65535 for 2 s (PROPS OFF)...")
        ok = write_param(link, "motorPowerSet", "enable", T_UINT8, 1)
        for m in ("m1", "m2", "m3", "m4"):
            ok &= write_param(link, "motorPowerSet", m, T_UINT16, power)
        time.sleep(2.0)
        for m in ("m1", "m2", "m3", "m4"):
            write_param(link, "motorPowerSet", m, T_UINT16, 0)
        write_param(link, "motorPowerSet", "enable", T_UINT8, 0)
        print("Done. Did the motors spin?")

    if "--spin" in sys.argv:
        def setpoint(thrust):
            link.send(bytes([0x30]) + struct.pack("<fffH", 0.0, 0.0, 0.0, thrust))
        print("Setpoint test: thrust 0 (unlock) then 15000 for 2 s (PROPS OFF)...")
        t_end = time.time() + 0.6
        while time.time() < t_end:
            setpoint(0)
            time.sleep(0.02)
        t_end = time.time() + 2.0
        while time.time() < t_end:
            setpoint(15000)
            time.sleep(0.02)
        for _ in range(25):
            setpoint(0)
            time.sleep(0.02)
        print("Done.")

    if not want and "--spin" not in sys.argv:
        print("Next: --status, --arm, --motors or --spin (see the top of this file).")


if __name__ == "__main__":
    main()
