"""rtrobot_sniff.py - listen to what RTrobot's ServoController program says to the servo board.

The board's own settings (its UART / RS485 baud, start positions, deviation, buzzer, auto run) and
Download / Read / Erase have no published commands. Their program can talk to the board over TCP
(meant for a WiFi module), so this sits in the middle: it takes their program's TCP connection,
passes every byte to the board's real serial port and every reply back, and writes both directions
down (hex and text) in rtrobot_sniff.log. Nothing of their program is taken apart: this only hears
what it sends, as the board does. Then Servo Studio can send the same.

    python scripts/rtrobot_sniff.py COM5            (the board's USB port; --baud 115200, --port 8080)
    then in ServoController: TCP, address 127.0.0.1, port 8080, Open; click the settings to learn
    (Setting -> Hardware -> UART Baud Rate -> Apply, Download, Read, Erase), one at a time.

Their manual says some functions need the USB connection itself; if a button stays grey over TCP,
the other way is a virtual COM pair (com0com) between their program and this relay.
Don't set the board's password: their program says that erases its flash.
"""

import argparse
import os
import socket
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
LOG = os.path.join(os.path.dirname(HERE), "rtrobot_sniff.log")


def show(data):
    text = "".join(chr(b) if 32 <= b < 127 else {13: "\\r", 10: "\\n"}.get(b, ".") for b in data)
    return f"{data.hex(' ')}    |{text}|"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("com", help="the servo board's serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200, help="115200 over the board's USB")
    ap.add_argument("--port", type=int, default=8080, help="the TCP port their program connects to")
    args = ap.parse_args()
    import serial
    ser = serial.Serial(args.com, args.baud, timeout=0.05)
    log = open(LOG, "a", encoding="utf-8")
    lock = threading.Lock()

    def write(direction, data):
        line = f"{time.strftime('%H:%M:%S')} {direction} {show(data)}"
        with lock:
            print(line, flush=True)
            log.write(line + "\n")
            log.flush()

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", args.port))
    srv.listen(1)
    print(f"Relay {args.com} @ {args.baud} <-> 127.0.0.1:{args.port}. In ServoController pick TCP, "
          f"127.0.0.1, {args.port}. Writing to {LOG}. Ctrl+C stops.", flush=True)
    while True:
        conn, _ = srv.accept()
        write("==", b"their program connected")
        alive = [True]

        def board_to_program():
            while alive[0]:
                data = ser.read(512)
                if data:
                    write("<-", data)
                    try:
                        conn.sendall(data)
                    except OSError:
                        alive[0] = False

        threading.Thread(target=board_to_program, daemon=True).start()
        try:
            while True:
                data = conn.recv(512)
                if not data:
                    break
                write("->", data)
                ser.write(data)
        except OSError:
            pass
        alive[0] = False
        conn.close()
        write("==", b"their program disconnected")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
