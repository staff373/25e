import os

from maix import app, camera, display, err, pinmap, time, uart


UART_DEVICE = "/dev/ttyS1"
UART_BAUDRATE = 115200

CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAPTURE_DIR = "/root/capture_dataset"
LINE_BUF_LIMIT = 96
LOOP_SLEEP_MS = 1
SHOW_DISPLAY = True
DISPLAY_EVERY_N_FRAMES = 2


def ensure_dir(path):
    try:
        if not os.path.exists(path):
            makedirs = getattr(os, "makedirs", None)
            if makedirs is not None:
                makedirs(path)
            else:
                os.mkdir(path)
        return True
    except Exception as exc:
        print("mkdir failed {}: {}".format(path, exc))
        return False


def is_capture_name(name):
    if not name.endswith(".jpg"):
        return False

    stem = name[:-4]
    if len(stem) == 0:
        return False

    for ch in stem:
        if ch < "0" or ch > "9":
            return False

    return True


def scan_next_index(path):
    max_index = 0

    if not ensure_dir(path):
        return 1

    try:
        for name in os.listdir(path):
            if is_capture_name(name):
                value = int(name[:-4])
                if value > max_index:
                    max_index = value
    except Exception as exc:
        print("scan dir failed {}: {}".format(path, exc))

    return max_index + 1


def init_uart1():
    err.check_raise(pinmap.set_pin_function("A19", "UART1_TX"), "set A19 UART1_TX failed")
    err.check_raise(pinmap.set_pin_function("A18", "UART1_RX"), "set A18 UART1_RX failed")
    return uart.UART(UART_DEVICE, UART_BAUDRATE)


def init_display():
    if not SHOW_DISPLAY:
        return None

    try:
        return display.Display()
    except Exception as exc:
        print("display disabled:", exc)
        return None


def send_line(serial_dev, text):
    line = "{}\n".format(text)
    serial_dev.write_str(line)
    print("tx:", text)


def read_uart_lines(serial_dev, state):
    try:
        data = serial_dev.read()
    except Exception as exc:
        print("uart read failed:", exc)
        return []

    if not data:
        return []

    lines = []
    for byte in data:
        value = byte
        if not isinstance(value, int):
            value = ord(value)

        if value == 10 or value == 13:
            if len(state["line"]) > 0:
                lines.append(state["line"])
                state["line"] = ""
            continue

        if value < 32 or value > 126:
            continue

        if len(state["line"]) < LINE_BUF_LIMIT:
            state["line"] += chr(value)
        else:
            state["line"] = ""

    return lines


def parse_cap_command(line):
    parts = line.strip().split()
    if len(parts) != 2 or parts[0] != "CAP":
        return None

    try:
        request_id = int(parts[1])
    except Exception:
        return None

    if request_id < 0:
        return None

    return request_id


def save_capture(ctx, request_id):
    img = ctx.get("latest_img")
    if img is None:
        img = ctx["camera"].read()

    if img is None:
        send_line(ctx["serial"], "CAP ERR {} no_frame".format(request_id))
        return

    filename = "{:04d}.jpg".format(ctx["next_index"])
    path = os.path.join(CAPTURE_DIR, filename)
    try:
        img.save(path)
        ctx["next_index"] += 1
        ctx["save_count"] += 1
        send_line(ctx["serial"], "CAP OK {} {}".format(request_id, filename))
        print("saved:", path)
    except Exception as exc:
        ctx["save_errors"] += 1
        print("save failed {}: {}".format(path, exc))
        send_line(ctx["serial"], "CAP ERR {} save_failed".format(request_id))


def process_line(ctx, line):
    print("rx:", line)
    request_id = parse_cap_command(line)
    if request_id is None:
        return

    save_capture(ctx, request_id)


def init_context():
    serial_dev = init_uart1()
    cam = camera.Camera(CAMERA_WIDTH, CAMERA_HEIGHT)
    disp = init_display()
    next_index = scan_next_index(CAPTURE_DIR)

    print("MaixCAM Pro capture UART")
    print("uart={} baud={}".format(UART_DEVICE, UART_BAUDRATE))
    print("camera={}x{}".format(CAMERA_WIDTH, CAMERA_HEIGHT))
    print("dir={} next={:04d}.jpg".format(CAPTURE_DIR, next_index))
    print("protocol: CAP <id> -> CAP OK <id> <file> / CAP ERR <id> <reason>")

    return {
        "serial": serial_dev,
        "camera": cam,
        "display": disp,
        "line": "",
        "latest_img": None,
        "next_index": next_index,
        "frame_count": 0,
        "save_count": 0,
        "save_errors": 0,
    }


def main():
    ctx = init_context()

    while not app.need_exit():
        try:
            img = ctx["camera"].read()
            if img is not None:
                ctx["latest_img"] = img
                ctx["frame_count"] += 1
                if (ctx["display"] is not None) and ((ctx["frame_count"] % DISPLAY_EVERY_N_FRAMES) == 0):
                    ctx["display"].show(img)

            for line in read_uart_lines(ctx["serial"], ctx):
                process_line(ctx, line)

            if LOOP_SLEEP_MS > 0:
                time.sleep_ms(LOOP_SLEEP_MS)
        except Exception as exc:
            print("capture loop error:", exc)
            time.sleep_ms(100)


if __name__ == "__main__":
    main()
