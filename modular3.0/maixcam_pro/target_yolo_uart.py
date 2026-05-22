import os

from maix import app, camera, display, err, image, nn, pinmap, time, uart


UART_DEVICE = "/dev/ttyS1"
UART_BAUDRATE = 115200

CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
AIM_X = 160
AIM_Y = 120

MODEL_FILENAME = "target_yolo.mud"
MODEL_TYPE = "AUTO"  # AUTO, YOLO11, YOLOv8, YOLOv5, YOLO26
TARGET_CLASS_IDS = ()  # Empty means accept every class.
CONF_THRESHOLD = 0.15
IOU_THRESHOLD = 0.45
MIN_TARGET_AREA = 16

LOOP_SLEEP_MS = 0
WARMUP_FRAMES = 5
ERROR_RETRY_MS = 1000
DISPLAY_INTERVAL_MS = 200
PRINT_INTERVAL_MS = 1000
DISPLAY_EVERY_N_FRAMES = 5
PRINT_EVERY_N_FRAMES = 20
TARGET_HOLD_MS = 200
TARGET_HOLD_FRAMES = 10
SAVE_MISS_IMAGES = True
MISS_SAVE_STREAK = 5
MISS_SAVE_INTERVAL_MS = 500
MISS_SAVE_EVERY_N_FRAMES = 28
MISS_SAVE_LIMIT = 40
MISS_IMAGE_DIR = "/root/miss_debug"
DEBUG_STATE_TRANSITIONS = False
SHOW_DISPLAY = True

STATE_INIT = "INIT"
STATE_WARMUP = "WARMUP"
STATE_DETECT = "DETECT"
STATE_SEND_TARGET = "SEND_TARGET"
STATE_SEND_EMPTY = "SEND_EMPTY"
STATE_ERROR = "ERROR"

g_serial = None


def checksum(payload):
    value = 0
    for ch in payload:
        value ^= ord(ch)
    return value


def build_frame(seq, valid, x, y, dx, dy, area):
    payload = "V,{},{},{},{},{},{},{}".format(seq, valid, x, y, dx, dy, area)
    return "${}*{:02X}\r\n".format(payload, checksum(payload))


def clamp_int(value, min_value, max_value):
    try:
        number = int(value)
    except Exception:
        number = 0

    if number < min_value:
        return min_value
    if number > max_value:
        return max_value
    return number


def now_ms():
    try:
        return int(time.ticks_ms())
    except Exception:
        try:
            return int(time.ticks_us() // 1000)
        except Exception:
            try:
                return int(time.time() * 1000)
            except Exception:
                return 0


def sleep_loop():
    if LOOP_SLEEP_MS > 0:
        time.sleep_ms(LOOP_SLEEP_MS)


def init_uart1():
    err.check_raise(pinmap.set_pin_function("A19", "UART1_TX"), "set A19 UART1_TX failed")
    err.check_raise(pinmap.set_pin_function("A18", "UART1_RX"), "set A18 UART1_RX failed")
    return uart.UART(UART_DEVICE, UART_BAUDRATE)


def file_exists(path):
    try:
        return os.path.exists(path)
    except Exception:
        try:
            fp = open(path, "rb")
            fp.close()
            return True
        except Exception:
            return False


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


def script_dir():
    try:
        return os.path.dirname(os.path.abspath(__file__))
    except Exception:
        return "."


def find_model_path():
    base_dir = script_dir()
    candidates = [
        os.path.join(base_dir, MODEL_FILENAME),
        MODEL_FILENAME,
        "/root/maixcam_pro/{}".format(MODEL_FILENAME),
        "/root/models/{}".format(MODEL_FILENAME),
    ]

    for path in candidates:
        if file_exists(path):
            return path

    raise RuntimeError("model not found: put {} in maixcam_pro folder".format(MODEL_FILENAME))


def detector_candidates():
    preferred = ["YOLO11", "YOLOv8", "YOLOv5", "YOLO26"]
    if MODEL_TYPE != "AUTO":
        preferred = [MODEL_TYPE]

    candidates = []
    for name in preferred:
        cls = getattr(nn, name, None)
        if cls is not None:
            candidates.append((name, cls))

    if len(candidates) == 0:
        raise RuntimeError("no supported detector class for MODEL_TYPE={}".format(MODEL_TYPE))

    return candidates


def init_detector(model_path):
    last_error = None
    for name, cls in detector_candidates():
        try:
            print("loading detector={} model={}".format(name, model_path))
            detector = cls(model=model_path, dual_buff=True)
            return detector, name
        except Exception as exc:
            last_error = exc
            print("detector {} failed: {}".format(name, exc))

    raise RuntimeError("load detector failed: {}".format(last_error))


def init_display():
    if not SHOW_DISPLAY:
        return None

    try:
        return display.Display()
    except Exception as exc:
        print("display disabled: {}".format(exc))
        return None


def init_context():
    global g_serial

    g_serial = init_uart1()
    model_path = find_model_path()
    detector, detector_name = init_detector(model_path)
    cam = camera.Camera(CAMERA_WIDTH, CAMERA_HEIGHT, detector.input_format())
    disp = init_display()

    print("MaixCAM Pro target YOLO UART")
    print("uart={} baud={}".format(UART_DEVICE, UART_BAUDRATE))
    print("camera={}x{} aim=({}, {})".format(CAMERA_WIDTH, CAMERA_HEIGHT, AIM_X, AIM_Y))
    print("detector={} input={}x{}".format(detector_name, detector.input_width(), detector.input_height()))
    print("detect conf={:.2f} iou={:.2f} hold_ms={}".format(CONF_THRESHOLD, IOU_THRESHOLD, TARGET_HOLD_MS))
    print("miss images={} dir={} limit={}".format(SAVE_MISS_IMAGES, os.path.join(script_dir(), MISS_IMAGE_DIR), MISS_SAVE_LIMIT))
    print("protocol=$V,<seq>,<valid>,<x>,<y>,<dx>,<dy>,<area>*<cs>")

    return {
        "serial": g_serial,
        "detector": detector,
        "detector_name": detector_name,
        "camera": cam,
        "display": disp,
        "ok_frames": 0,
        "empty_frames": 0,
        "raw_ok_frames": 0,
        "raw_miss_frames": 0,
        "raw_objects": 0,
        "candidate_objects": 0,
        "raw_object_frames": 0,
        "candidate_frames": 0,
        "held_frames": 0,
        "miss_streak": 0,
        "max_miss_streak": 0,
        "frame_count": 0,
        "fps": 0.0,
        "fps_window_frames": 0,
        "fps_window_ms": now_ms(),
        "last_display_ms": 0,
        "last_print_ms": 0,
        "last_valid_ms": 0,
        "last_valid_frame": 0,
        "last_valid_target": None,
        "last_miss_save_ms": 0,
        "last_miss_save_frame": 0,
        "miss_save_count": 0,
        "miss_save_errors": 0,
        "miss_image_dir": os.path.join(script_dir(), MISS_IMAGE_DIR),
    }


def enter_state(current, next_state, reason=""):
    if current != next_state and (DEBUG_STATE_TRANSITIONS or reason):
        if reason:
            print("state {} -> {}: {}".format(current, next_state, reason))
        else:
            print("state {} -> {}".format(current, next_state))
    return next_state


def allowed_class(obj):
    if len(TARGET_CLASS_IDS) == 0:
        return True
    return getattr(obj, "class_id", -1) in TARGET_CLASS_IDS


def obj_area(obj):
    width = max(0, int(getattr(obj, "w", 0)))
    height = max(0, int(getattr(obj, "h", 0)))
    return width * height


def select_target(objects):
    best = None
    best_score = -1.0
    raw_count = 0
    candidate_count = 0

    for obj in objects:
        raw_count += 1
        if not allowed_class(obj):
            continue

        area = obj_area(obj)
        if area < MIN_TARGET_AREA:
            continue

        candidate_count += 1
        score = float(getattr(obj, "score", 0.0))
        rank = (score * 100000.0) + float(area)
        if rank > best_score:
            best_score = rank
            best = obj

    if best is None:
        return None, raw_count, candidate_count

    x = clamp_int(int(getattr(best, "x", 0)) + (int(getattr(best, "w", 0)) // 2), -32768, 32767)
    y = clamp_int(int(getattr(best, "y", 0)) + (int(getattr(best, "h", 0)) // 2), -32768, 32767)
    dx = clamp_int(x - AIM_X, -32768, 32767)
    dy = clamp_int(y - AIM_Y, -32768, 32767)
    area = clamp_int(obj_area(best), 0, 65535)
    score = float(getattr(best, "score", 0.0))

    return (
        {
            "valid": 1,
            "x": x,
            "y": y,
            "dx": dx,
            "dy": dy,
            "area": area,
            "score": score,
            "class_id": int(getattr(best, "class_id", -1)),
            "held": 0,
            "age_ms": 0,
            "obj": best,
        },
        raw_count,
        candidate_count,
    )


def empty_target():
    return {
        "valid": 0,
        "x": 0,
        "y": 0,
        "dx": 0,
        "dy": 0,
        "area": 0,
        "score": 0.0,
        "class_id": -1,
        "held": 0,
        "age_ms": 0,
        "obj": None,
    }


def clone_held_target(target, age_ms):
    held = {
        "valid": 1,
        "x": target["x"],
        "y": target["y"],
        "dx": target["dx"],
        "dy": target["dy"],
        "area": target["area"],
        "score": target["score"],
        "class_id": target["class_id"],
        "held": 1,
        "age_ms": age_ms,
        "obj": None,
    }
    return held


def update_detection_stats(ctx, detected_target):
    if detected_target is not None:
        ctx["raw_ok_frames"] += 1
        ctx["miss_streak"] = 0
        ctx["last_valid_ms"] = now_ms()
        ctx["last_valid_frame"] = ctx.get("frame_count", 0)
        ctx["last_valid_target"] = detected_target
        return

    ctx["raw_miss_frames"] += 1
    ctx["miss_streak"] += 1
    if ctx["miss_streak"] > ctx["max_miss_streak"]:
        ctx["max_miss_streak"] = ctx["miss_streak"]


def target_age_ms(ctx):
    last_ms = ctx.get("last_valid_ms", 0)
    now = now_ms()
    if last_ms > 0 and now > 0:
        return max(0, now - last_ms)

    last_frame = ctx.get("last_valid_frame", 0)
    return max(0, ctx.get("frame_count", 0) - last_frame) * 20


def output_target_with_hold(ctx, detected_target):
    update_detection_stats(ctx, detected_target)
    if detected_target is not None:
        return detected_target

    last_target = ctx.get("last_valid_target")
    if last_target is None:
        return empty_target()

    age = target_age_ms(ctx)
    age_frames = max(0, ctx.get("frame_count", 0) - ctx.get("last_valid_frame", 0))
    has_time = ctx.get("last_valid_ms", 0) > 0 and now_ms() > 0
    if (has_time and age <= TARGET_HOLD_MS) or ((not has_time) and age_frames <= TARGET_HOLD_FRAMES):
        ctx["held_frames"] += 1
        return clone_held_target(last_target, age)

    return empty_target()


def send_target(serial_dev, seq, target):
    frame = build_frame(
        seq,
        target["valid"],
        target["x"],
        target["y"],
        target["dx"],
        target["dy"],
        target["area"],
    )
    serial_dev.write_str(frame)
    return (seq + 1) & 0xFFFF


def send_empty_heartbeat(seq):
    if g_serial is None:
        return seq

    try:
        return send_target(g_serial, seq, empty_target())
    except Exception as exc:
        print("error heartbeat failed: {}".format(exc))
        return seq


def update_frame_stats(ctx):
    now = now_ms()
    ctx["frame_count"] += 1
    ctx["fps_window_frames"] += 1

    start = ctx.get("fps_window_ms", 0)
    if now > 0 and start > 0:
        elapsed = now - start
        if elapsed >= 1000:
            ctx["fps"] = (ctx["fps_window_frames"] * 1000.0) / float(elapsed)
            ctx["fps_window_frames"] = 0
            ctx["fps_window_ms"] = now


def interval_due(ctx, key, interval_ms):
    now = now_ms()
    if now <= 0:
        frame_key = key + "_frame"
        interval_frames = PRINT_EVERY_N_FRAMES
        if key == "last_display_ms":
            interval_frames = DISPLAY_EVERY_N_FRAMES

        current = ctx.get("frame_count", 0)
        last = ctx.get(frame_key, -interval_frames)
        if (current - last) >= interval_frames:
            ctx[frame_key] = current
            return True
        return False

    last = ctx.get(key, 0)
    if last == 0 or (now - last) >= interval_ms:
        ctx[key] = now
        return True
    return False


def valid_ratio(ctx):
    total = ctx["raw_ok_frames"] + ctx["raw_miss_frames"]
    if total <= 0:
        return 0.0
    return (ctx["raw_ok_frames"] * 100.0) / float(total)


def frame_ratio(part, total):
    if total <= 0:
        return 0.0
    return (part * 100.0) / float(total)


def draw_overlay(ctx, img, state, target):
    disp = ctx.get("display")
    if disp is None:
        return
    if not interval_due(ctx, "last_display_ms", DISPLAY_INTERVAL_MS):
        return

    try:
        img.draw_rect(AIM_X - 3, AIM_Y - 3, 6, 6, color=image.COLOR_GREEN)
        img.draw_string(
            2,
            2,
            "{} fps={:.1f} raw={:.0f}% obj={}/{} ms={} max={}".format(
                state,
                ctx["fps"],
                valid_ratio(ctx),
                ctx["raw_objects"],
                ctx["candidate_objects"],
                ctx["miss_streak"],
                ctx["max_miss_streak"],
            ),
            color=image.COLOR_WHITE,
        )
        img.draw_string(
            2,
            18,
            "v={} h={} age={} dx={} dy={} s={:.2f}".format(
                target["valid"],
                target["held"],
                target_age_ms(ctx),
                target["dx"],
                target["dy"],
                target["score"],
            ),
            color=image.COLOR_WHITE,
        )

        obj = target.get("obj")
        if obj is not None:
            img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=image.COLOR_RED)
            img.draw_rect(target["x"] - 2, target["y"] - 2, 4, 4, color=image.COLOR_RED)
            img.draw_string(
                obj.x,
                obj.y,
                "dx={} dy={} a={}".format(target["dx"], target["dy"], target["area"]),
                color=image.COLOR_RED,
            )

        disp.show(img)
    except Exception as exc:
        print("display draw failed: {}".format(exc))


def print_summary(ctx, seq, state, target):
    if not interval_due(ctx, "last_print_ms", PRINT_INTERVAL_MS):
        return

    print(
        "fps={:.1f} state={} seq={} valid={} held={} age_ms={} x={} y={} dx={} dy={} area={} score={:.2f} ok={} miss={} raw_ok={} raw_miss={} raw_valid={:.1f}% raw_objects={} candidate_objects={} raw_obj_frames={:.1f}% candidate_frames={:.1f}% miss_streak={} max_miss={} held_frames={}".format(
            ctx["fps"],
            state,
            seq,
            target["valid"],
            target["held"],
            target_age_ms(ctx),
            target["x"],
            target["y"],
            target["dx"],
            target["dy"],
            target["area"],
            target["score"],
            ctx["ok_frames"],
            ctx["empty_frames"],
            ctx["raw_ok_frames"],
            ctx["raw_miss_frames"],
            valid_ratio(ctx),
            ctx["raw_objects"],
            ctx["candidate_objects"],
            frame_ratio(ctx["raw_object_frames"], ctx["frame_count"]),
            frame_ratio(ctx["candidate_frames"], ctx["frame_count"]),
            ctx["miss_streak"],
            ctx["max_miss_streak"],
            ctx["held_frames"],
        )
    )


def miss_save_due(ctx):
    if not SAVE_MISS_IMAGES:
        return False
    if ctx["miss_save_count"] >= MISS_SAVE_LIMIT:
        return False
    if ctx["miss_streak"] < MISS_SAVE_STREAK:
        return False

    now = now_ms()
    if now > 0:
        last = ctx.get("last_miss_save_ms", 0)
        if last == 0 or (now - last) >= MISS_SAVE_INTERVAL_MS:
            ctx["last_miss_save_ms"] = now
            return True
        return False

    current = ctx.get("frame_count", 0)
    last_frame = ctx.get("last_miss_save_frame", -MISS_SAVE_EVERY_N_FRAMES)
    if (current - last_frame) >= MISS_SAVE_EVERY_N_FRAMES:
        ctx["last_miss_save_frame"] = current
        return True
    return False


def maybe_save_miss_image(ctx, img, seq):
    if img is None or not miss_save_due(ctx):
        return
    if not ensure_dir(ctx["miss_image_dir"]):
        return

    filename = "miss_{:05d}_streak_{:04d}.jpg".format(seq, ctx["miss_streak"])
    path = os.path.join(ctx["miss_image_dir"], filename)
    try:
        img.save(path)
        ctx["miss_save_count"] += 1
        print("saved miss image:", path)
    except Exception as exc:
        ctx["miss_save_errors"] += 1
        print("save miss image failed {}: {}".format(path, exc))


def main():
    state = STATE_INIT
    ctx = None
    seq = 0
    warmup_left = WARMUP_FRAMES
    current_img = None
    current_target = empty_target()

    while not app.need_exit():
        try:
            if state == STATE_INIT:
                ctx = init_context()
                warmup_left = WARMUP_FRAMES
                state = enter_state(state, STATE_WARMUP)

            elif state == STATE_WARMUP:
                current_img = ctx["camera"].read()
                draw_overlay(ctx, current_img, state, empty_target())
                warmup_left -= 1
                if warmup_left <= 0:
                    state = enter_state(state, STATE_DETECT)
                sleep_loop()

            elif state == STATE_DETECT:
                current_img = ctx["camera"].read()
                objects = ctx["detector"].detect(current_img, conf_th=CONF_THRESHOLD, iou_th=IOU_THRESHOLD)
                if objects is None:
                    objects = []
                detected_target, raw_objects, candidate_objects = select_target(objects)
                ctx["raw_objects"] = raw_objects
                ctx["candidate_objects"] = candidate_objects
                if raw_objects > 0:
                    ctx["raw_object_frames"] += 1
                if candidate_objects > 0:
                    ctx["candidate_frames"] += 1
                current_target = output_target_with_hold(ctx, detected_target)
                if current_target is None:
                    current_target = empty_target()
                if detected_target is None:
                    maybe_save_miss_image(ctx, current_img, seq)
                if current_target["valid"] == 0:
                    state = enter_state(state, STATE_SEND_EMPTY)
                else:
                    state = enter_state(state, STATE_SEND_TARGET)

            elif state == STATE_SEND_TARGET:
                seq = send_target(ctx["serial"], seq, current_target)
                ctx["ok_frames"] += 1
                update_frame_stats(ctx)
                print_summary(ctx, seq, state, current_target)
                draw_overlay(ctx, current_img, state, current_target)
                state = enter_state(state, STATE_DETECT)
                sleep_loop()

            elif state == STATE_SEND_EMPTY:
                seq = send_target(ctx["serial"], seq, current_target)
                ctx["empty_frames"] += 1
                update_frame_stats(ctx)
                print_summary(ctx, seq, state, current_target)
                draw_overlay(ctx, current_img, state, current_target)
                state = enter_state(state, STATE_DETECT)
                sleep_loop()

        except Exception as exc:
            print("runtime error in {}: {}".format(state, exc))
            state = enter_state(state, STATE_ERROR)

        if state == STATE_ERROR:
            seq = send_empty_heartbeat(seq)
            time.sleep_ms(ERROR_RETRY_MS)
            state = enter_state(state, STATE_INIT, "retry")


if __name__ == "__main__":
    main()
