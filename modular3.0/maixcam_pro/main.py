import app_config


def main():
    mode = getattr(app_config, "APP_MODE", "capture")

    if mode == "capture":
        import capture_uart

        capture_uart.main()
        return

    if mode == "yolo":
        import target_yolo_uart

        target_yolo_uart.main()
        return

    raise RuntimeError("unknown APP_MODE: {}".format(mode))


if __name__ == "__main__":
    main()
