from maix import app, err, pinmap, time, uart


UART_DEVICE = "/dev/ttyS1"
UART_BAUDRATE = 115200
FRAME_INTERVAL_MS = 100
FRAME_CENTER_X = 160
FRAME_CENTER_Y = 120


def checksum(payload):
    value = 0
    for ch in payload:
        value ^= ord(ch)
    return value


def build_frame(seq, valid, x, y, dx, dy, area):
    payload = "V,{},{},{},{},{},{},{}".format(seq, valid, x, y, dx, dy, area)
    return "${}*{:02X}\r\n".format(payload, checksum(payload))


def init_uart1():
    err.check_raise(pinmap.set_pin_function("A19", "UART1_TX"), "set A19 UART1_TX failed")
    err.check_raise(pinmap.set_pin_function("A18", "UART1_RX"), "set A18 UART1_RX failed")
    return uart.UART(UART_DEVICE, UART_BAUDRATE)


def main():
    serial = init_uart1()
    seq = 0
    sweep = -80
    step = 8

    print("MaixCAM Pro STM32 UART smoke test")
    print("TX: A19/UART1_TX -> STM32 PD6/USART2_RX")
    print("port: {}, baud: {}".format(UART_DEVICE, UART_BAUDRATE))

    while not app.need_exit():
        dx = sweep
        dy = 0
        x = FRAME_CENTER_X + dx
        y = FRAME_CENTER_Y + dy
        area = 350
        frame = build_frame(seq, 1, x, y, dx, dy, area)
        serial.write_str(frame)
        print("sent:", frame.strip())

        seq = (seq + 1) & 0xFFFF
        sweep += step
        if sweep >= 80 or sweep <= -80:
            step = -step

        time.sleep_ms(FRAME_INTERVAL_MS)


if __name__ == "__main__":
    main()
