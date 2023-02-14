import serial
import serial.tools.list_ports


def serial_com(direction):
    mega_id = "/dev/ttyACM0"
    port = "not found"

    list_of_ports = serial.tools.list_ports.comports()

    for portInfo in list_of_ports:
        if portInfo.device == mega_id:
            port = portInfo.device
            break
        # print(portInfo.device)

    if port != "not found":
        ser_port = serial.Serial(port)
        ser_port.write(direction)
        ser_port.close()
    else:
        print("Make sure the right ESP32 is plugged in and re-run")
        exit(-1)


if __name__ == "__main__":
    serial_com(bytes('002 184 320 ', 'utf-8'))
