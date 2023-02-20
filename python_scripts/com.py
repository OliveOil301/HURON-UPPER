import serial
import serial.tools.list_ports


def serial_com(direction):
    mega_id = "85138313134351709030"
    port = "not found"

    list_of_ports = serial.tools.list_ports.comports()

    for portInfo in list_of_ports:
        if portInfo.serial_number == mega_id:
            port = portInfo.device
            break
        # print(portInfo.serial_number)

    if port != "not found":
        ser_port = serial.Serial(port)
        ser_port.write(direction)
        ser_port.close()
    else:
        print("Make sure the right Mega is plugged in and re-run")
        exit(-1)


if __name__ == "__main__":
    serial_com(bytes('002184320564', 'utf-8'))
