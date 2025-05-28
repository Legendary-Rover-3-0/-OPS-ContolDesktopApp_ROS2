import serial.tools.list_ports

def match(desc):
    if desc == "CP2102 USB to UART Bridge Controller - CP2102 USB to UART Bridge Controller":
        return "GiZ"
    elif desc == "CP2102N USB to UART Bridge Controller":
        return "GPS"
    elif desc == "USB-Serial Controller":
        return "SATEL"
    elif desc == "USB Serial":
        return "Science"
    else:
        return ""

ports = serial.tools.list_ports.comports()
for port in ports:
    print(f"{port.device} {match(port.description)}")
