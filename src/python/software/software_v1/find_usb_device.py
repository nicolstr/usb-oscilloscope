import serial.tools.list_ports as list_ports

def find_my_device(description_contains="Serielles USB"):
    ports = list_ports.comports()
    for port in ports:
        print(f"Port: {port.device}, Beschreibung: {port.description}")
        if description_contains.lower() in port.description.lower():
            print(f"Gefunden: {port.device}")
            return port.device
    print("Kein passender COM-Port gefunden.")
    return None
