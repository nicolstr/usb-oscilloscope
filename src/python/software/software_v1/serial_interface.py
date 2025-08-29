import serial

class SerialInterface:
    def __init__(self, port='COM3', baudrate=115200, timeout=2): # Timeout auf 0.1s gesetzt, da es in einer Polling-Schleife läuft
        self.ser = None
        self.is_connected = False # Eine klarere Flagge für den Verbindungsstatus
        self.timeout_ocurred = False

        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            self.is_connected = True
            print(f"Serielle Verbindung geöffnet: {port} @ {baudrate} Baud")
        except serial.SerialException as e:
            print(f"Fehler beim Öffnen der seriellen Verbindung: {e}")
            self.is_connected = False # Bei Fehler bleibt sie False

    def read_line(self):
        """
        Liest eine Zeile von der seriellen Schnittstelle.
        Gibt einen leeren String zurück, wenn keine Zeile innerhalb des Timeouts empfangen wird
        oder ein Fehler auftritt.
        """
        if not self.is_connected or not (self.ser and self.ser.is_open):
            return "" # Nicht verbunden oder Port nicht offen

        try:
            # readline() blockiert maximal 'timeout' Sekunden.
            # Gibt b'' zurück, wenn innerhalb des Timeouts keine Zeile empfangen wurde.
            # Gibt die Zeile (inkl. Newline) als Bytes zurück, wenn eine empfangen wurde.
            raw_line = self.ser.readline()
            if raw_line:
                try:
                    decoded_line = raw_line.decode('utf-8').strip()
                    return decoded_line
                except UnicodeDecodeError:
                    print(f"Warnung: Empfangene Daten konnten nicht als UTF-8 dekodiert werden: {raw_line}")
                    return "" # Bei Dekodierungsfehler leeren String zurückgeben
            else:
                self.timeout_ocurred = True
                return "" # Timeout abgelaufen, keine Zeile empfangen
        except serial.SerialException as e:
            print(f"Fehler beim Lesen von serieller Schnittstelle: {e}")
            self.is_connected = False # Verbindung als unterbrochen markieren
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close() # Versuch, den fehlerhaften Port zu schließen
                except serial.SerialException as close_e:
                    print(f"Fehler beim Schließen des fehlerhaften Ports: {close_e}")
            return "" # Bei seriellem Fehler leeren String zurückgeben
    
    def read(self, size):
        """
        Liest eine angegebene Anzahl von Bytes von der seriellen Schnittstelle.
        Blockiert maximal für die im Konstruktor definierte Timeout-Zeit.
        
        Args:
            size (int): Die Anzahl der Bytes, die gelesen werden sollen.
            
        Returns:
            bytes: Die gelesenen Bytes. Kann weniger als 'size' sein, wenn der Timeout abläuft
                   oder die Verbindung unterbrochen wird. Gibt b'' zurück, wenn nicht verbunden
                   oder ein Fehler auftritt.
        """
        if not self.is_connected or not (self.ser and self.ser.is_open):
            print("Serielle Verbindung nicht offen zum Lesen.")
            return b'' # Leere Bytes zurückgeben, wenn nicht verbunden

        try:
            # self.ser.read(size) blockiert, bis 'size' Bytes gelesen wurden
            # oder der Timeout abläuft.
            data = self.ser.read(size)
            return data
        except serial.SerialException as e:
            print(f"Fehler beim Lesen von {size} Bytes von serieller Schnittstelle: {e}")
            self.is_connected = False # Verbindung als unterbrochen markieren
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close() # Versuch, den fehlerhaften Port zu schließen
                except serial.SerialException as close_e:
                    print(f"Fehler beim Schließen des fehlerhaften Ports: {close_e}")
            return b'' # Leere Bytes zurückgeben bei Fehler

    def write(self, message):
        """
        Schreibt eine Nachricht auf die serielle Schnittstelle.
        """
        if not self.is_connected or not (self.ser and self.ser.is_open):
            print("Serielle Verbindung nicht offen zum Schreiben.")
            return

        try:
            self.ser.write((message + "\n").encode('utf-8')) # Explizit UTF-8 Kodierung
            # print(f"Gesendet: {message}") # Optional: für Debugging
        except serial.SerialException as e:
            print(f"Fehler beim Schreiben auf serielle Schnittstelle: {e}")
            self.is_connected = False # Verbindung als unterbrochen markieren
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close() # Versuch, den fehlerhaften Port zu schließen
                except serial.SerialException as close_e:
                    print(f"Fehler beim Schließen des fehlerhaften Ports: {close_e}")

    def close(self):
        """
        Schließt die serielle Verbindung.
        """
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                print("Serielle Verbindung geschlossen.")
            except serial.SerialException as e:
                print(f"Fehler beim Schließen der seriellen Verbindung: {e}")
        self.is_connected = False # Flagge zurücksetzen
        self.ser = None # Referenz löschen