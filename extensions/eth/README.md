## **1. nano_skript.txt – Arduino Nano Controller**

Dieses Programm läuft auf einem **Arduino Nano** und dient als **Steuereinheit für ein Flatpack2-Netzteil** (PSU = Power Supply Unit). Der Nano ist das zentrale Steuergerät mit folgenden Hauptaufgaben:

### Kernfunktionen:
- **Lokale Bedienung**: Über einen Drehencoder (Rotary Encoder) und ein OLED-Display können Spannung und Strom direkt am Gerät eingestellt werden
- **I2C-Kommunikation mit PSU**: Der Nano kommuniziert über I2C mit dem Flatpack2-Netzteil, um:
  - Sollwerte (Spannung, Strom) zu setzen
  - Messwerte (aktuelle Spannung, Strom, Temperaturen) auszulesen
  - Das Netzteil ein-/auszuschalten
  - Statuswerte zu überwachen

- **UART-Kommunikation**: Der Nano fungiert als "Slave" und kann über die serielle Schnittstelle (38400 Baud) von einem übergeordneten System (WT32) ferngesteuert werden
- **Dual-Modus**: Das System kann sowohl **autonom** (nur mit Nano) als auch **ferngesteuert** (über WT32) betrieben werden. Das Display zeigt an, ob ein Master-Gerät (z.B. "Cerbo") verbunden ist

### Besondere Features:
- Speicherung der Einstellungen im EEPROM
- Heartbeat-Mechanismus zur Verbindungsüberwachung
- CRC-gesicherte Kommunikation
- Display-Timeout zur Schonung

---

## **2. wt32-eth-skript.txt – WT32-ETH01 Webserver**

Dieses Programm läuft auf einem **WT32-ETH01** (ESP32 mit Ethernet) und dient als **Netzwerk-Gateway und Webinterface** für das Flatpack2-System:

### Kernfunktionen:
- **Web-Interface**: Stellt ein modernes HTML-Dashboard bereit, über das das Netzteil per Browser gesteuert werden kann
- **UART-Master**: Kommuniziert über Serial2 mit dem Nano und sendet Befehle (Spannung/Strom setzen, Ein/Aus schalten)
- **Ethernet/WiFi-Anbindung**: Ermöglicht Fernzugriff auf das System (feste IP: 192.168.1.234)
- **OTA-Updates**: Das Programm kann über das Netzwerk aktualisiert werden (Over-The-Air)

### API-Endpunkte:
- `/values` – Liefert aktuelle Messwerte und Status als JSON
- `/set_voltage?voltage=XX` – Setzt die Sollspannung
- `/set_current?current=XX` – Setzt den Sollstrom
- `/cmd?c=on|off` – Schaltet das Netzteil ein/aus

### Statusüberwachung:
- Erkennt, ob der Nano verbunden ist (Timeout: 15 Sekunden)
- Zeigt Gesundheitsstatus an (OK / WARN / FAULT)
- Überwacht Temperaturen und Statusbits des Netzteils

---

## **Zusammenspiel der beiden Systeme:**

```
[Flatpack2 PSU] ←I2C→ [Arduino Nano] ←UART→ [WT32-ETH01] ←Ethernet/WiFi→ [Browser/Victron Cerbo]
                         ↓
                  [Drehencoder + OLED]
```

Der **Nano** ist die direkte Hardwareschnittstelle zum Netzteil und kann eigenständig arbeiten. Der **WT32** erweitert das System um Netzwerkfunktionen und ein komfortables Webinterface, bleibt aber optional – der Nano funktioniert auch ohne ihn.