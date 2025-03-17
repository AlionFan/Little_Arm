# CAN Server for Raspberry Pi

This is the server component of the CAN communication system, designed to run on a Raspberry Pi with Ubuntu 24.04.

## Prerequisites

- Raspberry Pi with Ubuntu 24.04
- SLCAN USB device
- can-utils package

## Directory Structure

```
Server/
├── src/
│   └── can_server.cpp    # Main server program
├── scripts/
│   ├── setup-slcan.sh    # SLCAN setup script
│   └── slcan-setup.service # Systemd service file
└── README.md
```

## Installation

1. Install required packages:
```bash
sudo apt-get update
sudo apt-get install can-utils build-essential
```

2. Copy the SLCAN setup script:
```bash
sudo cp scripts/setup-slcan.sh /usr/local/bin/
sudo chmod +x /usr/local/bin/setup-slcan.sh
```

3. Install the systemd service:
```bash
sudo cp scripts/slcan-setup.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable slcan-setup
sudo systemctl start slcan-setup
```

4. Compile the server program:
```bash
g++ -o can_server src/can_server.cpp -pthread -std=c++11
```

## Usage

1. Start the server:
```bash
./can_server
```

The server will listen on port 5000 by default.

## Protocol Description

The server accepts the following commands:

1. Send CAN message:
   - Format: `SEND:<can_id>#<data>`
   - Example: `SEND:01#F6010ADC0A006B`

2. Monitor CAN bus:
   - Command: `MONITOR`
   - Returns: Latest CAN message

## Troubleshooting

1. Check SLCAN device status:
```bash
ip link show slcan0
```

2. Check service status:
```bash
sudo systemctl status slcan-setup
```

3. Monitor CAN messages manually:
```bash
candump slcan0
```

4. Send test message manually:
```bash
cansend slcan0 01#F6010ADC0A006B
```

## Motor Control Protocol

The motor control protocol follows this format:
- Command format: Address + 0xF6 + Direction + Speed + Acceleration + Sync Flag + Checksum
- Example: `01 F6 01 05 DC 0A 00 6B`

Where:
- Direction: 01 (CCW) or 00 (CW)
- Speed: 2 bytes (e.g., 05 DC = 1500 RPM)
- Acceleration: 1 byte (0-255, 0 = no acceleration curve)
- Sync Flag: 00 (disabled) or 01 (enabled)
- Checksum: Calculated value

## Security Considerations

1. The server accepts connections from any IP address
2. No authentication is implemented
3. Commands are sent in plain text
4. Consider implementing SSL/TLS for production use

## Maintenance

1. Check system logs:
```bash
journalctl -u slcan-setup
```

2. Monitor server process:
```bash
ps aux | grep can_server
```

## Support

For issues or questions, please check:
1. System logs
2. CAN device connection
3. Network connectivity
4. Firewall settings 