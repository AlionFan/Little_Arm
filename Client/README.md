# CAN Client Application

This is a Qt-based GUI client application for controlling and monitoring CAN devices through a Raspberry Pi server.

## Prerequisites

- CMake 3.16 or higher
- Qt 6.x (recommended) or Qt 5.x
- C++11 compatible compiler
- Network connection to Raspberry Pi server

## Directory Structure

```
Client/
├── src/
│   ├── main.cpp          # Application entry point
│   ├── mainwindow.h      # Main window header
│   ├── mainwindow.cpp    # Main window implementation
│   └── mainwindow.ui     # UI design file
├── CMakeLists.txt        # CMake build configuration
└── README.md
```

## Building the Application

### Using CMake Command Line

```bash
# Create and enter build directory
mkdir build
cd build

# Configure the project
cmake ..

# Build the project
cmake --build .
```

### Using Qt Creator with CMake

1. Open Qt Creator
2. Select "Open Project" and choose the `CMakeLists.txt` file
3. Configure the project for your Qt version
4. Build the project (Ctrl+B or Cmd+B)

### Building for Different Platforms

#### Linux
```bash
mkdir build && cd build
cmake ..
make
```

#### macOS
```bash
mkdir build && cd build
cmake ..
make
```

#### Windows
```bash
mkdir build && cd build
cmake ..
cmake --build .
```

## Features

1. Connect to Raspberry Pi CAN server
2. Send CAN messages to control motors
3. Monitor CAN bus in real-time
4. User-friendly interface for motor control

## Usage

1. Start the application
2. Enter the Raspberry Pi's IP address and port (default: 5000)
3. Click "Connect" to establish connection
4. Use the interface to:
   - Send motor control commands
   - Monitor CAN bus messages
   - View communication status

## Motor Control Protocol

The application supports the following motor control format:
- Command: Address + 0xF6 + Direction + Speed + Acceleration + Sync Flag + Checksum
- Example: `01 F6 01 05 DC 0A 00 6B`

Parameters:
- Direction: 01 (CCW) or 00 (CW)
- Speed: 2 bytes (e.g., 05 DC = 1500 RPM)
- Acceleration: 1 byte (0-255)
- Sync Flag: 00 (disabled) or 01 (enabled)

## Troubleshooting

1. CMake Build Issues:
   - Ensure CMake 3.16 or higher is installed
   - Verify Qt installation and environment variables
   - Check Qt version compatibility

2. Connection Issues:
   - Verify Raspberry Pi IP address
   - Check network connectivity
   - Ensure server is running
   - Check firewall settings

3. Command Issues:
   - Verify command format
   - Check motor status
   - Monitor server response

## Development

### Adding New Features

1. Modify UI in Qt Designer:
   - Open `mainwindow.ui`
   - Add/modify UI elements
   - Update corresponding slots in `mainwindow.h/cpp`

2. Adding New Commands:
   - Add command definition
   - Implement command handler
   - Update UI elements

## Contributing

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## Support

For issues or questions:
1. Check build configuration
2. Verify Qt and CMake versions
3. Check network connectivity
4. Review command format
1. Check network connectivity
2. Verify server status
3. Review command format
4. Check application logs 

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    connect(socket, QOverload<QAbstractSocket::SocketError>::of(&QAbstractSocket::error),
            this, &MainWindow::handleSocketError);
#else
    connect(socket, &QTcpSocket::errorOccurred, this, &MainWindow::handleSocketError);
#endif 