# CLAUDE.md - AI Assistant Guide for ESP32-mcp2515

## Project Overview

**Project Name**: autowp-mcp2515
**Version**: 1.3.1
**License**: MIT License
**Repository**: https://github.com/autowp/arduino-mcp2515
**Maintainer**: autowp <autowp@gmail.com>

### Description
This is an Arduino library for interfacing with the MCP2515 CAN (Controller Area Network) controller chip via SPI. The library provides CAN-BUS capability to Arduino/ESP32 boards, implementing CAN V2.0B protocol at speeds up to 1 Mb/s. It's commonly used for automotive diagnostics (OBD-II), industrial automation, and embedded systems communication.

### Key Features
- CAN V2.0B protocol support at up to 1 Mb/s
- SPI interface up to 10 MHz
- Standard (11-bit) and extended (29-bit) CAN frames
- Remote frames and data frames
- Two receive buffers with prioritized message storage
- Configurable filters and masks for selective message reception
- Multiple operating modes (Normal, Listen-Only, Loopback, One-Shot)

---

## Repository Structure

```
ESP32-mcp2515/
├── mcp2515.h           # Main MCP2515 driver class header
├── mcp2515.cpp         # Main MCP2515 driver implementation
├── can.h               # CAN frame structures and constants (Linux SocketCAN compatible)
├── library.properties  # Arduino library metadata
├── keywords.txt        # Arduino IDE syntax highlighting keywords
├── README.md           # User documentation
├── LICENSE.md          # MIT License
├── .travis.yml         # Travis CI configuration
├── examples/           # Example Arduino sketches
│   ├── CAN_read/       # Basic CAN frame reception example
│   ├── CAN_write/      # Basic CAN frame transmission example
│   ├── CAN_SpeedTest/  # Performance testing example
│   ├── wiring.png      # MCP2515 shield wiring diagram
│   └── wiring-diy.png  # DIY MCP2515 wiring diagram
└── .settings/          # Eclipse project settings
```

### Core Files

#### `can.h` (45 lines)
- Defines Linux SocketCAN-compatible structures and constants
- Main structure: `struct can_frame` with `can_id`, `can_dlc`, and `data[8]`
- CAN ID flags: `CAN_EFF_FLAG`, `CAN_RTR_FLAG`, `CAN_ERR_FLAG`
- Frame format masks: `CAN_SFF_MASK` (11-bit), `CAN_EFF_MASK` (29-bit)

#### `mcp2515.h` (~503 lines)
- Main driver class definition
- Bitrate configuration tables for 8 MHz, 16 MHz, and 20 MHz oscillators
- Enumerations for speeds, clocks, modes, errors, registers, and instructions
- Public API for initialization, configuration, send/receive operations
- Private SPI communication methods and register manipulation

#### `mcp2515.cpp` (~670 lines estimated)
- Complete implementation of the MCP2515 driver
- SPI communication protocol implementation
- Register read/write operations
- Mode switching, bitrate configuration
- Frame transmission and reception logic
- Filter and mask configuration

---

## Architecture and Design Patterns

### Class Design

**Main Class**: `MCP2515`

**Constructor**:
```cpp
MCP2515(const uint8_t _CS, const uint32_t _SPI_CLOCK = 10000000, SPIClass * _SPI = nullptr)
```
- `_CS`: SPI chip select pin number
- `_SPI_CLOCK`: SPI clock speed (default 10 MHz)
- `_SPI`: Optional custom SPI interface (for multi-SPI boards)

### Operating Modes

The MCP2515 supports several operating modes:

1. **Normal Mode** (`setNormalMode()`): Full CAN operation with acknowledgments
2. **Normal One-Shot Mode** (`setNormalOneShotMode()`): No automatic retransmission
3. **Listen-Only Mode** (`setListenOnlyMode()`): Receive-only, no acknowledgments sent
4. **Loopback Mode** (`setLoopbackMode()`): Internal loopback for testing
5. **Sleep Mode** (`setSleepMode()`): Low-power mode
6. **Configuration Mode** (`setConfigMode()`): Required for changing settings

### CAN Frame Structure

The library uses Linux SocketCAN-compatible frame structure:

```cpp
struct can_frame {
    canid_t can_id;  // 32-bit: CAN ID (11/29 bits) + flags (3 bits MSB)
    __u8 can_dlc;    // Data Length Code (0-8 bytes)
    __u8 data[8];    // Payload data (aligned to 8 bytes)
};
```

**CAN ID Layout** (32 bits):
- Bit 0-28: CAN identifier (11 or 29 bits depending on frame type)
- Bit 29: Error frame flag (`CAN_ERR_FLAG`)
- Bit 30: Remote Transmission Request (`CAN_RTR_FLAG`)
- Bit 31: Extended Frame Format flag (`CAN_EFF_FLAG`)

### Error Handling

All major operations return `MCP2515::ERROR` enum:
- `ERROR_OK`: Success
- `ERROR_FAIL`: General failure
- `ERROR_ALLTXBUSY`: All transmit buffers busy
- `ERROR_FAILINIT`: Initialization failed
- `ERROR_FAILTX`: Transmission failed
- `ERROR_NOMSG`: No message available

### Hardware Abstraction

The library abstracts SPI communication through:
- `startSPI()`: Begin SPI transaction with chip select
- `endSPI()`: End SPI transaction
- `readRegister()`: Read single MCP2515 register
- `setRegister()`: Write single register
- `modifyRegister()`: Bit-modify register (read-modify-write)

---

## Key Conventions and Standards

### Coding Style

1. **Naming Conventions**:
   - Classes: PascalCase (e.g., `MCP2515`)
   - Enums: UPPER_CASE (e.g., `CAN_SPEED`, `CANINTF`)
   - Constants: UPPER_CASE with underscores (e.g., `CAN_MAX_DLC`)
   - Private members: camelCase (e.g., `readRegister`)
   - Public methods: camelCase (e.g., `setBitrate`, `sendMessage`)

2. **Register Definitions**:
   - Configuration registers use prefix `MCP_` (e.g., `MCP_CNF1`, `MCP_CANCTRL`)
   - Bitrate configs use pattern: `MCP_<CLOCK>_<SPEED>_CFG<N>` (e.g., `MCP_16MHz_125kBPS_CFG1`)

3. **Hardware Constants**:
   - Default SPI clock: 10 MHz
   - Supported oscillator frequencies: 8 MHz, 16 MHz (default), 20 MHz
   - Number of TX buffers: 3 (`TXB0`, `TXB1`, `TXB2`)
   - Number of RX buffers: 2 (`RXB0`, `RXB1`)
   - Number of filters: 6 (`RXF0` through `RXF5`)
   - Number of masks: 2 (`MASK0`, `MASK1`)

### Arduino Library Standards

The library follows Arduino Library Specification 1.5:
- `library.properties`: Contains metadata (name, version, author, etc.)
- `keywords.txt`: Defines syntax highlighting for Arduino IDE
- `examples/`: Contains `.ino` sketch files in subdirectories
- Main include: `#include <mcp2515.h>` (lowercase)

---

## Common Development Tasks

### 1. Adding New Bitrate Configurations

When adding support for a new bitrate or clock frequency:

**Location**: `mcp2515.h` lines 8-174

**Pattern**:
```cpp
#define MCP_<CLOCK>MHz_<SPEED>kBPS_CFG1 (0xXX)
#define MCP_<CLOCK>MHz_<SPEED>kBPS_CFG2 (0xXX)
#define MCP_<CLOCK>MHz_<SPEED>kBPS_CFG3 (0xXX)
```

**Implementation**: Update `setBitrate()` function in `mcp2515.cpp`

**Important**: These values are calculated based on MCP2515 datasheet timing parameters (BRP, PRSEG, PHSEG1, PHSEG2, SJW). Verify with oscilloscope or CAN analyzer.

### 2. Adding New Operating Modes

**Existing modes** (see `mcp2515.h:275-283`):
- `CANCTRL_REQOP_NORMAL`
- `CANCTRL_REQOP_LOOPBACK`
- `CANCTRL_REQOP_LISTENONLY`
- `CANCTRL_REQOP_CONFIG`
- `CANCTRL_REQOP_OSM` (One-Shot Mode)

**To add new mode**:
1. Add enum value in `CANCTRL_REQOP_MODE`
2. Create public method: `ERROR set<Mode>Mode()`
3. Implement using `setMode(CANCTRL_REQOP_<MODE>)`

### 3. Extending Filter/Mask Functionality

**Current implementation**:
- `setFilterMask(MASK num, bool ext, uint32_t ulData)`
- `setFilter(RXF num, bool ext, uint32_t ulData)`

**Key considerations**:
- Filters are applied during initialization in `reset()` (mcp2515.cpp:72-87)
- Default: Accept all frames (masks set to 0)
- `ext` parameter: `false` for standard (11-bit), `true` for extended (29-bit)
- Filter matching uses bitwise AND with mask

### 4. Modifying Examples

**Guidelines**:
- Keep examples simple and focused on one concept
- Use Serial at 115200 baud for consistency
- Include `while (!Serial);` for boards with USB-Serial
- Always call `reset()`, `setBitrate()`, and set mode in `setup()`
- Use descriptive CAN IDs (avoid 0x000 in production)

**Example template**:
```cpp
#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(10);  // CS pin

void setup() {
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();
}

void loop() {
  // Your code here
}
```

### 5. Testing Changes

**Manual Testing**:
- Use two Arduino boards with MCP2515 modules
- One as sender (CAN_write example), one as receiver (CAN_read example)
- Connect CAN_H, CAN_L, and GND between modules
- Add 120Ω termination resistors at both ends of CAN bus

**Travis CI**:
- Automatic build testing on commit/PR
- Tests compilation for multiple Arduino platforms
- Located in `.travis.yml`

---

## Important Implementation Details

### SPI Communication Protocol

The MCP2515 uses SPI Mode 0 (CPOL=0, CPHA=0):
- Data sampled on rising edge, shifted on falling edge
- MSB first
- Maximum SPI clock: 10 MHz

**Key SPI Instructions**:
- `INSTRUCTION_RESET` (0xC0): Software reset
- `INSTRUCTION_READ` (0x03): Read register
- `INSTRUCTION_WRITE` (0x02): Write register
- `INSTRUCTION_BITMOD` (0x05): Bit modify
- `INSTRUCTION_READ_STATUS` (0xA0): Quick status read
- `INSTRUCTION_RTS_TXn`: Request to send from buffer n

### Timing Considerations

1. **Reset delay**: 10ms after reset command (mcp2515.cpp:47)
2. **Mode switching**: Waits for mode change confirmation (polling CANSTAT)
3. **Message transmission**: Non-blocking; check status with `getInterrupts()`

### Memory Layout

**TX Buffers** (`N_TXBUFFERS = 3`):
- Each has control register, ID registers, DLC, and 8 data bytes
- Can be loaded and triggered independently

**RX Buffers** (`N_RXBUFFERS = 2`):
- RXB0 has higher priority than RXB1
- RXB0 can rollover to RXB1 if full (`RXB0CTRL_BUKT` bit)

### Interrupt Handling

The MCP2515 supports interrupt-driven reception:

**Interrupt flags** (see `mcp2515.h:246-255`):
- `CANINTF_RX0IF`: Message in RXB0
- `CANINTF_RX1IF`: Message in RXB1
- `CANINTF_TX0IF/1IF/2IF`: Transmission complete
- `CANINTF_ERRIF`: Error interrupt
- `CANINTF_WAKIF`: Wake-up interrupt
- `CANINTF_MERRF`: Message error interrupt

**Methods**:
- `getInterrupts()`: Read interrupt flags
- `clearInterrupts()`: Clear all interrupt flags
- `clearTXInterrupts()`: Clear TX interrupt flags only

---

## AI Assistant Guidelines

### When Modifying Code

1. **Preserve compatibility**: This library is used in production systems. Breaking changes require major version bump.

2. **Hardware-specific code**: Be extremely careful with register values and timing. Incorrect values can prevent communication.

3. **SPI thread safety**: The library is not thread-safe. Document if adding multi-threading support.

4. **Memory constraints**: Arduino Uno has only 2KB RAM. Avoid large buffers or dynamic allocation.

5. **Test on hardware**: Simulation is insufficient for CAN/SPI communication. Always recommend hardware testing.

### Common Questions

**Q: Why use SocketCAN structure?**
A: Compatibility with Linux CAN tools and easy porting between platforms.

**Q: Can I use multiple MCP2515 chips?**
A: Yes, use different CS pins and create separate MCP2515 objects.

**Q: Why 120Ω termination?**
A: CAN bus impedance is 120Ω. Terminators prevent signal reflections.

**Q: What's the difference between Normal and One-Shot mode?**
A: One-Shot mode doesn't retry failed transmissions (useful for time-critical data).

### Code Review Checklist

When reviewing changes:
- [ ] Verify register addresses against MCP2515 datasheet
- [ ] Check for proper error handling (return `ERROR` enum)
- [ ] Ensure SPI transactions are wrapped in `startSPI()`/`endSPI()`
- [ ] Validate timing calculations for bitrate configurations
- [ ] Confirm examples compile for Arduino Uno, ESP32, and other targets
- [ ] Check that `library.properties` version is updated if needed
- [ ] Verify keywords.txt updated for new public constants/methods
- [ ] Test with actual hardware if possible

### Documentation Standards

- Keep README.md as primary user documentation
- Use C++ style comments for implementation details
- Document register bit manipulation with references to datasheet sections
- Include wiring diagrams for hardware changes
- Update CHANGELOG.md for user-facing changes (create if doesn't exist)

---

## Debugging Tips

### Common Issues

1. **"ERROR_FAILINIT"**:
   - Check SPI wiring (MISO, MOSI, SCK, CS)
   - Verify oscillator frequency matches code (8/16/20 MHz)
   - Check power supply (MCP2515 requires stable 5V or 3.3V)

2. **"ERROR_ALLTXBUSY"**:
   - No acknowledgment from other CAN node
   - Check CAN_H, CAN_L connections
   - Verify termination resistors (120Ω at both ends)
   - Confirm matching bitrate on all nodes

3. **Messages not received**:
   - Check filters/masks configuration
   - Verify bitrate matches sender
   - Ensure transceiver (MCP2551/2562/TJA1055) is powered

4. **Intermittent communication**:
   - Cable length too long (max ~1000m at 50kbps, ~40m at 1Mbps)
   - EMI interference (keep cables away from motors, high-current lines)
   - Improper grounding

### Debug Methods

```cpp
// Check if MCP2515 is responding
if (mcp2515.reset() != MCP2515::ERROR_OK) {
    Serial.println("MCP2515 not responding!");
}

// Monitor error flags
uint8_t errors = mcp2515.getErrorFlags();
if (errors & MCP2515::EFLG_RX0OVR) Serial.println("RX0 overflow");
if (errors & MCP2515::EFLG_RX1OVR) Serial.println("RX1 overflow");
if (errors & MCP2515::EFLG_TXBO) Serial.println("Bus-off");

// Check error counters
Serial.print("RX errors: "); Serial.println(mcp2515.errorCountRX());
Serial.print("TX errors: "); Serial.println(mcp2515.errorCountTX());
```

---

## External References

### Datasheets
- [MCP2515 Datasheet](https://www.microchip.com/wwwproducts/en/MCP2515) - CAN controller
- [MCP2551 Datasheet](https://www.microchip.com/wwwproducts/en/MCP2551) - CAN transceiver (legacy)
- [MCP2562 Datasheet](https://www.microchip.com/wwwproducts/en/MCP2562) - Modern CAN transceiver
- [TJA1055 Datasheet](https://www.nxp.com/docs/en/data-sheet/TJA1055.pdf) - Automotive CAN transceiver

### Standards
- [ISO 11898-1](https://www.iso.org/standard/63648.html) - CAN protocol specification
- [SocketCAN Documentation](https://www.kernel.org/doc/Documentation/networking/can.txt) - Linux CAN framework

### Related Projects
- [can-usb](https://github.com/autowp/can-usb) - CanHacker/lawicel protocol implementation
- [Seeed CAN-BUS Shield Wiki](http://www.seeedstudio.com/wiki/CAN-BUS_Shield) - Hardware reference

---

## Version History (Recent)

- **1.3.1**: One-Shot Mode refactoring, added `CANCTRL_REQOP_OSM`
- **1.3.0**: Can.h alignment fix for compatibility
- Custom SPI peripheral support added
- 95kbps @ 16MHz support added
- SPI clock speed customization

---

## Contributing

This is an active open-source project welcoming contributions:

1. Fork the repository
2. Create feature branch
3. Test changes on real hardware
4. Submit pull request with clear description
5. Update documentation as needed

**Maintainer Response Time**: Usually within 1-2 weeks for PRs

---

## Quick Reference: Essential Functions

### Initialization
```cpp
MCP2515 mcp2515(CS_PIN);
mcp2515.reset();
mcp2515.setBitrate(CAN_125KBPS, MCP_16MHZ);
mcp2515.setNormalMode();
```

### Sending
```cpp
struct can_frame frame;
frame.can_id = 0x123;
frame.can_dlc = 8;
frame.data[0] = 0xAA;
// ... set remaining data
mcp2515.sendMessage(&frame);
```

### Receiving (Polling)
```cpp
struct can_frame frame;
if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    // Process frame
}
```

### Receiving (Interrupt)
```cpp
void irqHandler() { /* set flag */ }
attachInterrupt(digitalPinToInterrupt(INT_PIN), irqHandler, FALLING);

if (interrupt_flag) {
    uint8_t irq = mcp2515.getInterrupts();
    if (irq & MCP2515::CANINTF_RX0IF) {
        mcp2515.readMessage(MCP2515::RXB0, &frame);
    }
}
```

---

**Last Updated**: 2025-11-14
**Document Version**: 1.0
**For Library Version**: 1.3.1
