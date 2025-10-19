# SHT3x Library

A portable C library for Sensirion SHT3x humidity and temperature sensors, supporting all microcontroller platforms through function pointers.

## Features

- ✅ Supports all SHT3x variants (SHT30, SHT31, SHT35)
- ✅ Portable - works on any microcontroller
- ✅ Single shot and periodic measurement modes
- ✅ Configurable repeatability (High/Medium/Low)
- ✅ CRC verification for data integrity
- ✅ Status register operations
- ✅ Internal heater control
- ✅ Strict C coding standard compliance
- ✅ Comprehensive error handling

## Library Structure

```
sht3x/
├── sht3x.h           # Main header file
├── sht3x.c           # Implementation
├── example.c         # Usage examples
└── README.md         # This documentation
```

## Quick Start

### 1. Initialize Sensor

```c
#include "sht3x.h"

// Declare handle
sht3x_t sht3x;

// Set function pointers for your platform
sht3x.i2c_write = your_i2c_write_function;
sht3x.i2c_read = your_i2c_read_function;
sht3x.delay = your_delay_function;
sht3x.addr = SHT3X_ADDR_A; // or SHT3X_ADDR_B
sht3x.user_data = NULL; // or pointer to your data

// Initialize sensor
sht3x_result_t result = sht3x_init(&sht3x);
if (result != SHT3X_OK) {
    // Handle error
}
```

### 2. Single Shot Measurement

```c
sht3x_data_t data;

// Set repeatability (optional)
sht3x_set_repeatability(&sht3x, SHT3X_REPEATABILITY_HIGH);

// Start measurement
result = sht3x_start_single_shot(&sht3x);
if (result != SHT3X_OK) {
    // Handle error
}

// Read data
result = sht3x_read_data(&sht3x, &data);
if (result == SHT3X_OK) {
    printf("Temperature: %.2f °C\n", data.temperature);
    printf("Humidity: %.2f %%RH\n", data.humidity);
}
```

### 3. Periodic Measurement

```c
// Set measurement frequency
sht3x_set_frequency(&sht3x, SHT3X_FREQUENCY_1_MPS);

// Start periodic measurement
result = sht3x_start_periodic(&sht3x);

// In main loop
uint8_t ready;
sht3x_data_t data;

result = sht3x_is_data_ready(&sht3x, &ready);
if (result == SHT3X_OK && ready) {
    result = sht3x_read_data(&sht3x, &data);
    if (result == SHT3X_OK) {
        printf("T: %.2f°C, RH: %.2f%%\n", data.temperature, data.humidity);
    }
}

// Stop periodic measurement when needed
sht3x_stop_periodic(&sht3x);
```

## Platform Implementation

### STM32 HAL

```c
// I2C Write function
static sht3x_result_t
stm32_i2c_write(sht3x_t* handle, uint8_t addr, const uint8_t* data, size_t len) {
    extern I2C_HandleTypeDef hi2c1;
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Master_Transmit(&hi2c1, addr << 1, (uint8_t*)data, len, 1000);
    return (status == HAL_OK) ? SHT3X_OK : SHT3X_ERR_I2C;
}

// I2C Read function  
static sht3x_result_t
stm32_i2c_read(sht3x_t* handle, uint8_t addr, uint8_t* data, size_t len) {
    extern I2C_HandleTypeDef hi2c1;
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Master_Receive(&hi2c1, addr << 1, data, len, 1000);
    return (status == HAL_OK) ? SHT3X_OK : SHT3X_ERR_I2C;
}

// Delay function
static void
stm32_delay(sht3x_t* handle, uint32_t ms) {
    HAL_Delay(ms);
}
```

### Arduino/ESP32

```c
#include <Wire.h>

// I2C Write function
static sht3x_result_t
arduino_i2c_write(sht3x_t* handle, uint8_t addr, const uint8_t* data, size_t len) {
    Wire.beginTransmission(addr);
    for (size_t i = 0; i < len; ++i) {
        Wire.write(data[i]);
    }
    return (Wire.endTransmission() == 0) ? SHT3X_OK : SHT3X_ERR_I2C;
}

// I2C Read function
static sht3x_result_t
arduino_i2c_read(sht3x_t* handle, uint8_t addr, uint8_t* data, size_t len) {
    size_t bytes_read = Wire.requestFrom(addr, len);
    if (bytes_read != len) {
        return SHT3X_ERR_I2C;
    }
    
    for (size_t i = 0; i < len; ++i) {
        data[i] = Wire.read();
    }
    return SHT3X_OK;
}

// Delay function
static void
arduino_delay(sht3x_t* handle, uint32_t ms) {
    delay(ms);
}
```

### Linux/Raspberry Pi

```c
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

typedef struct {
    int i2c_fd;
} linux_i2c_t;

static sht3x_result_t
linux_i2c_write(sht3x_t* handle, uint8_t addr, const uint8_t* data, size_t len) {
    linux_i2c_t* i2c = (linux_i2c_t*)handle->user_data;
    
    if (ioctl(i2c->i2c_fd, I2C_SLAVE, addr) < 0) {
        return SHT3X_ERR_I2C;
    }
    
    if (write(i2c->i2c_fd, data, len) != len) {
        return SHT3X_ERR_I2C;
    }
    
    return SHT3X_OK;
}

static sht3x_result_t
linux_i2c_read(sht3x_t* handle, uint8_t addr, uint8_t* data, size_t len) {
    linux_i2c_t* i2c = (linux_i2c_t*)handle->user_data;
    
    if (ioctl(i2c->i2c_fd, I2C_SLAVE, addr) < 0) {
        return SHT3X_ERR_I2C;
    }
    
    if (read(i2c->i2c_fd, data, len) != len) {
        return SHT3X_ERR_I2C;
    }
    
    return SHT3X_OK;
}

static void
linux_delay(sht3x_t* handle, uint32_t ms) {
    usleep(ms * 1000);
}
```

## API Reference

### Initialization Functions

| Function | Description |
|----------|-------------|
| `sht3x_init()` | Initialize sensor and perform soft reset |
| `sht3x_deinit()` | Deinitialize sensor |
| `sht3x_reset()` | Perform soft reset |

### Configuration Functions

| Function | Description |
|----------|-------------|
| `sht3x_set_addr()` | Set I2C address |
| `sht3x_set_repeatability()` | Set measurement repeatability |
| `sht3x_set_frequency()` | Set measurement frequency for periodic mode |

### Measurement Functions

| Function | Description |
|----------|-------------|
| `sht3x_start_single_shot()` | Start single shot measurement |
| `sht3x_start_periodic()` | Start periodic measurement |
| `sht3x_stop_periodic()` | Stop periodic measurement |
| `sht3x_read_data()` | Read measurement data |
| `sht3x_is_data_ready()` | Check if data is ready |

### Status & Control Functions

| Function | Description |
|----------|-------------|
| `sht3x_read_status()` | Read status register |
| `sht3x_clear_status()` | Clear status register |
| `sht3x_enable_heater()` | Enable internal heater |
| `sht3x_disable_heater()` | Disable internal heater |

## Error Codes

| Code | Description |
|------|-------------|
| `SHT3X_OK` | Success |
| `SHT3X_ERR` | Generic error |
| `SHT3X_ERR_PARAM` | Invalid parameter |
| `SHT3X_ERR_I2C` | I2C communication error |
| `SHT3X_ERR_CRC` | CRC check failed |
| `SHT3X_ERR_TIMEOUT` | Operation timeout |
| `SHT3X_ERR_NOT_READY` | Sensor not ready |

## Hardware Connection

### I2C Connections

| SHT3x Pin | Function | Connection |
|-----------|----------|------------|
| VDD | Power | 2.15V - 5.5V |
| VSS | Ground | Ground |
| SDA | I2C Data | MCU SDA + 10kΩ pull-up |
| SCL | I2C Clock | MCU SCL + 10kΩ pull-up |
| ADDR | Address Select | VDD (0x45) or VSS (0x44) |
| ALERT | Alert Output | MCU GPIO (optional) |
| nRESET | Reset | MCU GPIO or VDD |

### Typical Circuit

```
VDD ----+---- SHT3x VDD
        |
       100nF
        |
GND ----+---- SHT3x VSS
             SHT3x die pad

MCU SDA ---- 10kΩ ---- VDD
        |
        +---- SHT3x SDA

MCU SCL ---- 10kΩ ---- VDD  
        |
        +---- SHT3x SCL

ADDR pin ---- VDD (for 0x45) or GND (for 0x44)
```

## Specifications

### SHT35 (High accuracy)
- Temperature accuracy: ±0.1°C (20-60°C)
- Humidity accuracy: ±1.5%RH
- Resolution: 0.01°C, 0.01%RH

### SHT31 (Standard)
- Temperature accuracy: ±0.2°C (0-90°C)  
- Humidity accuracy: ±2%RH
- Resolution: 0.01°C, 0.01%RH

### SHT30 (Economy)
- Temperature accuracy: ±0.2°C (0-65°C)
- Humidity accuracy: ±2%RH  
- Resolution: 0.01°C, 0.01%RH

## Advanced Features

### Status Register Bits

| Bit | Name | Description |
|-----|------|-------------|
| 15 | Alert pending | Alert condition pending |
| 13 | Heater status | Heater status (0=OFF, 1=ON) |
| 11 | RH tracking alert | Humidity tracking alert |
| 10 | T tracking alert | Temperature tracking alert |
| 4 | System reset detected | System reset detected |
| 1 | Command status | Last command status (0=OK, 1=Failed) |
| 0 | Write data checksum | Write data checksum (0=OK, 1=Failed) |

### Heater Usage

The heater is used for:
- Sensor plausibility check
- Removing condensation in humid conditions
- Self-diagnostic purposes

```c
// Enable heater and check
sht3x_enable_heater(&sht3x);
sht3x->delay(sht3x, 2000); // Wait 2 seconds

uint16_t status;
sht3x_read_status(&sht3x, &status);
if (status & 0x2000) {
    printf("Heater is working\n");
}

sht3x_disable_heater(&sht3x);
```

## Troubleshooting

### Common Issues

1. **I2C Communication Failed**
   - Check pull-up resistors (10kΩ)
   - Verify I2C address (ADDR pin setting)
   - Check clock frequency (≤1MHz)
   - Verify power supply voltage

2. **CRC Error**
   - Noise on I2C bus
   - Timing issues
   - Hardware problem
   - Check ground connections

3. **No Data Ready**
   - Wait sufficient measurement time
   - Check measurement mode
   - Reset sensor if needed
   - Verify sensor initialization

### Debug Tips

```c
// Check status register for errors
uint16_t status;
if (sht3x_read_status(&sht3x, &status) == SHT3X_OK) {
    printf("Status: 0x%04X\n", status);
    if (status & 0x0002) {
        printf("Last command failed!\n");
    }
    if (status & 0x0001) {
        printf("Checksum error!\n");
    }
    if (status & 0x0010) {
        printf("Reset detected!\n");
    }
}

// Test I2C communication
result = sht3x_reset(&sht3x);
if (result != SHT3X_OK) {
    printf("I2C communication failed!\n");
}
```

### Measurement Timing

| Repeatability | Measurement Time |
|---------------|------------------|
| High | 12.5 - 15 ms |
| Medium | 4.5 - 6 ms |
| Low | 2.5 - 4 ms |

## Performance Considerations

### Power Consumption

| Mode | Current Consumption |
|------|-------------------|
| Idle (single shot) | 0.2 - 2.0 µA |
| Idle (periodic) | 45 µA |
| Measuring | 600 - 1500 µA |
| Average (1 mps) | ~1.7 µA |

### I2C Timing

- Maximum clock frequency: 1 MHz
- Pull-up resistor: 10kΩ (recommended)
- Bus capacitance: ≤400 pF

## Examples

See `example.c` for complete working examples including:
- Basic single shot measurement
- Periodic measurement with data ready checking
- Status register operations
- Heater control
- Error handling

## License

MIT License - see LICENSE file for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Follow the coding standard
4. Add tests if applicable
5. Create a Pull Request

## Changelog

### v1.0.0
- Initial release
- Support for all SHT3x variants
- Single shot and periodic modes
- Comprehensive error handling
- Cross-platform compatibility
- Complete documentation

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review the example code
3. Open an issue on GitHub

## References

- [SHT3x Datasheet](https://www.sensirion.com/en/environmental-sensors/humidity-sensors/digital-humidity-sensors-for-various-applications/)
- [I2C Specification](https://www.nxp.com/docs/en/user-guide/UM10204.pdf)
- [C Coding Standard](https://github.com/MaJerle/c-code-style)
