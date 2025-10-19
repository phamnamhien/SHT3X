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
└── sht3x.c           # Implementation
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

### STM32F103C8T6 (Blue Pill) Example

This example shows how to use the SHT3x library with STM32F103C8T6 using STM32CubeMX generated HAL code.

#### Hardware Setup
- **STM32F103C8T6** (Blue Pill board)
- **SHT3x sensor** connected via I2C1
- **Pin connections:**
  - PB6 → SHT3x SCL (I2C1_SCL)
  - PB7 → SHT3x SDA (I2C1_SDA)
  - PB15 → SHT3x nRESET (GPIO Output, High level, Pull-up)
  - PA9 → UART1_TX (Debug output)
  - PA10 → UART1_RX (Debug input)
  - PA8 → RS485_RXEN (GPIO Output, High level, always enabled for RX)
  - 3.3V → SHT3x VDD
  - GND → SHT3x VSS, die pad
  - GND → SHT3x ADDR (for address 0x44)

#### STM32CubeMX Configuration
1. **RCC Configuration:**
   - HSE: Crystal/Ceramic Resonator (8MHz)
   - PLLCLK: 72MHz
   
2. **I2C1 Configuration:**
   - Mode: I2C
   - Speed: 100kHz (Standard Mode)
   - Clock No Stretch: Disabled
   - PB6: I2C1_SCL
   - PB7: I2C1_SDA

3. **UART1 Configuration:**
   - Mode: Asynchronous
   - Baud Rate: 115200
   - Word Length: 8 bits
   - Stop Bits: 1
   - Parity: None
   - PA9: USART1_TX
   - PA10: USART1_RX

4. **GPIO Configuration:**
   - PB6, PB7: Alternate Function Open Drain, Pull-up
   - PB15: GPIO_Output, High level, Pull-up (SHT3x nRESET)
   - PA8: GPIO_Output, High level (RS485_RXEN)
   - PA9: Alternate Function Push Pull (UART1_TX)
   - PA10: Input with Pull-up (UART1_RX)
   - PC13: GPIO_Output (Built-in LED)

#### HAL Implementation

```c
/* main.h - Add these includes */
#include "sht3x.h"
#include <stdio.h>

/* main.c - Global variables */
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1; // For printf (optional)
sht3x_t sht3x_sensor;

/* I2C Write function for STM32F103C8T6 */
static sht3x_result_t
stm32f103_i2c_write(sht3x_t* handle, uint8_t addr, const uint8_t* data, size_t len) {
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Master_Transmit(&hi2c1, addr << 1, (uint8_t*)data, len, 1000);
    return (status == HAL_OK) ? SHT3X_OK : SHT3X_ERR_I2C;
}

/* I2C Read function for STM32F103C8T6 */
static sht3x_result_t
stm32f103_i2c_read(sht3x_t* handle, uint8_t addr, uint8_t* data, size_t len) {
    HAL_StatusTypeDef status;
    
    status = HAL_I2C_Master_Receive(&hi2c1, addr << 1, data, len, 1000);
    return (status == HAL_OK) ? SHT3X_OK : SHT3X_ERR_I2C;
}

/* Delay function for STM32F103C8T6 */
static void
stm32f103_delay(sht3x_t* handle, uint32_t ms) {
    HAL_Delay(ms);
}

/* Initialize SHT3x sensor */
static void
sht3x_sensor_init(void) {
    sht3x_result_t result;
    
    /* Configure SHT3x handle */
    sht3x_sensor.i2c_write = stm32f103_i2c_write;
    sht3x_sensor.i2c_read = stm32f103_i2c_read;
    sht3x_sensor.delay = stm32f103_delay;
    sht3x_sensor.addr = SHT3X_ADDR_A; // ADDR pin connected to GND
    sht3x_sensor.user_data = NULL;
    
    /* Initialize sensor */
    result = sht3x_init(&sht3x_sensor);
    if (result != SHT3X_OK) {
        printf("SHT3x initialization failed: %d\r\n", result);
        Error_Handler();
    }
    
    printf("SHT3x initialized successfully\r\n");
}

/* Read and display sensor data */
static void
sht3x_read_and_display(void) {
    sht3x_result_t result;
    sht3x_data_t data;
    
    /* Start single shot measurement */
    result = sht3x_start_single_shot(&sht3x_sensor);
    if (result != SHT3X_OK) {
        printf("Failed to start measurement: %d\r\n", result);
        return;
    }
    
    /* Read measurement data */
    result = sht3x_read_data(&sht3x_sensor, &data);
    if (result == SHT3X_OK) {
        printf("Temperature: %.2f°C, Humidity: %.2f%%RH\r\n", 
               data.temperature, data.humidity);
        
        /* Toggle LED to indicate successful reading */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    } else {
        printf("Failed to read data: %d\r\n", result);
    }
}

/* Main function */
int main(void) {
    /* Initialize HAL, clocks, peripherals */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init(); // Optional for printf
    
    /* Initialize SHT3x sensor */
    sht3x_sensor_init();
    
    /* Main loop */
    while (1) {
        sht3x_read_and_display();
        HAL_Delay(2000); // Read every 2 seconds
    }
}

/* Optional: Printf redirect to UART */
#ifdef __GNUC__
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 1000);
    return len;
}
#endif
```

#### Complete Project Structure

```
STM32F103_SHT3x/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   └── stm32f1xx_hal_conf.h
│   ├── Src/
│   │   ├── main.c            # Main application with SHT3x integration
│   │   ├── stm32f1xx_hal_msp.c
│   │   └── stm32f1xx_it.c
│   └── SHT3X/
│       ├── sht3x.h           # SHT3x library header
│       └── sht3x.c           # SHT3x library implementation
├── Drivers/
│   ├── STM32F1xx_HAL_Driver/
│   └── CMSIS/
└── STM32F103C8T6.ioc        # CubeMX project file
```

#### Compilation Notes

1. **Add source files** to your STM32CubeIDE project:
   - `sht3x.c` in SHT3X folder
   - `sht3x.h` in SHT3X folder

2. **Compiler settings:**
   - C standard: C11 or C99
   - Optimization: -O2 for release, -Og for debug

#### Advanced Features Example

```c
/* Advanced example with error checking and all features */
void
sht3x_advanced_example(void) {
    sht3x_t sht3x;
    sht3x_result_t result;
    sht3x_data_t data;
    uint16_t status;
    uint8_t ready;
    
    /* Initialize with error checking */
    sht3x.i2c_write = stm32f103_i2c_write;
    sht3x.i2c_read = stm32f103_i2c_read;
    sht3x.delay = stm32f103_delay;
    sht3x.addr = SHT3X_ADDR_A;
    sht3x.user_data = NULL;
    
    result = sht3x_init(&sht3x);
    if (result != SHT3X_OK) {
        printf("Sensor initialization failed!\r\n");
        Error_Handler();
    }
    
    /* Check sensor status */
    result = sht3x_read_status(&sht3x, &status);
    if (result == SHT3X_OK) {
        printf("Initial status: 0x%04X\r\n", status);
        
        /* Check for any errors */
        if (status & 0x0002) {
            printf("Warning: Last command failed\r\n");
        }
        if (status & 0x0001) {
            printf("Warning: Checksum error detected\r\n");
        }
        
        /* Clear status if needed */
        if (status & 0x8013) { /* Clear if any error flags set */
            sht3x_clear_status(&sht3x);
        }
    }
    
    /* Test heater functionality */
    printf("Testing heater...\r\n");
    sht3x_enable_heater(&sht3x);
    HAL_Delay(1000);
    
    sht3x_read_status(&sht3x, &status);
    if (status & 0x2000) {
        printf("Heater test passed\r\n");
    } else {
        printf("Heater test failed\r\n");
    }
    sht3x_disable_heater(&sht3x);
    
    /* Configure for high accuracy periodic measurement */
    sht3x_set_repeatability(&sht3x, SHT3X_REPEATABILITY_HIGH);
    sht3x_set_frequency(&sht3x, SHT3X_FREQUENCY_2_MPS);
    
    result = sht3x_start_periodic(&sht3x);
    if (result != SHT3X_OK) {
        printf("Failed to start periodic measurement\r\n");
        return;
    }
    
    /* Read 20 samples */
    for (int i = 0; i < 20; i++) {
        /* Wait for data with timeout */
        uint32_t timeout = 1000; /* 1 second timeout */
        do {
            result = sht3x_is_data_ready(&sht3x, &ready);
            if (result != SHT3X_OK) {
                printf("Error checking data ready\r\n");
                break;
            }
            
            if (!ready) {
                HAL_Delay(10);
                timeout -= 10;
            }
        } while (!ready && timeout > 0);
        
        if (timeout == 0) {
            printf("Timeout waiting for data\r\n");
            break;
        }
        
        /* Read and validate data */
        result = sht3x_read_data(&sht3x, &data);
        if (result == SHT3X_OK) {
            /* Basic data validation */
            if (data.temperature >= -40.0f && data.temperature <= 125.0f &&
                data.humidity >= 0.0f && data.humidity <= 100.0f) {
                printf("Sample %d: T=%.2f°C, RH=%.2f%%\r\n", 
                       i + 1, data.temperature, data.humidity);
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            } else {
                printf("Sample %d: Invalid data received\r\n", i + 1);
            }
        } else {
            printf("Sample %d: Read error %d\r\n", i + 1, result);
        }
        
        HAL_Delay(500); /* 2 Hz measurement rate */
    }
    
    /* Stop periodic measurement */
    sht3x_stop_periodic(&sht3x);
    
    /* Final status check */
    sht3x_read_status(&sht3x, &status);
    printf("Final status: 0x%04X\r\n", status);
    
    /* Deinitialize */
    sht3x_deinit(&sht3x);
}
```

#### Error Handling Best Practices

```c
/* Robust error handling example */
static void
handle_sht3x_error(sht3x_result_t error, const char* operation) {
    switch (error) {
        case SHT3X_OK:
            break; /* No error */
            
        case SHT3X_ERR_I2C:
            printf("I2C communication error during %s\r\n", operation);
            printf("Check wiring and pull-up resistors\r\n");
            break;
            
        case SHT3X_ERR_CRC:
            printf("CRC error during %s\r\n", operation);
            printf("Possible noise on I2C bus\r\n");
            break;
            
        case SHT3X_ERR_TIMEOUT:
            printf("Timeout error during %s\r\n", operation);
            break;
            
        case SHT3X_ERR_PARAM:
            printf("Invalid parameter during %s\r\n", operation);
            break;
            
        case SHT3X_ERR_NOT_READY:
            printf("Sensor not ready during %s\r\n", operation);
            break;
            
        default:
            printf("Unknown error %d during %s\r\n", error, operation);
            break;
    }
}

/* Usage example with error handling */
void
robust_sensor_reading(void) {
    sht3x_t sht3x;
    sht3x_result_t result;
    sht3x_data_t data;
    
    /* Initialize sensor */
    /* ... setup code ... */
    
    result = sht3x_start_single_shot(&sht3x);
    if (result != SHT3X_OK) {
        handle_sht3x_error(result, "start measurement");
        return;
    }
    
    result = sht3x_read_data(&sht3x, &data);
    if (result != SHT3X_OK) {
        handle_sht3x_error(result, "read data");
        
        /* Try to recover */
        printf("Attempting sensor reset...\r\n");
        result = sht3x_reset(&sht3x);
        if (result == SHT3X_OK) {
            printf("Sensor reset successful\r\n");
        } else {
            printf("Sensor reset failed\r\n");
        }
        return;
    }
    
    printf("T: %.2f°C, RH: %.2f%%\r\n", data.temperature, data.humidity);
}
```

#### Troubleshooting for STM32F103C8T6

**Common Issues:**

1. **I2C Communication Failure:**
   ```c
   /* Check I2C configuration */
   if (HAL_I2C_IsDeviceReady(&hi2c1, SHT3X_ADDR_A << 1, 3, 1000) != HAL_OK) {
       printf("SHT3x not detected on I2C bus\r\n");
       /* Check wiring, power, and pull-up resistors */
   }
   ```

2. **Clock Configuration:**
   - Ensure system clock is 72MHz
   - I2C clock should be 100kHz for reliable operation
   - Check HSE crystal is 8MHz

3. **Power Supply:**
   - SHT3x requires 2.15V - 5.5V
   - Use 3.3V from STM32F103C8T6
   - Add 100nF decoupling capacitor

4. **Wiring Checklist:**
   ```
   STM32F103C8T6    SHT3x
   ===============  =======
   3.3V       -->   VDD
   GND        -->   VSS, die pad
   PB6 (SCL)  -->   SCL (with 10kΩ pull-up to 3.3V)
   PB7 (SDA)  -->   SDA (with 10kΩ pull-up to 3.3V)
   GND        -->   ADDR (for address 0x44)
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

STM32F103C8T6 Pin Connections:
==============================
PB6 (I2C1_SCL) ---- 10kΩ ---- 3.3V
               |
               +---- SHT3x SCL

PB7 (I2C1_SDA) ---- 10kΩ ---- 3.3V  
               |
               +---- SHT3x SDA

PB15 (GPIO_OUT) ---- SHT3x nRESET (with internal pull-up)

PA9 (UART1_TX) ---- Debug TX Output
PA10 (UART1_RX) ---- Debug RX Input
PA8 (GPIO_OUT) ---- RS485_RXEN (High = RX enabled)

3.3V ---- SHT3x VDD
GND ---- SHT3x VSS, die pad, ADDR pin

Power Supply:
3.3V: SHT3x VDD, Pull-up resistors
GND: SHT3x VSS, die pad, ADDR pin, STM32 GND
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

## Support This Project

[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/Q5Q1JW4XS)
[![PayPal](https://img.shields.io/badge/PayPal-00457C?style=for-the-badge&logo=paypal&logoColor=white)](https://paypal.me/phamnamhien)

---

If this library helped your project, consider giving it a ⭐ on GitHub!
