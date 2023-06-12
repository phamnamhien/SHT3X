**SHT3X Temperature-Humidity Sensor Library**

This is a library for interfacing with the SHT3X temperature-humidity sensor. The project is designed to work with the STM32F103C8T6 microcontroller and utilizes the HAL-RTOS v2 library.

**Overview**

The SHT3X sensor is a popular choice for measuring temperature and humidity in various applications. This library provides a convenient and efficient way to interface with the sensor using the STM32F103C8T6 microcontroller. The library is built on top of the HAL-RTOS v2 library, which provides the necessary hardware abstraction layer and real-time operating system functionalities.

**Features**

- Read temperature and humidity values from the SHT3X sensor.
- Calculate the dew point from the measured temperature and humidity.
- Support for different measurement modes and accuracy settings.
- Error handling and status reporting for reliable data acquisition.
- Utilizes the HAL-RTOS v2 library for efficient resource management and multitasking.

**Installation**

1. Clone or download the repository to your local machine.
1. Open your STM32 development environment (e.g., STM32CubeIDE).
1. Create a new project or open an existing one.
1. Add the library source files to your project.
1. Configure the STM32F103C8T6 microcontroller settings in your project.
1. Build and flash the project to your microcontroller.

**Usage**

1. Include the necessary library header files in your project.
1. Initialize the SHT3X sensor by calling the appropriate initialization function.
1. Configure the measurement mode and accuracy settings, if needed.
1. Use the provided functions to read temperature, humidity, and dew point values.
1. Handle any errors or status flags that may occur during the operation.
1. Optionally, implement additional functionalities based on your project requirements.

**Examples**

```
#include "sht3x.h" // Initialize the SHT3X sensor SHT3X\_Init(); // Configure the measurement mode and accuracy settings SHT3X\_SetMeasurementMode(SHT3X\_MODE\_HIGH, SHT3X\_ACCURACY\_HIGH); // Read temperature and humidity values float temperature = SHT3X\_ReadTemperature(); float humidity = SHT3X\_ReadHumidity(); // Calculate the dew point float dewPoint = SHT3X\_CalculateDewPoint(temperature, humidity); // Handle errors or status flags if (SHT3X\_GetStatus() != SHT3X\_STATUS\_OK) { // Handle error condition } // Additional functionalities... 
```
**Contribution**

Contributions to the SHT3X Temperature-Humidity Sensor Library are welcome. If you encounter any issues or have suggestions for improvements, please feel free to submit an issue or pull request on the GitHub repository.

**License**

This project is licensed under the [MIT License](https://chat.openai.com/LICENSE). Please see the **LICENSE** file for more information.

**Acknowledgements**

This library makes use of the following open-source projects:

- [HAL-RTOS v2](https://github.com/stm32-rs/hal-rtos) library for STM32 microcontrollers.
- [SHT3x](https://github.com/Sensirion/arduino-sht) library by Sensirion for Arduino.

We would like to express our gratitude to the authors and contributors of these projects for their valuable work.

**Contact**

For any inquiries or further information, please contact <phamnamhien@gmail.com>.

