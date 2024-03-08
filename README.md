## Bangle.JS 2: Wireless Biometric and Weather Data Communication

### Overview

This project aims to establish wireless communication for biometric and weather data using Bangle.js, Arduino BLE, LoRa, and ROS Serial.

### Phase 1: BLE Connection

- **Objective**: Establish Bluetooth Low Energy (BLE) connection between Bangle.js and Espruino web IDE.
- **Functionality**: Reads biometric data (HRM, GPS, Accelerometer) and displays it on an HTML dashboard.
- **Implementation**: JavaScript program on Bangle.js communicates with Espruino web IDE.

### Phase 2a: LoRa Implementation

- **Objective**: Shift from Web IDE to Arduino 33 BLE microcontroller for portability, and transmit data via LoRa.
- **Functionality**: Utilises BLE advertising to share Bangle's data with connected Arduino. Arduino packs data into a LoRa packet (String) and transmits it.
- **Implementation**:
    - *Bangle*: Script loaded onto Bangle's Flash for autonomous operation.
    - *Transmitter Arduino*: Data packaged into compact String and transmitted via LoRa transceiver.
    - *Receiver Arduino*: Separate Arduino runs "receiver" sketch to decode LoRa packets and display them on terminal.

### Phase 2b: Integration with ROS

- **Objective**: Extend Phase 2a with ROS integration for enhanced data processing.
- **Modification**: Receiver sketch updated to publish LoRa data as ROS messages.
- **Functionality**: Enables ROS subscribers to access and utilise transmitted data efficiently.

### Conclusion

This project establishes a streamlined process for wireless communication of biometric and weather data, transitioning from BLE to LoRa for broader connectivity and integrating with ROS for enhanced data utilisation.