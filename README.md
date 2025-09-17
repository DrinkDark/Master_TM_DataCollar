# TM DataCollar
## Goal

### 1. Replace the microcontroller and microphone on an existing recording device.

1.1 Update the software to support the new microphone, ensuring it registers on trigger for a specified duration of x minutes.

1.2 Enhance the system's power efficiency by managing pin control and the mounting/unmounting of the SD card, among other optimizations.

1.3 Conduct a thorough power consumption analysis using the nRF PowerProfiler to accurately estimate the device's battery life.

### 2. Implement proximity detection using BLE (between devices and between device and station).

2.1 Review current BLE-based proximity detection solutions and recommend a low-power approach.

2.2 Enable detection of nearby devices or stations through BLE advertising and listening, ensuring that each device or station can both detect and be detected.

2.3 Examine the advertising and listening cycles to find an optimal balance between power consumption and detection efficiency.

2.4 Carry out a power consumption analysis to forecast battery longevity.

2.5 Incorporate BLE commands into devices and stations to allow programming of specific tasks, such as releasing a particular collar.

### 3. Data transfer via FTDI chip (USB-to-serial). -> The first two points must be completed and tested

3.1 Modify the software to support data transfer through an FTDI interface.

3.2 Create PC-side terminal or script to send/receive data.

3.3 Evaluate the data transfer capacity to understand its performance limits.

3.4 Perform a power consumption analysis to predict battery life during data transfer operations.

### Test and mesure the power consumption of the complet system
 
4.1 Define test scenarios.

4.2  Carry out a power consumption analysis of the global system.

