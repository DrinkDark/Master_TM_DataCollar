# TM DataCollar
## Project introduction & context
Ethology, the study of animal behavior, requires long-term, non-intrusive data collection in natural habitats. To support researchers studying Vervet monkeys in South Africa, the Monkeycall project - a collaboration between UNIL, HEI, and the ECS research group at HES-SO Valais/Wallis - developed the nRF Monkey bio-logging collar.

The initial system (V1) successfully provided 12 days of continuous audio recording, SD card storage, BLE mobile app management, and a remote release mechanism. However, field tests revealed that continuous recording led to excessive power consumption, data overload (mostly silence), and a lack of social context between individuals.

## Preliminary Foundations
This project builds upon two key preliminary studies:

- **PI MobileSens (2024)**: Focused on miniaturization, mechanical robustness, and long-range BLE communication. [GitHub repository](https://github.com/DrinkDark/Master_PI_MobilSens) 
- **PA Data Collar (2025)**: Evaluated GNSS integration and developed a custom, portable power profiling tool specifically for field use. [GitHub repository](https://github.com/DrinkDark/Master_PA_Data-collar)

The transition to V2 aims to bridge these findings into a final, highly efficient scientific tool capable of capturing meaningful acoustic data while providing new insights into primate social structures.

## Project Objectives
The primary goal of this second phase is to evolve the collar into a low-power, intelligent recorder using the nRF54L15 SoC and Zephyr RTOS. Key objectives include:

- **Intelligent audio triggering** : Integrating the TDK T5848 MEMS microphone to utilize Acoustic Activity Detection (AAD). This allows the system to remain in deep sleep, only recording when specific vocalization thresholds are met.

- **Social interaction monitoring**  : Implementing a low-power BLE proximity detection system. Each collar acts as both a broadcaster and observer, logging mutual detections to map social networks.

- **Power optimization** : Analyzing MCU states, peripheral usage (SD card power gating), and duty cycles to maximize the lifespan of a single lithium cell.

- **Validation**  :  profiling the V2 prototype's consumption across all states to calculate theoretical battery life improvements compared to the V1 architecture.

## Repository contain
1. **Report** : Report, poster, diagram and test results
2. **Hardware** : Schematics and datasheets
3. **Software** : Source code and BLE positioning methods evaluation source code
4. **Documentation** : Preliminary work report, collaring V1 report, biblography and thesis documentation
5. **Miscellaneous** 

## Results
The results show a significant improvement in efficiency. In real-world use, the intelligent trigger enables the V2 prototype to consume only about one tenth the power of the first version. Hardware and firmware improvements also cut current use by half during recording. This tool gives scientists a reliable way to study how primates communicate, while causing little disturbance to the animals.

## Authors and contributors :
* **Adrien Rey** - *Developer, Student*
* **Alexandra Andersson** - *Professor*
* **Pierre Pompili** - *Expert*