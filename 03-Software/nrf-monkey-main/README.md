# nRF Monkey - Speak No Evil

## 1. Generalities

This repository contains the source code for the firmware embedded in the project hardware.

The aim of the project is to record, via an embedded system, the sounds produced by monkeys in the wild. These sound captures are stored on an SD card for 10 days.

The system is embedded in a small case that the monkeys wear around their necks. Once the 10-day recording period has elapsed, researchers will be able to open the collar remotely, retrieve it and extract the data.

All the sounds will then be analyzed to deduce a language, if one exists... 

## 2. Uses cases

We have identified three modes of operation:

1. Setting up
2. Recording operation
3. Open the collar

### 2.1. Setting up the collar

In this use case, the operators need to format an SD Card, insert it into the box and attach the collar around the animal's neck. 
Once this step has been completed, all that remains is to start the device. The diagram below shows how to do this:

1. Launch a BLE search to obtain a list of all collars present in the area
2. Connect to one of the detected collars.
3. Start recording
![Setup](uml/out/uml/use\ cases/setup.png)

In the dedicated iOS app, we can have the following display
![iOS_Discovery](uml/out/ios/SpeakNoEvil_iOS.png)

### 2.2. Standard operation

The collar should record all sounds picked up around the collar for about ten days. In fact, the system will operate in recording mode until:

1. an operator connects again and triggers the opening of the collar.
2. there is no more space on the SD Card to store the sound files.
3. the battery level becomes low

When one of these three situations occurs, the system goes into low-power mode and cannot restart recording without external intervention.
![Standard Recording Operation](uml/out/uml/use\ cases/recorder.png)

### 2.3. Opening the collar

As with commissioning, this use case requires the intervention of an operator.

Using the application specially developed for the system, the operator will launch a collar search and connect to the desired collar. Then simply press the release button to free the collar from the animal's neck.

Once this operation has been completed, the system will go into power-saving mode, and the hardware will need to be manipulated to restart recording.

![Setup](uml/out/uml/use\ cases/open_collar.png)

## 3. Hardware

The main processor is a nRF54l15 from Nordic Seminconductor.

## 3.1. Development tools

### 3.1.1. nRF Connect SDK

The project has been develop using the nRF Connect SDK v2.4.0.

To get the SDK, download the latest version of the [nRF Command Line Tools](https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools/Download) and [nRF Connect for Desktop](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop/Download). Using nRF Connect for Desktop, use the "Toolchain Manager" to isntall nRF Connect SDK v2.4.0. You can also read the Getting started with nRF Connect SDK documentation, if you want to...

### 3.1.2. GIT

The project can be downloaded from our [Gitlab server](https://gitlab.hevs.ch/patrice.rudaz/nrf-monkey/-/archive/main/nrf-monkey-main.zip) and opened with Visual Studio Code. We use the nRF Connect Extension Pack plugin.

We can compile the source code directly and push it into hardware using the VS Code plugin or the appropriate command lines.

### 3.1.3. Debugging

The RTT protocol is used for DEBUG messages.

To view them, use the JLinkRTTViewer or JLinkRTTClient tools by connecting to the programming connector.

If you opt for JLinkRTTClient, the order must be set up in advance. To do this, 

1. open a command prompt and enter the command:
   `JLinkExe -if SWD -speed 4000 -autoconnect 1 -device NRF54l15_XXAA_APP [-USB <SEGGER_SERIAL>] [-RTTTelnetPort <PORT_NUMBER>] [-JLinkScriptFile <PATH_TO_SCRIPT_FILE>/<SCRIPT_FILE_NAME>.JlinkScript]`

  Where

  * `<SEGGER_SERIAL>` is the serial number of the SEGGER debugger (optional if only one segger is plugged in)
  * `<PORT_NUMBER>` let choose a specific port number
  * `-JLinkScriptFile` let give some more option to JLinkExe ! For example, to not reset the SOC when getting connected.

2. launch the RTT client in an other command prompt:
   `JLinkRTTClient [-RTTTelnetPort <PORT_NUMBER>] [ | TZ="UTC" ts '%d.%m.%Y %H:%M:%.S' | tee monkey_1.log]`

  where 

  * `PORT_NUMBER` must mathc the one used in `JLinkExe` command line
  * `[ | TZ="UTC" ts '%d.%m.%Y %H:%M:%.S' | tee <LOG_FILE_NAME>]`  let log the debug message in a file with the name `<LOG_FILE_NAME>`
   
  Example of script file:

  ```script
  /* ***************************************************************************************
   *
   *       ResetTarget()
   *
   *  Function description
   *    Replaces reset strategies of DLL. No matter what reset type is selected in the DLL, 
   *    if this function is present, it will be called instead of the DLL internal reset.
   *
   *  Notes
   *    (1) DLL expects target CPU to be halted / in debug mode, when leaving this function
   *    (2) May use MEM_API functions
   * *************************************************************************************** */
  
  int ResetTarget(void) {
      JLINK_SYS_Report("JLink script: Reset Target skiped.");
      return 0;
  }
  
  int InitTarget(void) 
  {
      JLINK_SYS_Report("J-Link script: InitTarget done.");
      return 0;
  }
  ```

#### addr2line

When a firmware using Zephyr crashes, you can find out at which line the problem occurs with the command `addr2line`. To use this tool, you can use:

```bash
user@macos ~ [PATH_TO_NCS_TOOLCHAIN]/opt/zephyr-sdk/arm-zephyr-eabi/bin/arm-zephyr-eabi-addr2line -e [PATH_TO_BUILD]/[BUILD_FILE].elf [MEMORY_ADDR]
```

where, for example, `[PATH_TO_NCS_TOOLCHAIN]` is for me `/opt/nordic/ncs/toolchains/v2.3.0`

### 3.2. Flashing firmware in hardware

Two scripts have been written to program the MonkeyCollar.

1. `update_firmware.bat`, for Windows users
2. `update_firmware.sh`, for Linux and macOS users

These scripts can be found in the `scripts` directory. They can be given two arguments: the collar identifier and the microphone input gain. The default mic input gain is 3.

Use a command prompt or terminal (depending on operating system)

```batch
C:[project_root_folder]\scripts > update_firmware.bat 45 3
```

```bash
user@macos scripts % ./update_fimrware.sh 45 3 
```

where:

- `45`: is the collar identifier
- `3`: is the microphone input gain. This value can be set from 1 to 5

### 3.3. SD CARD format

The maximum supported SD Card size is 32GB. To minimize the current consumption of the activity on the card (writing), it MUST be formatted in FAT32 with a cluster's size of 64k. Windows users can simply use the contextual menu and choose `format...` feature.

On Linux or macOS, you can format the card with the following command lines:

```bash
user@macOS / % diskutil info <VOLUME_PATH_AND_NAME>
user@macOS / % diskutil unmount <DEVICE_NODE>
user@macOS / % sudo newfs_msdos -F 32 -c 128 -v <VOLUME_NAME> <DEVICE_NODE>
```

