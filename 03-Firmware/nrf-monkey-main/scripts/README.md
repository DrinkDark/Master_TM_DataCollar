# Speak No Evil Scripting Tools

Two scripts have been prepared to update the firmware of a Speak No Evil device:
- `update_firmware.bat`: for Windows users
- `update_firmware.sh`: for Linux and MacOS users

## Installation and hardware requirements

### nRF Connect SDK

To be able to use those scripts, You must have installed the nRF connect SDK development tools provided by [Nordic Semiconductor](https://www.nordicsemi.com).

Just download and install the latest version of the [nRF Command Line Tools](https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools/Download) and [nRF Connect for Desktop](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-Desktop/Download). Through "nRF Connect for Desktop" application, launch the "Toolchain Manager" and install nRF Connect SDK v2.4.0 to your computer. You can also read the Getting started with nRF Connect SDK documentation, if you want to...

The development tools must be installed in the following path:
- under Windows: `C:\ncs`
- under macOS/Linux: `/opt/nordic/ncs` (this is the default path)

### SEGGER

To be able to update the firmware in the devices you need [SEGGER's J-Link debug probe](https://www.segger.com/products/debug-probes/j-link/#models).

Using the small cable you received, you can connect to the device...

## How to use

Open a command prompt (Windows) or terminal (Linux or macOS) in the project's home directory. Then go to the `scripts` subdirectory. 

For example:

- under macOS/Linux:

	```sh
	macOS@user ~ % cd Develop/GIT/Monkey/nrf-monkey
	macOS@user nrf-monkey % cd scripts
	macOS@user scripts %
	```

- under Windows:

	```batch
	C:\Users\toto>cd Develop\GIT\Monkey\nrf-monkey
	C:\Users\toto\Develop\GIT\Monkey\nrf-monkey>cd scripts
	C:\Users\toto\Develop\GIT\Monkey\nrf-monkey\scripts>
	
	```

Then simply run the script. 
You can give it the device ID number as an argument. If you don't, the script will ask for it...

- under macOS/Linux:

	```sh
	macOS@user scripts %./update_firmware.sh 45
	```

- under Windows:

	```batch
	C:\Users\toto\Develop\GIT\Monkey\nrf-monkey\scripts>.\update_firmware.bat 45
	
	```

... and that's it !