@echo off

cls
call ncs_env.cmd

echo.
echo ^ ========================================================================
echo ^ ^| Script to flash firmware in monkey device ...                        ^|
echo ^ ^| -------------------------------------------------------------------- ^|
echo ^ ^|                                                                      ^|
echo ^ ^| Just enter the identifier of the Speak N oEvil collar and the script ^|
echo ^ ^| will flash the firmware in the device, which MUST be connected to    ^|
echo ^ ^| the PC using a SEGGER J-Link programmer/debugger.                    ^|
echo ^ ^|  It will then store in FLASH the device's identifier.                ^|
echo ^ ^|                                                                      ^|
echo ^ ^| -------------------------------------------------------------------- ^|
echo ^ ^| version: 0.2.1                                                       ^|
echo ^ ========================================================================
echo.

set args_len=0
for %%x in (%*) do set /A args_len+=1

if %args_len% GTR 1 (
	set /A device_id=%1
	set /A mic_gain=%2
) else if %args_len% NEQ 0 (
	set /A device_id=%1
	set /p mic_gain=Enter the Mic INput Gain ^[1..5^]:^ 
) else (
	set /p device_id=Enter the device identifier:^ 
	set /p mic_gain=Enter the Mic INput Gain ^[1..5^]:^ 
)

set current_dir=%cd%\..
echo.
echo working directory: %current_dir%
echo Device identifier: %device_id% ...
echo Mic Input Gain: %mic_gain% ...
echo.

if NOT exist %current_dir%\nrf5340_ns\zephyr\merged_domains.hex (
	echo No .hex file found !
	echo building the project again ...
	echo.

	call west build --build-dir %current_dir%\nrf5340_ns\ %current_dir%
	echo.
)

if exist %current_dir%\nrf5340_ns\zephyr\merged_domains.hex (
	echo Recovering nRF5340 Network Core ...
	call nrfjprog --coprocessor CP_NETWORK --recover
	timeout /t 3 /nobreak > NUL
	echo.

	echo Recovering nRF5340 Application Core ...
	call nrfjprog --recover
	timeout /t 4 /nobreak > NUL
	echo. 

	call west flash -d  %current_dir%\nrf5340_ns\ --skip-rebuild --erase

	REM Store device ID and reset low batt detection counter
	REM Setup the FLASH Partition to be handled by NVS module at runtime.
	echo.
	echo Setup FLASH memory...
	echo -^> Writing device ID %device_id% ...
	call nrfjprog --memwr 0x000fe000 --val %device_id% > NUL
	call nrfjprog --memrd 0x000fe000
	echo.
	echo -^> Writing device ID %device_id% ...
	call nrfjprog --memwr 0x000fe004 --val %mic_gain% > NUL
	call nrfjprog --memrd 0x000fe004
	echo.
	echo -^> Reset Low Batt Detection counter ...
	call nrfjprog --memwr 0x000ff000 --val 0x00 > NUL
	call nrfjprog --memrd 0x000ff000
	echo.
	echo You are done !
) else (
	echo You should consider building the prject once again within VSCode...
)

echo.
