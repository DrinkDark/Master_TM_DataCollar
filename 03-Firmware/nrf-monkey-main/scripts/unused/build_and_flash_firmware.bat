@echo off

cls

call ncs_env.cmd

echo.
echo ^ ========================================================================
echo ^ ^| Script to build and flash firmware for a specific monkey device ...  ^|
echo ^ ^| -------------------------------------------------------------------- ^|
echo ^ ^|                                                                      ^|
echo ^ ^| Just enter the identifier of the Speak No Evil collar and the script ^|
echo ^ ^| will build the 'merged_domains.hex' file and flash it to the device, ^|
echo ^ ^| which MUST be connected to the PC using a SEGGER J-Link programmer.  ^|
echo ^ ^|                                                                      ^|
echo ^ ^| -------------------------------------------------------------------- ^|
echo ^ ^| version: 0.1.0                                                       ^|
echo ^ ========================================================================
echo.
set /p device_id=Enter the device identifier:^ 

echo Building firmware for Speak No Evil #%device_id% ...

set current_dir=%cd%
echo working directory: %current_dir%
echo.

rem Modifying the prj.conf to update the CONFIG_BT_DEVICE_NAME
call .\JREPL.BAT "Evil\ [0-9]*" "Evil %device_id%" /F prj.conf /O prj_temp.conf
del prj.conf
ren prj_temp.conf prj.conf

rem building the merged_domains.hex
call west build --build-dir %current_dir%\build %current_dir%

rem flashing .hex file in device
call west flash -d %current_dir%\build\ --skip-rebuild --erase

echo.
echo "Asta la vista, baby !"
echo.