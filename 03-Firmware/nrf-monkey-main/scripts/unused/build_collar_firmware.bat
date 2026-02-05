@echo off

cls
call ncs_env.cmd

echo.
echo ^ ========================================================================
echo ^ ^| Script to build firmware for a specific monkey device ...            ^|
echo ^ ^| -------------------------------------------------------------------- ^|
echo ^ ^|                                                                      ^|
echo ^ ^| Just enter the identifier of the Speak No Evil collar and the script ^|
echo ^ ^| will build the 'merged_domains.hex' file and save it with the name:  ^|
echo ^ ^| - speak_no_evil-v1.0.0_^<collar_identifier^>.hex                       ^|
echo ^ ^|                                                                      ^|
echo ^ ^| -------------------------------------------------------------------- ^|
echo ^ ^| version: 0.1.0                                                       ^|
echo ^ ========================================================================
echo.
set /p first_id=Enter the first device identifier:^ 
set /p num_id=Enter the number of firmware to build:^ 

set /A last_id = %first_id% + %num_id% - 1
echo Building firmware for Speak No Evil from #%first_id% to #%last_id% ...
echo Number of iteration: %num_id%
echo Iteration will starts with %first_id%
echo.

set current_dir=%cd%
echo.
echo working directory: %current_dir%
echo.

rem Looping for all firmware to build
set iter=%first_id%
for /L %%x in (%first_id%,1,%last_id%) do (
	set iter=%%x
	echo Evil %%x
	call .\JREPL.BAT "Evil\ [0-9]*" "Evil %%x" /F prj.conf /O prj_temp.conf

	rem delete prj.conf
	del prj.conf
	ren prj_temp.conf prj.conf

	call west build --build-dir %current_dir%\build %current_dir%

	rem moving merged_domains.hex to hex\speak_no_evil-v1.0.0_xxx.hex
	move %current_dir%\build\zephyr\merged_domains.hex %current_dir%\hex\speak_no_evil-v1.0.0_%%x.hex
)

echo.
echo "Asta la vista, baby !"
echo.