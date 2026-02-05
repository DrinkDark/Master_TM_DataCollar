#! /bin/bash

source ncs_v2.4.2_env.sh

echo
echo " ========================================================================"
echo " | Script to flash firmware in a monkey device ...                      |"
echo " | -------------------------------------------------------------------- |"
echo " |                                                                      |"
echo " | Just enter the identifier of the Speak No Evil collar and the script |"
echo " | will flash the firmware in the device, which MUST be connected to    |"
echo " | the PC using a SEGGER J-Link programmer/debugger.                    |"
echo " | It will then store in FLASH the device identifier                    |"
echo " |                                                                      |"
echo " | -------------------------------------------------------------------- |"
echo " | version: 0.2.1                                                       |"
echo " ========================================================================"
echo

args_len=$#

if [ $args_len -gt 1 ]
then
	device_id=$1
	mic_gain=$2
elif [ $args_len -gt 0 ]
then
	device_id=$1
	read -p "Enter Mic Input Gain [1..5]: " mic_gain
else
    read -p "Enter the first device identifier: " device_id
	read -p "Enter Mic Input Gain [1..5]: " mic_gain
fi

current_dir=$(pwd)/..

echo 
echo "working directory: "$current_dir
echo "Device identifier: $device_id ..."
echo "Mic Input Gain: $mic_gain ..."
echo

if ! [ -f $current_dir/nrf5340_ns/zephyr/merged_domains.hex ]; then
	echo "No .hex file found !"
	echo "building the project again ..."
	echo

	west build --build-dir $current_dir/nrf5340_ns/ $current_dir
	echo
fi


if [ -f $current_dir/nrf5340_ns/zephyr/merged_domains.hex ]; then
	echo "Recovering nRF5340 Network Core ..."
	nrfjprog --coprocessor CP_NETWORK --recover
	sleep 3
	echo

	echo "Recovering nRF5340 Application Core ..."
	nrfjprog --recover
	sleep 2
	echo

	west flash -d $current_dir/nrf5340_ns/ --skip-rebuild --erase

	# Store device ID and reset low batt detection counter
	# Setup the FLASH Partition to be handled by NVS module at runtime.
	echo
	echo "Setup FLASH memory..."
	echo "-> Writing device ID $device_id ..."

	res=255
	while [ $res -ne 0 ]
	do
		nrfjprog --memwr 0x000fe000 --val $device_id
		res=$?
		sleep 2
	done

	echo "-> Writing Mic Input Gain $mic_gain ..."
	res=255
	while [ $res -ne 0 ]
	do
		nrfjprog --memwr 0x000fe004 --val $mic_gain
		res=$?
		sleep 2
	done
	echo Done !
	echo

	echo "-> Reset Low Batt Detection counter ..."
	res=255
	while [ $res -ne 0 ]
	do
		nrfjprog --memwr 0x000ff000 --val 0x00
		res=$?
		sleep 2
	done
	echo Done !
	echo


	# +++++++++++++++++++++++++++++++ If using nvs API +++++++++++++++++++++++++++++++ 
	# res=255 
	# while [ $res -ne 0 ]
	# do
	# 	nrfjprog --memwr 0x000fe004 --val 0x00
	# 	res=$?
	# 	sleep 4
	# done

	# res=255 
	# while [ $res -ne 0 ]
	# do
	# 	nrfjprog --memwr 0x000fefe0 --val 0x00040001
	# 	res=$?
	# 	sleep 4
	# done

	# res=255 
	# while [ $res -ne 0 ]
	# do
	# 	nrfjprog --memwr 0x000fefe4 --val 0x4cff0004
	# 	res=$?
	# 	sleep 4
	# done

	# res=255 
	# while [ $res -ne 0 ]
	# do
	# 	nrfjprog --memwr 0x000fefe8 --val 0x00
	# 	res=$?
	# 	sleep 4
	# done

	# res=255 
	# while [ $res -ne 0 ]
	# do
	# 	nrfjprog --memwr 0x000fefec --val 0x1cff0004
	# 	res=$?
	# 	sleep 4
	# done

	# res=255 
	# while [ $res -ne 0 ]
	# do
	# 	nrfjprog --memwr 0x000feff0 --val 0x0000ffff
	# 	res=$?
	# 	sleep 4
	# done

	# res=255 
	# while [ $res -ne 0 ]
	# do
	# 	nrfjprog --memwr 0x000feff4 --val 0xaf000000
	# 	res=$?
	# 	sleep 4
	# done
	# nrfjprog --memrd 0x000fe000 --n 4096
	echo "Your're done !"
else 
	echo You should consider building the prject once again within VSCode...
fi

echo