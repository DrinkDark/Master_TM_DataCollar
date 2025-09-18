#! /bin/bash

source ncs_v2.4.0_env.sh

echo
echo " ========================================================================"
echo " | Script to build firmware for a specific monkey device ...            |"
echo " | -------------------------------------------------------------------- |"
echo " |                                                                      |"
echo " | Just enter the identifier of the Speak No Evil collar and the script |"
echo " | will build the 'merged_domains.hex' file and save it with the name:  |"
echo " | - speak_no_evil-v1.X.Y_<collar_identifier>.hex                       |"
echo " |                                                                      |"
echo " | -------------------------------------------------------------------- |"
echo " | version: 0.1.1                                                       |"
echo " ========================================================================"
echo
read -p "Enter the first device identifier: " first_id
read -p "Enter the number of firmware to build: " num_id

last_id=$((first_id + num_id - 1))
iter_id=$first_id
echo "Building firmware for Speak No Evil from #"$first_id" to #"$last_id" ..."
echo "Number of iteration: "$num_id
echo "Iteration will starts with "$iter_id
echo

current_dir=$(pwd)
echo 
echo "working directory: "$current_dir
echo

for (( i=0 ; i<$num_id ; i++ )) 
do
	echo "Building merged_domains.hex for Speak No Evil #"$iter_id" ..."
	sed -i '' -E 's/Evil\ [0-9]+/Evil\ '$iter_id'/' prj.conf

	west build --build-dir $current_dir/nrf5340_ns $current_dir

	echo 
	echo "Copying merged_domains.hex to /hex/speak_no_evil-v1.0.2_"$iter_id".hex ..." 
	cp $current_dir/nrf5340_ns/zephyr/merged_domains.hex $current_dir/hex/speak_no_evil-v1.0.2_$iter_id.hex
	echo "Done !"
	echo
	((iter_id++))
done	


echo "Asta la vista, baby !"
echo
