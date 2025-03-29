#!/bin/zsh

set -x

rm /Users/philbush/STM32CubeIDE/microSWIFT/microSWIFT_Configurator/firmware/microSWIFT_V2.2.elf

if [ "$1" == "d" ]; then
	cp /Users/philbush/STM32CubeIDE/microSWIFT/microSWIFT_V2.2/microSWIFT_V2.2/Debug/microSWIFT_V2.2.elf /Users/philbush/STM32CubeIDE/microSWIFT/microSWIFT_Configurator/firmware

elif [ "$1" == "dd" ]; then
	cp /Users/philbush/STM32CubeIDE/microSWIFT/microSWIFT_V2.2/microSWIFT_V2.2/Debug without debugging/microSWIFT_V2.2.elf /Users/philbush/STM32CubeIDE/microSWIFT/microSWIFT_Configurator/firmware

elif [ "$1" == "r" ]; then
	cp /Users/philbush/STM32CubeIDE/microSWIFT/microSWIFT_V2.2/microSWIFT_V2.2/Release/microSWIFT_V2.2.elf /Users/philbush/STM32CubeIDE/microSWIFT/microSWIFT_Configurator/firmware

else
	echo "Arg not caputred."

fi


cd /Users/philbush/STM32CubeIDE/microSWIFT/microSWIFT_Configurator/
python3 microSWIFT_programmer.py
