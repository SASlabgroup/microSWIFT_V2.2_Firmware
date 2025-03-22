#!/bin/zsh

cp /Users/philbush/STM32CubeIDE/microSWIFT/microSWIFT_V2.2/microSWIFT_V2.2/Debug/microSWIFT_V2.2.elf /Users/philbush/STM32CubeIDE/microSWIFT/microSWIFT_Configurator/firmware

cd /Users/philbush/STM32CubeIDE/microSWIFT/microSWIFT_Configurator/
python3 microSWIFT_programmer.py