#!/bin/bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program /mnt/c/Users/evanl/STM32CubeIDE/goofing/arm/Release/arm.elf verify reset exit"