#!/bin/bash

export C_INCLUDE_PATH=/opt/st/stm32cubeide_1.13.1/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.11.3.rel1.linux64_1.1.0.202305231506/tools/arm-none-eabi/include
export PATH="$PATH:/opt/st/stm32cubeide_1.13.1/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.11.3.rel1.linux64_1.1.0.202305231506/tools/bin" # /arm-none-eabi-gcc

if [[ $1 == "clean" ]]; then
    echo "Cleaning";
    make -j8 clean -C ./Debug;
    make -j8 clean -C ./Release;
    rm -rf compile_commands.json;
elif [[ $1 == "release" ]]; then
    echo "Building release";
    rm -rf compile_commands.json;
    make -j8 clean -C ./Release;
    bear -- make -j8 all -C ./Release;
    /opt/st/stm32cubeide_1.13.1/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.1.0.202305091550/tools/bin/STM32_Programmer_CLI -c port=swd -w ./Release/CHIP-8-STM32.elf -rst;
elif [[ $1 == "debug" ]]; then
    echo "Building debug";
    rm -rf compile_commands.json;
    make -j8 clean -C ./Debug;
    bear -- make -j8 all -C ./Debug;
    /opt/st/stm32cubeide_1.13.1/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_2.1.0.202305091550/tools/bin/STM32_Programmer_CLI -c port=swd -w ./Debug/CHIP-8-STM32.elf -rst;
fi
