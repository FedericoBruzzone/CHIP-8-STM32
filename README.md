# Chip-8 STM32

This repository contains the port of our [Chip-8 emulator](https://github.com/CHIP-8-Org/Core) for the STM32 microcontroller platform. The Chip-8 emulator allows you to run vintage video games originally designed for the CHIP-8, CHIP-48, S-CHIP 1.0 and S-CHIP 1.1 on STM32-based hardware.

## Features

- Emulates Chip-8 virtual machine on STM32 microcontroller.
- Display output on ILI9341 TFT screen.
- Read game files from an SD card for easy game loading.
- Beeper support for audio feedback during gameplay.
- Matrix keypad integration for user input.
- Low power consumption, suitable for battery-powered operation.


## Compilation and Usage

You need to have installed `bear` command in your machine in order to generate the `compile_commands.json`. You can find more information [here](https://github.com/rizsotto/Bear).

```bash
./build.sh [COMMAND]

Command:
    clean   [remove  the content of `./Release` and `./Debug` folders and `compile_commands.json` file]
    release [compile, generate `compile_commands.json` and flash in release mode]
    debug   [compile, generate `compile_commands.json` and flash in debug mode]
```

## Contributing

Contributions to this project are welcome! If you have any suggestions, improvements, or bug fixes, feel free to submit a pull request.

## License

This repository is licensed under the [GNU General Public License (GPL)](https://www.gnu.org/licenses/gpl-3.0.html). Please review the license file provided in the repository for more information regarding the terms and conditions of the GPL license.

## Contact

If you have any questions, suggestions, or feedback, do not hesitate to [contact me](https://federicobruzzone.github.io/).

Maintainers:
  - [FedericoBruzzone](https://github.com/FedericoBruzzone)
  - [Andreal2000](https://github.com/Andreal2000)
