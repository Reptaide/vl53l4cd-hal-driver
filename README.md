## :package: VL53L4CD HAL Driver Library
HAL Driver Library for the VL53L4CD Time-of-Flight ranging sensor.

This driver has been developed to be hardware-independent.
This way, the functions present in the `core` can be used as-is on any other microcontroller.
The only functions that need to be adapted are those contained in the `platform` folder, as these are the ones that interface with and depend on the hardware.
Currently, only the `ESP32` implementation is available.

## :bulb: Examples
To get started with the driver, check out the `examples` folder for practical usage references.
This folder contains the following files for each available microcontroller:
- `vl53l4cd_basic.c`
- `vl53l4cd_polling.c`
- `vl53l4cd_interrupt.c`
- `vl53l4cd_multiple_devices.c`

## :books: References

- [VL53L4CD Datasheet](https://www.st.com/resource/en/datasheet/vl53l4cd.pdf)
- [VL53L4CD ULD by STMicroelectronics](https://github.com/stm32duino/VL53L4CD)
- [A guide to using the VL53L4CD ultra lite driver (ULD)](https://www.st.com/resource/en/user_manual/um2931-a-guide-to-using-the-vl53l4cd-ultra-lite-driver-uld-stmicroelectronics.pdf)

## :heavy_exclamation_mark: Disclaimer
This software is provided **"AS IS"**, without warranty of any kind.
The author(s) accept no responsibility for any damage to hardware, data loss, device malfunction, or any other direct or indirect consequences arising from the use or misuse of this code.

**Use at your own risk.** It is your sole responsibility to ensure that this software is appropriate and safe for your specific hardware and use case.

## :scroll: License
This project is released under the [BSD 3-Clause License](LICENSE).

The core driver in `driver/core/` is derived from the official VL53L4CD Ultra Light Driver (ULD) by [STMicroelectronics](https://github.com/stm32duino/VL53L4CD), Copyright © 2021 STMicroelectronics, redistributed under the same license.
The rest of the repository (platform layer, examples and application code) is Copyright © 2026 Reptaide.
