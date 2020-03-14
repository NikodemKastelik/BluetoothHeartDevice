# BluetoothHeartDevice

This repository contains source code for heart-shaped electronic device running nRF52832. For the purpose of creation of the application, nRF5 SDK version 16.0.0 has been utilized.

### Main features

Application encompass following features:
- BLE connectivity via nRF _SoftDevice_ v7.0.1
- custom BLE service with characteristics for RGB LED remote control and battery level notifications
- secure DFU over BLE
- RGB LED control done via WS2812 module (**TBD**)
- LIS3DSH 3-axis MEMS (**TBD**)
- battery level monitoring done via _SAADC_ peripheral (**TBD**)

### How to install

- Download _nRF5 SDK v16.0.0_ and unzip it at desired location.
- Create directory named ```BluetoothHeartDevice``` in ```<nRF5SDK Directory>/examples/ble_peripheral/```
- Copy the repository contents or clone the repository to ```<nRF5SDK Directory>/examples/ble_peripheral/BluetoothHeartDevice/```.
- Open Segger Embedded Studio via ```<BluetoothHeartDevice directory>/pca10040/s132/ses/ble_custom_service_pca10040_s132.emProject``` file.

### How to flash

As this project utilizes secure bootloader, direct flashing of target application is not possible.

#### Prerequisites

Make sure following tools are present and added to ```PATH``` variable:
- ```nrfjprog``` - https://www.nordicsemi.com/Software-and-tools/Development-Tools/nRF-Command-Line-Tools/Download#infotabs
- ```mergehex``` - https://www.nordicsemi.com/Software-and-tools/Development-Tools/nRF-Command-Line-Tools/Download#infotabs
- ```nrfutil``` - https://github.com/NordicSemiconductor/pc-nrfutil/releases

#### Preparations

1. Install ```micro-ecc``` library by downloading the attached ```micro-ecc``` release (accessible under link: https://github.com/NikodemKastelik/BluetoothHeartDevice/releases/tag/v0.1)
and copying ```micro_ecc_lib_nrf52.a``` file to ```<SDK-dir>/external/micro-ecc/nrf52hf_armgcc/armgcc```.
2. Enter ```<SDK-dir>/examples/dfu```.
3. Invoke following command to generate private key - it is needed for signing firmware images:
<br>```nrfutil keys generate dfu_private_key.pem```
4. Invoke following command to generate public key - it will be included in bootloader code to validate images:
<br>```nrfutil keys display --key pk --format code --out_file dfu_public_key.c dfu_private_key.pem```
5. Open ```<SDK-dir>/examples/dfu/secure_bootloader/pca10040_s132_ble/ses/secure_bootloader_ble_s132_pca10040.emProject``` file and compile project.

#### Working with buttonless BLE DFU

During development complete firmware image (that is: bootloader, bootloader settings, _SoftDevice_ & application) needs to be loaded.
It can be done using the ```scripts/image_flash``` script. After programming, application will start immediately.

After device deployment programming port will not be accessible, so OTA DFU needs to be performed.
In order to load new application into memory of the target device via BLE, generate signed image using ```scripts/image_prepare``` script.
Then open ```nRF Connect```, connect to ```Custom_Service``` and use ```Secure DFU``` service to load ```output/app_image.zip``` into device.
