# BluetoothHeartDevice

This repository contains source code for heart-shaped electronic device running nRF52832. For the purpose of creation of the application, nRF5 SDK version 16.0.0 has been utilized.

### Main features

Application encompass following features:
- BLE connectivity via nRF _SoftDevice_ v7.0.1
- custom BLE service with characteristics for RGB LED remote control and battery level notifications
- RGB LED control done via WS2812 module (**TBD**)
- LIS3DSH 3-axis MEMS (**TBD**)
- battery level monitoring done via _SAADC_ peripheral (**TBD**)

### How to install

- Download _nRF5 SDK v16.0.0_ and unzip it at desired location.
- Create directory named ```BluetoothHeartDevice``` in ```<nRF5SDK Directory>/examples/ble_peripheral/```
- Copy the repository contents or clone the repository to ```<nRF5SDK Directory>/examples/ble_peripheral/BluetoothHeartDevice/```.
- Open Segger Embedded Studio via ```<BluetoothHeartDevice directory>/pca10040/s132/ses/ble_custom_service_pca10040_s132.emProject``` file.
