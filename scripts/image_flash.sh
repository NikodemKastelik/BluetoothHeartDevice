#!/bin/bash
cd "$(dirname "$0")"

SDK_DIR=../../../..
APP_DIR=..
OUTPUT_DIR=../output
mkdir -p ${OUTPUT_DIR}

# Generate bootloader settings for the specified application, so after flashing the complete image
# bootloader will treat the specified application as valid.
nrfutil settings generate \
    --family NRF52 \
    --application ${APP_DIR}/pca10040/s132/ses/Output/Release/Exe/ble_custom_service_pca10040_s132.hex \
    --application-version 0 \
    --bootloader-version 0 \
    --bl-settings-version 2 \
    ${OUTPUT_DIR}/bootloader_settings.hex

if [ ! $? -eq 0 ];
then
    echo Bootloader settings generation failed.
    exit
fi

# Merge together bootloader settings, application and SoftDevice into single .hex
mergehex --merge \
    ${OUTPUT_DIR}/bootloader_settings.hex \
    ${APP_DIR}/pca10040/s132/ses/Output/Release/Exe/ble_custom_service_pca10040_s132.hex \
    ${SDK_DIR}/components/softdevice/s132/hex/s132_nrf52_7.0.1_softdevice.hex \
    --output ${OUTPUT_DIR}/bootloader_settings_s132_application.hex

if [ ! $? -eq 0 ];
then
    echo Creation of combined .hex file failed.
    exit
fi

# Flash bootloader first. Erase whole memory to make sure no leftovers are preserved on the target device.
nrfjprog.exe \
    --program ${SDK_DIR}/examples/dfu/secure_bootloader/pca10040_s132_ble/ses/Output/Release/Exe/secure_bootloader_ble_s132_pca10040.hex \
    --chiperase

if [ ! $? -eq 0 ];
then
    echo Bootloader flashing failed.
    exit
fi

# Flash combined .hex file. Memory erase is not needed as it was done by previous `nrfjprog` call.
# Bootloader should threat the application as valid (because of provided bootloader settings) and boot it immediately.
nrfjprog.exe \
    --program ${OUTPUT_DIR}/bootloader_settings_s132_application.hex \
    --reset

if [ ! $? -eq 0 ];
then
    echo Combined .hex file flashing failed.
    exit
fi
