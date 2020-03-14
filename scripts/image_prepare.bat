@echo off
cd /D "%~dp0"

set SDK_DIR=..\..\..\..
set APP_DIR=..
set OUTPUT_DIR=..\output
if not exist %OUTPUT_DIR% mkdir %OUTPUT_DIR%

:: Generate application image and sign it using private key.
:: Then the application image can be loaded into target device via BLE DFU.
nrfutil pkg generate ^
    --application %APP_DIR%\pca10040\s132\ses\Output\Release\Exe\ble_custom_service_pca10040_s132.hex ^
    --application-version-string "1.0.0" ^
    --hw-version 52 ^
    --sd-req 0xCB ^
    --key-file %SDK_DIR%\examples\dfu\dfu_private_key.pem ^
    %OUTPUT_DIR%\app_image.zip

if %errorlevel% neq 0 (
    echo Image generation failed.
    pause
    exit /b
)
