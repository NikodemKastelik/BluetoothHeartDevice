@echo off
cd /D "%~dp0"

set SDK_DIR=..\..\..\..
set APP_DIR=..

:: Build SES project in Release configuration.
emBuild -config "Release" %APP_DIR%\pca10040\s132\ses\ble_custom_service_pca10040_s132.emProject
