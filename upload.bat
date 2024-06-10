@echo Uploading to %1
title esp32s3 uploader
esptool.exe --chip esp32s3 --port %1 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 40m --flash_size detect 0x0 .\.pio\build\esp32-s3-devkitc-1-n16r8v\bootloader.bin 0x8000 .\.pio\build\esp32-s3-devkitc-1-n16r8v\partitions.bin 0x10000 .\.pio\build\esp32-s3-devkitc-1-n16r8v\firmware.bin