# device-app

Application for LoRa bike device based on STM32 board, together with NEO-7N GPS module.

## Function

The app sends the following payload to the gateway, depending on the device running mode:
1. UnLock flag (when the bike is unlocked)
2. GPS data (when the bike keeps moving)
3. Lock flag (when the bike is locked)

## Todo list
1. Documentation for the app needs to be provided.
2. A way to trigger an interruption for the shifting between bike running modes is needed.
3. Currently the project is based on Keil IDE on Windows, which needs to be reproduced purely on CMake for system portability.
