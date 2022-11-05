@echo off
adb vivoroot

ping 127.0.0.1 -n 2 > nul
adb wait-for-devices

adb push run.sh /data/
adb push DirectCharger.bin /data/

adb shell chcon u:object_r:persist_file:s0 /data/DirectCharger.bin
adb shell chmod 777 /data/run.sh
adb shell setsid ./data/run.sh &
pause
