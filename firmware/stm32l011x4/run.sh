# 	Line 3164: <36>[  188.456125][07-04 09:02:32] type=1400 audit(1530666144.154:28467): avc: denied { read } for pid=125 comm="kworker/0:1" name="DirectCharger.bin" dev="dm-3" ino=12433 scontext=u:r:kernel:s0 tcontext=u:object_r:system_data_file:s0 tclass=file permissive=0

setenforce 0
echo 1 > /d/stm32l011/fw_update
sleep 1
cat /dev/kmsg |grep -i -e "dump_burn_wrap"  -e "mcu_fw"  -e "debug_fw_update"  -e "dump_reply_wrap" -e "fw_check"

