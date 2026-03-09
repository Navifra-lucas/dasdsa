### can interface up
``` bash
sudo ip link set can0 up can bitrate 500000 # 500k bps
sudo ip link set can0 txqueuelen 1000 # 반드시 해야함
```

### slcan 사용하는 USB2CAN 초기화 명령어
``` bash
sudo modprobe slcan # "slcan" 모듈 
sudo slcand -o -c -s6 /dev/ttyACM0 can0 # "slcand" 명령어, 500k bps
sudo slcand -o -c -s7 /dev/ttyACM0 can0 # 1M bps
sudo ip link set can0 up can bitrate 500000 
sudo ip link set can0 txqueuelen 1000
```

### 모든 CAN 메시지 보기
``` bash
candump can0
```

### candump 사용법
``` bash
# Node Id 1의 SDO Request | Response
candump can0 | grep -E "581|601" 

# Node Id 1의 Control Word + Mode of Operation
candump can0 | grep -E "can0 201" 

# Node Id 1의 Status Word + Mode of Operation Display
candump can0 | grep -E "can0 181" 

# Node Id 1의 Target Velocity + Target Position
candump can0 | grep -E "can0 301" 

# Node Id 1의 Actual Velocity + Actual Position
candump can0 | grep -E "can0 281" 

# Node Id 1의 Voltage + Current + Error Code
candump can0 | grep -E "can0 381"  
```

### candump 사용법 (모든노드 TPDO)
``` bash
# 모든 노드의 TPDO1 (181-1F1 범위)
candump can0 | grep -E " (18[1-9A-F]|19[0-9A-F]|1[A-E][0-9A-F]|1F[0-1]) "

# 모든 노드의 TPDO2 (281-2F1 범위)  
candump can0 | grep -E " (28[1-9A-F]|29[0-9A-F]|2[A-E][0-9A-F]|2F[0-1]) "

# 모든 노드의 TPDO3 (381-3F1 범위)
candump can0 | grep -E " (38[1-9A-F]|39[0-9A-F]|3[A-E][0-9A-F]|3F[0-1]) "

# 모든 노드의 TPDO4 (481-4F1 범위)  
candump can0 | grep -E " (48[1-9A-F]|49[0-9A-F]|4[A-E][0-9A-F]|4F[0-1]) "

# 모든 노드의 모든 TPDO
candump can0 | grep -E " ([1-4][8-F][1-9A-F]) "
```

### candump 사용법 (모든노드 RPDO)
``` bash
# 모든 노드의 RPDO1 (201-271 범위)
candump can0 | grep -E " (20[1-9A-F]|21[0-9A-F]|22[0-9A-F]|23[0-9A-F]|24[0-9A-F]|25[0-9A-F]|26[0-9A-F]|27[0-1]) "

# 모든 노드의 RPDO2 (301-371 범위)
candump can0 | grep -E " (30[1-9A-F]|31[0-9A-F]|32[0-9A-F]|33[0-9A-F]|34[0-9A-F]|35[0-9A-F]|36[0-9A-F]|37[0-1]) "

# 모든 노드의 RPDO3 (401-471 범위)
candump can0 | grep -E " (40[1-9A-F]|41[0-9A-F]|42[0-9A-F]|43[0-9A-F]|44[0-9A-F]|45[0-9A-F]|46[0-9A-F]|47[0-1]) "

# 모든 노드의 RPDO4 (501-571 범위)
candump can0 | grep -E " (50[1-9A-F]|51[0-9A-F]|52[0-9A-F]|53[0-9A-F]|54[0-9A-F]|55[0-9A-F]|56[0-9A-F]|57[0-1]) "

# 모든 노드의 모든 RPDO
candump can0 | grep -E " ([2-5][0-7][1-9A-F]) "
```

### candump 사용법 (TPDO + RPDO)

``` bash
# TPDO1 + RPDO1 함께
candump can0 | grep -E " (1[8-9A-F][1-9A-F]|1F[0-1]|2[0-6][1-9A-F]|27[0-1]) "

# TPDO2 + RPDO2 함께  
candump can0 | grep -E " (2[8-9A-F][1-9A-F]|2F[0-1]|3[0-6][1-9A-F]|37[0-1]) "

# TPDO3 + RPDO3 함께
candump can0 | grep -E " (3[8-9A-F][1-9A-F]|3F[0-1]|4[0-6][1-9A-F]|47[0-1]) "

# TPDO4 + RPDO4 함께
candump can0 | grep -E " (4[8-9A-F][1-9A-F]|4F[0-1]|5[0-6][1-9A-F]|57[0-1]) "

# 모든 TPDO + RPDO 함께 
candump can0 | grep -E " [1-5][0-9A-F][1-9A-F] "
```

### 타임스탬프(업타임)와 함께 보기
``` bash
candump -t z can0
```

### 타임스탬프(현재시간)와 함께 보기
``` bash
candump -tA can0
```
