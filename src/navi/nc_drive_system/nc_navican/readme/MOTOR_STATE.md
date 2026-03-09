# CANopen 모터 상태 제어

## ControlWord와 StatusWord

- **Control Word(0x6040)**: 모터에게 보내는 명령어 (target)
- **Status Word(0x6041)**: 모터가 알려주는 현재 상태 (feedback)

## 📋 Control Word

| 비트 | 이름 | 설명 | 0일 때 | 1일 때 |
|------|------|------|--------|--------|
| **0** | **Switch On** | 스위치 온/오프 제어 | 스위치 오프 | 스위치 온 |
| **1** | **Enable Voltage** | 전압 활성화 제어 | 전압 비활성화 | 전압 활성화 |
| **2** | **Quick Stop** | 빠른 정지 제어 | 빠른 정지 활성화 | 정상 동작 |
| **3** | **Enable Operation** | 동작 허용 제어 | 동작 금지 | 동작 허용 |
| **4** | **New Set-point** | 새로운 목표값 | 이전 목표값 | 새로운 목표값 |
| **5** | **Change Set Immediately** | 즉시 변경 | 램프 적용 | 즉시 적용 |
| **6** | **Abs/Rel** | 절대/상대 위치 | 상대 위치 | 절대 위치 |
| **7** | **Fault Reset** | 오류 리셋 | 정상 상태 | 오류 리셋 실행 |
| **8** | **Halt** | 정지 명령 | 정상 동작 | 정지 |
| **9** | **Reserved** | 예약됨 | - | - |
| **10** | **Reserved** | 예약됨 | - | - |
| **11** | **Manufacturer Specific** | 제조사별 정의 | 제조사 정의 | 제조사 정의 |
| **12** | **Manufacturer Specific** | 제조사별 정의 | 제조사 정의 | 제조사 정의 |
| **13** | **Manufacturer Specific** | 제조사별 정의 | 제조사 정의 | 제조사 정의 |
| **14** | **Manufacturer Specific** | 제조사별 정의 | 제조사 정의 | 제조사 정의 |
| **15** | **Manufacturer Specific** | 제조사별 정의 | 제조사 정의 | 제조사 정의 |

### Control Word 주요 조합 4 가지

**Ready To Switch On**
```
0x0006 = 0000 0000 0000 0110

b3 = 0:
b2 = 1: Quick Stop
b1 = 1: Enable Voltage
b0 = 0: 
```

**Switch On**
```
0x0007 = 0000 0000 0000 0111

b3 = 0:
b2 = 1: Quick Stop
b1 = 1: Enable Voltage
b0 = 1: Switch On 
```

**Operation Enable**
```
0x000F = 0000 0000 0000 1111

b3 = 1: Operation Enable
b2 = 1: Quick Stop
b1 = 1: Enable Voltage
b0 = 1: Switch On 
```

**Fault Reset**
```
0x0086 = 0000 0000 1000 0110
b7 = 1: Fault Reset()
b6 = 0:
b5 = 0:
b4 = 0:
b3 = 0:
b2 = 1: Quick Stop
b1 = 1: Enable Voltage
b0 = 0: Switch On 

```
## 📊 Status Word

| 비트 | 이름 | 설명 | 0일 때 | 1일 때 |
|------|------|------|--------|--------|
| **0** | **Ready to Switch On** | 스위치 온 준비 상태 | 준비 안됨 | 준비 완료 |
| **1** | **Switched On** | 스위치 온 상태 | 스위치 오프 | 스위치 온 |
| **2** | **Operation Enabled** | 동작 활성화 상태 | 동작 비활성 | 동작 활성 |
| **3** | **Fault** | 오류 상태 | 정상 | 오류 발생 |
| **4** | **Voltage Enabled** | 전압 활성화 상태 | 전압 비활성 | 전압 활성 |
| **5** | **Quick Stop** | 빠른 정지 상태 | 빠른 정지 중 | 정상 |
| **6** | **Switch On Disabled** | 스위치 온 비활성 | 활성화 가능 | 비활성화됨 |
| **7** | **Warning** | 경고 상태 | 정상 | 경고 있음 |
| **8** | **Sync** | 동기화 상태 | 비동기 | 동기 |
| **9** | **Remote** | 원격 제어 상태 | 로컬 제어 | 원격 제어 |
| **10** | **Target Reached** | 목표 도달 | 진행 중 | 목표 도달 |
| **11** | **Internal Limit Active** | 내부 제한 활성 | 제한 없음 | 제한 활성 |
| **12** | **Set-point Ack** | 목표값 확인 | 미확인 | 확인됨 |
| **13** | **Following Error** | 추종 오차 | 정상 | 오차 발생 |
| **14** | **Manufacturer Specific** | 제조사별 정의 | 제조사 정의 | 제조사 정의 |
| **15** | **Manufacturer Specific** | 제조사별 정의 | 제조사 정의 | 제조사 정의 |

### Status Word 주요 조합 4 가지

**Ready to Switch on**
```
0x0031 = 0000 0000 0011 0001 (2진수)

b5 = 1: Quick Stop 준비 (정상 상태)
b4 = 1: Voltae Enabled 
b3 = 0: Fault (정상)
b2 = 0: 
b1 = 0: 
b0 = 1: Ready to Switch On (스위치 온 준비 완료)
```

**Switch On**
```
0x0033 = 0000 0000 0010 0011

b5 = 1: Quick Stop 준비 (정상 상태)
b4 = 1: Voltae Enabled 
b3 = 0: Fault (정상)
b2 = 0: 
b1 = 1: Switched On
b0 = 1: Ready to Switch On
```

**Operation Enable**
```
0x0037 = 0000 0000 0010 0111

b5 = 1: Quick Stop 준비 (정상 상태)
b4 = 1: Voltae Enabled 
b3 = 0: Fault (정상)
b2 = 1: Operation Enabled
b1 = 1: Switched On
b0 = 1: Ready to Switch On
```
**Fault**
```
0x0038 = 0000 0000 0010 1000 (2진수)

b5 = 1: Quick Stop 준비 (정상 상태)
b4 = 1: Voltae Enabled 
b3 = 1: Fault (비정상)
b2 = 0:
b1 = 0:
b0 = 0:
```

## 리눅스에서 cansend 명령어

### 모터 켜기 - cansend 명령어로 테스트

#### 1: Ready to Swtich On
```bash
# Control Word 0x0006을 전송
cansend can0 601#2B4060000600  

# 설명:
# 601: CAN ID (0x600 + Node ID 1)
# 2B: SDO Write 명령
# 40: Object Index 하위 바이트 (0x6040 - Control Word)
# 60: Object Index 상위 바이트  
# 00: Sub Index
# 0600: Control Word 값 (0x0006, 리틀 엔디안)
```

**Status Word 확인하기:**
```bash
# Status Word 읽기
cansend can0 601#404160000000

# 응답을 candump로 확인
candump can0 | grep 581
# 예상: 581#4B4160003100 (Status Word = 0x0031)
```

#### 2: Switch On
```bash
# Control Word 0x0007 전송
cansend can0 601#2B4060000700

# Status Word 확인
# 예상: Status Word = 0x0033
```

#### 3: Operation Enable
```bash
# Control Word 0x000F 전송  
cansend can0 601#2B40600F0000

# Status Word 확인
# 예상: Status Word = 0x0037
```

### Fault Reset

```bash
# Control Word 0x0086 전송 (Fault Reset)
cansend can0 601#2B4060008600
```


## ! 실 수 방 지 !
```
❌ Control Word만 보내고 Status Word 확인 안함
❌ 단계를 건너뛰려고 함 (0x0006 → 바로 0x000F)
❌ Status Word 값을 대충 확인
❌ 오류 발생해도 계속 진행
```

