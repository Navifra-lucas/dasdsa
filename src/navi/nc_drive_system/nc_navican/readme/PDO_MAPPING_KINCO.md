# NaviCAN 모터 초기화 및 PDO MAPPING 설명서

## 개요
NaviCAN으로 PDO Mapping 과정을 설명합니다.

## CANopen 기본 개념

### CAN ID 구조
- **NMT(Network Management)**: 0x000
- **SYNC**: 0x080
- **Emergency**: 0x081-0x0FF - 비상 메시지
- **PDO(Process Data Object)**: 실시간 데이터 전송
  - TPDO1: 0x180+NodeID (T:송신)
  - RPDO1: 0x200+NodeID (R:수신)
  - TPDO2: 0x280+NodeID
  - RPDO2: 0x300+NodeID
  - TPDO3: 0x380+NodeID
  - RPDO3: 0x400+NodeID
  - TPDO4: 0x480+NodeID
  - RPDO4: 0x500+NodeID  
- **SDO(Service Data Object)**: 파라미터 설정
  - SDO TX: 0x580+NodeID (모터 드라이버 → 마스터)
  - SDO RX: 0x600+NodeID (마스터 → 모터 드라이버)
- **NMT Error Control**: 0x700+NodeID - Heartbeat / Node Guarding

### SDO 명령 코드 (첫번째 바이트)
- `0x2F`: 1바이트 쓰기
- `0x2B`: 2바이트 쓰기
- `0x23`: 4바이트 쓰기
- `0x40`: 읽기 요청
- `0x43`: 1바이트 읽기 응답
- `0x60`: 쓰기 응답 (성공)

## 초기화 과정 설명

### 1단계: 초기 통신 및 장치 정보 확인

```
can0  764   [1]  00                    # Master 시작
can0  000   [2]  82 00                 # NMT: 모든 노드 Pre-operational 상태로 전환
can0  702   [1]  00                    # Node 2 하트비트 (부팅 메시지)
can0  701   [1]  00                    # Node 1 하트비트 (부팅 메시지)
```

### 2단계: DeviceType 확인 (Object 0x1000)

```
can0  602   [8]  40 00 10 00 00 00 00 00  # Node 2: DeviceType 읽기
can0  601   [8]  40 00 10 00 00 00 00 00  # Node 1: DeviceType 읽기
can0  582   [8]  43 00 10 00 92 01 02 00  # Node 2: DeviceType - 0x00020192
can0  581   [8]  43 00 10 00 92 01 02 00  # Node 1: DeviceType - 0x00020192
```

### 3단계: Vendor Id, Product Code 확인 (Object 0x1018:01, Object 0x1018:02)

```
can0  602   [8]  40 18 10 01 00 00 00 00  # Node 2: Vendor Id 읽기
can0  601   [8]  40 18 10 01 00 00 00 00  # Node 1: Vendor Id 읽기
can0  582   [8]  43 18 10 01 00 03 00 00  # Node 2: Vendor Id = 0x00000300
can0  581   [8]  43 18 10 01 00 03 00 00  # Node 1: Vendor Id = 0x00000300

can0  602   [8]  40 18 10 02 00 00 00 00  # Node 2: Product Code 읽기
can0  601   [8]  40 18 10 02 00 00 00 00  # Node 1: Product Code 읽기
can0  582   [8]  43 18 10 02 44 46 00 00  # Node 2: Product Code = 0x00004644
can0  581   [8]  43 18 10 02 44 46 00 00  # Node 1: Product Code = 0x00004644
```

### 4단계: Consumer Heartbeat 설정 (Object 0x1016) (Optional)

```
can0  602   [8]  23 16 10 01 58 02 64 00  # Node 2: Consumer Heartbeat 설정
can0  601   [8]  23 16 10 01 58 02 64 00  # Node 1: Consumer Heartbeat 설정
                    ^^^^^^^^ Object 0x1016, Sub 01
                                  ^^^^^^ Node Id = 0x0064(100)
                             ^^^^^ Time = 0x0258(600) ms 
can0  582   [8]  60 16 10 01 58 02 64 00  # Node 2: 설정 완료
can0  581   [8]  60 16 10 01 58 02 64 00  # Node 1: 설정 완료
```

### 5단계: RPDO1 (Receive PDO 1) 매핑 - COB-ID 0x200+NodeID

#### 5.1 RPDO1 통신 파라미터 (0x1400)

```
can0  602   [8]  23 00 14 01 02 02 00 80  # Node 2: RPDO1 COB-ID 설정 (비활성화)
can0  601   [8]  23 00 14 01 01 02 00 80  # Node 1: RPDO1 COB-ID 설정 (비활성화)
                    ^^^^^^^^ Object 0x1400, Sub 01
                             ^^^^^^^^^^^ 0x80000202(Node2), 0x80000201(Node1)
                                      ^^ 0x80 = PDO 비활성화 (매핑 변경을 위해)

can0  602   [8]  2F 00 14 02 01 00 00 00  # Node 2: 전송 타입 = 1 (동기)
can0  601   [8]  2F 00 14 02 01 00 00 00  # Node 1: 전송 타입 = 1 (동기)
```

#### 5.2 RPDO1 매핑 파라미터 (0x1600)

```
can0  602   [8]  2F 00 16 00 00 00 00 00  # Node 2: 매핑 개수 = 0 (매핑 시작)
can0  601   [8]  2F 00 16 00 00 00 00 00  # Node 1: 매핑 개수 = 0 (매핑 시작)

can0  602   [8]  23 00 16 01 10 00 40 60  # Node 2: 첫 번째 매핑
can0  601   [8]  23 00 16 01 10 00 40 60  # Node 1: 첫 번째 매핑
                          ^^^^^^^^^^^^^^ 0x60400010
                                   ^^^^^ Object 0x6040 (Control Word)
                                ^^ Sub-index 0x00
                             ^^ 16비트 (0x10)

can0  602   [8]  23 00 16 02 08 00 60 60  # Node 2: 두 번째 매핑
can0  601   [8]  23 00 16 02 08 00 60 60  # Node 1: 두 번째 매핑
                             ^^^^^^^^^^^ 0x60600008
                                   ^^^^^ Object 0x6060 (Mode of Operation)
                                ^^ Sub-index 0x00
                             ^^ 8비트 (0x08)

can0  602   [8]  2F 00 16 00 02 00 00 00  # Node 2: 매핑 개수 = 2 (매핑 완료)
can0  601   [8]  2F 00 16 00 02 00 00 00  # Node 1: 매핑 개수 = 2 (매핑 완료)
```

#### 5.3 RPDO1 활성화

```
can0  602   [8]  23 00 14 01 02 02 00 00  # Node 2: RPDO1 활성화 (0x00000202)
can0  601   [8]  23 00 14 01 01 02 00 00  # Node 1: RPDO1 활성화 (0x00000201)
```

### 6단계: RPDO2 (Receive PDO 2) 매핑 - COB-ID 0x300+NodeID

#### 6.1 RPDO2 통신 파라미터 (0x1401)

```
can0  602   [8]  23 01 14 01 02 03 00 80  # Node 2: RPDO2 COB-ID 설정 (비활성화)
can0  601   [8]  23 01 14 01 01 03 00 80  # Node 1: RPDO2 COB-ID 설정 (비활성화)
                    ^^^^^^^^ Object 0x1401, Sub 01
                             ^^^^^^^^^^^ 0x80000302(Node2), 0x80000301(Node1)

can0  602   [8]  2F 01 14 02 01 00 00 00  # Node 2: 전송 타입 = 1 (동기)
can0  601   [8]  2F 01 14 02 01 00 00 00  # Node 1: 전송 타입 = 1 (동기)
```

#### 6.2 RPDO2 매핑 파라미터 (0x1601)

```
can0  602   [8]  2F 01 16 00 00 00 00 00  # Node 2: 매핑 개수 = 0
can0  601   [8]  2F 01 16 00 00 00 00 00  # Node 1: 매핑 개수 = 0

can0  602   [8]  23 01 16 01 20 00 FF 60  # Node 2: 첫 번째 매핑
can0  601   [8]  23 01 16 01 20 00 FF 60  # Node 1: 첫 번째 매핑
                             ^^^^^^^^^^^ 0x60FF0020
                                   ^^^^^ Object 0x60FF (Target Velocity)
                                ^^ Sub-index 0x00
                             ^^ 32비트 (0x20)

can0  602   [8]  23 01 16 02 20 00 7A 60  # Node 2: 두 번째 매핑
can0  601   [8]  23 01 16 02 20 00 7A 60  # Node 1: 두 번째 매핑
                          ^^^^^^^^^^^^^^ 0x607A0020
                                    ^^^^ Object 0x607A (Target Position)
                                ^^ Sub-index 0x00
                             ^^ 32비트 (0x20)

can0  602   [8]  2F 01 16 00 02 00 00 00  # Node 2: 매핑 개수 = 2
can0  601   [8]  2F 01 16 00 02 00 00 00  # Node 1: 매핑 개수 = 2
```

#### 6.3 RPDO2 활성화

```
can0  602   [8]  23 01 14 01 02 03 00 00  # Node 2: RPDO2 활성화 (0x00000302)
can0  601   [8]  23 01 14 01 01 03 00 00  # Node 1: RPDO2 활성화 (0x00000301)
```

### 7단계: TPDO1 (Transmit PDO 1) 매핑 - COB-ID 0x180+NodeID

#### 7.1 TPDO1 통신 파라미터 (0x1800)

```
can0  602   [8]  23 00 18 01 82 01 00 80  # Node 2: TPDO1 COB-ID 설정 (비활성화)
can0  601   [8]  23 00 18 01 81 01 00 80  # Node 1: TPDO1 COB-ID 설정 (비활성화)
                             ^^^^^^^^^^^ 0x80000182(Node2), 0x80000181(Node1)

can0  602   [8]  2F 00 18 02 01 00 00 00  # Node 2: 전송 타입 = 1 (동기)
can0  601   [8]  2F 00 18 02 01 00 00 00  # Node 1: 전송 타입 = 1 (동기)
```

#### 7.2 TPDO1 매핑 파라미터 (0x1A00)

```
can0  602   [8]  2F 00 1A 00 00 00 00 00  # Node 2: 매핑 개수 = 0
can0  601   [8]  2F 00 1A 00 00 00 00 00  # Node 1: 매핑 개수 = 0

can0  602   [8]  23 00 1A 01 10 00 41 60  # Node 2: 첫 번째 매핑
can0  601   [8]  23 00 1A 01 10 00 41 60  # Node 1: 첫 번째 매핑
                             ^^^^^^^^^^^ 0x60410010
                                    ^^^^ Object 0x6041 (Status Word)
                                ^^ Sub-index 0x00
                             ^^ 16비트 (0x10)

can0  602   [8]  23 00 1A 02 08 00 61 60  # Node 2: 두 번째 매핑
can0  601   [8]  23 00 1A 02 08 00 61 60  # Node 1: 두 번째 매핑
                             ^^^^^^^^^^^ 0x60610008
                                    ^^^^ Object 0x6061 (Mode of Operation Display)
                                ^^ Sub-index 0x00
                             ^^ 8비트 (0x08)

can0  602   [8]  2F 00 1A 00 02 00 00 00  # Node 2: 매핑 개수 = 2
can0  601   [8]  2F 00 1A 00 02 00 00 00  # Node 1: 매핑 개수 = 2
```

#### 7.3 TPDO1 활성화

```
can0  602   [8]  23 00 18 01 82 01 00 00  # Node 2: TPDO1 활성화 (0x00000182)
can0  601   [8]  23 00 18 01 81 01 00 00  # Node 1: TPDO1 활성화 (0x00000181)
```

### 8단계: TPDO2 (Transmit PDO 2) 매핑 - COB-ID 0x280+NodeID

#### 8.1 TPDO2 통신 파라미터 (0x1801)

```
can0  602   [8]  23 01 18 01 82 02 00 80  # Node 2: TPDO2 COB-ID 설정 (비활성화)
can0  601   [8]  23 01 18 01 81 02 00 80  # Node 1: TPDO2 COB-ID 설정 (비활성화)
                             ^^^^^^^^^^^ 0x80000282(Node2), 0x80000281(Node1)

can0  602   [8]  2F 01 18 02 01 00 00 00  # Node 2: Transmission Type = 1
can0  601   [8]  2F 01 18 02 01 00 00 00  # Node 1: Transmission Type = 1
```

#### 8.2 TPDO2 매핑 파라미터 (0x1A01)

```
can0  602   [8]  2F 01 1A 00 00 00 00 00  # Node 2: 매핑 개수 = 0
can0  601   [8]  2F 01 1A 00 00 00 00 00  # Node 1: 매핑 개수 = 0

can0  602   [8]  23 01 1A 01 20 00 6C 60  # Node 2: 첫 번째 매핑
can0  601   [8]  23 01 1A 01 20 00 6C 60  # Node 1: 첫 번째 매핑
                             ^^^^^^^^^^^ 0x606C0020
                                    ^^^^ Object 0x606C (Actual Velocity)
                                ^^ Sub-index 0x00
                             ^^ 32비트 (0x20)

can0  602   [8]  23 01 1A 02 20 00 64 60  # Node 2: 두 번째 매핑
can0  601   [8]  23 01 1A 02 20 00 64 60  # Node 1: 두 번째 매핑
                             ^^^^^^^^^^^ 0x60640020
                                    ^^^^ Object 0x6064 (Actual Position)
                                ^^ Sub-index 0x00
                             ^^ 32비트 (0x20)

can0  602   [8]  2F 01 1A 00 02 00 00 00  # Node 2: 매핑 개수 = 2
can0  601   [8]  2F 01 1A 00 02 00 00 00  # Node 1: 매핑 개수 = 2
```

#### 8.3 TPDO2 활성화

```
can0  602   [8]  23 01 18 01 82 02 00 00  # Node 2: TPDO2 활성화 (0x00000282)
can0  601   [8]  23 01 18 01 81 02 00 00  # Node 1: TPDO2 활성화 (0x00000281)
```

### 9단계: TPDO3 (Transmit PDO 3) 매핑 - COB-ID 0x380+NodeID

#### 9.1 TPDO3 통신 파라미터 (0x1802)

```
can0  602   [8]  23 02 18 01 82 03 00 80  # Node 2: TPDO3 COB-ID 설정 (비활성화)
can0  601   [8]  23 02 18 01 81 03 00 80  # Node 1: TPDO3 COB-ID 설정 (비활성화)
                             ^^^^^^^^^^^ 0x80000382(Node2), 0x80000381(Node1)

can0  602   [8]  2F 02 18 02 01 00 00 00  # Node 2: Transmission Type = 1
can0  601   [8]  2F 02 18 02 01 00 00 00  # Node 1: Transmission Type = 1
```

#### 9.2 TPDO3 매핑 파라미터 (0x1A02)

```
can0  602   [8]  2F 02 1A 00 00 00 00 00  # Node 2: 매핑 개수 = 0
can0  601   [8]  2F 02 1A 00 00 00 00 00  # Node 1: 매핑 개수 = 0

can0  602   [8]  23 02 1A 01 20 00 79 60  # Node 2: 첫 번째 매핑
can0  601   [8]  23 02 1A 01 20 00 79 60  # Node 1: 첫 번째 매핑
                             ^^^^^^^^^^^ 0x60790020
                                   ^^^^^ Object 0x6079 (DC Link Circuit Voltage)
                                ^^ Sub-index 0x00
                             ^^ 32비트 (0x20)

can0  602   [8]  23 02 1A 02 10 00 78 60  # Node 2: 두 번째 매핑
can0  601   [8]  23 02 1A 02 10 00 78 60  # Node 1: 두 번째 매핑
                             ^^^^^^^^^^^ 0x60780010
                                   ^^^^^ Object 0x6078 (Current Actual Value)
                                ^^ Sub-index 0x00
                             ^^ 16비트 (0x10)

can0  602   [8]  23 02 1A 03 10 00 3F 60  # Node 2: 세 번째 매핑
can0  601   [8]  23 02 1A 03 10 00 3F 60  # Node 1: 세 번째 매핑
                             ^^^^^^^^^^^ 0x603F0010
                                   ^^^^^ Object 0x603F (Error Code)
                                ^^ Sub-index 0x00
                             ^^ 16비트 (0x10)

can0  602   [8]  2F 02 1A 00 03 00 00 00  # Node 2: 매핑 개수 = 3
can0  601   [8]  2F 02 1A 00 03 00 00 00  # Node 1: 매핑 개수 = 3
```

#### 9.3 TPDO3 활성화

```
can0  602   [8]  23 02 18 01 82 03 00 00  # Node 2: TPDO3 활성화 (0x00000382)
can0  601   [8]  23 02 18 01 81 03 00 00  # Node 1: TPDO3 활성화 (0x00000381)
```

### 10단계: CAN 통신 끊어졌을 때 동작 설정 (Optional)

```
can0  602   [8]  2B 07 60 00 01 00 00 00  # Node 2: Abort Connection Mode
can0  601   [8]  2B 07 60 00 01 00 00 00  # Node 1: Abort Connection Mode

can0  601   [8]  2B 5A 60 00 06 00 00 00  # Node 1: Quick Stop Option Code
can0  602   [8]  2B 5A 60 00 06 00 00 00  # Node 2: Quick Stop Option Code

can0  601   [8]  2B 5C 60 00 01 00 00 00  # Node 1: Disable Stop Mode
can0  602   [8]  2B 5C 60 00 01 00 00 00  # Node 2: Disable Stop Mode
```

### 11단계: 노드 시작

```
can0  000   [2]  01 01                    # NMT: Node 1 Operational 상태로 전환
can0  000   [2]  01 02                    # NMT: Node 2 Operational 상태로 전환
```

### 12단계: PDO 통신 시작

```
# TPDO1 - Motor State Feedback (0x181, 0x182)
can0  181   [3]  38 42 00                 # Node 1: Status Word(0x4238) + Mode Display(0x00)
can0  182   [3]  38 42 00                 # Node 2: Status Word(0x4238) + Mode Display(0x00)

# TPDO3 - Motor Extra Info (0x381, 0x382)
can0  381   [8]  5C BB 00 00 01 00 00 81  # Node 1: 전압, 전류, 에러코드
can0  382   [8]  22 BB 00 00 00 00 00 81  # Node 2: 전압, 전류, 에러코드

# TPDO2 - Velocity/Position Feedback (0x281, 0x282)
can0  281   [8]  00 00 00 00 FE FF FF FF  # Node 1: 속도(0) + 위치(-2)
can0  282   [8]  00 00 00 00 01 00 00 00  # Node 2: 속도(0) + 위치(1)

# RPDO1 - Motor State Control (0x201, 0x202)
can0  201   [3]  00 01 00                 # Node 1: Control Word(0x0100) + Mode(0x00)
can0  202   [3]  00 01 00                 # Node 2: Control Word(0x0100) + Mode(0x00)

# RPDO2 - Velocity/Position Target (0x301, 0x302)
can0  301   [8]  00 00 00 00 00 00 00 00  # Node 1: 목표 속도(0) + 목표 위치(0)
can0  302   [8]  00 00 00 00 00 00 00 00  # Node 2: 목표 속도(0) + 목표 위치(0)
```

## PDO 매핑 정보

### RPDO1, TPDO1 (드라이버 제어/상태, 모드)
| PDO | COB-ID | 매핑된 객체 | 설명 |
|-----|--------|------------|------|
| RPDO1 | 0x200+NodeID | 0x6040 (16bit) + 0x6060 (8bit) | Control Word + Mode of Operation |
| TPDO1 | 0x180+NodeID | 0x6041 (16bit) + 0x6061 (8bit) | Status Word + Mode Display |

### RPDO1, TPDO1 (동작 제어)
| PDO | COB-ID | 매핑된 객체 | 설명 |
|-----|--------|------------|------|
| RPDO2 | 0x300+NodeID | 0x60FF (32bit) + 0x607A (32bit) | Target Velocity + Target Position |
| TPDO2 | 0x280+NodeID | 0x606C (32bit) + 0x6064 (32bit) | Actual Velocity + Actual Position |

### TPDO3 (모터 상태, 구동제어 외 정보)
| PDO | COB-ID | 매핑된 객체 | 설명 |
|-----|--------|------------|------|
| TPDO3 | 0x380+NodeID | 0x6079 (32bit) + 0x6078 (16bit) + 0x603F (16bit) | DC Voltage + Current + Error Code |

## 사용하는 Object Dictionary Index

- **0x1000**: Device Type - 장치 타입
- **0x1018**: Identity Object - 제조사 ID, 제품 코드 등
- **0x1400-0x1403**: RPDO 통신 파라미터
- **0x1600-0x1603**: RPDO 매핑 파라미터
- **0x1800-0x1803**: TPDO 통신 파라미터
- **0x1A00-0x1A03**: TPDO 매핑 파라미터
- **0x6040**: Control Word - 드라이버 제어
- **0x6041**: Status Word - 드라이버 상태
- **0x6060**: Mode of Operation - 동작 모드 설정
- **0x6061**: Mode of Operation Display - 현재 동작 모드
- **0x6064**: Position Actual Value - 현재 위치
- **0x606C**: Velocity Actual Value - 현재 속도
- **0x607A**: Target Position - 목표 위치
- **0x60FF**: Target Velocity - 목표 속도

## 참고사항

1. PDO 매핑을 변경하기 전에는 반드시 해당 PDO를 비활성화(0x80000000 OR)해야 합니다.
2. 매핑이 완료된 후에는 PDO를 다시 활성화해야 합니다.
3. Sync 메시지(0x080)는 동기 전송 타입의 PDO를 트리거합니다.
4. 하트비트 메시지는 노드의 상태를 주기적으로 확인하는 데 사용됩니다.
5. 모든 SDO 응답에서 0x60은 성공을 의미합니다.

## NMT 상태 머신

CANopen 노드는 다음과 같은 상태를 가집니다:

- **Initializing**: 부팅 중
- **Pre-operational** (0x80): SDO 통신 가능, PDO 통신 불가
- **Operational** (0x01): SDO와 PDO 모두 통신 가능
- **Stopped** (0x02): 최소한의 통신만 가능

## 문제 해결

### PDO 통신이 안 될 때
1. 노드가 Operational 상태인지 확인
2. PDO 매핑이 올바르게 설정되었는지 확인
3. COB-ID가 활성화되었는지 확인 (0x80000000 비트가 0인지)
4. Sync 메시지가 전송되고 있는지 확인 (동기 전송 모드의 경우)

### SDO 응답 에러 코드
- 0x80: SDO Abort - 에러 발생
- 0x60: 성공적으로 처리됨
- 0x43, 0x47, 0x4B, 0x4F: 읽기 응답 (데이터 크기에 따라 다름)

## 실제 데이터 해석 예시

### TPDO1 데이터 (Status Word)
```
can0  181   [3]  38 42 00
                 ^^^^^ Status Word = 0x4238
                       - Bit 0: Ready to switch on = 0
                       - Bit 1: Switched on = 0  
                       - Bit 2: Operation enabled = 0
                       - Bit 3: Fault = 1
                       - Bit 4: Voltage enabled = 1
                       - Bit 5: Quick stop = 1
                       - Bit 6: Switch on disabled = 0
                       - Bit 9: Remote = 1
                       - Bit 14: Manufacturer specific = 1
                    ^^ Mode of Operation Display = 0x00 (모드 없음)
```

### TPDO2 데이터 (실제 위치/속도)
```
can0  281   [8]  00 00 00 00 FE FF FF FF
                 ^^^^^^^^^^^ Actual Velocity = 0x00000000 (0)
                             ^^^^^^^^^^^ Actual Position = 0xFFFFFFFE (-2)
```

### RPDO1 데이터 (Control Word)
```
can0  201   [3]  00 01 00
                 ^^^^^ Control Word = 0x0100
                       - Bit 8: Halt = 1 (정지 명령)
                       ^^ Mode of Operation = 0x00
```

## 추가 정보

### PDO 전송 타입
- **0x00**: 동기, acyclic (SYNC 수신 시 전송)
- **0x01-0xF0**: 동기, cyclic (매 n번째 SYNC마다 전송)
- **0xFE-0xFF**: 비동기 (이벤트 발생 시 즉시 전송)

### 매핑 데이터 구조
PDO 매핑 파라미터는 32비트로 구성:
- Bit 31-16: Object Index (16비트)
- Bit 15-8: Sub-index (8비트)  
- Bit 7-0: 데이터 크기 (비트 단위)
  - 0x08 = 8비트 (1바이트)
  - 0x10 = 16비트 (2바이트)
  - 0x20 = 32비트 (4바이트)
