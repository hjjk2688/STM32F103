# STM32 RC Car Bluetooth Controller

### 문서 관리 정보
- 문서 ID: RC-CAR-DO178C-001-Rev1
- 버전 & 날짜 : 1.0 / 2025년 9월 19일
- 분류: 설계 보증 레벨 D (DAL-D)
- 시스템: STM32F103 기반 RC카 제어 시스템
- 주요 변경사항: 초기 문서 작성

---

## 1. 소프트웨어 인증 계획서 (PSAC)
#### 1.1 소프트웨어 개요
**소프트웨어 항목**: RC카 블루투스 제어기
**기능**:
- 블루투스(UART)를 통해 수신된 명령어로 RC카 제어
- 전진, 후진, 좌회전, 우회전, 정지 기능 구현
- 초음파 센서를 이용한 전방 장애물 감지 및 자동 정지
**중요도 레벨**: DAL-D (경미한 고장 상황)

**입출력**:
- 입력:
    - USART3: 블루투스 모듈로부터 제어 명령어 수신 (w, a, s, d, p)
    - GPIO (Echo): 초음파 센서로부터 거리 측정 펄스 입력
- 출력:
    - GPIO (Motor Pins): 모터 드라이버에 제어 신호 출력
    - GPIO (Trigger): 초음파 센서에 트리거 신호 출력
    - USART2: 디버깅 메시지 출력

#### 1.2 소프트웨어 생명주기 프로세스
- 계획 수립 프로세스
- 소프트웨어 개발 프로세스
- 소프트웨어 검증 프로세스
- 소프트웨어 형상관리 프로세스
- 소프트웨어 품질보증 프로세스

---

## 2. 소프트웨어 요구사항 표준 (SRS)
#### 2.1 상위레벨 요구사항 (HLR)

**HLR-001**: 시스템 초기화
- 소프트웨어는 시작 시 모든 필수 주변장치(GPIO, UART, TIM)를 초기화해야 한다.

**HLR-002**: 블루투스 명령어 수신
- 소프트웨어는 USART3를 통해 비동기적으로 제어 명령어를 수신해야 한다.

**HLR-003**: 모터 제어
- 소프트웨어는 수신된 명령어에 따라 4개의 모터를 제어하여 RC카를 전진, 후진, 좌회전, 우회전, 정지시켜야 한다.

**HLR-004**: 장애물 감지
- 소프트웨어는 전방의 두 개 초음파 센서를 주기적으로 확인하여 장애물과의 거리를 측정해야 한다.

**HLR-005**: 자동 정지
- 소프트웨어는 전진 중 장애물이 250mm 이내로 감지되면 자동으로 RC카를 정지시켜야 한다.

**HLR-006**: 오류 처리
- 소프트웨어는 HAL 초기화 또는 설정 중 발생하는 오류를 감지하고 `Error_Handler()`를 호출해야 한다.

#### 2.2 저수준 요구사항 (LLR)

**LLR-001**: 주변장치 설정
- `MX_GPIO_Init()`: 모터 제어, 초음파 센서, LED에 사용될 GPIO 핀을 출력 또는 입력으로 설정한다.
- `MX_USART3_UART_Init()`: 블루투스 통신을 위해 9600 Baudrate로 UART를 설정한다.
- `MX_TIM1_Init()`, `MX_TIM2_Init()`: 초음파 센서 거리 측정을 위한 타이머를 1MHz 클럭으로 설정한다.

**LLR-002**: 명령어 수신 처리
- `HAL_UART_RxCpltCallback()`: USART3로부터 1바이트 데이터 수신 시 `rx3_data` 변수에 저장하고 다음 수신을 대기한다.

**LLR-003**: 모터 제어 함수 구현
- `smartcar_forward()`: 모든 모터를 전진 방향으로 회전시킨다.
- `smartcar_back()`: 모든 모터를 후진 방향으로 회전시킨다.
- `smartcar_left()`: 왼쪽 바퀴는 후진, 오른쪽 바퀴는 전진시켜 좌회전한다.
- `smartcar_right()`: 오른쪽 바퀴는 후진, 왼쪽 바퀴는 전진시켜 우회전한다.
- `smartcar_stop()`: 모든 모터의 회전을 멈춘다.

**LLR-004**: 초음파 센서 제어 함수
- `trig()`: 해당 Trigger 핀에 10us 길이의 펄스를 출력한다.
- `echo()`: 해당 Echo 핀에서 HIGH 펄스의 길이를 마이크로초(us) 단위로 측정하여 반환한다. 타임아웃(30000us)을 적용한다.

**LLR-005**: 거리 계산 및 필터링
- `main()` 루프 내에서 500ms 주기로 센서를 읽는다.
- 측정된 `echo_time`을 `(17 * echo_time / 100)` 공식을 사용하여 mm 단위 거리로 변환한다.
- `get_median()`: 5개의 최근 거리 측정값 중 중간값을 반환하여 노이즈를 제거한다.

**LLR-006**: 주행 로직 구현
- `main()` 루프에서 `rx3_data` 변수 값을 확인한다.
- 'w' 수신 시, `obstacle_detected` 플래그가 0이면 `smartcar_forward()` 호출, 1이면 `smartcar_stop()` 호출.
- 'a', 'd', 's', 'p' 수신 시 각각 `smartcar_left()`, `smartcar_right()`, `smartcar_back()`, `smartcar_stop()`을 호출한다.

---

## 3. 소프트웨어 설계 표준 (SDS)
#### 3.1 아키텍처 설계
```
STM32_RC_Car_Controller
├── HAL_Driver (STM32 Hardware Abstraction Layer)
├── Application
│   ├── main()
│   │   ├── System_Initialization
│   │   └── Main_Loop
│   │       ├── Ultrasonic_Sensor_Handler (Non-blocking)
│   │       └── Bluetooth_Command_Processor
│   ├── UART_Callback_Handler
│   │   └── HAL_UART_RxCpltCallback()
│   └── Motor_Control_Module
│       ├── smartcar_forward()
│       ├── smartcar_back()
│       ├── smartcar_left()
│       ├── smartcar_right()
│       └── smartcar_stop()
└── Error_Handler
```

#### 3.2 데이터 구조 설계
- `UART_HandleTypeDef huart3`: 블루투스 통신용 핸들
- `TIM_HandleTypeDef htim1`, `htim2`: 초음파 센서용 타이머 핸들
- `uint8_t rx3_data`: 수신된 블루투스 명령어를 저장하는 변수
- `int right_history[5]`, `left_history[5]`: 센서 값 필터링을 위한 배열

#### 3.3 주요 함수 설계
**핵심 함수**:
- `main()`: 전체 프로세스 제어, 시스템 초기화 및 메인 루프 실행
- `HAL_UART_RxCpltCallback()`: 블루투스(UART) 데이터 수신 처리
- `smartcar_*()`: 전/후/좌/우/정지 모터 제어 함수
- `trig()`, `echo()`: 초음파 센서 제어 및 거리 측정
- `get_median()`: 센서 데이터 안정화를 위한 미디언 필터

**명령어 처리 콜백 함수**:
```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) { // 블루투스 모듈 연결 확인
    // 수신된 데이터를 PC로 전달 (디버깅용)
    HAL_UART_Transmit(&huart2, &rx3_data, 1, 10); 
    // 다음 1바이트를 인터럽트 방식으로 다시 수신 대기
    HAL_UART_Receive_IT(&huart3, &rx3_data, 1);
  }
}
```

---

## 4. 소프트웨어 코드 표준 (SCS)
#### 4.1 코딩 규칙
- **명명 규칙**: STM32 HAL 라이브러리 규칙을 따름 (PascalCase for Types, camelCase for variables, UPPER_CASE for macros). 사용자 정의 함수는 `snake_case` 사용.
- **주석**: 코드의 복잡한 부분을 설명하기 위해 간결한 주석 사용.
- **오류 처리**: `Error_Handler()` 함수를 통해 치명적인 오류 발생 시 무한 루프에 진입하여 시스템을 정지시킴.

---

## 5. 소프트웨어 검증 계획 (SVP)
#### 5.1 검증 목표
- 모든 요구사항의 올바른 구현 확인
- 모터 제어의 정확성 및 반응성 검증
- 장애물 감지 및 자동 정지 기능의 신뢰성 검증
- 블루투스 명령어 처리의 안정성 확인

#### 5.2 상세 테스트 케이스
- **TC-001**: 전진 명령어 ('w')
  - **입력**: 블루투스로 'w' 전송
  - **예상 결과**: RC카가 전진. 장애물 없을 시 계속 전진.
- **TC-002**: 후진 명령어 ('s')
  - **입력**: 블루투스로 's' 전송
  - **예상 결과**: RC카가 후진.
- **TC-003**: 좌회전 명령어 ('a')
  - **입력**: 블루투스로 'a' 전송
  - **예상 결과**: RC카가 좌회전.
- **TC-004**: 우회전 명령어 ('d')
  - **입력**: 블루투스로 'd' 전송
  - **예상 결과**: RC카가 우회전.
- **TC-005**: 정지 명령어 ('p')
  - **입력**: 블루투스로 'p' 전송
  - **예상 결과**: RC카가 정지.
- **TC-006**: 장애물 감지 및 자동 정지
  - **조건**: 전진('w') 명령 수행 중
  - **입력**: RC카 전방 25cm 이내에 장애물 배치
  - **예상 결과**: RC카가 자동으로 정지.
- **TC-007**: 유효하지 않은 명령어
  - **입력**: 'w, a, s, d, p' 이외의 문자 전송
  - **예상 결과**: RC카가 현재 상태를 유지하고 아무 동작도 하지 않음.

---

## 6. 소프트웨어 형상관리 계획 (SCMP)
#### 6.1 형상관리 대상
- **소스 코드**: `Core/Src/main.c`
- **설정 파일**: `RC_Bluetooth.ioc` (STM32CubeMX 설정)
- **문서**: `README.md`

#### 6.2 버전 관리 규칙
- **버전 체계**: MAJOR.MINOR.PATCH
- **현재 버전**: v1.0.0 (초기 개발 완료)

---

## 7. 추적성 매트릭스

| 요구사항 ID | 설계 요소 | 코드 구현 | 테스트 케이스 |
|:---:|:---:|:---:|:---:|
| HLR-001 | System_Initialization | `main()` 내 `MX_*_Init()` 호출 | - |
| HLR-002 | UART_Callback_Handler | `HAL_UART_RxCpltCallback()` | TC-001 ~ TC-007 |
| HLR-003 | Motor_Control_Module | `smartcar_*()` 함수들 | TC-001 ~ TC-005 |
| HLR-004 | Ultrasonic_Sensor_Handler | `trig()`, `echo()`, `get_median()` | TC-006 |
| HLR-005 | Bluetooth_Command_Processor | `main()` 루프 내 자동 정지 로직 | TC-006 |
| HLR-006 | Error_Handler | `Error_Handler()` | - |

---

## 8. 인증 결론

본 STM32 RC카 제어 소프트웨어 v1.0은 DO-178C DAL-D 수준의 요구사항을 충족하도록 개발되었습니다.

**주요 기능**:
1. **블루투스 원격 제어**: 스마트폰 등 외부 기기에서 전송된 명령어로 RC카를 실시간 제어합니다.
2. **장애물 감지 및 회피**: 전방 초음파 센서를 이용해 장애물을 감지하고, 충돌 위험 시 자동으로 정지합니다.
3. **안정적인 센서 데이터 처리**: 미디언 필터를 적용하여 초음파 센서의 측정 오류를 최소화하고 반응 신뢰도를 높였습니다.
4. **모듈식 모터 제어**: 직관적인 함수(`smartcar_forward`, `smartcar_stop` 등)를 통해 모터 동작을 명확하게 제어합니다.

**검증 완료 항목**:
- ✅ HLR-001: 시스템 초기화
- ✅ HLR-002: 블루투스 명령어 수신
- ✅ HLR-003: 모터 제어
- ✅ HLR-004: 장애물 감지
- ✅ HLR-005: 자동 정지
- ✅ HLR-006: 오류 처리

**인증 승인**:
- 개발팀장: [서명 대기]
- 품질보증 관리자: [서명 대기]
- 인증 담당자: [서명 대기]

**최종 승인 날짜**: 2025년 9월 19일

---
*본 문서는 실제 C 소스 코드를 기반으로 작성되었으며, 모든 구현 세부사항이 요구사항과 일치함을 확인하였습니다.*
