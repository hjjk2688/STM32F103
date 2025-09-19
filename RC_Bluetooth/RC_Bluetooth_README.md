
# STM32F103 주변장치 기능 구현 및 분석

### 문서 관리 정보
- **문서 ID**: STM32-PERIPH-DESIGN-001-Rev1
- **버전 & 날짜**: 1.0 / 2025년 9월 17일
- **분류**: 개인 학습 포트폴리오
- **시스템**: STM32F103 기반 임베디드 시스템 예제
- **주요 변경사항**: 각 주변장치별 소스 코드 분석 기반으로 요구사항, 설계, 검증 항목 상세화. 최종 프로젝트(RC_BLUETOOTH) 포함.
---

## 1. 레포지토리 개요 (Repository Overview)
#### 1.1 소프트웨어 개요
**소프트웨어 항목**: STM32F103 주변장치별 드라이버 및 통합 애플리케이션
**기능**: 
- STM32F103 MCU의 핵심 주변장치(GPIO, UART, EXTI, TIMER, ADC, NVIC) 기능 구현
- 각 주변장치 기능을 통합한 최종 애플리케이션(블루투스 RC카) 제작
- HAL(Hardware Abstraction Layer) 라이브러리 기반 개발

**중요도 레벨**: N/A (개인 학습용)

#### 1.2 소프트웨어 개발 프로세스
- **요구사항 분석**: 각 주변장치의 기본 동작 및 최종 프로젝트의 기능 정의
- **설계**: 각 기능 구현을 위한 HAL 함수 및 로직 흐름 설계
- **구현**: STM32CubeIDE를 사용한 C언어 코딩
- **단위 검증**: 각 주변장치별 프로젝트를 개별적으로 테스트
- **통합 검증**: 모든 기능을 통합한 `RC_BLUETOOTH` 프로젝트 테스트

---
## 2. 소프트웨어 요구사항 표준 (SRS)
#### 2.1 상위레벨 요구사항 (HLR)

**최종 프로젝트: RC_BLUETOOTH**
- **HLR-RC-001**: 시스템은 UART를 통해 수신된 문자('F', 'B', 'L', 'R', 'S')에 따라 지정된 동작을 수행해야 한다.
- **HLR-RC-002**: 시스템은 GPIO를 통해 모터 드라이버의 방향 제어 핀(IN1~IN4)을 제어하여 모터의 정/역회전을 결정해야 한다.
- **HLR-RC-003**: 시스템은 TIMER의 PWM 기능을 사용하여 모터의 속도를 제어해야 한다.

**기본 예제**
- **HLR-GPIO-001**: 시스템은 특정 GPIO 핀의 출력을 1초 주기로 변경하여 LED를 점멸시켜야 한다.
- **HLR-UART-001**: 시스템은 `printf` 출력을 UART로 리다이렉션하여 PC 터미널에 메시지를 표시해야 한다.
- **HLR-EXTI-001**: 시스템은 특정 버튼 입력의 하강 엣지(Falling Edge)를 감지하여 인터럽트를 발생시켜야 한다.
- **HLR-TIM-001**: 시스템은 하드웨어 타이머를 사용하여 1ms 주기의 주기적인 인터럽트를 생성해야 한다.
- **HLR-ADC-001**: 시스템은 아날로그 입력 핀의 전압을 12비트 디지털 값으로 변환할 수 있어야 한다.
- **HLR-NVIC-001**: 시스템은 두 개 이상의 인터럽트 발생 시, 사전에 정의된 우선순위에 따라 인터럽트를 처리해야 한다.

---
## 3. 소프트웨어 설계 표준 (SDS)
#### 3.1 아키텍처 설계 (`RC_BLUETOOTH` 기준)
```
[스마트폰 앱] ---> [블루투스 모듈(HC-06)] ---> [STM32: UART 수신]
                                                     |
                                                     +--- [STM32: GPIO 제어] ---> [모터 드라이버(L298N)] ---> [DC 모터]
                                                     |
                                                     +--- [STM32: TIMER(PWM)] --> [모터 드라이버(L298N)] ---> [DC 모터]
```

#### 3.2 주요 함수 및 HAL 라이브러리 설계
- **GPIO**: `HAL_GPIO_WritePin`, `HAL_GPIO_TogglePin`, `HAL_GPIO_ReadPin`
- **UART**: `HAL_UART_Receive_IT`, `HAL_UART_Transmit`, `_write` (재정의)
- **EXTI**: `HAL_GPIO_EXTI_Callback`
- **TIMER**: `HAL_TIM_Base_Start_IT`, `HAL_TIM_PWM_Start`, `HAL_TIM_PeriodElapsedCallback`
- **ADC**: `HAL_ADC_Start`, `HAL_ADC_PollForConversion`, `HAL_ADC_GetValue`
- **NVIC**: `HAL_NVIC_SetPriority`, `HAL_NVIC_EnableIRQ`

#### 3.3 인터페이스 설계 (`RC_BLUETOOTH` 기준)
- **블루투스/UART 제어 문자 인터페이스**
  - 'F': 전진 (모터 2개 정회전)
  - 'B': 후진 (모터 2개 역회전)
  - 'L': 좌회전 (한쪽 정회전, 다른 쪽 역회전)
  - 'R': 우회전 (한쪽 역회전, 다른 쪽 정회전)
  - 'S': 정지 (모터 2개 정지)

---
## 4. 소프트웨어 검증 계획 (SVP)
#### 4.1 상세 테스트 케이스 (TC)

- **TC-RC-001**: 전진 기능 테스트
  - **입력**: UART로 문자 'F' 수신
  - **예상 결과**: 두 DC 모터가 모두 정회전하며 RC카가 전진한다.
  - **검증 방법**: 육안 확인

- **TC-RC-002**: 정지 기능 테스트
  - **입력**: UART로 문자 'S' 수신
  - **예상 결과**: 두 DC 모터가 모두 정지한다.
  - **검증 방법**: 육안 확인

- **TC-GPIO-001**: LED 점멸 테스트
  - **입력**: 없음
  - **예상 결과**: NUCLEO 보드의 LD2(PA5) LED가 1초 주기로 점멸한다.
  - **검증 방법**: 육안 확인

- **TC-EXTI-001**: 버튼 인터럽트 테스트
  - **입력**: B1(PC13) 사용자 버튼 누름
  - **예상 결과**: 버튼을 누를 때마다 LD2 LED의 상태가 즉시 반전된다.
  - **검증 방법**: 육안 및 반응성 확인

- **TC-ADC-001**: ADC 값 변환 테스트
  - **입력**: PA0 핀에 연결된 가변저항 조작
  - **예상 결과**: PC 시리얼 터미널에 0부터 4095 사이의 값이 실시간으로 출력된다.
  - **검증 방법**: 터미널 프로그램 확인

---
## 5. 추적성 매트릭스 (Traceability Matrix)

| 요구사항 ID | 설계 요소 (모듈) | 주요 함수 | 테스트 케이스 ID |
|:---:|:---:|:---|:---:|
| HLR-RC-001 | `RC_BLUETOOTH` | `HAL_UART_Receive_IT` | TC-RC-001, 002 |
| HLR-RC-002 | `RC_BLUETOOTH` | `HAL_GPIO_WritePin` | TC-RC-001, 002 |
| HLR-RC-003 | `RC_BLUETOOTH` | `HAL_TIM_PWM_Start` | TC-RC-001 |
| HLR-GPIO-001 | `GPIO` | `HAL_GPIO_TogglePin`, `HAL_Delay` | TC-GPIO-001 |
| HLR-EXTI-001 | `EXTI` | `HAL_GPIO_EXTI_Callback` | TC-EXTI-001 |
| HLR-ADC-001 | `ADC` | `HAL_ADC_GetValue` | TC-ADC-001 |

---
## 6. 결론

본 레포지토리는 STM32F103의 주요 주변장치에 대한 학습 내용을 체계적으로 정리하고, 이를 바탕으로 통합 애플리케이션(블루투스 RC카)을 성공적으로 구현하였음을 보인다. 각 프로젝트는 요구사항 정의, 설계, 구현, 검증의 흐름에 따라 문서화되었다.
````
