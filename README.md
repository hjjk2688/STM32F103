
# STM32F103 주변장치별 동작 원리 및 실습
**A Repository for Hands-on Practice with STM32F103 Peripherals, Culminating in a Hybrid Bluetooth RC Car with Obstacle Avoidance**

![C](https://img.shields.io/badge/Language-C-blue.svg)
![STM32CubeIDE](https://img.shields.io/badge/IDE-STM32CubeIDE-orange.svg)
![STM32F103](https://img.shields.io/badge/MCU-STM32F103-blueviolet.svg)


---

## 📖 개요 (Overview)

이 레포지토리는 STM32F103 MCU의 핵심 주변장치들을 **HAL(Hardware Abstraction Layer) 라이브러리**를 기반으로 학습하고, 각 기능의 동작 원리를 깊이 있게 이해하기 위한 실습 코드 모음입니다. 기본적인 주변장치 제어 예제를 포함하고있습니다.

최종 프로젝트는 **장애물 감지 기능이 탑재된 하이브리드 블루투스 RC카** 입니다.

---

## 🏆 최종 프로젝트: 하이브리드 블루투스 RC카 (Final Project: Hybrid Bluetooth RC Car)

이 레포지토리의 모든 기본 예제들을 종합하여 만든 최종 애플리케이션입니다. 기본적으로는 스마트폰의 블루투스 명령으로 조작하지만, **전방에 장애물이 감지되면 사용자의 명령을 무시하고 자동으로 정지하는 안전 기능**이 탑재된 스마트카 입니다.

본 프로젝트의 DO-178C 표준 기반 요구사항, 설계, 검증에 대한 상세 문서는 아래 링크를 참고 하십시오.

📄 **상세 설계 문서:** [DO-178C 기반 문서 확인하기](./RC_Bluetooth/RC_Bluetooth_README.md)

#### 시스템 아키텍처
```
[스마트폰 앱] ---> [블루투스 모듈] --(UART)-->+
                                             |
[초음파 센서] --(Trig/Echo)--> [STM32 MCU] --(판단 로직)--> [모터 드라이버] ---> [DC 모터]
```

#### 주요 기능 및 구현
*   **하이브리드 제어 로직 (Hybrid Control Logic):**
    *   `main` 함수의 `while(1)` 루프에서 주기적으로 초음파 센서로 전방 거리를 측정합니다.
    *   동시에 UART 수신 인터럽트를 통해 스마트폰의 제어 명령('F', 'B', 'S' 등)을 대기합니다.
    *   **핵심 기능:** 사용자가 'F'(전진) 명령을 내렸을 때, 측정된 전방 거리가 20cm 미만이면 **사용자 명령을 무시하고 자동으로 모터를 정지**시켜 충돌을 방지합니다. 거리가 20cm 이상일 때만 전진 명령을 수행합니다.

*   **수동 조작 (Manual Control via Bluetooth):**
    *   `USART2`와 `HAL_UART_Receive_IT`를 사용하여 블루투스 모듈(HC-06)로부터 제어 문자를 비동기 수신합니다.
    *   수신된 문자에 따라 모터의 전진, 후진, 좌/우회전, 정지 동작을 결정합니다.

*   **거리 측정 (Distance Measurement):**
    *   `TIM1`을 마이크로초(us) 단위 시간 측정에 사용하고, `EXTI`로 Echo 핀의 상승/하강 엣지를 감지합니다.
    *   `Trig` 핀에 10us 펄스를 보내 초음파를 발사하고, `Echo` 핀이 High로 유지되는 시간을 측정하여 cm 단위의 거리로 환산합니다.

*   **모터 제어 (Motor Control):**
    *   `TIM2`, `TIM3`의 PWM 모드를 사용하여 모터의 속도를 제어합니다.
    *   GPIO 출력 핀으로 모터 드라이버(L298N)의 방향을 제어합니다.

---

## 📚 기본 예제 (Basic Examples)

아래는 최종 프로젝트를 만들기 위해 학습한 각 주변장치별 기본 기능 구현 예제입니다.

| 프로젝트 (Project) | 주요 학습 내용 (Key Learning Points) | 소스 코드 기반 상세 설명 |
| :--- | :--- | :--- |
| **💡 GPIO** | `HAL_GPIO_WritePin`, `HAL_GPIO_ReadPin`, `HAL_Delay` | `LD2 (PA5)` 핀에 연결된 LED를 `while` 루프와 `HAL_Delay(1000)`을 이용해 1초 간격으로 점멸시킵니다. |
| **SERIAL UART** | `printf` 리다이렉션, `HAL_UART_Receive_IT` | `USART2`를 설정하고 `_write` 함수를 재정의하여 `printf` 출력을 PC 시리얼 터미널로 전송합니다. 또한, 수신 인터럽트를 이용해 PC로부터 받은 데이터를 그대로 다시 보내는 에코(Echo) 기능을 구현합니다. |
| **⚡ EXTI** | `HAL_GPIO_EXTI_Callback`, 인터럽트 처리 | `B1 (PC13)` 사용자 버튼에 외부 인터럽트(하강 엣지)를 설정합니다. 버튼을 누를 때마다 인터럽트 콜백 함수가 호출되어 `LD2` LED의 상태를 반전(`TogglePin`)시킵니다. |
| **⏰ TIMER** | `HAL_TIM_Base_Start_IT`, `HAL_TIM_PeriodElapsedCallback` | `TIM2`를 1ms 주기 업데이트 인터럽트 모드로 설정합니다. 콜백 함수 내에서 카운터를 1000까지 세어, 정확히 1초마다 `LD2` LED를 점멸시킵니다. |
| **📈 ADC** | `HAL_ADC_Start`, `HAL_ADC_GetValue` | `ADC1`의 `IN0 (PA0)` 채널을 사용하여 아날로그 값을 읽습니다. `while` 루프 내에서 주기적으로 ADC 변환을 수행하고, 12비트 결과값을 `printf`를 통해 UART 터미널로 출력합니다. |
| **🧲 NVIC** | `HAL_NVIC_SetPriority`, 인터럽트 우선순위 | `TIM2` 인터럽트와 `EXTI` 버튼 인터럽트의 우선순위를 다르게 설정합니다. 우선순위가 높은 인터럽트가 낮은 우선순위의 인터럽트 처리 중에 발생했을 때, 먼저 처리되는(Preemption) 현상을 확인합니다. |

---

## 📌 필요 사항 (Hardware & Software)

*   **하드웨어**
    *   STM32F103 기반 개발 보드 (NUCLEO-F103RB)
    *   초음파 센서 (HC-SR04)
    *   블루투스 모듈 (HC-06 등)
    *   모터 드라이버 (L298N 등)
    *   DC 모터 2개 이상 및 바퀴, RC카 섀시
    *   모터 및 MCU 구동용 외부 전원 (배터리 등)
    *   ST-LINK 디버거
*   **소프트웨어**
    *   STM32CubeIDE (v1.9.0 이상 권장)
    *   스마트폰 블루투스 시리얼 통신 앱

---

## 🚀 사용 방법 (How to Use)

1.  이 레포지토리를 로컬 컴퓨터에 클론(Clone)합니다.
2.  STM32CubeIDE에서 `File` > `Import...` > `General` > `Existing Projects into Workspace`를 선택합니다.
3.  `Select root directory:`에서 이 레포지토리의 최상위 폴더를 선택하면 모든 프로젝트가 목록에 나타납니다.
4.  원하는 프로젝트를 체크하고 `Finish`를 눌러 워크스페이스로 불러옵니다.

---

## ✍️ 작성자 (Author)

*   **HJ**

