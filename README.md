````markdown
<div align="center">

# STM32F103 주변장치별 동작 원리 및 실습
**A Repository for Hands-on Practice with STM32F103 Peripherals, Culminating in a Bluetooth RC Car Project**

![C](https://img.shields.io/badge/Language-C-blue.svg)
![STM32CubeIDE](https://img.shields.io/badge/IDE-STM32CubeIDE-orange.svg)
![STM32F103](https://img.shields.io/badge/MCU-STM32F103-blueviolet.svg)

</div>

---

## 📖 개요 (Overview)

이 레포지토리는 STM32F103 MCU의 핵심 주변장치들을 **HAL(Hardware Abstraction Layer) 라이브러리**를 기반으로 학습하고, 각 기능의 동작 원리를 깊이 있게 이해하기 위한 실습 코드 모음입니다. 기본적인 주변장치 제어 예제부터, 이를 응용한 최종 프로젝트 **블루투스 RC카**까지 포함하고 있습니다.

---

## 🏆 최종 프로젝트: 블루투스 RC카 (Final Project: Bluetooth RC Car)

이 레포지토리의 모든 기본 예제들을 종합하여 만든 최종 애플리케이션입니다. 스마트폰 앱과 블루투스 통신을 통해 DC 모터로 구동되는 RC카를 제어합니다.

#### 시스템 아키텍처
`스마트폰 앱` → `블루투스 모듈 (HC-06)` → `STM32 (UART)` → `STM32 (GPIO/TIMER)` → `모터 드라이버 (L298N)` → `DC 모터`

#### 주요 기능 및 구현
*   **블루투스 통신 (UART):**
    *   `USART2`를 사용하여 블루투스 모듈(HC-06)과 연결합니다.
    *   스마트폰 앱으로부터 'F'(전진), 'B'(후진), 'L'(좌회전), 'R'(우회전), 'S'(정지) 등의 제어 문자를 수신합니다.
    *   `HAL_UART_Receive_IT`를 사용하여 데이터 수신 시 인터럽트 방식으로 즉각적인 제어 명령을 처리합니다.

*   **모터 속도 제어 (TIMER - PWM):**
    *   `TIM2`와 `TIM3`를 PWM 생성 모드로 설정합니다.
    *   모터 드라이버의 Enable(ENA, ENB) 핀에 PWM 신호를 인가하여 DC 모터의 속도를 제어합니다.

*   **모터 방향 제어 (GPIO):**
    *   모터 드라이버(L298N)의 방향 제어 핀(IN1, IN2, IN3, IN4)에 연결된 GPIO 핀들의 High/Low 상태를 조합하여 모터의 정회전/역회전을 제어합니다.

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

*   **공통**
    *   STM32F103 기반 개발 보드 (NUCLEO-F103RB)
    *   ST-LINK 디버거
    *   USB to Serial 변환기
*   **RC카 프로젝트용 추가 부품**
    *   블루투스 모듈 (HC-06 등)
    *   모터 드라이버 (L298N 등)
    *   DC 모터 2개 이상
    *   RC카 섀시 및 바퀴
    *   모터 및 MCU 구동용 외부 전원 (배터리 등)
*   **소프트웨어**
    *   STM32CubeIDE (v1.9.0 이상 권장)
    *   PC 시리얼 터미널 프로그램 (Tera Term, PuTTY 등)
    *   스마트폰 블루투스 시리얼 통신 앱

---

## 🚀 사용 방법 (How to Use)

1.  이 레포지토리를 로컬 컴퓨터에 클론(Clone)합니다.
2.  STM32CubeIDE에서 `File` > `Import...` > `General` > `Existing Projects into Workspace`를 선택합니다.
3.  `Select root directory:`에서 이 레포지토리의 최상위 폴더를 선택하면 모든 프로젝트가 목록에 나타납니다.
4.  원하는 프로젝트를 체크하고 `Finish`를 눌러 워크스페이스로 불러옵니다.

---

## ✍️ 작성자 (Author)

*   **hjjk2688**
````
