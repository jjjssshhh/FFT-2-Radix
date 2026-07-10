# FFT-2-Radix

256-point DIT Radix-2 FFT를 FPGA에 구현한 프로젝트.  
PC에서 UART로 256샘플을 전송하면 FFT 연산 후 최대 주파수 인덱스를 7-세그먼트에 표시한다.

2 * pi * f * i / N

ex) N = 256, f = 5, fs = 2560hz, 실제 f = 5*2560/256 = 50hz

## 데모

[![Demo](https://img.youtube.com/vi/f2Y4J2dDIZk/0.jpg)](https://youtu.be/f2Y4J2dDIZk)

## 구성

| 파일 | 설명 |
|---|---|
| `fft_top.sv` | 최상위 모듈, 핑퐁 메모리 제어 |
| `fft_ctrl.sv` | 8스테이지 순차 실행 제어기 |
| `fft.sv` | Butterfly 연산 코어 |
| `UART_controller.sv` | 115200bps UART 수신기 |
| `FND_ctrl.sv` | 7-세그먼트 표시기 (Double Dabble BCD 변환) |
| `div_8.sv` | UART 오버샘플링 8분주기 |
| `uart_fifo_tb.sv` | 시뮬레이션 테스트벤치 |

## 주요 설계 포인트

- **핑퐁 메모리**: 스테이지마다 BRAM 두 개를 교대로 읽기/쓰기 전환
- **Twiddle Factor**: 8bit 고정소수점 (×127 스케일), 3단계 파이프라인으로 오버플로 방지
- **CDC**: UART(uartclk) → FFT(wclk) 도메인 2-FF 동기화기 적용
- **비트-역순 주소**: UART 수신 시 bit_reversal()로 DIT 입력 순서 자동 변환

## 개발 환경

- **Tool**: Vivado 2024.2
- **Target Board**: Basys3 (Artix-7)
- **Simulation**: Vivado Simulator
