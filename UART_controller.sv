`timescale 1ns / 1ps
//==============================================================================
// 모듈명 : UART_controller
// 기  능 : 115200 bps, 8N1 UART 수신기.
//          수신된 바이트를 FFT 입력 BRAM에 순차 저장하고,
//          256바이트 완료 시 uart_done=1을 출력한다.
//
// 오버샘플링 구조:
//   uartclk = 14.7456 MHz (외부 공급)
//   div_8 펄스 = uartclk / 8 = 1.8432 MHz
//   비트 주기 = 16 × div_8 주기 = 115200 Hz
//   → 각 비트의 중앙(8번째 div_8 펄스)에서 샘플링하여 노이즈 마진 확보
//
// 상태머신:
//   IDLE : RsRx 하강 에지 감지 (Start bit) → START 상태로 진입
//   START: 7번 div_8 대기 후 RsRx=0 확인 → 비트 중앙 기준점 설정
//   BUSY : 16 div_8마다 1bit 샘플링, LSB 우선으로 8bit 수집
//   STOP : 스톱 비트(RsRx=1) 확인 후 IDLE 복귀
//
// 비트-역순 주소 매핑:
//   FFT DIT 알고리즘은 입력이 비트-역순(bit-reversed order)으로 배치되어야 한다.
//   wt_ptr를 1씩 증가시키되, BRAM 쓰기 주소(wt_ptr)에 bit_reversal()을 적용하여
//   UART 수신 순서(자연 순서) → FFT 비트-역순 주소로 자동 변환한다.
//   예) wt_ptr=0b00000001 → BRAM주소=0b10000000
//
// 수신 완료 신호:
//   256바이트 수신(cal_cnt==255) 후 uart_done=1 유지.
//   FFT 연산 완료 후 다음 256바이트 수신을 기다리는 구조.
//==============================================================================

module UART_controller(FT.uart mi0);

    typedef enum logic [2:0]
    {
        IDLE,
        START,
        BUSY,
        STOP
    } st_t;

    st_t state;
    logic [3:0] buff2;
    logic [1:0] rgb_cnt;
    logic [3:0] buff0_cnt;  // 수집된 비트 수 (0~7)
    logic [1:0] buff1_cnt;
    logic [3:0] sig_tick;   // div_8 펄스 카운터 (비트 중앙 타이밍 제어)

    logic [7:0] wt_ptr_d;   // 비트-역순 변환 전 순차 쓰기 포인터

    // 8bit 비트 순서 반전: FFT DIT 비트-역순 주소 생성
    function [7:0] bit_reversal(input [7:0] A);
        logic [7:0] temp;
        for(int i=0; i<8; i=i+1)
            temp[7-i] = A[i];
        return temp;
    endfunction

    // --- UART 수신 상태머신 (uartclk 도메인) ---
    always@(posedge mi0.uartclk or negedge mi0.rstn)begin
        if(!mi0.rstn)begin
            state      <= IDLE;
            buff0_cnt  <= 0;
            sig_tick   <= 0;
            rgb_cnt    <= 0;
            mi0.en_div <= 0;
            mi0.cal_valid <= 0;
            mi0.wt_ptr <= 0; wt_ptr_d <= 0;
        end else begin
            // 쓰기 주소: wt_ptr_d를 비트-역순 변환하여 출력 (1클럭 등록 딜레이 포함)
            mi0.wt_ptr <= bit_reversal(wt_ptr_d);
            case(state)
                IDLE: begin
                    if(mi0.RsRx == 0) begin  // Start bit 하강 에지 감지
                        sig_tick   <= 0;
                        mi0.en_div <= 1;      // 8분주기 활성화
                        state      <= START;
                    end
                end

                START: begin
                    // 7번 div_8 대기: 비트 중앙 기준점 설정 (RsRx=0 유지 확인)
                    if(mi0.div_8)begin
                        if(sig_tick == 7 && mi0.RsRx == 0)begin
                            sig_tick <= 0;
                            state    <= BUSY;
                        end else begin
                            sig_tick <= sig_tick + 1;
                        end
                    end
                end

                BUSY: begin
                    // 16 div_8마다 비트 중앙에서 1bit 샘플링 (LSB 우선)
                    if(mi0.div_8)begin
                        if(sig_tick == 15)begin
                            sig_tick <= 0;
                            mi0.buff0[buff0_cnt] <= mi0.RsRx;  // LSB 우선 비트 저장
                            if(buff0_cnt == 7) begin  // 8bit 수집 완료
                                state         <= STOP;
                                buff0_cnt     <= 0;
                                mi0.cal_valid <= 1;     // 1바이트 수신 완료 펄스
                                wt_ptr_d      <= wt_ptr_d + 1;
                            end
                            else buff0_cnt <= buff0_cnt + 1;
                        end else begin
                            sig_tick <= sig_tick + 1;
                        end
                    end
                end

                STOP: begin
                    mi0.cal_valid <= 0;
                    // 스톱 비트(RsRx=1) 확인 후 IDLE 복귀
                    if(mi0.div_8)begin
                        if(sig_tick == 15 && mi0.RsRx == 1)begin
                            sig_tick <= 0;
                            state    <= IDLE;
                        end else sig_tick <= sig_tick + 1;
                    end
                end
            endcase
        end
    end

    // --- 256바이트 수신 완료 감지 → uart_done 생성 ---
    logic [7:0] cal_cnt;
    logic room_d;
    always@(posedge mi0.uartclk or negedge mi0.rstn)begin
        if(!mi0.rstn)begin
            mi0.uart_done <= 0; cal_cnt <= 0; room_d <= 0;
        end else begin
            if(mi0.cal_valid)begin
                if(cal_cnt == 255)begin  // 256번째 바이트 수신
                    cal_cnt <= 0;
                    room_d  <= 1;        // 1클럭 후 uart_done에 반영 (등록 지연)
                end else begin
                    cal_cnt <= cal_cnt + 1;
                end
            end
            mi0.uart_done <= room_d;
        end
    end

endmodule
