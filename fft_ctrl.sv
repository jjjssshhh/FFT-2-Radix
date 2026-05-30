`timescale 1ns / 1ps
//==============================================================================
// 모듈명 : fft_ctrl
// 기  능 : 256-point FFT의 8개 스테이지를 순차적으로 실행하는 제어기.
//          각 스테이지에서 butterfly 코어(fft)를 기동하고,
//          완료 신호(fft_done)를 받아 다음 스테이지로 진행한다.
//
// Cooley-Tukey DIT Radix-2 FFT 스테이지 구조 (256-point):
//   스테이지 0: butterfly 거리(distance) = 1,  twiddle step = 128
//   스테이지 1: butterfly 거리          = 2,  twiddle step = 64
//   스테이지 2: butterfly 거리          = 4,  twiddle step = 32
//      ...
//   스테이지 7: butterfly 거리          = 128, twiddle step = 1
//   interval = 스테이지 번호 → distance = 2^interval, step = 128 >> interval
//
// 핑퐁 메모리 교환:
//   스테이지마다 room_change를 토글하여 읽기/쓰기 BRAM을 교환.
//   짝수 스테이지: room_change=0 (bram0 읽기, bram1 쓰기)
//   홀수 스테이지: room_change=1 (bram1 읽기, bram0 쓰기)
//
// 시작 조건:
//   uart_done 상승 에지 감지 → start=1 세팅 → 스테이지 0부터 순차 실행
//
// 종료 조건:
//   state_fft=8: 모든 스테이지 완료 → fnd_en=1 (FND 표시 활성화)
//   다음 uart_done 변화 시 state_fft=0으로 리셋하여 재측정 대기
//==============================================================================

module fft_ctrl(FT.fft m);

    fft fft0 (.m(m));  // butterfly 연산 코어 인스턴스

    // --- UART 수신 완료 상승 에지 감지 → FFT 시작 ---
    logic uart_done_d;
    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
            m.start <= 0;
        end else begin
            uart_done_d <= m.uart_done;
            // uart_done 상승 에지: 256샘플 수신 완료 → FFT 시작 신호
            if({uart_done_d, m.uart_done} == 2'b01)begin
                m.start <= 1;
            end
        end
    end

    // --- 스테이지 순차 실행 상태머신 ---
    // pulse: start_fft를 1클럭 펄스로 만들기 위한 플래그
    // state_fft: 현재 실행 중인 FFT 스테이지 번호 (0~8)
    logic pulse;
    logic [5:0] state_fft;

    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
            state_fft   <= 0; m.stage    <= 0;
            m.n         <= 0; m.start_fft <= 0; pulse <= 0;
            m.fnd_en    <= 0; m.room_change <= 0;
        end else if(m.start)begin
            m.stage    <= state_fft;
            m.interval <= state_fft;  // interval = 스테이지 번호 = log2(distance)
            case(state_fft)
                // 스테이지 0~7: start_fft 1클럭 펄스 발생 후 fft_done 대기
                // room_change: 짝수=0(bram0 읽기), 홀수=1(bram1 읽기)
                0: begin
                    if(!pulse) begin m.start_fft <= 1; pulse <= 1; m.room_change <= 0;
                    end else         m.start_fft <= 0;
                    m.num <= 128;
                    if(m.fft_done) begin state_fft <= 1; pulse <= 0; end
                end
                1: begin
                    if(!pulse) begin m.start_fft <= 1; pulse <= 1; m.room_change <= 1;
                    end else         m.start_fft <= 0;
                    if(m.fft_done) begin state_fft <= 2; pulse <= 0; end
                end
                2: begin
                    if(!pulse) begin m.start_fft <= 1; pulse <= 1; m.room_change <= 0;
                    end else         m.start_fft <= 0;
                    if(m.fft_done) begin state_fft <= 3; pulse <= 0; end
                end
                3: begin
                    if(!pulse) begin m.start_fft <= 1; pulse <= 1; m.room_change <= 1;
                    end else         m.start_fft <= 0;
                    if(m.fft_done) begin state_fft <= 4; pulse <= 0; end
                end
                4: begin
                    if(!pulse) begin m.start_fft <= 1; pulse <= 1; m.room_change <= 0;
                    end else         m.start_fft <= 0;
                    if(m.fft_done) begin state_fft <= 5; pulse <= 0; end
                end
                5: begin
                    if(!pulse) begin m.start_fft <= 1; pulse <= 1; m.room_change <= 1;
                    end else         m.start_fft <= 0;
                    if(m.fft_done) begin state_fft <= 6; pulse <= 0; end
                end
                6: begin
                    if(!pulse) begin m.start_fft <= 1; pulse <= 1; m.room_change <= 0;
                    end else         m.start_fft <= 0;
                    if(m.fft_done) begin state_fft <= 7; pulse <= 0; end
                end
                7: begin
                    if(!pulse) begin m.start_fft <= 1; pulse <= 1; m.room_change <= 1;
                    end else         m.start_fft <= 0;
                    if(m.fft_done) begin state_fft <= 8; pulse <= 0; end
                end
                8: begin
                    // 모든 스테이지 완료: FND 표시 활성화, 다음 수신 대기
                    m.fnd_en <= 1;
                    if(uart_done_d != m.uart_done) state_fft <= 0;
                end
            endcase
        end
    end

endmodule
