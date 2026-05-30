`timescale 1ns / 1ps
//==============================================================================
// 모듈명 : uart_fifo_tb
// 기  능 : fft_top 검증 테스트벤치.
//          UART 프로토콜로 256샘플을 전송하고,
//          FFT 연산 후 $display로 출력되는 butterfly 결과를 확인한다.
//
// 클럭 설정:
//   wclk    : 20ns 주기 (50MHz)
//   uartclk : 1736ns 주기 ≈ 575.8kHz
//             → div_8 후 71.97kHz, ×16으로 4498Hz ≈ 실제와 다르나 시뮬 속도 우선
//
// 테스트 시나리오:
//   sine_test: 주파수 5Hz 사인파 256샘플 → 빈 인덱스 5에서 최대 진폭 기대
//   real_test: input_data_2.mem에서 로드한 실측 데이터 전송
//   one_test : 모든 샘플 = 100 (DC 성분 테스트)
//   test     : 랜덤 데이터 (기본 동작 확인)
//
// UART 프레임 구조 (8N1, LSB 우선):
//   [1bit Start=0] [8bit Data, LSB→MSB] [1bit Stop=1]
//   각 비트 = 128 uartclk 주기 유지
//==============================================================================

module uart_fifo_tb();

    logic wclk, uartclk;
    logic rstn;
    logic RsRx;

    always #10   wclk    = ~wclk;    // 50MHz
    always #868  uartclk = ~uartclk; // ≈575kHz (시뮬레이션 가속용)

    fft_top top0(wclk, uartclk, rstn, RsRx);

    logic signed [7:0] in_dt [0:255];  // 실측 데이터 파일 로드용
    initial begin
        $readmemh("input_data_2.mem", in_dt);
    end

    initial begin
        wclk = 0; uartclk = 0;
        RsRx = 0;
        top0.main_bus.rd_ready = 0;
        rstn = 0;
        repeat(128) @(posedge uartclk);
        rstn = 1;

        sine_test();  // 사인파 256샘플 FFT 검증
        #1000;
        $finish;
    end

    // --- 사인파 테스트: 256샘플 안에 freq 사이클 포함 ---
    // 결과: max_idx ≈ freq 위치에서 최대 진폭 기대
    task sine_test();
        real pi        = 3.1415926535;
        real amplitude = 120.0;  // 8bit signed 포화 방지 (최대 127)
        real freq      = 5.0;    // 256샘플 내 5 사이클
        integer val;
        logic signed [7:0] temp;

        for(int j=0; j<256; j=j+1) begin
            val  = $rtoi(amplitude * $sin(2.0 * pi * freq * (j / 256.0)));
            temp = val[7:0];

            RsRx = 1;                          // Idle
            repeat(128) @(posedge uartclk);
            RsRx = 0;                          // Start bit
            repeat(128) @(posedge uartclk);

            for(int i=0; i<8; i=i+1) begin    // Data bits (LSB 우선)
                RsRx = temp[i];
                repeat(128) @(posedge uartclk);
            end

            RsRx = 1;                          // Stop bit
            repeat(128) @(posedge uartclk);
        end
    endtask

    // --- 실측 데이터 테스트: input_data_2.mem에서 로드 ---
    task real_test();
        logic signed [7:0] temp;
        for(int j=0; j<256; j=j+1)begin
            temp = in_dt[j];
            RsRx = 1;
            repeat(128) @(posedge uartclk);
            RsRx = 0;
            repeat(128) @(posedge uartclk);
            for(int i=0; i<8; i=i+1)begin
                RsRx = temp[i];
                repeat(128) @(posedge uartclk);
            end
            RsRx = 1;
            repeat(128) @(posedge uartclk);
        end
    endtask

    // --- DC 테스트: 모든 샘플 = 100 (빈 0에서 최대 진폭 기대) ---
    task one_test();
        logic signed [7:0] temp;
        for(int j=0; j<256; j=j+1)begin
            temp = 100;
            RsRx = 1;
            repeat(128) @(posedge uartclk);
            RsRx = 0;
            repeat(128) @(posedge uartclk);
            for(int i=0; i<8; i=i+1)begin
                RsRx = temp[i];
                repeat(128) @(posedge uartclk);
            end
            RsRx = 1;
            repeat(128) @(posedge uartclk);
        end
    endtask

    // --- 랜덤 테스트: 기본 동작 확인 ---
    task test();
        repeat(256)begin
            RsRx = 1;
            repeat(128) @(posedge uartclk);
            RsRx = 0;
            repeat(128) @(posedge uartclk);
            for(int i=0; i<8; i=i+1)begin
                RsRx = $urandom_range(0, 1);
                repeat(128) @(posedge uartclk);
            end
            RsRx = 1;
            repeat(128) @(posedge uartclk);
        end
    endtask

endmodule
