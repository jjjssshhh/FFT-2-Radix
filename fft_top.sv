`timescale 1ns / 1ps
//==============================================================================
// 인터페이스 : FT (FFT Top Bus)
// 기      능 : 시스템 내 모든 서브모듈 간 신호를 하나의 버스로 묶는 인터페이스.
//              modport로 각 모듈의 방향을 정의하여 연결 오류를 컴파일 타임에 방지.
//
// 포트 그룹:
//   div8  : UART 오버샘플링 클럭 8분주기 (uartclk → div_8 펄스)
//   uart  : UART 수신기 (RsRx 수신 → buff0 바이트 출력, cal_valid 펄스)
//   fft   : FFT 연산 코어 + 제어기 (메모리 읽기/쓰기 주소·데이터, 스테이지 제어)
//   fnd   : 7-세그먼트 표시기 (max_idx → seg/an 출력)
//==============================================================================
//
//==============================================================================
// 모듈명 : fft_top
// 기  능 : UART로 수신한 256샘플에 대해 256-point FFT를 수행하고,
//          최대 진폭 주파수 빈(bin) 인덱스를 7-세그먼트에 표시한다.
//
// 전체 데이터 흐름:
//   PC → UART RX → BRAM(ping) → FFT 8스테이지 연산(ping↔pong 교환) → max_idx 추출 → FND 표시
//
// 핑퐁 메모리 구조:
//   BRAM 2개(bram0, bram1)를 핑퐁으로 운용한다.
//   - UART 수신 중:  bram0에 256샘플 기록 (UART 도메인)
//   - FFT 연산 중:   bram0 읽기 / bram1 쓰기 (room_change=0)
//                    bram1 읽기 / bram0 쓰기 (room_change=1)  → 스테이지마다 교환
//   각 스테이지 완료 시 fft_ctrl이 room_change를 토글하여 read/write 대상을 전환.
//
// CDC 처리:
//   uart_done, cal_valid, buff0, wt_ptr은 uartclk 도메인에서 생성되므로
//   2-FF 동기화기를 거쳐 wclk 도메인으로 전달한다.
//
// 클럭 도메인:
//   wclk    : 100MHz, FFT 연산 / BRAM 포트 A·B 공유 / FND 구동
//   uartclk : 14.7456MHz (115200 × 16 × 8 = 오버샘플링 기준), UART RX 전용
//==============================================================================

interface FT(input wclk, input uartclk);

    localparam size = 32;

    logic rstn;
    logic RsRx;

    // UART 수신기 제어 신호
    logic wt_ready;           // BRAM 쓰기 가능 상태
    logic rd_ready;           // BRAM 읽기 요청
    logic rd_valid;           // BRAM 읽기 데이터 유효
    logic [size-1:0] fifo_dt0, fifo_dt1;  // butterfly 입력 데이터 쌍
    logic en_div;             // 8분주기 활성화
    logic div_8;              // 8분주 펄스 (매 8 uartclk마다 1클럭 High)
    logic uart_done;          // 256바이트 수신 완료 (uartclk 도메인)
    logic cal_valid;          // 바이트 수신 완료 펄스
    logic [7:0] buff0;        // 수신된 1바이트

    logic [7:0] wt_ptr;       // 비트-역순 변환된 BRAM 쓰기 주소
    modport div8(input uartclk,rstn,en_div, output div_8);
    modport uart(input wclk,uartclk,rstn,RsRx,rd_ready,div_8,
                            output rd_valid,wt_ready,en_div,uart_done,
                                        buff0, cal_valid,wt_ptr);

    // FFT 연산 제어 신호
    logic [size-1:0] fft_out0, fft_out1;  // butterfly 연산 결과 쌍
    logic wt_valid;           // FFT 결과 BRAM 쓰기 유효
    logic fft_en, fft_done;   // FFT 활성화 / 1스테이지 완료
    logic start;              // FFT 전체 시작 (uart_done 상승 에지에서 세팅)
    logic [8:0] num;          // 현재 스테이지 butterfly 횟수
    logic [3:0] stage;        // 현재 스테이지 번호 (0~7)
    logic [7:0] addr_wt0,addr_wt1;  // butterfly 결과 쓰기 주소 쌍
    logic [7:0] addr_rd0,addr_rd1;  // butterfly 입력 읽기 주소 쌍
    logic [7:0] interval;     // 현재 스테이지의 butterfly 거리 (2^stage)
    logic [7:0] n;
    logic start_fft;          // 한 스테이지 시작 펄스
    logic fnd_en;             // FND 표시 활성화 (모든 스테이지 완료 후)
    logic [8:0] max_idx;      // 최대 진폭 주파수 빈 인덱스
    logic room_change;        // 핑퐁 메모리 선택 (0: bram0 읽기, 1: bram1 읽기)
    modport fft (input wclk, rstn,rd_valid,fifo_dt0, fifo_dt1, wt_ready,uart_done,num,fft_en,stage,interval,n,start,
                            output rd_ready,fft_out0, fft_out1, wt_valid,addr_wt0, addr_wt1, fft_done,addr_rd0, addr_rd1,
                                        start_fft,fnd_en,max_idx,room_change);

    logic [7:0] seg;
    logic [3:0] an;
    modport fnd(input wclk, rstn,fnd_en,max_idx, output seg, an);

endinterface


module fft_top
(
    input  wclk,
    input  uartclk,   // 외부 입력 (14.7456MHz, UART 오버샘플링 기준 클럭)
    input  rstn,
    input  RsRx,
    output [7:0] seg,
    output [3:0] an
);

    FT main_bus(wclk, uartclk);

    assign main_bus.rstn = rstn;
    assign main_bus.RsRx = RsRx;
    assign seg = main_bus.seg;
    assign an  = main_bus.an;

    fft_ctrl          fft_ctrl0 (.m(main_bus.fft));
    FND_ctrl          fnd0      (.m(main_bus.fnd));
    UART_controller   uart0     (.mi0(main_bus.uart));
    div_8             div0      (.mi0(main_bus.div8));

    // --- 핑퐁 메모리 제어 신호 ---
    logic wt_valid0_0, wt_valid0_1, wt_valid1_0, wt_valid1_1;
    logic rd_ready0, rd_ready1;
    logic rd_valid0, rd_valid1;
    logic wt_ready0, wt_ready1;

    logic [31:0] fifo0_dt0, fifo0_dt1;  // bram0 읽기 데이터
    logic [31:0] fifo1_dt0, fifo1_dt1;  // bram1 읽기 데이터

    logic [31:0] in_dt0_wt0, in_dt0_wt1;  // bram0 쓰기 데이터
    logic [31:0] in_dt1_wt0, in_dt1_wt1;  // bram1 쓰기 데이터

    logic [7:0] addr0_0, addr0_1;
    logic [7:0] addr1_0, addr1_1;
    logic we0_0, we0_1, we1_0, we1_1;
    logic fft_en0, fft_en1;

    // --- CDC: uartclk 도메인 신호를 wclk 도메인으로 2-FF 동기화 ---
    logic [1:0] uart_done_dsync;
    logic [7:0] uart_out_dsync[0:1];
    logic [1:0] uart_valid_dsync;
    logic [7:0] uart_addr_dsync[0:1];
    always@(posedge wclk)begin
        uart_done_dsync  <= {uart_done_dsync[0],  main_bus.uart_done};
        uart_out_dsync   <= {uart_out_dsync[0],   main_bus.buff0};
        uart_valid_dsync <= {uart_valid_dsync[0], main_bus.cal_valid};
        uart_addr_dsync  <= {uart_addr_dsync[0],  main_bus.wt_ptr};
    end

    // --- 핑퐁 메모리 스위칭 로직 ---
    // uart_done이 High(256샘플 수신 완료) → FFT 모드: bram 읽기·쓰기 교환
    // uart_done이 Low(수신 중)            → UART 모드: bram0에 샘플 순차 기록
    always_comb begin
        // latch 방지: 모든 출력 기본값 초기화
        wt_valid0_0 = 0; wt_valid0_1 = 0; wt_valid1_0 = 0; wt_valid1_1 = 0;
        rd_ready0 = 0; rd_ready1 = 0;
        main_bus.rd_valid = 0;
        in_dt0_wt0 = 0; in_dt0_wt1 = 0;
        in_dt1_wt0 = 0; in_dt1_wt1 = 0;
        main_bus.wt_ready = 0;
        fft_en0 = 0; fft_en1 = 0;
        addr0_0 = 0; addr0_1 = 0;
        addr1_0 = 0; addr1_1 = 0;
        we0_0 = 0; we0_1 = 0;
        we1_0 = 0; we1_1 = 0;

        if(uart_done_dsync[1]) begin  // FFT 연산 모드
            case(main_bus.room_change)
                0: begin
                    // bram0: FFT 읽기 포트 (butterfly 입력)
                    we0_0 = 0; we0_1 = 0;
                    addr0_0 = main_bus.addr_rd0;
                    addr0_1 = main_bus.addr_rd1;
                    wt_valid0_0 = main_bus.rd_ready;
                    wt_valid0_1 = main_bus.rd_ready;
                    main_bus.fifo_dt0 = fifo0_dt0;
                    main_bus.fifo_dt1 = fifo0_dt1;

                    // bram1: FFT 쓰기 포트 (butterfly 결과)
                    we1_0 = 1; we1_1 = 1;
                    wt_valid1_0 = main_bus.wt_valid;
                    wt_valid1_1 = main_bus.wt_valid;
                    addr1_0 = main_bus.addr_wt0;
                    addr1_1 = main_bus.addr_wt1;
                    in_dt1_wt0 = main_bus.fft_out0;
                    in_dt1_wt1 = main_bus.fft_out1;
                end
                1: begin
                    // bram1: FFT 읽기 포트
                    we1_0 = 0; we1_1 = 0;
                    addr1_0 = main_bus.addr_rd0;
                    addr1_1 = main_bus.addr_rd1;
                    wt_valid1_0 = main_bus.rd_ready;
                    wt_valid1_1 = main_bus.rd_ready;
                    main_bus.fifo_dt0 = fifo1_dt0;
                    main_bus.fifo_dt1 = fifo1_dt1;

                    // bram0: FFT 쓰기 포트
                    we0_0 = 1; we0_1 = 1;
                    wt_valid0_0 = main_bus.wt_valid;
                    wt_valid0_1 = main_bus.wt_valid;
                    addr0_0 = main_bus.addr_wt0;
                    addr0_1 = main_bus.addr_wt1;
                    in_dt0_wt0 = main_bus.fft_out0;
                    in_dt0_wt1 = main_bus.fft_out1;
                end
            endcase
        end
        else begin  // UART 수신 모드: bram0에 샘플 기록
            we0_0      = 1;
            wt_valid0_0 = uart_valid_dsync[1];
            addr0_0     = uart_addr_dsync[1];
            // 8bit 수신 샘플을 32bit BRAM 상위에 저장 (FFT 입력 정수부)
            in_dt0_wt0  = uart_out_dsync[1] <<< 8;
        end
    end

    // 듀얼포트 BRAM: 포트 A/B 각각 butterfly 입출력 주소 독립 접근
    blk_mem_gen_0 bram0(
        .addra(addr0_0), .clka(wclk), .dina(in_dt0_wt0), .douta(fifo0_dt0), .ena(wt_valid0_0), .wea(we0_0),
        .addrb(addr0_1), .clkb(wclk), .dinb(in_dt0_wt1), .doutb(fifo0_dt1), .enb(wt_valid0_1), .web(we0_1));
    blk_mem_gen_0 bram1(
        .addra(addr1_0), .clka(wclk), .dina(in_dt1_wt0), .douta(fifo1_dt0), .ena(wt_valid1_0), .wea(we1_0),
        .addrb(addr1_1), .clkb(wclk), .dinb(in_dt1_wt1), .doutb(fifo1_dt1), .enb(wt_valid1_1), .web(we1_1));

endmodule
