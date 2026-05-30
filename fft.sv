`timescale 1ns / 1ps
//==============================================================================
// 모듈명 : fft
// 기  능 : 256-point DIT Radix-2 FFT의 단일 butterfly 연산 코어.
//          한 스테이지에서 128쌍의 butterfly를 순차 처리하며,
//          완료 시 fft_done을 1클럭 펄스로 출력한다.
//
// Butterfly 연산 (Cooley-Tukey):
//   X[k]     = A + W^n * B
//   X[k+N/2] = A - W^n * B
//
// Twiddle Factor (W) 표현:
//   twiddle_table.mem에서 로드 (256엔트리, 각 16bit)
//   W[n][15:8] = round(cos(2π*n/256) * 127)  ← 고정소수점 (×127 스케일)
//   W[n][7:0]  = round(-sin(2π*n/256) * 127)
//   → 연산 결과가 127배로 커지므로, 마지막에 >>>7 시프트로 스케일 복원
//
// 스테이지별 데이터 형식:
//   스테이지 0 (입력): A/B = BRAM 상위 16bit만 유효 (8bit 정수 sign extension)
//                      허수부 = 0 (실수 입력)
//   스테이지 1~7     : A/B = {실수[31:16], 허수[15:0]} 32bit 복소수
//
// 3-stage 파이프라인:
//   1단계 (CALC): W*B 곱셈 → calc_re_k, calc_im_k
//   2단계 (ADD) : A*128 ± W*B → temp_re/im_k, temp_re/im_n_2
//   3단계 (SCALE): >>>7 스케일 복원 → re/im_k, re/im_n_2
//   (×128 후 ÷128이므로 정밀도 손실 없이 오버플로 방지)
//
// 주소 생성:
//   idx: BRAM 읽기 주소 (group 내 순차 증가, group 경계에서 distance+1 점프)
//   idx_d[6]: 읽기 주소를 6클럭 지연 → 파이프라인 딜레이 보상 후 쓰기 주소 사용
//
// 최대 진폭 추출 (스테이지 7 전용):
//   마지막 스테이지 출력에서 각 주파수 빈의 크기(|X[k]|)를 계산.
//   |X[k]| ≈ max(re,im) + min(re,im)/2  (α-β approximation)
//   전체 128빈 중 최댓값 인덱스를 max_idx에 기록.
//==============================================================================

module fft(FT.fft m);

    typedef enum logic [2:0]
    {
        IDLE,
        START,
        WAIT_A,
        READ_B,
        WAIT_B,
        CALC,
        END_S
    } st_t;

    st_t en_0_state;

    // Twiddle factor 테이블: 256엔트리, [15:8]=cos, [7:0]=-sin (×127 고정소수점)
    logic signed [15:0] W [0:255];
    initial begin
        $readmemh("twiddle_table.mem", W);
    end

    // --- BRAM 읽기 데이터 1클럭 지연 (파이프라인 입력 안정화) ---
    logic [31:0] fifo_dt_d[0:1];
    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
            fifo_dt_d[0] <= 0; fifo_dt_d[1] <= 0;
        end else begin
            fifo_dt_d[0] <= m.fifo_dt0;
            fifo_dt_d[1] <= m.fifo_dt1;
        end
    end

    logic [31:0] A, B;            // butterfly 입력: A=X[k], B=X[k+distance]
    logic signed [15:0] re_k, im_k;
    logic signed [15:0] re_n_2, im_n_2;
    logic [8:0] idx;              // BRAM 읽기 주소 (현재 butterfly 상위 인덱스)
    logic [2:0] st_sw;
    logic [9:0] jdx;
    logic [31:0] fft_out0, fft_out1;
    logic [9:0] addr;
    logic send_en;                // butterfly 결과 쓰기 유효 신호

    logic [8:0] cnt_st;
    logic en_calc;                // butterfly 연산 활성화
    logic [3:0] en_pipe;
    logic [8:0] cnt_wait;         // 한 스테이지 butterfly 횟수 카운터 (0~127)

    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
            en_pipe <= 0;
        end else begin
            en_pipe <= {en_pipe[2:0], (cnt_wait == 127) ? 1'b1 : 1'b0};
        end
    end

    // distance: 현재 스테이지의 butterfly 파트너 거리 = 2^interval
    // step    : twiddle factor 테이블 인덱스 증분 = 128 >> interval
    logic [8:0] distance, group_cnt;
    assign distance = (1 << m.interval);

    // --- BRAM 읽기 주소 생성 + 상태머신 ---
    // butterfly 쌍 (idx, idx+distance)을 순차적으로 읽음
    // group_cnt가 distance-1에 도달하면 다음 그룹으로 점프
    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
            st_sw <= 0; idx <= 0; jdx <= 0;
            m.rd_ready <= 0;
            cnt_st <= 0; en_calc <= 0; group_cnt <= 0; cnt_wait <= 0;
            en_0_state <= IDLE;
        end else begin
            if(en_0_state != IDLE) begin
                cnt_st     <= (cnt_st == 2) ? 0 : cnt_st + 1;
                m.rd_ready <= 1;
                m.addr_rd0 <= idx;
                m.addr_rd1 <= idx + distance;  // butterfly 파트너 주소
                // 그룹 내 마지막 쌍: 다음 그룹으로 점프 (distance+1 증가)
                if(group_cnt == distance - 1) begin
                    idx       <= idx + distance + 1;
                    group_cnt <= 0;
                end else begin
                    idx       <= idx + 1;
                    group_cnt <= group_cnt + 1;
                end
            end
            else cnt_st <= 0;

            case(en_0_state)
                IDLE: begin
                    if(m.start_fft) en_0_state <= START;
                    else            en_0_state <= IDLE;
                    idx <= 0; jdx <= 0;
                    m.rd_ready <= 0;
                    en_calc    <= 0;
                    group_cnt  <= 0;
                    cnt_wait   <= 0;
                end
                START: begin
                    // BRAM 읽기 레이턴시(2클럭) 대기 후 연산 시작
                    if(cnt_st == 2) en_0_state <= WAIT_A;
                    else            en_0_state <= START;
                end
                WAIT_A: begin
                    en_calc <= 1;
                    A <= fifo_dt_d[0];  // butterfly 상위 샘플
                    B <= fifo_dt_d[1];  // butterfly 하위 샘플
                    cnt_wait <= (cnt_wait == 127) ? 0 : cnt_wait + 1;
                    if(cnt_wait == 127) en_0_state <= IDLE;  // 128쌍 완료
                    else                en_0_state <= WAIT_A;
                end
            endcase
        end
    end

    logic [8:0] step;
    assign step = (8'd128 >> m.interval);  // twiddle 인덱스 증분

    logic comb;

    // --- 스테이지별 데이터 형식 분리 ---
    // 스테이지 0: 8bit 실수 입력 → 32bit 복소수로 sign extension
    // 스테이지 1~7: BRAM에 저장된 {real[31:16], imag[15:0]} 그대로 사용
    logic signed [31:0] a_re, b_re, a_im, b_im;
    logic signed [31:0] a_re_d, a_im_d;  // 1클럭 지연 (파이프라인 정렬)

    logic signed [31:0] calc_re_k, calc_im_k;
    logic signed [31:0] calc_re_n_2, calc_im_n_2;
    logic signed [31:0] temp_re_k, temp_im_k;
    logic signed [31:0] temp_re_n_2, temp_im_n_2;
    logic [8:0] w_step_cnt;
    logic [8:0] w_step;       // 현재 butterfly의 twiddle factor 인덱스
    logic signed [31:0] product_re, product_im;

    always_comb begin
        if(m.stage == 0) begin
            // 스테이지 0: 8bit signed 입력, 허수부 = 0
            a_re = $signed({{8{A[15]}}, A[15:8]}); a_im = 0;
            b_re = $signed({{8{B[15]}}, B[15:8]}); b_im = 0;
        end else begin
            // 스테이지 1~7: 32bit 복소수 {실수[31:16], 허수[15:0]}
            a_re = $signed(A[31:16]); a_im = $signed(A[15:0]);
            b_re = $signed(B[31:16]); b_im = $signed(B[15:0]);
        end
    end

    // --- 파이프라인 1단계: W*B 복소수 곱셈 ---
    // W[w_step] = cos - j*sin (×127 스케일)
    // W*B = (Wr*Br - Wi*Bi) + j(Wr*Bi + Wi*Br)
    logic calc_comb;
    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
            calc_comb <= 0;
            w_step <= 0; w_step_cnt <= 0;
        end else if(en_calc) begin
            // twiddle 인덱스: step씩 증가, 128 도달 시 순환
            w_step <= ((w_step + step) >= 128) ? 0 : w_step + step;
            a_re_d <= a_re;
            a_im_d <= a_im;
            calc_re_k   <= $signed($signed(W[w_step][15:8]) * $signed(b_re))
                         - $signed($signed(W[w_step][7:0])  * $signed(b_im));
            calc_im_k   <= $signed($signed(W[w_step][15:8]) * $signed(b_im))
                         + $signed($signed(W[w_step][7:0])  * $signed(b_re));
            calc_re_n_2 <= $signed($signed(W[w_step][15:8]) * $signed(b_re))
                         - $signed($signed(W[w_step][7:0])  * $signed(b_im));
            calc_im_n_2 <= $signed($signed(W[w_step][15:8]) * $signed(b_im))
                         + $signed($signed(W[w_step][7:0])  * $signed(b_re));
            calc_comb <= 1;
        end
        else begin
            calc_comb  <= 0;
            w_step     <= 0;
            w_step_cnt <= 0;
        end
    end

    // --- 파이프라인 2단계: A*128 ± W*B (덧셈/뺄셈) ---
    // A를 <<<7 하여 W*B의 ×127 스케일과 맞춤 (실질적으로 A*128 ≈ A*127)
    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
            comb <= 0;
        end else begin
            if(calc_comb)begin
                comb        <= 1;
                temp_re_k   <= $signed($signed(a_re_d <<< 7) + $signed(calc_re_k));
                temp_im_k   <= $signed($signed(a_im_d <<< 7) + $signed(calc_im_k));
                temp_re_n_2 <= $signed($signed(a_re_d <<< 7) - $signed(calc_re_n_2));
                temp_im_n_2 <= $signed($signed(a_im_d <<< 7) - $signed(calc_im_n_2));
            end
            else comb <= 0;
        end
    end

    logic shift_d;

    // --- 파이프라인 3단계: >>>7 스케일 복원 (×128 → ×1) ---
    // 결과를 16bit로 축소하여 다음 스테이지 BRAM 저장 형식에 맞춤
    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
            re_k <= 0; im_k <= 0;
            re_n_2 <= 0; im_n_2 <= 0; shift_d <= 0;
        end else begin
            if(comb)begin
                shift_d <= 1;
                re_k   <= $signed(temp_re_k   >>> 7);
                im_k   <= $signed(temp_im_k   >>> 7);
                re_n_2 <= $signed(temp_re_n_2 >>> 7);
                im_n_2 <= $signed(temp_im_n_2 >>> 7);
            end
            else shift_d <= 0;
        end
    end

    // --- 출력 패킹: {실수[31:16], 허수[15:0]} 형식으로 BRAM 쓰기 ---
    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
            fft_out0 <= 0; fft_out1 <= 0;
            send_en  <= 0;
        end else begin
            if(shift_d)begin
                fft_out0 <= {re_k,   im_k};    // X[k]
                fft_out1 <= {re_n_2, im_n_2};  // X[k+N/2]
                send_en  <= 1;
            end else send_en <= 0;
        end
    end

    // --- 쓰기 주소: 읽기 주소(idx)를 6클럭 지연하여 파이프라인과 정렬 ---
    logic [8:0] idx_d[0:9];
    logic [8:0] send_addr0, send_addr1;

    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
            for(int i=0; i<=9; i=i+1) idx_d[i] <= 0;
            send_addr0 <= 0; send_addr1 <= 0;
        end else begin
            idx_d[0] <= idx;
            for(int i=1; i<=9; i=i+1)
                idx_d[i] <= idx_d[i-1];
            send_addr0 <= idx_d[6];
            send_addr1 <= idx_d[6] + distance;
        end
    end

    // --- BRAM 쓰기 출력 ---
    logic send_st;
    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
            send_st <= 0;
        end else if(send_en) begin
            m.wt_valid <= 1;
            m.fft_out0 <= fft_out0;
            m.fft_out1 <= fft_out1;
            m.addr_wt0 <= send_addr0;
            m.addr_wt1 <= send_addr1;
        end else begin
            send_st    <= 0;
            m.wt_valid <= 0;
        end
    end

    // --- fft_done: send_en 하강 에지 = 마지막 결과 출력 완료 ---
    logic [1:0] send_en_d;
    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
            m.fft_done   <= 0;
            send_en_d    <= 0;
        end else begin
            send_en_d <= {send_en_d[0], send_en};
            m.fft_done <= (send_en_d == 2'b10) ? 1 : 0;  // 하강 에지 감지
        end
    end

    // -----------------------------------------------------------------------
    // 최대 진폭 추출 (스테이지 7 전용)
    // -----------------------------------------------------------------------
    // α-β 근사를 이용한 크기 추정: |X| ≈ max(|re|,|im|) + min(|re|,|im|)/2
    // 절댓값 → 비교 → 누적 최댓값 갱신의 3-클럭 파이프라인

    logic signed [31:0] com_re_k, com_im_k;

    // 1단계: 절댓값 계산 (2의 보수 → 크기)
    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
        end else if(m.stage == 7 && shift_d)begin
            com_re_k <= (re_k >= 0) ? re_k : ~re_k + 1;
            com_im_k <= (im_k >= 0) ? im_k : ~im_k + 1;
        end
    end

    // 2단계: max/min 분리
    logic signed [31:0] com_max, com_min;
    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
        end else if(m.stage == 7 && send_en)begin
            com_max <= (com_re_k >= com_im_k) ? com_re_k : com_im_k;
            com_min <= (com_re_k >= com_im_k) ? com_im_k : com_re_k;
        end
    end

    // 3단계: α-β 근사 크기 비교 → max_idx 갱신
    logic signed [31:0] com;
    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
            m.max_idx <= 0; com <= 0;
        end else begin
            if(m.stage == 7 && send_en_d[0])begin
                if(com < (com_max + com_min >>> 1))begin
                    com       <= (com_max + com_min >>> 1);
                    m.max_idx <= idx_d[8];
                end
            end
        end
    end

    // --- 시뮬레이션 로그: 각 스테이지 butterfly 결과 출력 ---
    always@(posedge m.wclk)begin
        for(int i=0; i<8; i=i+1)begin
            if(m.interval == i && send_en)begin
                $display(" %t X[k]:%d, %d+%di  X[k+N/2]:%d, %d+%di ",
                    $time,
                    send_addr0, $signed(fft_out0[31:16]), $signed(fft_out0[15:0]),
                    send_addr1, $signed(fft_out1[31:16]), $signed(fft_out1[15:0]));
            end
        end
    end

endmodule
