`timescale 1ns / 1ps
//==============================================================================
// 모듈명 : FND_ctrl
// 기  능 : FFT 최대 진폭 주파수 빈 인덱스(max_idx, 9bit)를
//          4자리 Common Anode 7-세그먼트 디스플레이에 10진수로 표시한다.
//
// 동작:
//   1ms마다 an을 순환하여 4자리를 시분할 구동 (Dynamic Scanning).
//   max_idx(0~255)를 Double Dabble 알고리즘으로 BCD 3자리로 변환 후
//   현재 선택된 자릿수(an)에 맞는 세그먼트 코드를 seg에 출력한다.
//
// Double Dabble (Binary → BCD 변환):
//   MSB부터 1비트씩 BCD 레지스터에 시프트 인.
//   각 시프트 전에 BCD 자리가 5 이상이면 +3 보정 (BCD 올림 규칙).
//   9bit 입력 → 최대 511 → 백의 자리 최대 5, 3자리 BCD로 표현 가능.
//
// 7-세그먼트 인코딩: Common Anode (Low = 점등)
//   fnd_decode_dec[0~9]: 0~9 숫자 세그먼트 코드
//==============================================================================

module FND_ctrl(FT.fnd m);

    // --- 1ms 타이머 (100MHz 기준, 49999 카운트 = 1ms) ---
    logic [15:0] usec_cnt;
    logic usec;
    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
            usec <= 0; usec_cnt <= 0;
        end else if(m.fnd_en) begin
            if(usec_cnt == 49999)begin
                usec_cnt <= 0;
                usec     <= ~usec;  // 1ms마다 토글
            end else begin
                usec_cnt <= usec_cnt + 1;
            end
        end
    end

    // --- usec 에지 검출: 자리 전환 트리거 ---
    logic cp;
    logic usec_pedge, usec_nedge;
    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
            cp <= 0; usec_pedge <= 0; usec_nedge <= 0;
        end else if(m.fnd_en) begin
            cp         <= usec;
            usec_pedge <= ({cp, usec} == 2'b01) ? 1 : 0;
            usec_nedge <= ({cp, usec} == 2'b10) ? 1 : 0;
        end
    end

    // --- 자리 선택 순환 (1110 → 1101 → 1011 → 0111 → 1110, 1ms 간격) ---
    always@(posedge m.wclk, negedge m.rstn)begin
        if(!m.rstn)begin
            m.an <= 4'b0000;
        end else if(m.fnd_en) begin
            if(usec_pedge)begin
                case(m.an)
                    4'b1110 : m.an = 4'b1101;
                    4'b1101 : m.an = 4'b1011;
                    4'b1011 : m.an = 4'b0111;
                    4'b0111 : m.an = 4'b1110;
                    default : m.an = 4'b1110;  // 초기 자리: 최하위 자리
                endcase
            end
        end
    end

    // --- 7-세그먼트 디코드 테이블 (Common Anode, Active Low) ---
    logic [7:0] fnd_decode_dec[0:9] =
        {8'hC0, 8'hF9, 8'hA4, 8'hB0, 8'h99,
         8'h92, 8'h82, 8'hF8, 8'h80, 8'h90};
    logic [7:0] fnd_decode_hex[0:15] =
        {8'hC0, 8'hF9, 8'hA4, 8'hB0, 8'h99, 8'h92, 8'h82, 8'hF8,
         8'h80, 8'h90, 8'h88, 8'h83, 8'hC6, 8'hA1, 8'h86, 8'h8E};

    // --- Double Dabble: max_idx(9bit 이진수) → BCD 3자리 변환 ---
    logic [11:0] bin_to_dec;  // {백의자리[11:8], 십의자리[7:4], 일의자리[3:0]}
    always_comb begin
        if(m.fnd_en) begin
            bin_to_dec = 0;
            for(int i=8; i>=0; i=i-1)begin
                // 각 BCD 자리가 5 이상이면 +3 (시프트 전 보정)
                if(bin_to_dec[3:0]  >= 5) bin_to_dec[3:0]  = bin_to_dec[3:0]  + 3;
                if(bin_to_dec[7:4]  >= 5) bin_to_dec[7:4]  = bin_to_dec[7:4]  + 3;
                if(bin_to_dec[11:8] >= 5) bin_to_dec[11:8] = bin_to_dec[11:8] + 3;
                bin_to_dec = {bin_to_dec[10:0], m.max_idx[i]};  // 1비트 시프트 인
            end
        end
    end

    // --- 현재 an에 해당하는 BCD 자리 선택 ---
    logic [3:0] select_bits;
    always_comb begin
        case(m.an)
            4'b1110 : select_bits = bin_to_dec[3:0];   // 일의 자리
            4'b1101 : select_bits = bin_to_dec[7:4];   // 십의 자리
            4'b1011 : select_bits = bin_to_dec[11:8];  // 백의 자리
            default : select_bits = 0;
        endcase
    end

    assign m.seg = fnd_decode_dec[select_bits];

endmodule
