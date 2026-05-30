`timescale 1ns / 1ps
//==============================================================================
// 모듈명 : div_8
// 기  능 : uartclk을 8분주하여 UART 오버샘플링 기준 펄스(div_8)를 생성한다.
//
// 동작:
//   en_div=1이 되는 시점부터 카운터를 시작한다.
//   매 8 uartclk마다 div_8을 1클럭 High로 출력 (펄스 형태).
//   이 펄스를 UART_controller가 비트 타이밍 기준으로 사용한다.
//
// 오버샘플링 배율:
//   uartclk(14.7456MHz) / 8 / 16 = 115200 Hz (UART 비트 레이트)
//   → 각 비트 구간에서 16회 div_8 펄스 발생, 중앙(8번째)에서 샘플링
//==============================================================================

module div_8(FT.div8 mi0);

    logic [2:0] sig_cnt;  // 8분주 카운터 (0~7)

    always@(posedge mi0.uartclk or negedge mi0.rstn)begin
        if(!mi0.rstn)begin
            sig_cnt  <= 0;
            mi0.div_8 <= 0;
        end
        else begin
            if(mi0.en_div)begin
                if(sig_cnt == 7) begin
                    sig_cnt   <= 0;
                    mi0.div_8 <= 1;  // 8클럭마다 1클럭 펄스 출력
                end else begin
                    mi0.div_8 <= 0;
                    sig_cnt   <= sig_cnt + 1;
                end
            end
        end
    end

endmodule
