`timescale 1ns / 1ps

module memory
#(
    parameter SIZE = 8
)(
    input logic uartclk,
    input logic wclk,
    input logic rstn,
    input logic wt_valid,
    input logic [SIZE-1:0] in_dt_wt0,
    input logic [SIZE-1:0] in_dt_wt1,
    input logic rd_ready,
    input logic [8:0] addr_wt0,
    input logic [8:0] addr_wt1,
    input logic [8:0] addr_rd0,
    input logic [8:0] addr_rd1,

    input logic fft_en,
    output logic rd_valid,
    output logic [SIZE-1:0] fifo_dt0,
    output logic [SIZE-1:0] fifo_dt1,
    output logic wt_ready
);

    function [7:0] bit_reverse(input [7:0] wt_ptr);
        logic [7:0] temp;
        for(int i=0; i<8;i=i+1)begin
            temp[7-i] = wt_ptr[i];
        end
        return temp;
    endfunction

   (* ram_style = "block" *) logic [SIZE-1:0] memory[0:255];
    logic [8:0] wt_ptr,rd_ptr;

    logic [8:0] wt_ptr_d[0:1];
    logic [8:0] rd_ptr_d[0:1];
    always@(posedge uartclk,negedge rstn)begin
        if(!rstn)begin
            rd_ptr <= 0;
            rd_ptr_d[0] <= 0; rd_ptr_d[1] <= 0;
        end else begin
            rd_ptr_d[0] <= rd_ptr;
            rd_ptr_d[1] <= rd_ptr_d[0];
        end
    end
    always@(posedge wclk)begin
        if(!rstn)begin
            wt_ptr_d[0] <= 0; wt_ptr_d[1] <= 0;
        end else begin
            wt_ptr_d[0] <= wt_ptr;
            wt_ptr_d[1] <= wt_ptr_d[0];
        end
    end

    logic [2:0] wt_valid_d;
    always@(posedge wclk)begin
        if(!rstn)begin
            wt_valid_d <= 0;
        end else begin
            wt_valid_d <= {wt_valid_d[1:0], wt_valid};
        end
    end

    logic wt_valid_pulse;
    always@(posedge wclk)begin
        wt_valid_pulse <= (wt_valid_d[2:1] == 2'b10) ? 1 : 0;
    end

    /* Port A */
    always@(posedge wclk)begin
        if(fft_en) begin
            if(wt_valid)begin
                memory[addr_wt0] <= in_dt_wt0;
            end
            if(rd_ready)begin
                fifo_dt0 <= memory[addr_rd0];
            end
        end
        else begin
            if(wt_valid_pulse) memory[bit_reverse(wt_ptr[7:0])] <= in_dt_wt0;
        end
    end

    always@(posedge wclk,negedge rstn)begin
        if(!rstn)begin
            rd_valid <= 0;
            wt_ptr <= 0; wt_ready <= 1;
        end
        else begin
            if(wt_valid_pulse && wt_ptr_d[1][8] == rd_ptr_d[1][8] )begin
                wt_ptr <= wt_ptr + 1;
                wt_ready <= 1;
            end
            if(rd_ready)begin
                rd_valid <= 1;
            end
            else rd_valid <= 0;
        end
    end

    /* Port B */
    always@(posedge wclk)begin
        if(fft_en) begin
            if(wt_valid)begin
                memory[addr_wt1] <= in_dt_wt1;
            end
            if(rd_ready)begin
                fifo_dt1 <= memory[addr_rd1];
            end
        end
    end

endmodule
