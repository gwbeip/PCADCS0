// This is the top module of RVCPU.
// Wenbo Guo
// 2023.03.16

module rvcpu(
    input wire clk,
    input wire rst,

    // with main memory
    output          mode, // mode=0 -> read; mode -> 1 write;
    output          valid,
    input           ready,
    output  [63:0]  addr,
    output  [ 7:0]  w_data,
    input           r_data_valid,
    input [ 7:0]    r_data
);


    wire              r_mem_ena;
    wire              w_mem_ena;
    wire  [64-1 : 0]  rw_mem_addr;
    wire  [   2 : 0]  rw_mem_bytes;
    wire  [64-1 : 0]  w_mem_data;

    wire             mem_data_ready;
    wire [64-1 : 0]  mem_data;


    rvcpu_core cpu_core(
        .clk(clk),
        .rst(rst),

        .mtime_intr(1'b0),

        .r_mem_ena(r_mem_ena),
        .w_mem_ena(w_mem_ena),
        .rw_mem_addr(rw_mem_addr),
        .rw_mem_bytes(rw_mem_bytes),
        .w_mem_data(w_mem_data),

        .mem_data_ready(mem_data_ready),
        .mem_data(mem_data)
    );

    memory_access_ctrl mac(
        .clk(clk),
        .rst(rst),

        // with cpu
        .r_mem_ena(r_mem_ena),
        .w_mem_ena(w_mem_ena),
        .rw_mem_addr(rw_mem_addr),
        .rw_mem_bytes(rw_mem_bytes),
        .w_mem_data(w_mem_data),

        .mem_data_ready(mem_data_ready),
        .mem_data(mem_data),

        //with main memory
        .mode(mode), // mode=0 -> read; mode -> 1 write;
        .valid(valid),
        .ready(ready),
        .addr(addr),
        .w_data(w_data),
        .r_data_valid(r_data_valid),
        .r_data(r_data)
    );


endmodule
