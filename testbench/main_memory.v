// Fake main memory for simulation
// Wenbo Guo
// 2023.03.14

module main_memory(
    input clk, // posedge active
    input rst, // low active

    // read/write channel
    input           mode, // mode=0 -> read; mode -> 1 write;
    input           valid,
    output          ready,
    input  [63:0]   addr,
    input  [ 7:0]   w_data,
    output          r_data_valid,
    output [ 7:0]   r_data,

    // Interact with simulation
    output          invalid_addr, // invalid memory access
    output          write_char_io, // write a character into address 0x1000
    output          simulation_stop // end the similation, i.e., write 0xff into address 0x2000
);

    reg [7:0] mem [0 : 65536-1]; // main memory
    initial begin
        $readmemh("program.hex", mem); // program file, hexadecimal
    end

    // measure the simulation status
    wire read_addr_is_valid = (~mode) & (addr[63:16] == 48'h8000);
    wire write_addr_is_valid = mode & ((addr[63:16] == 48'h8000) | (addr == 64'h1000) | (addr == 64'h2000));
    assign invalid_addr = valid & ((~read_addr_is_valid) & (~write_addr_is_valid));
    assign write_char_io = valid & mode & addr == 64'h1000;
    assign simulation_stop = valid & mode & addr == 64'h2000 & w_data == 8'hff;

    // read memory
    assign r_data_valid = valid & (~mode);
    assign r_data = {8{read_addr_is_valid & valid}} & mem[addr[15:0]];

    // write memoey
    always @(posedge clk) begin
        if(valid & mode & (addr[63:16] == 48'h8000)) begin
            mem[addr[15:0]] <= w_data;
        end
    end

    reg [7:0] char_0x1000;
    always @(posedge clk) begin
        if(valid & mode & (addr == 64'h1000)) begin
            char_0x1000 <= w_data;
            $write("%c", w_data);
        end
    end

    assign ready = 1'b1;


endmodule