
module regfile(
    input  wire clk,
	input  wire rst,
	
	input  wire  [4  : 0] w_addr,
	input  wire  [64-1 : 0] w_data,
	input  wire 		  w_ena,
	
	input  wire  [4  : 0] r_addr1,
	output reg   [64-1 : 0] r_data1,
	
	input  wire  [4  : 0] r_addr2,
	output reg   [64-1 : 0] r_data2
    );

    // 32 registers
	reg [64-1 : 0] 	regs[0 : 31];
	
	always @(posedge clk or negedge rst) 
	begin
		if ( rst == 1'b0 ) 
		begin
			regs[ 0] <= 64'h0;
			regs[ 1] <= 64'h0;
			regs[ 2] <= 64'h0;
			regs[ 3] <= 64'h0;
			regs[ 4] <= 64'h0;
			regs[ 5] <= 64'h0;
			regs[ 6] <= 64'h0;
			regs[ 7] <= 64'h0;
			regs[ 8] <= 64'h0;
			regs[ 9] <= 64'h0;
			regs[10] <= 64'h0;
			regs[11] <= 64'h0;
			regs[12] <= 64'h0;
			regs[13] <= 64'h0;
			regs[14] <= 64'h0;
			regs[15] <= 64'h0;
			regs[16] <= 64'h0;
			regs[17] <= 64'h0;
			regs[18] <= 64'h0;
			regs[19] <= 64'h0;
			regs[20] <= 64'h0;
			regs[21] <= 64'h0;
			regs[22] <= 64'h0;
			regs[23] <= 64'h0;
			regs[24] <= 64'h0;
			regs[25] <= 64'h0;
			regs[26] <= 64'h0;
			regs[27] <= 64'h0;
			regs[28] <= 64'h0;
			regs[29] <= 64'h0;
			regs[30] <= 64'h0;
			regs[31] <= 64'h0;
		end
		else 
		begin
			if ((w_ena == 1'b1) && (w_addr != 5'h00))	
				regs[w_addr] <= w_data;
		end
	end
	
	always @(*) begin
		if (rst == 1'b0)
			r_data1 = 64'h0;
		else
			// r_data1 = (r_addr1 == w_addr)&w_ena ? w_data : regs[r_addr1];
			r_data1 = regs[r_addr1];
	end
	
	always @(*) begin
		if (rst == 1'b0)
			r_data2 = 64'h0;
		else
			// r_data2 = (r_addr2 == w_addr)&w_ena ? w_data : regs[r_addr2];
			r_data2 = regs[r_addr2];
	end


	// wire [63:0] rr1;
	// assign rr1 = regs[1];

endmodule
