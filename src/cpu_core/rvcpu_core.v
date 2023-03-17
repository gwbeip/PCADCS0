// This is a simple multi-cycle processor core which satisfies RV64I ISA in machine mode only.
// Wenbo Guo
// 2023.03.16

module rvcpu_core(
    input wire            clk,
    input wire            rst,

    // simple interrupt signals
    input wire              mtime_intr, // timer interrupt signal, high active

    // memory access signals
    output wire             r_mem_ena, // read memory enable, high active
    output wire             w_mem_ena, // write memory enable, high active
    output wire [64-1 : 0]  rw_mem_addr,
    output wire [   2 : 0]  rw_mem_bytes, // number of bytes
    output wire [64-1 : 0]  w_mem_data,

    input wire              mem_data_ready, // indicates the validity of the data
    input wire [64-1 : 0]   mem_data // the data read from main memory

    // functional signal
    // output wire [63 : 0] value_x10,
    // output wire [63:0] pc_o
);

    // assign pc_o = pc;
    // assign value_x10 = RegFile.regs[10];

    reg [1:0] state, nxt_state;

    reg [63:0] pc;
    reg [63:0] nxt_pc;
    reg [31:0] inst;

    wire [6  :   2] opcode;
    wire [4  :   0] rs1;
    wire [4  :   0] rs2;
    wire [4  :   0] rd;
    wire [6  :   0] func7;
    wire [2  :   0] func3;


    wire [31:0] inst_i;
    wire is_32_inst;

    wire is_R_alu;
    wire is_R_alu_w;
    wire is_mret;
    wire is_I_alu;
    wire is_I_alu_w;
    wire is_csr;
    wire is_ecall;
    wire is_jalr;
    wire is_load;
    wire is_store;
    wire is_B;
    wire is_auipc;
    wire is_lui;
    wire is_J;

    wire is_I;
    wire is_S;
    wire is_U;
    reg is_B_jump;

    wire [63 : 0] imm;
    wire [  3:   0] mode_ALU;
    wire is_R_AL_OP;
    wire is_I_AL_OP;
    wire is_shift; //is atithmetic or ligical shift operation
    wire [1 : 0] n_bytes_ALU;
    wire w_rd_ena;
    wire [63:0] r_csr_data, csr_mtvec, csr_mepc;
    wire MIE, MTIE;
    wire [64-1 : 0] op1;
    wire [64-1 : 0] op2;
    wire [64-1 : 0] r_add_sub;
    wire [64-1 : 0] r_and;
    wire [64-1 : 0] r_or;
    wire [64-1 : 0] r_xor;
    wire [64-1 : 0] r_slt;
    wire is_equal, is_less_than, r_slt_lsb;
    wire s, s1, s2, c1, c2;

    wire [63:0] result_ALU;

    wire [63:0] w_rd_data, r_rs1_data, r_rs2_data;

    wire is_byte;
    wire expend_shift; 
    wire is_sll;
    wire [64-1 : 0] shift_op1;
    wire [5:0] shift_op2;
    reg [64-1 : 0] shift_op1_reverse;
    wire [64-1 : 0] shift_op1_in;
    wire [64-1 : 0] shift_result;
    reg [64-1 : 0] shift_result_reverse;
    wire [64-1 : 0] shift_out;

    assign r_mem_ena = (state==2'b00) | ( (state==2'b10) & (is_load) );
    assign w_mem_ena = (state==2'b10) & is_store;
    assign rw_mem_addr =    {64{(state==2'b00)}} & pc | 
                            {64{(state==2'b10)}} & result_ALU;
    assign rw_mem_bytes =   {3{(state==2'b00)}} & 3'b010 | 
                            {3{(state==2'b10)}} & func3;
    assign w_mem_data = {64{(state==2'b10)&is_store}} & r_rs2_data;


    always @(posedge clk or negedge rst) begin
        if(!rst) state <= 2'b0;
        else state <= nxt_state;
    end

    always@(*)case(state)
        2'b00: begin
            if(mem_data_ready) nxt_state = 2'b01;
            else nxt_state = 2'b00;
        end
        2'b01: begin
            if(is_load | is_store) nxt_state = 2'b10;
            else nxt_state = 2'b11;
        end
        2'b10: begin
            if(mem_data_ready) nxt_state = 2'b11;
            else nxt_state = 2'b10;
        end
        2'b11: begin
            if(is_32_inst) nxt_state = 2'b00;
            else nxt_state = 2'b00;
        end

    endcase



    always @(posedge clk or negedge rst) begin
        if(!rst) inst <= 32'h13;
        else if( (state==2'b00) & (mem_data_ready) ) inst <= mem_data[31:0];
        else inst <= inst;
    end

    always @(posedge clk or negedge rst) begin
        if(!rst) pc <= 64'h80000000;
        else begin
            if(state==2'b11) begin 
                // if(mtime_intr&MIE&MTIE) pc <= csr_mtvec;
                // else if(is_ecall) pc <= csr_mtvec; 
                // else if(is_mret) pc <= csr_mepc;
                // else if(is_jalr)  pc <= (r_rs1_data + imm) & ( ~(64'b1) );
                // else if(is_B & is_B_jump) pc <= pc + imm;
                // else if(is_J) pc <= pc + imm;
                // else pc <= pc + 64'd4;                
                pc <= (mtime_intr&MIE&MTIE) ? csr_mtvec : nxt_pc;
            end
            else pc <= pc;
        end
    end

    always @(*) begin
        // if(mtime_intr&MIE&MTIE) nxt_pc = csr_mtvec;
        if(is_ecall) nxt_pc = csr_mtvec; 
        else if(is_mret) nxt_pc = csr_mepc;
        else if(is_jalr)  nxt_pc = (r_rs1_data + imm) & ( ~(64'b1) );
        else if(is_B & is_B_jump) nxt_pc = pc + imm;
        else if(is_J) nxt_pc = pc + imm;
        else nxt_pc = pc + 64'd4;  
    end




    // INST input
    assign inst_i = inst;
    
    assign is_32_inst = inst_i[1:0] == 2'b11;


    //Instruction decode

    assign opcode = inst_i[ 6: 2];
    assign    rs1 = inst_i[19:15];
    assign    rs2 = inst_i[24:20];
    assign     rd = inst_i[11: 7];
    assign  func3 = inst_i[14:12];
    assign  func7 = inst_i[31:25];



    assign is_R_alu = (opcode == 5'b01100);
    assign is_R_alu_w = (opcode == 5'b01110);
    assign  is_mret = (opcode==5'b11100) & (func3==3'b000) & (func7==7'b0011000) & (rs2==5'b00010);
    assign is_I_alu = (opcode == 5'b00100);
    assign is_I_alu_w = (opcode == 5'b00110);
    assign is_csr = (opcode==5'b11100) & ( (func3==3'b011) | (func3==3'b111) | (func3==3'b010) | (func3==3'b110) | (func3==3'b001) | (func3==3'b101) );
    assign is_ecall = (opcode==5'b11100) & (func3==3'b000) & (func7==7'b0000000) & (rs2==5'b00000);
    assign is_jalr = (opcode==5'b11001) & (func3==3'b000);
    assign is_load = (opcode==5'b00000);
    assign is_store = (opcode==5'b01000);
    assign is_B = (opcode==5'b11000);
    assign is_auipc = opcode==5'b00101;
    assign is_lui = opcode==5'b01101;
    assign is_J = (opcode==5'b11011);

    assign is_I = ( is_I_alu | is_I_alu_w | is_csr | is_ecall | is_jalr | is_load );
    assign is_S = is_store;
    assign is_U = ( is_auipc | is_lui);


    assign imm =    ({64{is_I}} & {{(64-12){inst[32-1]}}, inst[31:20]}) |
                    ({64{is_S}} & {{(64-12){inst[32-1]}}, inst[31:25], inst[11:7]}) |
                    ({64{is_B}} & {{(64-13){inst[32-1]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0}) |
                    ({64{is_U}} & {{(64-32){inst[32-1]}}, inst[31:12], 12'b0}) |
                    ({64{is_J}} & {{(64-21){inst[32-1]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0});

    assign is_shift = (func3==3'b001 | func3==3'b101);
    assign is_R_AL_OP = ( is_R_alu | is_R_alu_w );
    assign is_I_AL_OP = ( is_I_alu | is_I_alu_w );
    assign mode_ALU =   ( {4{is_R_AL_OP}} & {inst_i[30], func3} ) |
                        ( {4{is_I_AL_OP}} & {inst_i[30]&is_shift, func3} ) |
                        ({4{is_load | is_store | is_jalr | is_csr | is_auipc}} & {4'b0000}); // B inst has special signals

    assign n_bytes_ALU = (opcode==5'b01100 || opcode==5'b00100) ? 2'b11 :
                                (opcode==5'b01110 || opcode==5'b00110) ? 2'b10 :
                                2'b11;
    

    assign w_rd_ena = ( is_R_AL_OP | is_I_AL_OP | is_load | is_J | is_jalr | is_U | is_csr ) & (state==2'b11);
    assign w_rd_data =  {64{is_R_AL_OP}} & result_ALU | 
                        {64{is_I_AL_OP}} & result_ALU |
                        {64{is_load   }} & mem_data   |
                        {64{is_J      }} & pc + 64'd4 |
                        {64{is_jalr   }} & pc + 64'd4 |
                        {64{is_U      }} & result_ALU |
                        {64{is_csr    }} & r_csr_data ;
                        


    regfile RegFile(
        .clk(clk),
        .rst(rst),

        .w_addr(rd),
        .w_data(w_rd_data),
        .w_ena(w_rd_ena),

        .r_addr1(rs1),
        .r_data1(r_rs1_data),

        .r_addr2(rs2),
        .r_data2(r_rs2_data)
        );





    csr CSR(
            .clk(clk),
            .rst(rst),

            .is_ecall(is_ecall & (state==2'b11)),
            .is_mret(is_mret & (state==2'b11)),
            
            .w_addr(imm[11:0]),
            .w_data(func3[2] ? {59'b0, rs1} : r_rs1_data),
            .w_ena(is_csr & (state==2'b11)),
            .w_mode(func3[1:0]),
            
            .r_addr(imm[11:0]),

            .MIE(MIE),
            .MTIE(MTIE),

            .r_data(r_csr_data),
            .csr_mtvec_o(csr_mtvec),
            .csr_mepc_o(csr_mepc),
            .pc_from_ex(pc),
            .pc_intr(nxt_pc),

            .inst_valid(state==2'b11),
            .mtime_intr_i(mtime_intr & (state==2'b11)),
            .mtime_intr_enable_i(mtime_intr & MIE & MTIE & (state==2'b11))

            // .r_exception(),
            // .w_exception()
    );



    // ALU

    assign op1 =    {64{is_R_AL_OP}} & r_rs1_data |
                    {64{is_I_AL_OP}} & r_rs1_data | 
                    {64{is_load}}    & r_rs1_data |
                    {64{is_store}}   & r_rs1_data |
                    {64{is_B}}       & r_rs1_data |
                    {64{is_auipc}}   & pc         |
                    {64{is_lui}}     & 64'b0      |
                    {64{is_jalr}}    & pc         |
                    {64{is_J}}       & pc         ;
    
    
    // (is_lui_i | is_csr_i) ? 64'b0 :
    //                 is_auipc_i ? pc_ID_i :
    //                 ((rs1 == rd_ex_mem)&wb_ena_ex_mem) ? wb_data_ex_mem :
    //                 ((rs1 == rd_mem_wb)&wb_ena_mem_wb) ? wb_mem_data_wb : 
    //                 op1_alu;
    assign op2 =    {64{is_R_AL_OP}} & r_rs2_data |
                    {64{is_I_AL_OP}} & imm        |
                    {64{is_load}}    & imm        |
                    {64{is_store}}   & imm        |
                    {64{is_B}}       & r_rs2_data |
                    {64{is_auipc}}   & imm        |
                    {64{is_lui}}     & imm        |
                    {64{is_jalr}}    & 64'd4      |
                    {64{is_J}}       & 64'd4      ;


    
    // (is_I_AL_OP | load_ena_i | store_ena_i | is_csr_i) ? op2_alu :
    //                 ((rs2 == rd_ex_mem)&wb_ena_ex_mem) ? wb_data_ex_mem :
    //                 ((rs2 == rd_mem_wb)&wb_ena_mem_wb) ? wb_mem_data_wb : 
    //                 op2_alu;
    

    // Addition and subtraction
    assign r_add_sub = op1 + (op2^{64{mode_ALU[3]}}) + {63'b0, mode_ALU[3]};

    // AND
    assign r_and = op1 & op2;

    // OR
    assign r_or = op1 | op2;

    // XOR
    assign r_xor = op1 ^ op2;

    // set less than : signed and unsigned
    assign is_equal = (op1[64-2:0] == op2[64-2:0]);
    assign is_less_than = (op1[64-2:0] < op2[64-2:0]);
    assign s = mode_ALU[0];
    assign s1 = op1[64-1];
    assign s2 = op2[64-1];
    assign c1 = is_less_than;
    assign c2 = is_equal;
    // assign r_slt_lsb = ((~s)&s1&(~s2)) | (s&(~s1)&s2) | (c1&(s|((~s1)&(~s2)))) | (((~s)&s1)&((~c1)&(~c2)));
    assign r_slt_lsb = s ? (({s1, s2}==2'b01) | ((s1==s2)&c1)) : (({s1, s2}==2'b10) | ((s1==s2)&c1));
    assign r_slt = {{(64-1){1'b0}}, r_slt_lsb};

    // Shift
    assign is_sll = (mode_ALU[2:0] == 3'b001);
    assign is_byte = (n_bytes_ALU == 2'b10);
    assign expend_shift = op1[31] & mode_ALU[3];

    assign shift_op1 = is_byte ? {{32{expend_shift}}, op1[31:0]} : op1;

    assign shift_op2 = {op2[5]&(~is_byte), op2[4:0]};

    integer i;
    always@(*)
    for(i=0; i<64; i=i+1) begin
        shift_op1_reverse[i] = shift_op1[63-i];
    end

    assign shift_op1_in = is_sll ? shift_op1 : shift_op1_reverse;

    assign shift_result = (shift_op1_in << shift_op2);
    
    always@(*)
    for(i=0; i<64; i=i+1) begin
        shift_result_reverse[i] = shift_result[63-i];
    end

    assign shift_out = is_sll ? shift_result : shift_result_reverse;

    wire signed [64-1 : 0] sign_original;
    assign sign_original = {mode_ALU[3] & (n_bytes_ALU==2'b11) & op1[63], 63'b0};

    wire signed [64-1 : 0] sign_out;
    assign sign_out = (sign_original >>> shift_op2);

    wire [64-1 : 0] sub_result;
    assign sub_result = shift_out | sign_out;

    wire [64-1 : 0] r_shift;
    assign r_shift = (n_bytes_ALU==2'b11) ? sub_result : {{32{sub_result[31]}}, sub_result[31:0]};

    reg [64-1 : 0] result_ALU_no_signext;
    always @(*) case(mode_ALU[2:0])
        3'b000        : result_ALU_no_signext = r_add_sub;
        3'b001, 3'b101: result_ALU_no_signext = r_shift;
        3'b010, 3'b011: result_ALU_no_signext = r_slt;
        3'b100        : result_ALU_no_signext = r_xor;
        3'b110        : result_ALU_no_signext = r_or;
        3'b111        : result_ALU_no_signext = r_and;
    endcase

    assign result_ALU = is_byte ? {{32{result_ALU_no_signext[31]}}, result_ALU_no_signext[31:0]} : result_ALU_no_signext;


    always @ (*) case (func3)
        3'b000 : is_B_jump = (s1==s2) & c2;
        // 3'b101 : is_B_jump_wire = ((s1==s2)&c2) | (s1==1'b0 & s2==1'b1) | (({s1, s2}==2'b00)&({c1, c2}==2'b00)) | (({s1, s2}==2'b11)&c1); 
        3'b101 : is_B_jump = ((s1==s2)&c2) | (s1==1'b0 & s2==1'b1) | ((s1==s2)&({c1, c2}==2'b00)); 
        3'b111 : is_B_jump = ((s1==s2)&c2) | (s1==1'b1 & s2==1'b0) | (s1==s2 & {c1, c2}==2'b00); 
        // 3'b100 : is_B_jump_wire = (s1==1'b1 & s2==1'b0) | (({s1, s2}==2'b00)&c1) | (({s1, s2}==2'b11)&({c1, c2}==2'b00)); 
        3'b100 : is_B_jump = (s1==1'b1 & s2==1'b0) | ((s1==s2)&c1); 
        3'b110 : is_B_jump = ((s1==s2)&c1) | ({s1, s2}==2'b01); 
        3'b001 : is_B_jump = ~((s1==s2)&c2);
        default: is_B_jump = 1'b0;
    endcase

endmodule
