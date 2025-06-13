module top(clk, reset);
input clk, reset;
wire [31:0] PC_top, instruction_top, Rd1_top, Rd2_top, ImmExt_top, mux1_top, Sum_out_top, NextoPC_top, PCin_top, addres s_top, Memdata_top, WriteBack_top;
wire RegWrite_top, ALUSrc_top, zero_top, branch_top, sel2_top, MemtoReg_top, MemWrite_top, MemRead_top;
wire [1:0] ALUOp_top;
wire [3:0] control_top;
 
Program_Counter PC(.clk(clk), reset(reset), PC_in (PCin_top), PC_out (PC_top));
 
PCplus4 PC_Adder(.fromPC (PC_top), NextoPC(NextoPC_top));
 
Instruction_Memory Inst_Memory(.clk(clk), reset(reset), read_address (PC_top), instruction_out(instruction_top));
 
Reg_File Reg_File(.clk(clk), .reset(reset), RegWrite(RegWrite_top), Rs1(instruction_top [19:15]), Rs2 (instruction_top [24:20]), Rd(instruction_top[11:7]), Write_data(WriteBack_top), .read_data1 (Rd1_top), .read_data2(Rd2_top));
 
ImmGen ImmGen(.Opcode (instruction_top [6:0]), instruction (instruction_top), ImmExt (ImmExt_top));
 
Control_Unit Control_Unit(instruction (instruction_top [6:0]), Branch (branch_top), MemRead (MemRead_top), MemtoReg (Me mtoReg_top), ALUOP (ALUOp_top), MemWrite(MemWrite_top), ALUSrc (ALUSrc_top), RegWrite(RegWrite_top));
 
ALU_Control ALU_Control (ALUOP (ALUOp_top), fun7 (instruction_top[30]), .fun3(instruction_top [14:12]), Control_out(control_top));
 
ALU_unit ALU(.A(Rd1_top), B(mux1_top), Control_in(control_top), ALU_Result (address_top), zero (zero_top));
 
Mux1 ALU_mux(.sel1 (ALUSrc_top), A1 (Rd2_top), .81 (ImmExt_top), Mux1_out(mux1_top));
 
Adder Adder(.in_1(PC_top), in_2(ImmExt_top), Sum_out (Sum_out_top));
 
AND logic AND(.branch (branch_top), zero (zero_top), and_out (sel2_top));
 
Mux2 Adder_mux(.sel2(sel2_top), A2 (NextoPC_top), B2(Sum_out_top), Mux2_out (PCin_top));
 
Data Memory Data_mem(.clk(clk), reset (reset), MemWrite(MemWrite_top), MemRead (MemRead_top), .read_address(address_t op), Write_data (Rd2_top), MemData_out (Memdata_top));
 
Mux3 Memory_mux(.sel3(MemtoReg_top), A3(address_top), B3(Memdata_top), Mux3_out WriteBack_top);
endmodule
