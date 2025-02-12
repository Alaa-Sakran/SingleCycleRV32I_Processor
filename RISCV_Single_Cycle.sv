module RISCV_Single_Cycle(input clk,reset);

logic [31:0] PC,ALUOUT,MUX2OUTPUT,PCNEXT,Instruction;
logic [3:0] ALUOP;
logic [2:0] EXTDEC_CODE;
logic WEM,MUX3INPUT,M1,J,WER,zero,LessThan,LUI;
logic [1:0] M2,M3;

datapath dp(LUI,clk,reset,Instruction,PC,ALUOUT,MUX2OUTPUT,PCNEXT,LessThan,zero,
M2,M3, WEM,MUX3INPUT,M1,J,WER,EXTDEC_CODE,ALUOP); 

controller control_unit(LUI,Instruction,M2,M3,WEM,MUX3INPUT,M1,J,WER,EXTDEC_CODE,zero,LessThan,ALUOP);

endmodule 

module controller(output logic LUI, input logic [31:0] Instruction,output logic [1:0] M2,M3, output logic WEM,MUX3INPUT,M1,J,WER,output logic [2:0] EXTDEC_CODE,
input logic zero,LessThan, output logic [3:0] ALUOP);

logic [2:0] ALUOPDEC;
main_dec U1(LUI,Instruction[6:0],M2,WER,WEM,M1,J,EXTDEC_CODE,ALUOPDEC);
ALUDEC aludec(ALUOPDEC,Instruction[14:12], Instruction[31:25],ALUOP,M3);
assign MUX3INPUT=(zero&(M3==2'b01))|(LessThan&(M3==2'b10))|(J);
endmodule

module datapath(input logic LUI,input logic clk,reset, output logic [31:0] Instruction, output logic [31:0] PC,ALUOUT,MUX2OUTPUT,PCNEXT, output logic LessThan,zero,
input logic [1:0] M2,M3,
input logic WEM,MUX3INPUT,M1,J,WER, input logic [2:0] EXTDEC_CODE, input logic [3:0] ALUOP);

logic [31:0] RDA1,RDA2,MUX1OUTPUT,EXTOUT,BranchAddress,RDB1,MUX3OUTPUT;
instruction_memory IM1(PC,Instruction);
REGFILE regfile(LUI,Instruction[19:15], Instruction[24:20], Instruction[11:7], WER,clk,reset,MUX2OUTPUT,RDA1,RDA2);
Extendor E1(EXTDEC_CODE,Instruction[31:7],EXTOUT);
SimpleAdder A1(PC,32'd4,PCNEXT);
flop #(32) F1(clk,reset,MUX3OUTPUT,PC);
ALU alu(RDA1,MUX1OUTPUT,ALUOP,ALUOUT,zero,LessThan);
SimpleAdder A2(EXTOUT,PC,BranchAddress);
data_memory DM(ALUOUT,RDA2,WEM,clk,RDB1);
MUX2TO1 #(32) MUX1(RDA2,EXTOUT,M1,MUX1OUTPUT);
MUX4TO1 #(32) MUX2(ALUOUT,RDB1,PCNEXT,EXTOUT,M2,MUX2OUTPUT);
MUX2TO1 #(32) MUX3(PCNEXT,BranchAddress,MUX3INPUT,MUX3OUTPUT);

endmodule

module ALU( input logic [31:0] operand1,operand2, input logic [3:0] ALUOP, output logic [31:0] result, output logic zero,LessThan);
always_comb //operand1 is RDA1 while operand2 is the output of MUX2, result is also connected to both data memory and MUX3
begin
case(ALUOP)
4'b0000: result=operand1+operand2;
4'b0001: result=operand1-operand2;
4'b0010: result=operand1&operand2;
4'b0011: result=operand1|operand2;
4'b0100: result=operand1^operand2;
4'b0101: result=operand1>>operand2;
4'b0110: result=operand1<<operand2;
4'b0111: result=operand1>>>operand2;
4'b1000: result=(operand1<operand2)?32'd1:32'd0;
default: result=result;
endcase
end
assign zero=(result==32'b0);
assign LessThan=result[31];
endmodule 


module Extendor(input logic [2:0] EXT_CODE, input logic [24:0]IMMIN, output logic [31:0]IMMOUT); //extendor will have [31:7]
always_comb
begin
case(EXT_CODE)
3'b000: IMMOUT = {{20{IMMIN[24]}}, IMMIN[24:13]}; //lw,immediate
3'b001: IMMOUT = {{20{IMMIN[24]}}, IMMIN[24:18], IMMIN[4:0]};//s-type
3'b010: IMMOUT = {{20{IMMIN[24]}}, IMMIN[24], IMMIN[0], IMMIN[23:18],IMMIN[4:1],1'b0};//branch
3'b011: IMMOUT = {{11{IMMIN[24]}}, IMMIN[24], IMMIN[12:5], IMMIN[13], IMMIN[23:14], 1'b0 }; //jump
3'b111: IMMOUT = {IMMIN[24:5],{12{1'bx}}};//Lui
default: IMMOUT=IMMOUT;
endcase

end

endmodule

module flop #(parameter WIDTH=8) (input logic clk, reset, input logic [WIDTH-1:0] D, output logic [WIDTH-1:0] Q);

always_ff @(posedge clk, negedge reset)
begin
if(!reset)
Q<=0;
else
Q<=D;
end
endmodule

module MUX2TO1 #(parameter WIDTH=8)(input logic [WIDTH-1:0] MUXIN1, MUXIN2, input logic select, output logic [WIDTH-1:0] MUXOUT);
assign MUXOUT = select?MUXIN2:MUXIN1;
endmodule

module MUX4TO1 #(parameter WIDTH=8)(input logic [WIDTH-1:0] MUXIN1, MUXIN2,MUXIN3,MUXIN4 , input logic [1:0] select, output logic [WIDTH-1:0] MUXOUT);
always_comb
begin
case(select)
2'b00: MUXOUT=MUXIN1;
2'b01: MUXOUT=MUXIN2;
2'b10: MUXOUT=MUXIN3;
2'b11: MUXOUT=MUXIN4;
default: MUXOUT=MUXOUT;
endcase
end
endmodule

module SimpleAdder(input logic [31:0]INPUT1, INPUT2, output logic [31:0] OUTPUT);
always_comb
begin
OUTPUT= INPUT1+INPUT2;
end

endmodule

module REGFILE (input logic LUI,input logic [4:0] A1,A2,A3, input logic WER,clk,reset, input logic [31:0] WDR, output logic [31:0]RDA1, RDA2);
logic [31:0] REGS[31:0];


always_ff @(posedge clk, negedge reset)
begin
if(!reset) for (int i=0; i<32; i++)begin REGS[i]<=32'b0; end
if(WER) if(A3!=0)begin if(LUI)begin REGS[A3][31:12]<=WDR[31:12];  end else REGS[A3]<=WDR; end
end
always_comb
begin
RDA1=REGS[A1];
RDA2=REGS[A2];
end
endmodule

module main_dec(output logic LUI, input logic [6:0] OPCODE, output logic [1:0] M2, output logic WER,WEM,M1,J, output logic [2:0]EXT_CODE, output logic [2:0] ALUOPDEC);
always_comb
begin
case(OPCODE)
7'b0000011:begin  WER=1; WEM=0; M1=1;LUI=0; M2=2'b01;  J=0; ALUOPDEC=3'b010; EXT_CODE=3'b000;  end //load
7'b0010011:begin  WER=1; WEM=0; M1=1;LUI=0; M2=2'b00;  J=0; ALUOPDEC=3'b000; EXT_CODE=3'b000;  end //addi..
7'b1100111:begin  WER=1; WEM=0; M1=1;LUI=0; M2=2'b10;  J=1; ALUOPDEC=3'b000; EXT_CODE=3'b000;  end //jalr
7'b0100011:begin  WER=0; WEM=1; M1=1;   J=0; ALUOPDEC=3'b010; EXT_CODE=3'b001;  end //s-type
7'b0110011:begin  WER=1; WEM=0; M1=0;LUI=0; M2=2'b00;  J=0; ALUOPDEC=3'b100;   end //R-type
7'b0110111:begin  WER=1; WEM=0; M2=2'b11;LUI=1;  J=0; EXT_CODE=3'b111;   end //U-type
7'b1100011:begin  WER=0; WEM=0; M1=0;  J=0; ALUOPDEC=3'b110; EXT_CODE=3'b010;   end //B-type
7'b1101111:begin  WER=1; WEM=0; M2=2'b10;LUI=0;  J=1;  EXT_CODE=3'b011;   end //J
default:  begin WER=0; WEM=0;J=0; end //make sure if a false OPCODE was inserted no writing to existing data occurs
endcase

end
endmodule
module ALUDEC(input logic [2:0] ALUOPDEC, input logic [2:0] FUNCT3,input logic [6:0] FUNCT7 , output logic [3:0] ALUOP, output logic [1:0] M3);
always_comb
begin
case(ALUOPDEC)
3'b000:begin  
	case(FUNCT3)
	3'b000:begin ALUOP=4'b0000; M3=2'b00; end  //addi
	3'b010:begin ALUOP=4'b1000; M3=2'b00; end //slti
	3'b100:begin ALUOP=4'b0100; M3=2'b00; end //xori
	3'b110:begin ALUOP=4'b0011; M3=2'b00; end //ori
	3'b111:begin ALUOP=4'b0010; M3=2'b00; end //andi
	3'b001:begin ALUOP=4'b0110; M3=2'b00; end //slli
	3'b101:begin if(FUNCT7==7'd0) begin ALUOP=4'b0101; M3=2'b00; end if(FUNCT7==7'b0100000) begin ALUOP=4'b0111; M3=2'b00; end end //srl, sra
	default: begin ALUOP=4'bxxxx; M3=2'b00; end //ALUOP could be anything it won't matter as long as write controls are LOW
	endcase   end

3'b100:begin   if(FUNCT7==7'd0) begin
	case(FUNCT3)
	3'b000:begin ALUOP=4'b0000; M3=2'b00; end  //add
	3'b010:begin ALUOP=4'b1000; M3=2'b00; end //slt
	3'b100:begin ALUOP=4'b0100; M3=2'b00; end //xor
	3'b110:begin ALUOP=4'b0011; M3=2'b00; end //or
	3'b111:begin ALUOP=4'b0010; M3=2'b00; end //and
	3'b001:begin ALUOP=4'b0110; M3=2'b00; end //sll
	3'b101:begin  ALUOP=4'b0101; M3=2'b00; end 
	default: begin ALUOP=4'bxxxx; M3=2'b00; end
	endcase   end
   
if(FUNCT7==7'b0100000) begin  case(FUNCT3) 
	 3'b000:begin ALUOP=4'b0001; M3=2'b00; end //sub
	 3'b101:begin ALUOP=4'b0111; M3=2'b00; end //sra
	 default: begin ALUOP=4'bxxxx; M3=2'b00; end
	 endcase end
end
3'b110:begin
case(FUNCT3)
3'b000: begin ALUOP=4'b0001; M3=2'b01; end //beq
3'b100: begin ALUOP=4'b0001; M3=2'b10; end //blt
default:begin ALUOP=4'bxxxx; M3=2'b00; end
endcase

end
3'b010:begin if(FUNCT3==3'b010)begin ALUOP=4'b0000; M3=2'b00; end end //lw,sw
default: begin ALUOP=4'bxxxx; M3=2'b00; end
endcase
end
endmodule

module data_memory(input logic [31:0] B1,WDM,input logic WEM,clk, output logic [31:0] RDB1);
logic [31:0] RAM [63:0];
always_comb
begin
RDB1=RAM[B1[31:2]];
end
always_ff@(posedge clk)
begin
RAM[B1[31:2]]<=WDM;
end
endmodule

module instruction_memory(input logic [31:0] PC, output logic [31:0] instruction);

logic [31:0] instruction_ROM[63:0];
always_comb begin 
instruction=instruction_ROM[PC[31:2]]; 
end

endmodule
module test_bench();
logic clk,reset;

RISCV_Single_Cycle RISCV_Single_Cycle(clk,reset);
initial begin $readmemh("instruction_memory.txt", RISCV_Single_Cycle.dp.IM1.instruction_ROM); end
initial begin reset<=1; #22 reset<=0; #5 reset<=1; end
initial
begin
#25
clk<=0;
forever begin #5 clk<=~clk; end
end

initial #290 $finish;
initial begin
$monitor("Time is %d  reset is %b clk is %b current instruction is %h, \n REGs from 0 to 18 are: x00 is %h x01 is %h x02 is %h x03 is %h  \n x04 is %h x05 is %h x06 is %h x07 is %h x08 is %h \n x09 is %h x10 is %h x11 is %h x12 is %h \n x13 is %h x14 is %h x15 is %h \n x16 is %h x17 is %h x18 is %h",$time,
reset,clk,
RISCV_Single_Cycle.Instruction,RISCV_Single_Cycle.dp.regfile.REGS[0],RISCV_Single_Cycle.dp.regfile.REGS[1],RISCV_Single_Cycle.dp.regfile.REGS[2],
RISCV_Single_Cycle.dp.regfile.REGS[3],RISCV_Single_Cycle.dp.regfile.REGS[4],RISCV_Single_Cycle.dp.regfile.REGS[5],RISCV_Single_Cycle.dp.regfile.REGS[6],
RISCV_Single_Cycle.dp.regfile.REGS[7],RISCV_Single_Cycle.dp.regfile.REGS[8],RISCV_Single_Cycle.dp.regfile.REGS[9],RISCV_Single_Cycle.dp.regfile.REGS[10],
RISCV_Single_Cycle.dp.regfile.REGS[11],RISCV_Single_Cycle.dp.regfile.REGS[12],RISCV_Single_Cycle.dp.regfile.REGS[13],RISCV_Single_Cycle.dp.regfile.REGS[14],
RISCV_Single_Cycle.dp.regfile.REGS[15],RISCV_Single_Cycle.dp.regfile.REGS[16],RISCV_Single_Cycle.dp.regfile.REGS[17],RISCV_Single_Cycle.dp.regfile.REGS[18],
);
end

endmodule

