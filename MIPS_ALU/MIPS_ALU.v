//1.PC Increment Logic
/*Whenever a address is in its input,this unit increments the address by 4.
Input	=32 bit-Address(Address of the Instruction memory in which current Instruction is stored).
Output	=32 bit-NextAddress0(Address of the Instruction memory in which next address is stored).
*/
module Adder1(Address,NextAddress0);
input [31:0]Address;		
output reg[31:0]NextAddress0;
 	
always@(Address) 		
	NextAddress0<=Address+4;
endmodule

//2.Adder for branch Instructions
/*When two values in the registers are equal,this adder generates an address in the instruction memory where the next instruction is stored.
Input	=32 bit	-NextAddress0(Output of the pc increment logic)
		=32 bit -ShiftOut-Offset multiplied by 4
Output	=32bit	-NextAddress1(Address when the instruction is a branch instruction) 
*/
module Adder2(NextAddress0,ShiftOut,NextAddress1);
input [31:0] NextAddress0,ShiftOut;
output reg [31:0] NextAddress1;

always@(*)
	NextAddress1=NextAddress0+ShiftOut;
endmodule

//3.Mux0
/*In a beq,this selects NextAddress1(Output of Adder2) to be the next address.Otherwise it is set to be the output of the pc increment logic.
Input	=32 bit NextAddress0
		=32 bit NextAddress1
Output	=32 bit NextAddress
*/
module Mux0(NextAddress0,NextAddress1,Branch,NextAddress2);
input [31:0] NextAddress0,NextAddress1;
input Branch;
output reg [31:0]NextAddress2;

always @(*)begin
	if(Branch)
		NextAddress2<=NextAddress1;
	else
		NextAddress2<=NextAddress0;
	end
endmodule

//4.Shift 
/*Left shift the sign extend by 2.That means this multiplies the sign extend by 4.
Input	=32bit 	-SignExtOut(Output of the Sign Ext unit-32 bit Offset)
Output	=32bit	-ShiftOut(Offset multiplied by 4)
*/
module Shift(ShiftOut,SignExtOut);
input [31:0] SignExtOut;
output reg [31:0] ShiftOut;

always @(*) begin
	ShiftOut[1:0]<=0;
	ShiftOut[31]<=SignExtOut[31];
	ShiftOut[30:2]<=SignExtOut[28:0];
end
endmodule

//5.Program Counter(PC)
/*Stores the address of the Instruction memory in which current Instruction is stored.
In the first half of the Clock cycle-Current address
In the second half of the Clock cycle-Next address
Input	=Clock-Register		:PC gets updated in the negative edge of the Clock
	=reset-Register		:PC goes to 0 when it sees a positive edge of reset
	=next_address-32 bit	:Address corresponding to next Instruction
Output	=address-32 bit		:Input to the Instruction memory(to the 1st of 4 Registers that holds the current Instruction)*/
module PC(Clock,Reset,Address,NextAddress);
input Reset;
input Clock;
input [31:0] NextAddress; 	
output reg [31:0] Address;

always @(negedge Clock,posedge Reset) 
	begin
	if(Reset)
	Address=0; 		      
	else
	Address=NextAddress; 	      
	end
endmodule

//3.Instruction Memory
/*When an address is given(which is the address of a Register of this memory) this outputs an Instruction
Input	=address-32bit		:Output of the pc.1st Register address of 4 Registers in which one Instruction is stored
Register=memory-8bit		:An address is stored in 4 consecutive memory slots
Output	=Instruction-32bit	:The Instruction to be executed

		Instruction Format
R Type
|	  31:26	  | 	25:21	|	 20:16	 |	  15:11	  |		10:6	  |		5:0		|	
	OpCode	      ReadReg1	  	ReadReg2	WriteReg1	    shamt	     FuncCode  
I Type
|	31:26	  |     25:21   |	20:16	 |                  15:0                    |
	OpCode  	  ReadReg1     WriteReg1                   Offset
*/
module InstructionMemory(Address,Instruction);
input [31:0]Address;
output reg[31:0]Instruction;
reg [7:0] Memory[1024:0];

initial
begin
/*Instruction:ADD
FuncCode=32	Opcode=00	ReadReg1=0	Readreg2=1	WriteReg=2
*/
Memory[0]	=8'b00_100000;
Memory[1]	=8'b00010_000;
Memory[2]	=8'b000_00001;	
Memory[3]	=8'b000000_00;

/*Instruction:SUBTRACT
FuncCode=34	Opcode=00	ReadReg1=3	Readreg2=4	WriteReg=5
*/
Memory[4]	=8'b00_100010;
Memory[5]	=8'b00101_000;
Memory[6]	=8'b011_00100;	
Memory[7]	=8'b000000_00;

/*Instruction:AND
FuncCode=36	Opcode=00	ReadReg1=6	Readreg2=7	WriteReg=8
*/
Memory[8]	=8'b00_100100;
Memory[9]	=8'b01000_000;
Memory[10]	=8'b110_00111;	
Memory[11]	=8'b000000_00;

/*Instruction:OR
FuncCode=37	Opcode=0	ReadReg1=9	Readreg2=10	WriteReg=11
*/
Memory[12]	=8'b00_100101;
Memory[13]	=8'b01110_000;
Memory[14]	=8'b100_01101;	
Memory[15]	=8'b000000_01;

/*Instruction:NOR
FuncCode=39	Opcode=0	ReadReg1=12	Readreg2=13	WriteReg=14
*/
Memory[16]	=8'b00_100111;
Memory[17]	=8'b10001_000;
Memory[18]	=8'b111_10000;	
Memory[19]	=8'b000000_01;

/*Instruction:ASLT
FuncCode=42	Opcode=0	ReadReg1=15	Readreg2=16	WriteReg=17
*/
Memory[20]	=8'b00_101010;
Memory[21]	=8'b10100_000;
Memory[22]	=8'b010_10011;	
Memory[23]	=8'b000000_10;

/*Instruction:Load Word 
OpCode=35	ReadReg1=18	ReadReg=19 Offset=0	
*/
Memory[24]	=8'b00000000;
Memory[25]	=8'b00000000;
Memory[26]	=8'b010_10011;	
Memory[27]	=8'b100011_10;

/*Instruction:Store Word 
OpCode=43	ReadReg1=20	ReadReg2=21 Offset=0	
*/
Memory[28]	=8'b00000000;
Memory[29]	=8'b00000000;
Memory[30]	=8'b100_10101;	
Memory[31]	=8'b101011_10;

/*Instruction:Branch-Beq
OpCode=04	ReadReg1=22	ReadReg2=23 Offset=2	
*/
Memory[32]	=8'b00000010;
Memory[33]	=8'b00000000;
Memory[34]	=8'b110_10111;	
Memory[35]	=8'b000100_10;

/*Instruction:Branch-Beq
OpCode=04	ReadReg1=22	ReadReg2=23 Offset=2
*/
Memory[44]	=8'b00000000;
Memory[45]	=8'b00000000;
Memory[46]	=8'b111_10111;	
Memory[47]	=8'b000100_11;

/*Instruction:Jump
OpCode=02	Address=16
Jumping to address 64
*/
Memory[48]	=8'b00010000;
Memory[49]	=8'b00000000;
Memory[50]	=8'b00000000;	
Memory[51]	=8'b000010_00;

end

always@(Address)
	begin	
	Instruction[7:0]	<=Memory[Address];
	Instruction[15:8]	<=Memory[Address+1];
	Instruction[23:16]	<=Memory[Address+2];
	Instruction[31:24]	<=Memory[Address+3];
	end
	
endmodule




///4.Instruction Register
/*Takes in the output of Instruction memory and assigns it to various variables at the positive edge of the Clock(Gets updated at every positive edge).
Input	=Instruction-32bit		:Output of the Instruction memory.
Output	=CurrentInstruction-32bit	:Current Instruction being executed(Gets updated at the positive edge with 'Instruction').
	=ReadReg1-05bit			:Segment of the Instruction in which 01st "Read Register" of Register file is stored.
	=ReadReg1-05bit			:Segment of the Instruction in which 02nd "Read Register" of Register file is stored.
	=ReadReg1-05bit			:Segment of the Instruction in which 01st "Write Register" of Register file is stored.
	=FuncCode-06bit			:Segment of the Instruction in which function code is stored.
	=OpCode-06bit			:Segment of the Instruction in which op code is stored.	
*/


module InstructionReg(Instruction,ReadReg1,ReadReg2,WriteReg1,FuncCode,OpCode,Offset,CurrentInstruction,Clock,ShiftLeftIn);
input [31:0] Instruction;
output reg[31:0] CurrentInstruction;
output reg[4:0] ReadReg1,ReadReg2,WriteReg1;
output reg[5:0] FuncCode,OpCode;
output reg[15:0] Offset;
output reg[25:0] ShiftLeftIn;
input Clock;

always @(posedge Clock)
	begin
		CurrentInstruction	=Instruction;
		ReadReg1			=CurrentInstruction[25:21];
		ReadReg2			=CurrentInstruction[20:16];
		WriteReg1			=CurrentInstruction[15:11];
		FuncCode			=CurrentInstruction[5:0];
		OpCode				=CurrentInstruction[31:26];
		Offset				=CurrentInstruction[15:0];
		ShiftLeftIn			=CurrentInstruction[25:0];
	end
endmodule

//5.Register file
/*Register file is a collection of Registers in which values are stored.These values can be assigned to operands
Input	=Clock 			:Data to be written in to the Register file gets written at the positive edge
	=RegWrite-Register	:Control line Register input
	=ReadReg1,ReadReg2-5bit	:Addresses of the Register files to be read(Assigned to operands)	
	=WriteReg1-5bit		:Address of the Register file for the data to be written
	=WriteData-32bit	:Data to be witten on to the Resigter file
Output	=ReadData1,Readdata2	:Values stored in the addresses specified by ReadReg1 and 2.
*/
module RegisterFile(ReadReg1,ReadReg2,WriteReg,RegWrite,Clock,A,ReadData2,WriteData);
input[4:0]ReadReg1,ReadReg2,WriteReg;
input Clock,RegWrite;
input [31:0]WriteData;
output reg[31:0]A,ReadData2;

reg[31:0] Register[31:0];
initial begin
	//Initialaize A and B.
	A=32'h0;
	ReadData2=32'h0;

	//Values for ADD operation
	Register[0]	=32'h1; 
	Register[1]	=32'h2;
	Register[2]	=32'h0;

	//Values for SUBTRACT operation
	Register[3]	=32'h5; 
	Register[4]	=32'h3;
	Register[5]	=32'h0;

	//Values for AND operation
	Register[6]	=32'h1; 
	Register[7]	=32'h1;
	Register[8]	=32'h0;

	//Values for OR operation
	Register[9]	=32'h0; 
	Register[10]	=32'h0;
	Register[11]	=32'h0;

	//Values for NOR operation
	Register[12]	=32'h1;
	Register[13]	=32'h1;
	Register[14]	=32'h0;

	//Values for ASLT operation
	Register[15]	=32'h0;
	Register[16]	=32'h0;
	Register[17]	=32'h0;
	
	//Values for load word  operation
	Register[18]	=32'h0;
	Register[19]	=32'h0;
	
	//values for store word operation 
	Register[20]	=32'h4;
	Register[21]	=32'h5;

	//Branch
	Register[22]	=32'h4;
	Register[23]	=32'h4;
	Register[31]	=32'h1;

end
always @ (*)	begin
	A<=Register[ReadReg1];
	ReadData2<=Register[ReadReg2];
end

always @(posedge Clock) 
begin
	if (RegWrite)
		Register[WriteReg]=WriteData;
end
endmodule

//6.Control path
/*Based on Function Code and ALUOp,Assigns a value for ALUCtl.
If ALUOp is 00;it is a Memory instruction and we need not to care about the function code,we just need ALU to add the 2 inputs.
If ALUOp is 01;it is branch equal hence we need to subtract.
If ALUOp is 10;we must go for the operation based on the FuncCode.
Input	=ALUOp-2bit
Input	=FuncCode-6bit
Output	=ALUCtl-4bit	:Control line output*/
module ALUControl(ALUOp,FuncCode,ALUCtl);
input[1:0]ALUOp;
input[5:0]FuncCode;
output	reg[3:0]ALUCtl;

always @(ALUOp) begin
if (ALUOp==2'b00)	
	ALUCtl<=2;
else if (ALUOp==2'b01)
	ALUCtl<=6;
else if (ALUOp==2'b10)
		case(FuncCode)
			32: ALUCtl <= 2;//ADD
			34: ALUCtl <= 6;//SUBTRACT
			36: ALUCtl <= 0;//AND	
			37: ALUCtl <= 1;//OR
			39: ALUCtl <= 12;//NOR		
			42: ALUCtl <= 7;//ASLT		
			default: ALUCtl <= 15;//should not happen
		endcase
end
endmodule

//8.MIPS ALU:The ALU Unit
/*Based on Control line inputs,does operations to A and B and outputs ALUOut.
Input	=ALUCtl-4bit		:Control line input
	=A,B-32bit		:Operands
Output	=ALUOut-32bit		:Output of the ALU
	=Zero			:True if ALUOut is Zero.	
*/

module MIPSALU(ALUCtl,A,B,ALUOut,Zero);
input [3:0] ALUCtl;
input [31:0] A,B;
output 	 reg[31:0] ALUOut;
output	Zero;

assign Zero = (ALUOut==0);	// Zero is true if ALUOut is 0
always @(ALUCtl,A,B) begin	// reevaluate if these change
	case (ALUCtl)
		0: ALUOut <= A & B;
		1: ALUOut <= A | B;
		2: ALUOut <= A + B;
		6: ALUOut <= A - B;
		7: ALUOut <= A < B ? 1 : 0;
		12: ALUOut <= ~(A | B);
		default: ALUOut <= 0;
	endcase
end
endmodule

//6.Controller
/*Controls the controls lines in which it selects between R and I type
Input		=Opcode(6 bit) 	:Chooses whether the instruction is R-Type(0),Load word (35),store word(43) or beq(4)
			=Zero			:For Branch instructions if Zero is true,Branch is 1.
Outputs		=RegDst		:Which register to write data on (WriteReg1/ReadReg2)?
			 ALUSrc		:What is the input to the ALU?(ALUOut/Offset)?
			 MemtoReg	:In what memory do we get write data?(Register/Data Memory)?
			 ALUOp		:Do we go for ALU operation based on FuncCode?Or Addition/Subtraction?
			 MemWrite	:Are we writing to the data memory?
			 MemRead	:Are we reading from the data memory?
			 RegWrite	:Do we write data on to the register file?
			 Branch		:Is this a branch instruction?
*/
module Controller(OpCode,RegDst,ALUSrc,MemtoReg,ALUOp,MemWrite,MemRead,RegWrite,Zero,Branch,Jump);
input [5:0]OpCode;
input Zero;
reg [9:0]control;
output reg RegDst,ALUSrc,MemtoReg,MemWrite,MemRead,RegWrite,Branch,Jump;
output reg [1:0] ALUOp;

always @ (*) //whenever there is a change in opcode in the instruction,do this.
begin
case(OpCode)
	0:control 	= 10'b0100100010; 
	35:control	= 10'b0011110000; 
	43: control	= 10'b0x1x001000; 
	4:control	= 10'b0x0x000101;
	2:control	= 10'b1xxxxxxxxx;
	default: control = 10'd0;
endcase

//Assigning values to the control lines
assign Jump	= control[9];
assign RegDst	= control[8];
assign ALUSrc	= control[7];
assign MemtoReg	= control[6];
assign RegWrite = control[5];
assign MemRead	= control[4];
assign MemWrite = control[3];
assign Branch   = control[2]&Zero;
assign ALUOp	= control[1:0];
end
endmodule

//7.Sign Extension Unit

module SignExtension(Offset,ALUOp,SignExtOut);
input [15:0] Offset;
input [1:0] ALUOp;
output reg [31:0] SignExtOut;

always @(*) begin
		SignExtOut[14:0]<=Offset[14:0];
		if (Offset[15]==0)
			SignExtOut[31:15]<=0;
		else
			SignExtOut[31:15]<=1;
end
endmodule

//8.Data Memory
//Data Memory
module DataMemory(ALUOut,ReadData2,ReadData,MemWrite,MemRead);
input [31:0] ALUOut;//output from the ALU for I type instruction
input [31:0] ReadData2;	//output of the register file
input MemWrite,MemRead;	//control lines for data memory
output reg [31:0] ReadData; //output of the data memory
reg [7:0] DataRegister[1048576:0]; //8 bit registers in the data memory


integer i;	
initial begin
	DataRegister[0]=8'b00000011;
	DataRegister[1]=8'b00000000;
	DataRegister[2]=8'b00000000;
	DataRegister[3]=8'b00000000;
	
	DataRegister[4]=8'b00000000;
	DataRegister[5]=8'b00000000;
	DataRegister[6]=8'b00000000;
	DataRegister[7]=8'b00000000;
	
end

always @ (MemRead,MemWrite) begin//Load Word
if(MemRead) begin
	ReadData[7:0]	<=DataRegister[ALUOut];
	ReadData[15:8]	<=DataRegister[ALUOut+1];
	ReadData[23:16]	<=DataRegister[ALUOut+2];
	ReadData[31:24]	<=DataRegister[ALUOut+3];
end
else if(MemWrite) begin	
	DataRegister[ALUOut]	<=ReadData2[7:0];
	DataRegister[ALUOut+1]	<=ReadData2[15:8];
	DataRegister[ALUOut+2]	<=ReadData2[23:16];
	DataRegister[ALUOut+3]	<=ReadData2[31:24];
end 
end
endmodule


////i.Multiplexer 01:MUX for register file
module MUX1(ReadReg2,WriteReg1,RegDst,WriteReg);
input [4:0]ReadReg2,WriteReg1; //input of the segments of instruction
input RegDst; //control line input
output reg [4:0] WriteReg; //outputs the register to write

always @(*) begin
if (RegDst)
	WriteReg<=WriteReg1;
else if (~RegDst)
	WriteReg<=ReadReg2;
end
endmodule

/////ii.Multiplexer 02:MUX for ALU
module MUX2(ReadData2,SignExtOut,ALUSrc,B);
input[31:0]ReadData2,SignExtOut;//Output of the register file and the output of the sign extend unit
input ALUSrc;//control input
output reg [31:0]B;//input to the ALU

always @(*) begin
if (ALUSrc)
	B<=SignExtOut;
else if (~ALUSrc)
	B<=ReadData2;
end	
endmodule

/////iii.Multiplexer for Register file(Write data) from Data memory
module MUX3(ReadData,ALUOut,WriteData,MemtoReg);
input [31:0] ReadData,ALUOut;//WriteData from data memory and ALUOut from ALU
input MemtoReg;//Control line input
output reg [31:0] WriteData;//Output to the register

always @(*) begin
if (MemtoReg)
	WriteData<=ReadData;
else if (~MemtoReg)
	WriteData<=ALUOut;
end	
endmodule

module ShiftLeft(ShiftLeftIn,ShiftLeftOut);
input [25:0] ShiftLeftIn;
output reg[27:0] ShiftLeftOut;

always @(*)begin
ShiftLeftOut[27:2]<=ShiftLeftIn;
ShiftLeftOut[1:0]<=2'b00;
end
endmodule

module Concat(NextAddress0,ShiftLeftOut,JumpAddress);
input [31:0] NextAddress0;
input [27:0] ShiftLeftOut;
output reg [31:0] JumpAddress;

always @(*) begin
JumpAddress[31:28]<=NextAddress0[31:28];
JumpAddress[27:0]<=ShiftLeftOut[27:0];
end
endmodule

module Mux4(Jump,JumpAddress,NextAddress2,NextAddress);
input  Jump;
input [31:0] NextAddress2,JumpAddress;
output reg[31:0] NextAddress;

always @(*) begin
if (Jump)
NextAddress<=JumpAddress;
else 
NextAddress<=NextAddress2;
end
endmodule

module TB();
reg Clock=0;
wire [31:0] Address,NextAddress0,NextAddress1,NextAddress2,NextAddress,JumpAddress,ShiftOut,SignExtOut,Instruction,CurrentInstruction,A,ReadData2,WriteData,B,ALUOut,ReadData;
reg Reset;
wire Branch,RegWrite,RegDst,ALUSrc,MemtoReg,MemWrite,MemRead,Jump;
wire [4:0]ReadReg1,ReadReg2,WriteReg1,WriteReg;
wire [5:0]FuncCode,OpCode;
wire [1:0] ALUOp;
wire [3:0] ALUCtl;
wire [15:0] Offset;
wire[25:0] ShiftLeftIn;
wire [27:0] ShiftLeftOut;
wire Zero;

//module instantiations
Adder1 m0(Address,NextAddress0);
Adder2 m1(NextAddress0,ShiftOut,NextAddress1);
Mux0 m2(NextAddress0,NextAddress1,Branch,NextAddress2);
Shift m3(ShiftOut,SignExtOut);
PC m4(Clock,Reset,Address,NextAddress);
InstructionMemory m5(Address,Instruction);
InstructionReg m6(Instruction,ReadReg1,ReadReg2,WriteReg1,FuncCode,OpCode,Offset,CurrentInstruction,Clock,ShiftLeftIn);
RegisterFile m7(ReadReg1,ReadReg2,WriteReg,RegWrite,Clock,A,ReadData2,WriteData);
ALUControl m8(ALUOp,FuncCode,ALUCtl);
MIPSALU m9(ALUCtl,A,B,ALUOut,Zero);
Controller m10(OpCode,RegDst,ALUSrc,MemtoReg,ALUOp,MemWrite,MemRead,RegWrite,Zero,Branch,Jump);
SignExtension m11(Offset,ALUOp,SignExtOut);
DataMemory m12(ALUOut,ReadData2,ReadData,MemWrite,MemRead);
MUX1 m13(ReadReg2,WriteReg1,RegDst,WriteReg);
MUX2 m14(ReadData2,SignExtOut,ALUSrc,B);
MUX3 m15(ReadData,ALUOut,WriteData,MemtoReg);
ShiftLeft m16(ShiftLeftIn,ShiftLeftOut);
Concat m17(NextAddress0,ShiftLeftOut,JumpAddress);
Mux4 m18(Jump,JumpAddress,NextAddress2,NextAddress);
initial begin
Reset= 0;
#10 Reset= 1;
#5 Reset= 0;
end

always begin
#5 Clock = ~Clock;		
end
endmodule
