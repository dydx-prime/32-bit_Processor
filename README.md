#32-bit_Risc_Processor

Design of a 32-bit Processor using ARM instructions. Design includes controller, datapath, data memory, and instruction memory.

## Table of contents

[Instruction Sets & Microarchitecture](#instruction-sets--microarchitecture)
[Schematics](#schematics)
[Verilog Implementation](#verilog-implementation)
[Verifcation of CPU](#verification-of-cpu)


## Instruction Sets / Microarchitecture

### Instruction Register

| 31:28 | 27:26 | 25:20 | 19:16 | 15:12 | 11:0 |
|-------|-------|-------|-------|-------|------|
| cond  | op    | funct | Rn    | Rd    | Src2 |

which can be simplified to

| 31:28 | 27:26 | 25 | 24:21| 20| 19:16 | 15:12 | 11:0 |
|-------|-------|----|------|---|-------|-------|------|
| cond  | op    | I  |funct | S | Rn    | Rd    | Src2 |

The tables below describe the different types of operations, mnemonics, and the type of instruction being executed.

| cond | mnemonic | name          | condEx |
|------|----------|---------------|--------|
| 0000 | EQ       | Equal         | Z      |
| 0001 | NE       | NotEqual      | ~Z     |
| 0100 | MI       | Minus/Negative| N      |
| 1110 | AL       | Unconditional | Ignored|

| cmd | operation |
|-----|-----------|
| 0000| AND       |
| 0100| ADD       |
| 0010| SUB       |
| 1100| ORR       |

| op | instruction type  |
|----|-------------------|
| 00 | Data-Processing   |
| 01 | Memory-Instruction|
| 10 | Branch-Instruction|

### Data Proccessing

The data-processing instruction type is used for register and immediate valued operations.

| 31:28 | 27:26 | 25 | 24:21 | 20 | 19:16 | 15:12 | 11:0 |
|-------|-------|----|-------|----|-------|-------|------|
| cond  | 00    | I  | cmd   | S  | Rn    | Rd    | Src2 |

If I equals 1, then it means an immediate value is being used. Thus, the table below is used for forming the proper instruction.

| 11:8 | 7:0 |
|------|-----|
| rot  | imm8|

If I equals 0, then it indicates a register value is being used. Thus, the table below is used for forming the proper instruction,

| 11:7 | 6:5 | 4 | 3:0 |
|------|-----|---|-----|
| shamt| sh  | 0 | Rm  |

where the table below is used for describing the type of shift operation.

| instruction | sh | operation              |
|-------------|----|------------------------|
| LSL         | 00 | Logical Shift Left     |
| LSR         | 01 | Logical Shift Right    |
| ASR         | 10 | Arithmetic Shift Right |
| ROR         | 11 | Rotate Right           |

### Memory Instruction

The memory-instruction type is used for loading and storing operations (LDR/STR).

| 31:28 | 27:26 | 25 | 24 | 23 | 22 | 21 | 20 | 19:16 | 15:12 | 11:0 |
|-------|-------|----|----|----|----|----|----|-------|-------|------|
| cond  | 01    | I  | P  | U  | B  | W  | L  | Rn    | Rd    | Src2 |

The tables below show how I, P, U, B, W, & L contribute to the instruction.

| L | B | Instruction |
|---|---|-------------|
| 0 | 0 | STR         |
| 1 | 0 | LDR         |
| 1 | 1 | LDRB        |

| P | W | Index Mode |
|---|---|------------|
| 0 | 0 | Post-index |
| 1 | 0 | Offset     |
| 1 | 1 | Pre-index  |

| bit | I                        | U          |
|-----|--------------------------|------------|
| 0   | Immediate offset in Src2 | Subtract 1 |
| 1   | Register offset in Src2  | Add        |

### Branch Instruction

The branch instruction structure is much simpler as it only accounts for wheter it is a standard branch, or a branch with link. The branch with link is not implemented in this project, so "L" only has the option of 0.

| 31:28 | 27:26 | 25:24 | 23:0 |
|-------|-------|-------|------|
| cond  | 10    | 1L    | imm24|

| L      | Branch Type  |
|--------|--------------|
| Branch | Branch & Link|

### Condition Flags

The Current Program Status Register (CPSR) is responsible for holidng information on the condition flags. Thus, the structure will be as seen below,

| 31 | 30 | 29 | 28 | ...|
|----|----|----|----|----|
| N  | Z  | C  | V  | ...|

where N, Z, C, and V represent negative, zero, carry, and overflow, respectively.

The table below showcases the different types of mnemonics/conditions with flags.

| cond | mnemonic | name                                |
|------|----------|-------------------------------------|
| 0000 | EQ       | Equal                               |
| 0001 | NE       | NotEqual                            |
| 0010 | CS/HS    | Carry Set / Unsigned Higher or Same |
| 0011 | CC/LO    | Carry Clear / Unsigned Lower        |
| 0100 | MI       | Minus / Negative                    |
| 0101 | PL       | Plus / Positive or Zero             |
| 0110 | VS       | Overflow / Overflow Set             |
| 0111 | VC       | No Overflow / Overflow Clear        |
| 1000 | HI       | Unsigned Higher                     |
| 1001 | LS       | Unsigned Lower or Same              |
| 1010 | GE       | Signed Greater Than or Equal        |
| 1011 | LT       | Signed Less Than                    |
| 1100 | GT       | Signed Greater Than                 |
| 1101 | LE       | Signed Less Than or Equal           |
| 1110 | AL       | Always / Unconditional              |

## Schematics

The top level abstraction is illustrated below, where the Controller, Datapath, Instruction Memory, and Data Memory, when connected together, form the CPU. 

![top_level_abs](/schematics/top_level_design.svg)

The datapath, when connected to the data memory and controller, is illustrated below.

![datapath_datamem_controller](/schematics/microarchitecture.svg)

A closer look into the controller and the decoder within the controller.

![controller](/schematics/controller.svg)

![decoder](/schematics/decoder.svg)

For the controller, the following table is used to identify the operation of the instruction being passed.

| ALUOp | Funct 4:1 (cmd) | Funct][0] (S) | Type  | ALUControl 1:0 | FlagW 1:0 |
|-------|-----------------|--------------|-------|----------------|-----------|
| 0     | X               | X            | NOT DP| 00             | 00        |
| 1     | 100             | 0            | ADD   | 00             | 00        |
| 1     | 100             | 1            | ADD   | 00             | 11        |
| 1     | 010             | 0            | SUB   | 01             | 00        |
| 1     | 010             | 1            | SUB   | 01             | 11        |
| 1     | 000             | 0            | AND   | 10             | 00        |
| 1     | 000             | 1            | AND   | 10             | 10        |
| 1     | 1100            | 0            | ORR   | 11             | 00        |
| 1     | 1100            | 1            | ORR   | 11             | 10        |

For the instruction memory, a vector of 64 members (32 bits) will contain memory file, filled with instructions, which is then assigned to the ouput pin, for reading data. Below is the verilog implementation of the instruction memory.

```sv
module imem(
  input logic [31:0] a,
  output logic [31:0] rd);
  reg [31:0] ram[0:63];

  initial
    $readmemh("memfile.dat", ram);
    // word aligned for proper indexing
    assign rd = ram[a[31:2]]; 
endmodule
```

Similarly, the data memory creates a vector, where the read data is assigned to, with the addition of the always block, which will drive the writing data if the write enable signal is high.
```sv
module dmem(
  input clk, we,
  input [31:0] a, wd,
  output reg [31:0] rd);

reg [31:0] RAM[63:0];
// word aligned for proper indexing
assign rd = RAM[a[31:2]];

always @(posedge clk)
    if (we) RAM[a[31:2]] <= wd;

endmodule
```

## Verilog Implementation

This section covers the implementation of the higher level modules to complete the datpath/controller, as well as some of the lower level modules.

The ALU covers the ADD, SUB, AND, and OR operations. Each case updates the Carry and Overflow flags, and the Zero/Negative flags are updated outside of the always block. The 4 bit ALUFlag is then updated with the NZCV flags for future instructions.

```sv
module alu(
  input [31:0] SrcA, // Source A
  input [31:0] SrcB, // Source B
  input [1:0] ALUControl, // ALU Select signal
  output reg [31:0] ALUResult, // ALU output
  output reg [3:0] ALUFlag // N Z C V flags
  );

  reg Negative, Zero, Carry, Overflow;

  always @(*) begin
    case (ALUControl)
      2'b00: begin // ADD operation
        {Carry, ALUResult} = SrcA + SrcB;
        Overflow = ((SrcA[31] == SrcB[31]) && (ALUResult[31] != SrcA[31]));
      end
      2'b01: begin // SUB operation
        {Carry, ALUResult} = SrcA - SrcB;
        Overflow = ((SrcA[31] != SrcB[31]) && (ALUResult[31] != SrcA[31]));
      end
      2'b10: begin // AND operation
        ALUResult = SrcA & SrcB;
        Carry = 0;
        Overflow = 0;
      end
      2'b11: begin // OR operation
        ALUResult = SrcA | SrcB;
        Carry = 0;
        Overflow = 0;
      end
    default: begin
      ALUResult = 32'b0;
      Carry = 0;
      Overflow = 0;
    end
  endcase
end

  assign Zero = (ALUResult == 32'b0) ? 1'b1 : 1'b0; // Zero flag
  assign Negative = ALUResult[31];
  assign ALUFlag = {Negative, Zero, Carry, Overflow};

endmodule
```

The condition logic is implemented by defining Flagwrite, Flags, and CondEx to drive the data into the dflip flop when it is enabled, which updates the control signals given to the datapath. Once the condition is found, the condition to execute is updated to its respective value.

```sv
`include "dflipen.sv"
module condlogic(input logic clk, reset,
  input  [3:0] Cond,
  input  [3:0] ALUFlags,
  input  [1:0] FlagW,
  input  PCS, RegW, MemW,
  output reg PCSrc, RegWrite,MemWrite);
  
  //internal wires
  wire [1:0] FlagWrite;
  wire [3:0] Flags;
  reg CondEx;
  
  dflipen #(.WIDTH(2)) flagreg1(clk, reset, FlagWrite[1],ALUFlags[3:2], Flags[3:2]);
  dflipen #(.WIDTH(2)) flagreg0(clk, reset, FlagWrite[0],ALUFlags[1:0], Flags[1:0]);
  // write controls are conditional
  condcheck cc(Cond, Flags, CondEx);
  assign FlagWrite = FlagW & {2{CondEx}};
  assign RegWrite = RegW & CondEx;
  assign MemWrite = MemW & CondEx;
  assign PCSrc = PCS & CondEx;
endmodule


module condcheck(input  [3:0] Cond,
  input  [3:0] Flags,
  output  reg CondEx);

  wire neg, zero, carry, overflow, ge;
  assign {neg, zero, carry, overflow} = Flags;
  assign ge= ~(neg^overflow);

  always@(*)
    case(Cond)
      4'b0000: CondEx = zero; // EQ
      4'b0001: CondEx = ~zero; // NE
      4'b0010: CondEx = carry; // CS
      4'b0011: CondEx = ~carry; // CC
      4'b0100: CondEx = neg; // MI
      4'b0101: CondEx = ~neg; // PL
      4'b0110: CondEx = overflow; // VS
      4'b0111: CondEx = ~overflow; // VC
      4'b1000: CondEx = carry & ~zero; // HI
      4'b1001: CondEx = ~(carry & ~zero); // LS
      4'b1010: CondEx = ge; // GE
      4'b1011: CondEx = ~ge; // LT
      4'b1100: CondEx = ~zero & ge; // GT

      4'b1101: CondEx = ~(~zero & ge); // LE
      4'b1110: CondEx = 1'b1; // Always
      default: CondEx = 1'bx; // undefined
    endcase
endmodule
```

The datapath contains the connections between the PC register, Register File, and ALU. Since the multiplexer, D flip-flops, adder, alu, and register file have their own modules, they are simply instantiated. The output of the PC will be dependent on the PCsrc signal, as well as the data from the result of the ALU, or the ReadData output from the Data Memory. The register file will take the necessary bits from the Instruction Memory, where the mux's, based on their signal, will allow the correct data to flow. The ALU will take in a signal from the ReadData1 port, and ReadData2 port, where the signal will determine the instruction type, which then outputs the result, and updates the NZCV flags.

```sv
`include "extender.sv"
`include "alu.sv"
`include "regfile.sv"
`include "mux2.sv"
`include "dflip.sv"
`include "adder.sv"

module datapath(input  clk, reset,
	input  [1:0] RegSrc,
	input  RegWrite,
	input  [1:0] ImmSrc,
	input  ALUSrc,
	input  [1:0] ALUControl,
	input  MemtoReg,
	input  PCSrc,
	output reg [3:0] ALUFlags,
	output reg [31:0] PC,
	input  [31:0] Instr,
	output reg [31:0] ALUResult, WriteData,
	input  [31:0] ReadData);
  
wire [31:0] PCNext, PCPlus4, PCPlus8;
wire [31:0] ExtImm, SrcA, SrcB, Result;
wire [3:0] RA1, RA2;
  
//PC logic 
  mux2	#( .WIDTH(32) ) pcmux(.y(PCNext),.d0(PCPlus4),.d1(Result),.s(PCSrc));
  dflip  pcreg(clk, reset, PCNext, PC);
  adder pcadd1(PC, 32'b100, PCPlus4);
  adder pcadd2(PCPlus4, 32'b100, PCPlus8);
                 

// register file logic
  mux2 #(.WIDTH(4)) ra1mux(Instr[19:16], 4'b1111, RegSrc[0], RA1);
  mux2 #(.WIDTH(4)) ra2mux(Instr[3:0], Instr[15:12], RegSrc[1], RA2);
  regfile rf(clk, RegWrite, RA1, RA2,Instr[15:12], Result, PCPlus8,SrcA, WriteData);
  mux2 #(.WIDTH(32)) resmux(ALUResult, ReadData, MemtoReg, Result);
  extender ext(Instr[23:0], ImmSrc, ExtImm);
                   

// ALU logic
  mux2 #(.WIDTH(32)) srcbmux(WriteData, ExtImm, ALUSrc, SrcB);
  alu alu(SrcA, SrcB, ALUControl, ALUResult, ALUFlags);


endmodule
```

The decoder will be needed to breakdown the instruction given in the Instruction Memory. The Op code is first checked to determine if the instruciton is of R-type or I-type, followed by whether it is an LDR, STR, or Branch instruction. The ALU decoder then extracts the data of the function, and then ouputs the ALUControl and FlagW signals. The PC logic is then updated based on the Branch and RegW comparison, which then updates the PCsrc signal.

```sv
module decoder(
  input [1:0] Op,
  input  [5:0] Funct,
  input  [3:0] Rd,
  output reg [1:0] FlagW,
  output reg PCS, RegW, MemW,
  output reg MemtoReg, ALUSrc,
  output reg [1:0] ImmSrc, RegSrc, ALUControl
  );

 //internal wires 
  reg Branch, ALUOp;
  reg [9:0] controls;
  assign {Branch,MemtoReg,MemW,ALUSrc,ImmSrc,RegW,RegSrc, ALUOp} = controls ;
  //main decoder 
  always@(*) 
    casex(Op)
      2'b00: if (Funct[5]) controls = 10'b0001001001; // Data-processing immediate
			// Data-processing register
			else controls = 10'b0000001001;
            // LDR
      2'b01: if (Funct[0]) controls = 10'b0101011000;
          	// STR
          	else controls = 10'b0011010100;
          	// B
          	2'b10: controls = 10'b1001100010;
          	// Unimplemented
         	default: controls = 10'bx;
    endcase
  // ALU Decoder
always@(*) begin 
  if (ALUOp) begin // which DataPath Instr?
      case(Funct[4:1])
          4'b0100: ALUControl = 2'b00; // ADD
          4'b0010: ALUControl = 2'b01; // SUB
          4'b0000: ALUControl = 2'b10; // AND
          4'b1100: ALUControl = 2'b11; // ORR
          default: ALUControl = 2'bx; // unimplemented
      endcase
  // update flags if bit is set (C & V only for arith)
      FlagW[1] = Funct[0];
      FlagW[0] = Funct[0] & (ALUControl == 2'b00 | ALUControl == 2'b01);
  end

  else begin
      ALUControl = 2'b00; // add for non-DataPath instructions

      FlagW = 2'b00; // don't update Flags

  end

end
assign PCS = ((Rd == 4'b1111) & RegW) | Branch;
endmodule
```

The cpu module connects the controller and datapath.

```sv
`include "controller.sv"
`include "datapath.sv"

module cpu(
  input clk,
  input reset,
  output [31:0] PC,
  input [31:0] Instr,
  output MemWrite,
  output [31:0] ALUResult, WriteData,
  input [31:0] ReadData
  );

  wire [3:0] ALUFlags;
  wire [1:0] RegSrc, ImmSrc, ALUControl;
  wire RegWrite, ALUSrc, MemtoReg, PCSrc;

  controller c(clk, reset, Instr[31:12], ALUFlags, RegSrc, RegWrite,
    ImmSrc, ALUSrc, ALUControl, MemWrite, MemtoReg, PCSrc);

  datapath dp(clk, reset, RegSrc, RegWrite, ImmSrc, ALUSrc, ALUControl,
    MemtoReg, PCSrc, ALUFlags, PC, Instr, ALUResult, WriteData, ReadData);

  endmodule
```

The top module connects the cpu, data memory, and instruction memory, finalizing the top view of the project.

```sv
`include "cpu.sv"
`include "dmem.sv"
`include "imem.sv"

module top(
  input clk,
  input reset
  );

  wire [31:0] PC, Instr, ReadData;
  wire [31:0] WriteData, DataAdr;
  wire MemWrite;

  // cpu and memory inst.

  cpu cpu (clk, reset, PC, Instr, MemWrite, DataAdr, WriteData, ReadData);
  imem imem(PC, Instr);
  dmem dmem(clk, MemWrite, DataAdr, WriteData, ReadData);

  endmodule
```

## Verification of CPU

A testbench is written to verify a wide range of instructions. The table below showcases the stimulus being driven to the DUT.

| address | program          | binary machine code                     |
|---------|------------------|-----------------------------------------|
| 00      | SUB R0, R15, R15 | 1110 0000 0100 1111 0000 0000 0000 1111 |
| 04      | ADD R2, R0, #5   | 1110 0010 1000 0000 0010 0000 0000 0101 |
| 08      | ADD R3, R0, #12  | 1110 0010 1000 0000 0011 0000 0000 1100 |
| 0C      | SUB R7, R3, #9   | 1110 0010 0100 0011 0111 0000 0000 1001 |
| 10      | ORR R4, R7, R2   | 1110 0001 1000 0111 0100 0000 0000 0010 |
| 14      | AND R5, R3, R4   | 1110 0000 0000 0011 0101 0000 0000 0100 |
| 18      | ADD R5, R5, R4   | 1110 0000 1000 0101 0101 0000 0000 0100 |
| 1C      | SUBS R8, R5, R7  | 1110 0000 0101 0101 1000 0000 0000 0111 |
| 20      | BEQ END (not taken)|  0000 1010 0000 0000 0000 0000 0000 1100 |
| 24      | SUBS R8, R3, R4  | 1110 0000 0101 0011 1000 0000 0000 0100 |
| 28      | BGE AROUND (taken) | 1010 1010 0000 0000 0000 0000 0000 0000 |
| 2C      | ADD R5, R0, #0 (skipped)  |  1110 0010 1000 0000 0101 0000 0000 0000 |
| 30 AROUND | SUBS R8, R7, R2  | 1110 0000 0101 0111 1000 0000 0000 0010 |
| 34      | ADDLT R7, R5, #1 | 1011 0010 1000 0101 0111 0000 0000 0001 |
| 38      | SUBS R7, R7, R2  | 1110 0000 0100 0111 0111 0000 0000 0010 |
| 3C      | STR R7, [R3, #84]| 1110 0101 1000 0011 0111 0000 0101 0100 |
| 40      | LDR R2, [R0, #96]| 1110 0101 1001 0000 0010 0000 0110 0000 |
| 44      | ADD R15, R15, R0 | 1110 0000 1000 1111 1111 0000 0000 0000 |
| 48      | ADD R2, R0, #14  | 1110 0010 1000 0000 0010 0000 0000 1110 |
| 4C      | B END (taken)    | 1110 1010 0000 0000 0000 0000 0000 0001 |
| 50      | ADD R2, R0, #13  | 1110 0010 1000 0000 0010 0000 0000 1101 |
| 54      | ADD R2, R0, #10  | 1110 0010 1000 0000 0010 0000 0000 1010 |
| 58 END  | STR R2, [R0, #84]| 1110 0101 1000 0000 0010 0000 0101 0100 |

```sv
`include "top.sv"

// iverilog -g2012 -o results testbench.sv
// vvp results

module testbench();
	reg clk;
	reg reset;
  
// instantiate device to be tested
top dut(clk, reset);
  
// initialize test
initial begin
	reset <= 1; 
  	#10 reset <= 0;
  	clk <= 1;
end
  
// generate clock to sequence tests
always #5 clk=~clk;
initial begin 
  #10000
  if (dut.dmem.RAM[21] === 32'd7) begin
    $display("Test Passed: Memory[84] contains 7");
        end else begin
          $display("Test Failed: Memory[84] = %d, expected 7", dut.dmem.RAM[21]);
        end
$finish;
end
endmodule
```

The result of the testbench is showcased below, where it can be seen that the memory address correctly contains the value of 7. 

![verification_message](/schematics/verification_message.svg)

The CPU is then concluded to be a 32 bit processor that supports many instructions. However, there are many areas of improvement. The lack of features such as branching with link, pipelining, and FPGA implementation for further verification, can prove to make a project like this stronger. I thank you for reading this repository, and I hope you gained something from it.

