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

