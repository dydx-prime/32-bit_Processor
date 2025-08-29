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
