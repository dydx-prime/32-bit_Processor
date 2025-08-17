`include "decoder.sv"
`include "condlogic.sv"

module controller(
  input logic clk, reset,
  input [31:12] Instr,
  input [3:0] ALUFlags,
  output reg [1:0] RegSrc,
  output reg RegWrite,
  output reg [1:0] ImmSrc,
  output reg ALUSrc,
  output reg [1:0] ALUControl,
  output reg MemWrite, MemtoReg,
  output reg PCSrc
  );

  // internal wire
  wire [1:0] FlagW;
  wire PCS, RegW, MemW;

  // decoder integration (PC logic, main decode, ALU decoder)
  decoder dec(Instr[27:26], Instr[25:20], Instr[15:12],
              FlagW, PCS, RegW, MemW, MemtoReg, ALUSrc,
              ImmSrc, RegSrc, ALUControl);

  // conditional logic block integration
  condlogic cl(clk, reset, Instr[31:28], ALUFlags,
              FlagW, PCS, RegW, MemW, PCSrc, RegWrite,
              MemWrite);

endmodule
