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
