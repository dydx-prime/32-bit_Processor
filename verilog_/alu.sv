module alu(
  input [31:0] SrcA, // Source A
  input [31:0] SrcB, // Source B
  input [1:0] ALUControl, // ALU Select signal
  output reg [31:0] ALUResult, // ALU output
  // output reg [3:0] ALUFlag // flags - N, Z, C, V
  output reg Zero,
  output reg Negative,
  output reg Overflow,
  output reg Carry
  );

  always @(*) begin
    case (ALUControl)
      2'b00: begin // AND operation
        ALUResult = SrcA & SrcB;
        Carry = 0;
        Overflow = 0;
      end
      2'b01: begin // OR operation
        ALUResult = SrcA | SrcB;
        Carry = 0;
        Overflow = 0;
      end
      2'b10: begin // ADD operation
      {Carry, ALUResult} = SrcA + SrcB;
      Overflow = Carry;
      end
      2'b11: begin // SUB operation
      {Carry, ALUResult} = SrcA - SrcB;
      Overflow = Carry;
      end
    default: begin
      ALUResult = 32'b0;
      Carry = 0;
      Overflow = 0;
    end
  endcase
end

  assign Zero = (ALUResult == 32'b0) ? 1'b1 : 1'b0; // Zero flag

endmodule
