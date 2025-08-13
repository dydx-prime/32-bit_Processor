module regfile (
  input clk,
  input we3,
  input [3:0] ra1, ra2, wa3, // addy ports, two for read, one for write
  input [31:0] wd3, r15, // writing data and PC + 8
  output [31:0] rd1, rd2
  );

  reg [31:0] rf[14:0]; // 15 registers, each 32 bits
  always @(posedge clk)
  begin
    if (we3) rf[wa3] <= wd3;
    assign rd1 = (ra1 == 4'b1111) ? 15 : rf[ra1];
    assign rd2 = (ra2 == 4'b1111) ? 15 : rf[ra2];
  end
  endmodule
