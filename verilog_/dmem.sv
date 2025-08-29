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
