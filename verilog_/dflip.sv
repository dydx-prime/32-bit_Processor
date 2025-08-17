module dflip(
  input [31:0] d,
  input clk, reset,
  output reg [31:0] q
  );
  
  always @(posedge clk, posedge reset) 
    if (reset) q <= 0;
    else q <= d;

endmodule
