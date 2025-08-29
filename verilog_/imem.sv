module imem(
  input logic [31:0] a,
  output logic [31:0] rd);
  reg [31:0] RAM[0:63];

  initial
    $readmemh("memfile.dat", RAM);
    // word aligned for proper indexing
    assign rd = RAM[a[31:2]]; 
endmodule


