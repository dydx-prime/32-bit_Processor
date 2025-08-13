`include "alu.sv"

module test ;
  reg  [31:0] SrcA,SrcB,ALUResult;           
  reg [1:0] ALUControl;
  wire Zero,Negative,Overflow,Carry ; 
  
  reg clk ;

  //extend f(Instr,ImmSrc,ExtImm);
  alu dut(SrcA,SrcB,ALUControl,ALUResult,Zero, Negative, Overflow, Carry);
  
  initial begin 
      clk=1'b0;
      #300 $finish;
    end 
  
  always #5 clk=~clk;
  
initial begin 
 SrcA=32'h00000004;
 SrcB=32'h00000005;
 #16 ALUControl=2'b00;
 #10 ALUControl=2'b01;
 #10 ALUControl=2'b10;
 #10 ALUControl=2'b11; 
end 
  
  initial begin 
    $monitor ($time," %d %d %d",ALUResult,SrcA,SrcB);
    $dumpfile ("extender.vcd") ;
    $dumpvars (1, test);
  end 
endmodule
