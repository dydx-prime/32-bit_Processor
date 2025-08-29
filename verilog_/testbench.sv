`include "top.sv"

// iverilog -g2012 -o results testbench.sv
// vvp results

module testbench();
	reg clk;
	reg reset;
  
// instantiate device to be tested
top dut(clk, reset);
  
// initialize test
initial begin
	reset <= 1; 
  	#10 reset <= 0;
  	clk <= 1;
end
  
// generate clock to sequence tests
always #5 clk=~clk;
initial begin 
  #10000
  if (dut.dmem.RAM[21] === 32'd7) begin
    $display("Test Passed: Memory[84] contains 7");
        end else begin
          $display("Test Failed: Memory[84] = %d, expected 7", dut.dmem.RAM[21]);
        end
$finish;
end
endmodule
