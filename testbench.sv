module tb_top;
  reg clk, reset;
  top uut(.clk(clk), .reset(reset));
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(0,tb_top);
    clk=0;
    reset=1;
    #5;
    reset=0;
    #400 $finish;
  end
  always begin
    #5
    clk = ~clk;
  end
endmodule
