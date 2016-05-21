module cpu_test;

   reg clk, reset;
   wire [6:0] seg7_0, seg7_1, seg7_2, seg7_3, seg7_4, seg7_5;

   cpu cpu(clk, reset, seg7_0, seg7_1, seg7_2, seg7_3, seg7_4, seg7_5);

   integer i;
   initial begin
      $dumpfile("test_cpu.vcd");
      $dumpvars(0, cpu_test);
      $monitor ("%t: clk = 0b%b, reset = 0b%b", $time, clk, reset);

      clk = 0; reset = 1;
      for ( i=0; i<100; i++ ) begin
        #10 clk = 1; reset = 0;
        #10 clk = 0; reset = 0;
      end
      #10 $finish;
   end

endmodule // cpu_test
