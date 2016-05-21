module seg7 ( in, out );

   input [15:0] in;
   output [6:0] out;

   assign out = in == 16'h0000 ? 7'b1000000
              : in == 16'h0001 ? 7'b1111001
              : in == 16'h0002 ? 7'b0100100
              : in == 16'h0003 ? 7'b0110000
              : in == 16'h0004 ? 7'b0011001
              : in == 16'h0005 ? 7'b0010010
              : in == 16'h0006 ? 7'b0000010
              : in == 16'h0007 ? 7'b1111000
              : in == 16'h0008 ? 7'b0000000
              : in == 16'h0009 ? 7'b0011000
              : in == 16'h000a ? 7'b0001000
              : in == 16'h000b ? 7'b0000011
              : in == 16'h000c ? 7'b1000110
              : in == 16'h000d ? 7'b0100001
              : in == 16'h000e ? 7'b0000110
              : in == 16'h000f ? 7'b0001110
              : 7'b1111111;

endmodule // seg7