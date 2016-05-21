module cpu ( clk, reset, o_seg7_0, o_seg7_1, o_seg7_2, o_seg7_3, o_seg7_4, o_seg7_5 );

   input clk, reset;
   output [6:0] o_seg7_0, o_seg7_1, o_seg7_2, o_seg7_3, o_seg7_4, o_seg7_5;

   // PC
   reg [7:0] pc, pc1;

   // Flag
   reg       flag_eq, flag_eq1;

   // Instruction
   reg [14:0] inst;
   wire [3:0] op = inst[14:11];
   wire [2:0] reg_a = inst[10:8];
   wire [2:0] reg_b = inst[7:5];
   wire [7:0] reg_data = inst[7:0];
   wire [7:0] reg_addr = inst[7:0];

   // Registers
   reg [15:0] _reg [7:0];
   reg [2:0]  reg_idx;
   reg [15:0] reg_in;
   reg        reg_we;
   wire [15:0] reg0 = _reg[0];
   wire [15:0] reg1 = _reg[1];
   wire [15:0] reg2 = _reg[2];
   wire [15:0] reg3 = _reg[3];
   wire [15:0] reg4 = _reg[4];
   wire [15:0] reg5 = _reg[5];
   wire [15:0] reg6 = _reg[6];
   wire [15:0] reg7 = _reg[7];

   // ROM
   reg [14:0] rom [255:0];

   // RAM
   reg [15:0] ram [255:0];
   reg [7:0]  ram_addr;
   reg [15:0] ram_in;
   reg        ram_we;

   // Fetch
   always @*
     inst <= rom[pc];

   // Execute
   always @*
     case (op)
       4'b0000: begin           // MOV
          pc1 <= pc + 1;
          flag_eq1 <= flag_eq;
          reg_we <= 1'b1;
          reg_idx <= reg_a;
          reg_in <= _reg[reg_b];
          ram_we <= 1'b0;
       end
       4'b0001: begin           // ADD
          pc1 <= pc + 1;
          flag_eq1 <= flag_eq;
          reg_we <= 1'b1;
          reg_idx <= reg_a;
          reg_in <= _reg[reg_a] + _reg[reg_b];
          ram_we <= 1'b0;
       end
       4'b0010: begin           // SUB
          pc1 <= pc + 1;
          flag_eq1 <= flag_eq;
          reg_we <= 1'b1;
          reg_idx <= reg_a;
          reg_in <= _reg[reg_a] - _reg[reg_b];
          ram_we <= 1'b0;
       end
       4'b0011: begin           // AND
          pc1 <= pc + 1;
          flag_eq1 <= flag_eq;
          reg_we <= 1'b1;
          reg_idx <= reg_a;
          reg_in <= _reg[reg_a] & _reg[reg_b];
          ram_we <= 1'b0;
       end
       4'b0100: begin           // OR
          pc1 <= pc + 1;
          flag_eq1 <= flag_eq;
          reg_we <= 1'b1;
          reg_idx <= reg_a;
          reg_in <= _reg[reg_a] | _reg[reg_b];
          ram_we <= 1'b0;
       end
       4'b0101: begin           // SL
          pc1 <= pc + 1;
          flag_eq1 <= flag_eq;
          reg_we <= 1'b1;
          reg_idx <= reg_a;
          reg_in <= _reg[reg_a] << 1;
          ram_we <= 1'b0;
       end
       4'b0110: begin           // SR
          pc1 <= pc + 1;
          flag_eq1 <= flag_eq;
          reg_we <= 1'b1;
          reg_idx <= reg_a;
          reg_in <= { 1'b0, _reg[reg_a][15:1] };
          ram_we <= 1'b0;
       end
       4'b0111: begin           // SRA
          pc1 <= pc + 1;
          flag_eq1 <= flag_eq;
          reg_we <= 1'b1;
          reg_idx <= reg_a;
          reg_in <= { _reg[reg_a][15], _reg[reg_a][15:1] };
          ram_we <= 1'b0;
       end
       4'b1000: begin           // LDL
          pc1 <= pc + 1;
          flag_eq1 <= flag_eq;
          reg_we <= 1'b1;
          reg_idx <= reg_a;
          reg_in <= (_reg[reg_a] & 16'hff00) | { 8'h00, reg_data };
          ram_we <= 1'b0;
       end
       4'b1001: begin           // LDH
          pc1 <= pc + 1;
          flag_eq1 <= flag_eq;
          reg_we <= 1'b1;
          reg_idx <= reg_a;
          reg_in <= (_reg[reg_a] & 16'h00ff) | { reg_data, 8'h00 };
          ram_we <= 1'b0;
       end
       4'b1010: begin           // CMP
          pc1 <= pc + 1;
          flag_eq1 <= (_reg[reg_a] == _reg[reg_b]);
          reg_we <= 1'b0;
          ram_we <= 1'b0;
       end
       4'b1011: begin           // JE
          pc1 <= flag_eq ? reg_addr : pc + 1;
          flag_eq1 <= flag_eq;
          reg_we <= 1'b0;
          ram_we <= 1'b0;
       end
       4'b1100: begin           // JMP
          pc1 <= reg_addr;
          flag_eq1 <= flag_eq;
          reg_we <= 1'b0;
          ram_we <= 1'b0;
       end
       4'b1101: begin           // LD
          pc1 <= pc + 1;
          flag_eq1 <= flag_eq;
          reg_we <= 1'b1;
          reg_idx <= reg_a;
          reg_in <= ram[reg_addr];
          ram_we <= 1'b0;
       end
       4'b1110: begin           // ST
          pc1 <= pc + 1;
          flag_eq1 <= flag_eq;
          reg_we <= 1'b0;
          ram_we <= 1'b1;
          ram_addr <= reg_addr;
          ram_in <= _reg[reg_a];
       end
       4'b1111: begin           // HLT
          pc1 <= pc;
          flag_eq1 <= flag_eq;
          reg_we <= 1'b0;
          ram_we <= 1'b0;
       end
       default: begin
          pc1 <= 16'hxxxx;
          flag_eq1 <= 1'bx;
          reg_we <= 1'bx;
          ram_we <= 1'bx;
       end
     endcase

   // Write back
   always @(posedge clk) begin
      // PC
      pc <= pc1;
      // Flag
      flag_eq <= flag_eq1;
      // Registers
      if (reg_we)
         _reg[reg_idx] <= reg_in;
      // RAM
      if (ram_we)
         ram[ram_addr] <= ram_in;
   end

   // 7 seg
   seg7 seg7_0 ( ram[8'hf0], o_seg7_0 );
   seg7 seg7_1 ( ram[8'hf1], o_seg7_1 );
   seg7 seg7_2 ( ram[8'hf2], o_seg7_2 );
   seg7 seg7_3 ( ram[8'hf3], o_seg7_3 );
   seg7 seg7_4 ( ram[8'hf4], o_seg7_4 );
   seg7 seg7_5 ( ram[8'hf5], o_seg7_5 );

   // Reset
   integer i;
   always @(negedge reset) begin
      // PC
      pc <= 0;
      // Flag
      flag_eq <= 0;
      // Registers
      for ( i=0; i<8; i++ )
        _reg[i] <= 0;
      // ROM
      rom[0] <= 15'b100000000000001;   // MOV: ldl reg0 0x01
      rom[1] <= 15'b100000100000010;   //      ldl reg1 0x02
      rom[2] <= 15'b000000000100000;   //      mov reg0 reg1
      rom[3] <= 15'b100000000000001;   // ADD: ldl reg0 0x01
      rom[4] <= 15'b100000100000010;   //      ldl reg1 0x02
      rom[5] <= 15'b000100000100000;   //      add reg0 reg1
      rom[6] <= 15'b100000000000010;   // SUB: ldl reg0 0x02
      rom[7] <= 15'b100000100000001;   //      ldl reg1 0x01
      rom[8] <= 15'b001000000100000;   //      sub reg0 reg1
      rom[9] <= 15'b100000000000011;   // AND: ldl reg0 0x03
      rom[10] <= 15'b100000100000001;  //      ldl reg1 0x01
      rom[11] <= 15'b001100000100000;  //      and reg0 reg1
      rom[12] <= 15'b100000000000001;  // OR:  ldl reg0 0x01
      rom[13] <= 15'b100000100000010;  //      ldl reg1 0x02
      rom[14] <= 15'b010000000100000;  //      or  reg0 reg1
      rom[15] <= 15'b100000000000001;  // SL:  ldl reg0 0x01
      rom[16] <= 15'b010100000000000;  //      sl
      rom[17] <= 15'b100000011111111;  // SR:  ldl reg0 0xff
      rom[18] <= 15'b100100011111111;  //      ldh reg0 0xff
      rom[19] <= 15'b011000000000000;  //      sr
      rom[20] <= 15'b100000011111111;  // SRA: ldl reg0 0xff
      rom[21] <= 15'b100100010000000;  //      ldh reg0 0x80
      rom[22] <= 15'b011100000000000;  //      sra
      rom[23] <= 15'b100000011111111;  //      ldl reg0 0xff
      rom[24] <= 15'b100100000000000;  //      ldh reg0 0x00
      rom[25] <= 15'b011100000000000;  //      sra
      rom[26] <= 15'b100000000000001;  // CMP: ldl reg0 0x01
      rom[27] <= 15'b100000100000001;  //      ldl reg1 0x01
      rom[28] <= 15'b101000000100000;  //      cmp reg0 reg1
      rom[29] <= 15'b100000000000001;  //      ldl reg0 0x01
      rom[30] <= 15'b100000100000010;  //      ldl reg1 0x02
      rom[31] <= 15'b101000000100000;  //      cmp reg0 reg1
      rom[32] <= 15'b100000000000001;  // JE:  ldl reg0 0x01
      rom[33] <= 15'b100000100000001;  //      ldl reg1 0x01
      rom[34] <= 15'b101000000100000;  //      cmp reg0 reg1
      rom[35] <= 15'b101100000100101;  //      je  0x25
      rom[36] <= 15'b000000000000000;  //      mov reg0 reg0
      rom[37] <= 15'b100000000000001;  //      ldl reg0 0x01
      rom[38] <= 15'b100000100000010;  //      ldl reg1 0x02
      rom[39] <= 15'b101000000100000;  //      cmp reg0 reg1
      rom[40] <= 15'b101100000000000;  //      je  0x00
      rom[41] <= 15'b110000000101011;  // JMP: jmp 0x2b
      rom[42] <= 15'b000000000000000;  //      mov reg0 reg0
      rom[43] <= 15'b100000000000001;  // ST:  ldl reg0 0x01
      rom[44] <= 15'b100000100000000;  //      ldl reg1 0x00
      rom[45] <= 15'b111000000000000;  //      st  reg0 0x00
      rom[46] <= 15'b110100100000000;  // LD:  ld  reg1 0x00
      rom[47] <= 15'b100000000000001;  // SEG: ldl reg0 0x01
      rom[48] <= 15'b111000011110000;  //      st  reg0 0xf0
      rom[49] <= 15'b100000000000010;  //      ldl reg0 0x02
      rom[50] <= 15'b111000011110001;  //      st  reg0 0xf1
      rom[51] <= 15'b100000000000011;  //      ldl reg0 0x03
      rom[52] <= 15'b111000011110010;  //      st  reg0 0xf2
      rom[53] <= 15'b100000000000100;  //      ldl reg0 0x04
      rom[54] <= 15'b111000011110011;  //      st  reg0 0xf3
      rom[55] <= 15'b100000000000101;  //      ldl reg0 0x05
      rom[56] <= 15'b111000011110100;  //      st  reg0 0xf4
      rom[57] <= 15'b100000000000110;  //      ldl reg0 0x06
      rom[58] <= 15'b111000011110101;  //      st  reg0 0xf5
      rom[59] <= 15'b111100000000000;  // HLT: hlt
      for ( i=60; i<256; i++ )
        rom[i] <= 0;
      // RAM
      for ( i=0; i<256; i++ )
        ram[i] <= 0;
   end

endmodule
