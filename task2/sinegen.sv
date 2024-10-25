module sinegen #(
  parameter WIDTH = 8
)(
  input logic clk,
  input logic rst,
  input logic en,
  input logic phase_diff,
  output logic [WIDTH-1:0] dout1, 
  output logic [WIDTH-1:0] dout2
);

  logic [WIDTH-1:0] count1;
  logic [WIDTH-1:0] count2;

counter myCounter (
  .clk (clk),
  .rst (rst),
  .en (en),
  .diff (phase_diff),
  .count1 (count1),
  .count2 (count2)
);

dual_rom myDualRom (
  .clk (clk),
  .addr1 (count1),
  .addr2 (count2),
  .dout1 (dout1),
  .dout2 (dout2)
);

endmodule
