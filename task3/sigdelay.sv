module sinegen #(
  parameter WIDTH = 8
)(
  input logic clk,
  input logic rst,
  input logic wr,
  input logic rd,
  input logic offset,
  input logic mic_signal,
  output logic [WIDTH-1:0] delayed_signal
);

  logic [WIDTH-1:0] count1;
  logic [WIDTH-1:0] count2;

counter myCounter (
  .clk (clk),
  .rst (rst),
  .en (1),
  .diff (offset),
  .count1 (count1),
  .count2 (count2)
);

dual_ram myDualRam (
  .clk (clk),
  .wr_en (wr),
  .rd_en (rd),
  .wr_addr (count2),
  .rd_addr (count1),
  .din (mic_signal),
  .dout (delayed_signal)
);

endmodule
