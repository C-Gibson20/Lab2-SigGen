module sinegen #(
  parameter WIDTH = 8
)(
  input  logic clk,
  input  logic rst,
  input  logic en,
  input logic incr,
  output logic [WIDTH-1:0] dout
);

  logic [WIDTH-1:0] address;

counter myCounter (
  .clk (clk),
  .rst (rst),
  .en (en),
  .incr (incr),
  .count (address)
);

rom myRom (
  .clk (clk),
  .addr (address),
  .dout (dout)
);

endmodule
