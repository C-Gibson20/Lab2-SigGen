module counter #(
    parameter WIDTH = 8
)(
    input logic clk,
    input logic rst,
    input logic en,
    input logic diff,
    output logic [WIDTH-1:0] count1, 
    output logic [WIDTH-1:0] count2
);

always_ff @ (posedge clk or posedge rst) begin
    if (rst)begin
        count1 <= {WIDTH{1'b0}};
        count2 <= count1 + diff;
    end
    else begin
        count1 <= count1 + {{WIDTH-1{1'b0}}, en};
        count2 <= count1 + diff;
    end
end

endmodule
