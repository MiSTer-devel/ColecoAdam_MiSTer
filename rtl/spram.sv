`timescale 1ns / 1ps

module spramv #(
    parameter widthad_a = 10,
    parameter width_a = 8,
    parameter init_file= ""
) (
    input   wire                clock,
    input   wire                wren,
    input   wire    [widthad_a-1:0]  address,
    input   wire    [width_a-1:0]  data,
    output  reg     [width_a-1:0]  q,
     
    input wire cs,
    input wire enable
);

    initial begin
        $display("Loading rom.");
        $display(init_file);
        if (init_file>0)
        	$readmemh(init_file, mem);
    end

 
// Shared memory
reg [width_a-1:0] mem [(2**widthad_a)-1:0];

always @(posedge clock) begin
    q      <= mem[address];
    if(wren) begin
//$display("writing %x %x",address,data);
        q      <= data;
        mem[address] <= data;
    end
end
 
 
endmodule
