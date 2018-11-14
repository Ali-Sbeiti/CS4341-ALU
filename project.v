////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Cohort: 404_cohort_not_found
// Software: Icarus iVerilog
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// -- Data Lengths: N-Bits
`define DATALEN = 8;
`define MODELEN = 4;
`define ERRORLEN = 2;
// -- Function Code Definitions
`define NoChange 4'b0000
`define Load 4'b0100
//Logic Function Codes
`define NOT 4'b0001
`define AND 4'b0101
`define OR 4'b0110
`define XOR 4'b0111
//Math Function Codes
`define Add 4'b1000
`define Subtract 4'b1001
`define ShiftLeft 4'b0010
`define ShiftRight 4'b0011
//Error Code Definitions
`define NoError 2'b00
`define OverflowError 2'b01
`define UnderflowError 2'b10

// -- Rising Edge, D Flip Flop Register
module DFF(q, clk, d, reset);
    //Default register size
    parameter datalen = 8;

    //Outputs
    output reg [datalen-1:0] q;
    //Inputs
    input clk;
    input reset;
    input [datalen-1:0] d;

    //Change register value on each rising edge of clk
    always @(posedge clk) 
        begin
            q = reset ? {datalen{1'b0}} : d;
        end
endmodule

// -- ALU
module ALU(out, error, inA, inB, mode, clear,clk);
    //Default register lengths
    parameter datalen = 8;
    parameter modelen = 4;
    parameter errorlen = 2;

    //outputs
    output [datalen-1:0] out;
    output reg[errorlen-1:0] error;
    //Inputs
    input [datalen-1:0] inA;
    input [datalen-1:0] inB;
    input [modelen-1:0] mode;
    input clear; 
    input clk;
    //Wire
    wire [datalen-1:0] store;
    reg [datalen-1:0] str;
    //N-bit register module
    DFF #(datalen) acu(out, clk, str, clear);

    //Op selection
    always @(*) begin
        case(mode)
            `NOT:
                begin
                    str = ~inA;
                end
        endcase
    end
endmodule

// ------------------------------- Test Bench ------------------------------- //
module TestBench();

//Data lengths
parameter datalen = 8;
parameter modelen = 4;
parameter errorlen = 2;

//Sync system clock
reg clk;
//System reset
reg reset;

//Data inputs/Outputs
wire [datalen-1:0] out;
wire [errorlen-1:0] err;
reg [datalen-1:0] inA;
reg [datalen-1:0] inB;
reg [modelen-1:0] mode;
reg clear;

//ALU module
ALU #(datalen, modelen, errorlen) alu(out,err, inA, inB, mode, clear,clk);

//TUDO: No program lifetime
initial begin
    #17 $finish;
end

//Clock
initial begin
    clk = 0;
    forever begin
        #5 clk = ~clk;
    end
end

//$display 
initial begin
    #6 //Offset until just after posedge
    forever begin
        #10 $display("%b --> %b", inA, out);
    end
end

//Test cases
initial begin
    #4 //Offset until just before posedge
    #10 inA = 8'b01010001; mode = 4'b0001; clear = 0;
end
endmodule