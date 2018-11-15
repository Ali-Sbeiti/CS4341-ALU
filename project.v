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
//Import Adder
`include "./full_adder.v"

// -- ToString: Assigns Readable Name to Mode/Error Events
module ModeToString(modeStr, errorStr, mode, err);
    parameter datalen = 8;
    parameter modelen = 4;
    parameter errorlen = 2;

    output reg[11*datalen:0] modeStr;
    output reg[9*datalen:0] errorStr;

    input [modelen-1:0] mode;
    input [errorlen-1:0] err;

    always @(*) begin
        case(mode)
            `NoChange: modeStr = "No Change";
            `NOT: modeStr = "NOT";
            `ShiftLeft: modeStr = "Shift Left";
            `ShiftRight: modeStr = "Shift Right";
            `Load: modeStr = "Load";
            `AND: modeStr = "AND";
            `OR: modeStr = "OR";
            `XOR: modeStr = "XOR";
            `Add: modeStr = "Add";
            `Subtract: modeStr = "Subtract";
        endcase

        case(err)
            `NoError: errorStr = "No Error";
            `OverflowError: errorStr = "Overflow";
            `UnderflowError: errorStr = "Underflow";
        endcase
    end
endmodule

// -- Rising Edge, D Flip Flop Register
//Modified module to produce a register of flops, instance this to create new registers of size datalen
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
    //wire [datalen-1:0] store; TUDO: Connect wire imbed DFF accumulator?
    reg [datalen-1:0] str;
    reg [datalen-1:0] n; //Counter
    //Adder
    wire [datalen-1:0] addOut;
    reg carry_in;
    wire carry_out;

    //N-bit accumulator register
    DFF #(datalen) accumulator(out, clk, str, clear);
    //8-Bit Adder module
    nAdder #(8) nAdd(addOut, carry_out, inA, inB, carry_in);

    //Op selection
    always @(*) begin
        case(mode)
            `NoChange:
                begin
                    //DFF output is fed back into DFF input
                    str = out;
                    error = `NoError;
                end
            `Load:
                begin
                    str = inA;
                    error = `NoError;
                end
            `NOT:
                begin
                    str = ~inA;
                    error = `NoError;
                end
            `AND:
                begin
                    str = inA & inB;
                    error = `NoError;
                end
            `OR:
                begin
                    str = inA | inB;
                    error = `NoError;
                end
            `XOR:
                begin
                    str = inA ^ inB;
                    error = `NoError;
                end
            `Add:
                begin
                    carry_in = 1'b0;
                    str = addOut;
                    error = carry_out ? `OverflowError : `NoError;
                end

            //TUDO: As of right now, these two shift the value of the last result, subject to change
            `ShiftLeft:
                begin
                    str = {out[datalen-2:0], 1'b0};
                end
            `ShiftRight:
                begin
                    str = {1'b0, out[datalen-1:1]};
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

//Data input registers /Outputs
wire [datalen-1:0] out;
wire [errorlen-1:0] err;
reg [datalen-1:0] inA;
reg [datalen-1:0] inB;
reg [modelen-1:0] mode;
reg clear;
//Mode/Error ToString
wire [11*datalen:0] modeStr;
wire [9*datalen:0] errorStr;

//ALU module
ALU #(datalen, modelen, errorlen) alu(out,err, inA, inB, mode, clear,clk);
//To String module
ModeToString #(datalen, modelen, errorlen) modestr(modeStr, errorStr, mode, err);

//TUDO: No program lifetime
initial begin
    #57 $finish;    //Add 10 time units for every new test case
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
    $display("Input A \t Input B \t\t Mode \t\t\t Clear \t Result \t\t Error");
    #6 //Offset until just after posedge
    forever begin
    #10  $display(
          "%b (%d) \t %b (%d) \t %b (%s) \t %b \t %b (%d) \t %b (%s)",
          inA, inA, inB, inB, mode, modeStr, clear, out, out, err, errorStr
        );
    end
end

//Test cases
initial begin
    #4 //Offset until just before posedge
    #10 inA = 8'b01010001; inB = 8'b00011000; mode = `Add; clear = 0;
    #10 inA = 10; mode = 4'b000; clear = 0;
    #10 inA = 8'b01010101; inB = 8'b01011000; mode = `OR; clear = 0;
    #10 mode = `ShiftLeft;
    #10 mode = `ShiftRight;
end
endmodule