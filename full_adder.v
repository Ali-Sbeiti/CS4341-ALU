////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Cohort: 404_cohort_not_found
// Software: Icarus iVerilog
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// -- 1-Bit Half Adder
module Half_Add(output out, carry, input a,b);
    xor x1(out, a,b);
    and a1(carry, a,b);
endmodule

// -- 1-Bit Adder
module Full_Add(output out, carry, input a, b, cin);
    wire w1,w2,w3;
    Half_Add h1(w1,w2,a,b);
    Half_Add h2(out, w3, w1, cin);
    or o1(carry, w2, w3);
endmodule

//FA 4-Bit Grouping
module Four_Full(output [3:0] out, output carry, input [3:0] a,b, input cin);
    wire c_in1, c_in2, c_in3; 
    Full_Add FA0(out[0], cin_1, a[0], b[0], cin);
    Full_Add FA1(out[1], cin_2, a[1], b[1], cin_1);
    Full_Add FA2(out[2], cin_3, a[2], b[2], cin_2);
    Full_Add FA3(out[3], carry, a[3], b[3], cin_3);
endmodule

//Multi-Bit Adder
module nAdder(out, carry, a, b, cin);
    //Data length
    parameter datalen = 8;

    //Output registers / wires
    output [datalen-1:0] out;
    output carry;
    wire c_in1;

    //Input
    input [datalen-1:0] a;
    input [datalen-1:0] b;
    input cin;

    //Full Adders - Add more to increase bit count
    Four_Full FF0(out[3:0], c_in1, a[3:0], b[3:0], cin);
    Four_Full FF1(out[7:4], carry, a[7:4], b[7:4], c_in1);
endmodule



// -------------------- Test Bench -------------------- //
module TestBench();
parameter datalen = 8;
//Inputs
reg [datalen-1:0] a, b;
reg cin;

//Output data
wire [datalen-1:0] out;
wire carry;
//Adder
nAdder #(8) nAdd(out, carry, a, b, cin);

initial begin
    a=8'b00000111; b=8'b00000110; cin=0;
    #1 $display("%b + %b (%b) = %b {%b}", a,b,cin,out,carry);

end

endmodule