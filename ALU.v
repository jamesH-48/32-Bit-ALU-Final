/* ========================================================= */
/*                 32BIT BITWISE XOR MODULE                  */
/* ========================================================= */
module Xor_1( A, B, out );
    input wire A, B;
    output reg out;

    always @(*) 
    begin
        out <= A ^ B;
    end
endmodule

module Xor_8( A, B, out );
    input wire[7:0] A, B;
    output wire[7:0] out;

    Xor_1 A0( A[0], B[0], out[0] );
    Xor_1 A1( A[1], B[1], out[1] );
    Xor_1 A2( A[2], B[2], out[2] );
    Xor_1 A3( A[3], B[3], out[3] );
    Xor_1 A4( A[4], B[4], out[4] );
    Xor_1 A5( A[5], B[5], out[5] );
    Xor_1 A6( A[6], B[6], out[6] );
    Xor_1 A7( A[7], B[7], out[7] );

endmodule

module Xor_16( A, B, out );
    input wire[15:0] A, B;
    output wire[15:0] out;

    Xor_8 A0( A[7:0], B[7:0], out[7:0] );
    Xor_8 A1( A[15:8], B[15:8], out[15:8] );
	
endmodule


module Xor_32( A, B, out );
    input wire[31:0] A, B;
    output wire[31:0] out;

    Xor_16 A0( A[15:0], B[15:0], out[15:0] );
    Xor_16 A1( A[31:16], B[31:16], out[31:16] );

endmodule

/* ========================================================= */
/*                 32BIT BITWISE NOT MODULE                  */
/* ========================================================= */
module Not_1( in, out );
    input wire in;
    output reg out;

    always @(*) 
    begin
        out = !in;
    end
endmodule

module Not_8( in, out );
    input wire[7:0] in;
    output wire[7:0] out;

    Not_1 N0( in[0], out[0] );
    Not_1 N1( in[1], out[1] );
    Not_1 N2( in[2], out[2] );
    Not_1 N3( in[3], out[3] );
    Not_1 N4( in[4], out[4] );
    Not_1 N5( in[5], out[5] );
    Not_1 N6( in[6], out[6] );
    Not_1 N7( in[7], out[7] );
endmodule

module Not_16( in, out );
    input wire[15:0] in;
    output wire[15:0] out;

    Not_8 N0( in[7:0], out[7:0] );
    Not_8 N1( in[15:8], out[15:8] );

endmodule

module Not_32( in, out );
    input wire[31:0] in;
    output wire[31:0] out;

    Not_16 N0( in[15:0], out[15:0] );
    Not_16 N1( in[31:16], out[31:16] );

endmodule

/* ========================================================= */
/*                 32BIT BITWISE AND MODULE                  */
/* ========================================================= */
module And_1( A, B, out );
    input wire A, B;
    output reg out;

    always @(*) 
    begin
        out <= A & B;
    end
endmodule

module And_8( A, B, out );
    input wire[7:0] A, B;
    output wire[7:0] out;

    And_1 A0( A[0], B[0], out[0] );
    And_1 A1( A[1], B[1], out[1] );
    And_1 A2( A[2], B[2], out[2] );
    And_1 A3( A[3], B[3], out[3] );
    And_1 A4( A[4], B[4], out[4] );
    And_1 A5( A[5], B[5], out[5] );
    And_1 A6( A[6], B[6], out[6] );
    And_1 A7( A[7], B[7], out[7] );

endmodule

module And_16( A, B, out );
    input wire[15:0] A, B;
    output wire[15:0] out;

    And_8 A0( A[7:0], B[7:0], out[7:0] );
    And_8 A1( A[15:8], B[15:8], out[15:8] );
	
endmodule


module And_32( A, B, out );
    input wire[31:0] A, B;
    output wire[31:0] out;

    And_16 A0( A[15:0], B[15:0], out[15:0] );
    And_16 A1( A[31:16], B[31:16], out[31:16] );

endmodule

/* ========================================================= */
/*                 32BIT BITWISE OR MODULE                   */
/* ========================================================= */
module Or_1( A, B, out );
    input wire A, B;
    output reg out;

    always @(*) 
    begin
        out <= A | B;
    end
endmodule

module Or_8( A, B, out );
    input wire[7:0] A, B;
    output wire[7:0] out;

    Or_1 A0( A[0], B[0], out[0] );
    Or_1 A1( A[1], B[1], out[1] );
    Or_1 A2( A[2], B[2], out[2] );
    Or_1 A3( A[3], B[3], out[3] );
    Or_1 A4( A[4], B[4], out[4] );
    Or_1 A5( A[5], B[5], out[5] );
    Or_1 A6( A[6], B[6], out[6] );
    Or_1 A7( A[7], B[7], out[7] );

endmodule

module Or_16( A, B, out );
    input wire[15:0] A, B;
    output wire[15:0] out;

    Or_8 A0( A[7:0], B[7:0], out[7:0] );
    Or_8 A1( A[15:8], B[15:8], out[15:8] );
	
endmodule


module Or_32( A, B, out );
    input wire[31:0] A, B;
    output wire[31:0] out;

    Or_16 A0( A[15:0], B[15:0], out[15:0] );
    Or_16 A1( A[31:16], B[31:16], out[31:16] );

endmodule

/* ========================================================= */
/*                 1 BIT Full Adder                          */
/* ========================================================= */
module FullAdder1Bit(cin, A, B, S, cout);
    input cin, A, B;
    output S, cout;
    wire [2:0] temp;
    
    Xor_1 X0( A, B, temp[0] );
    Xor_1 X1( temp[0], cin, S );
    And_1 A0( A, B, temp[1] );
    And_1 A1( temp[0], cin, temp[2] );
    Or_1  O1( temp[1], temp[2], cout );
    
endmodule

/* ========================================================= */
/*                 8 BIT Full Adder                          */
/* ========================================================= */
module FullAdder8Bit( cin, A, B, S, cout );
    input cin;
    input [7:0] A, B;
    output cout;
    output [7:0] S;
    wire[7:1] carry;
    
    FullAdder1Bit FA0( cin, A[0], B[0], S[0], carry[1] );
    FullAdder1Bit FA1( carry[1], A[1], B[1], S[1], carry[2] );
    FullAdder1Bit FA2( carry[2], A[2], B[2], S[2], carry[3] );
    FullAdder1Bit FA3( carry[3], A[3], B[3], S[3], carry[4] );
    FullAdder1Bit FA4( carry[4], A[4], B[4], S[4], carry[5] );
    FullAdder1Bit FA5( carry[5], A[5], B[5], S[5], carry[6] );
    FullAdder1Bit FA6( carry[6], A[6], B[6], S[6], carry[7] );
    FullAdder1Bit FA7( carry[7], A[7], B[7], S[7], cout );
    
endmodule

/* ========================================================= */
/*                 16 BIT Full Adder                          */
/* ========================================================= */
module FullAdder16Bit( cin, A, B, S, cout );
    input wire cin;
    input wire[15:0] A, B;
    output wire cout;
    output wire[15:0] S;
    wire carry;
    
    FullAdder8Bit FA0( cin, A[7:0], B[7:0], S[7:0], carry );
    FullAdder8Bit FA1( carry, A[15:8], B[15:8], S[15:8], cout );
endmodule

/* ========================================================= */
/*                 32 BIT Full Adder                          */
/* ========================================================= */
module FullAdder32Bit( cin, A, B, S, cout );
    input wire cin;
    input wire[31:0] A, B;
    output wire cout;
    output wire[31:0] S;
    wire carry;
    
    FullAdder16Bit FA0( cin, A[15:0], B[15:0], S[15:0], carry );
    FullAdder16Bit FA1( carry, A[31:16], B[31:16], S[31:16], cout );
endmodule

/* ========================================================= */
/*                 16 BIT Multiplier                         */
/* ========================================================= */
module Multiplier16Bit(A, B, P) ;
  input [15:0] A, B ;
  output [31:0] P ;
  
  // form partial products 
  wire [15:0] pp0 = A & {16{B[0]}} ;
  wire [15:0] pp1 = A & {16{B[1]}} ;
  wire [15:0] pp2 = A & {16{B[2]}} ;
  wire [15:0] pp3 = A & {16{B[3]}} ;
  wire [15:0] pp4 = A & {16{B[4]}} ;
  wire [15:0] pp5 = A & {16{B[5]}} ;
  wire [15:0] pp6 = A & {16{B[6]}} ;
  wire [15:0] pp7 = A & {16{B[7]}} ;
  wire [15:0] pp8 = A & {16{B[8]}} ;
  wire [15:0] pp9 = A & {16{B[9]}} ;
  wire [15:0] pp10 = A & {16{B[10]}} ;
  wire [15:0] pp11 = A & {16{B[11]}} ;
  wire [15:0] pp12 = A & {16{B[12]}} ;
  wire [15:0] pp13 = A & {16{B[13]}} ;
  wire [15:0] pp14 = A & {16{B[14]}} ;
  wire [15:0] pp15 = A & {16{B[15]}} ;


  wire cout1, cout2, cout3, cout4, cout5, cout6, cout7, cout8, cout9, cout10, cout11, cout12, cout13, cout14, cout15;
  
  wire [15:0] s1, s2, s3, s4, s5, s6, s7, s8, s9, s10, s11, s12, s13, s14, s15;
  
  FullAdder16Bit #(16) add1(1'b0, pp1, {1'b0,pp0[15:1]}, s1, cout1);
  FullAdder16Bit #(16) add2(1'b0, pp2, {cout1,s1[15:1]}, s2, cout2);
  FullAdder16Bit #(16) add3(1'b0, pp3, {cout2,s2[15:1]}, s3, cout3);
  FullAdder16Bit #(16) add4(1'b0, pp4, {cout3,s3[15:1]}, s4, cout4);
  FullAdder16Bit #(16) add5(1'b0, pp5, {cout4,s4[15:1]}, s5, cout5);
  FullAdder16Bit #(16) add6(1'b0, pp6, {cout5,s5[15:1]}, s6, cout6);
  FullAdder16Bit #(16) add7(1'b0, pp7, {cout6,s6[15:1]}, s7, cout7);
  FullAdder16Bit #(16) add8(1'b0, pp8, {cout7,s7[15:1]}, s8, cout8);
  FullAdder16Bit #(16) add9(1'b0, pp9, {cout8,s8[15:1]}, s9, cout9);
  FullAdder16Bit #(16) add10(1'b0, pp10, {cout9,s9[15:1]}, s10, cout10);
  FullAdder16Bit #(16) add11(1'b0, pp11, {cout10,s10[15:1]}, s11, cout11);
  FullAdder16Bit #(16) add12(1'b0, pp12, {cout11,s11[15:1]}, s12, cout12);
  FullAdder16Bit #(16) add13(1'b0, pp13, {cout12,s12[15:1]}, s13, cout13);
  FullAdder16Bit #(16) add14(1'b0, pp14, {cout13,s13[15:1]}, s14, cout14);
  FullAdder16Bit #(16) add15(1'b0, pp15, {cout14,s14[15:1]}, s15, cout15);
 
 
  // collect the result
    assign P = { cout15, s15, s14[0], s13[0], s12[0], s11[0], s10[0], s9[0], s8[0], s7[0], s6[0], s5[0], s4[0], s3[0], s2[0], s1[0], pp0[0]} ;

endmodule

/* ========================================================= */
/*                 32 BIT SUBTRACTOR                         */
/* ========================================================= */
module AdderSub(S, C, V, Cout, A, B, Op);
   output [31:0] S;   // The 4-bit sum/difference.
   output 	C;   // The 1-bit carry/borrow status.
   output 	V;   // The 1-bit overflow status.
   output Cout;	 // Carry-Out Bit
   input [31:0] 	A;   // The 4-bit augend/minuend.
   input [31:0] 	B;   // The 4-bit addend/subtrahend.
   input 	Op;  // The operation: 0 => Add, 1=>Subtract.
   
   wire C0,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14,C15,C16,C17,C18,C19,C20,C21,C22,C23,C24,C25,C26,C27,C28,C29,C30,C31;
   wire B0,B1,B2,B3,B4,B5,B6,B7,B8,B9,B10,B11,B12,B13,B14,B15,B16,B17,B18,B19,B20,B21,B22,B23,B24,B25,B26,B27,B28,B29,B30,B31;
	
   // Looking at the truth table for xor we see that  
   // B xor 0 = B, and
   // B xor 1 = not(B).
   // So, if Op==1 means we are subtracting, then
   // adding A and B xor Op alog with setting the first
   // carry bit to Op, will give us a result of
   // A+B when Op==0, and A+not(B)+1 when Op==1.
   // Note that not(B)+1 is the 2's complement of B, so
   // this gives us subtraction.     
   xor(B0, B[0], Op);
   xor(B1, B[1], Op);
   xor(B2, B[2], Op);
   xor(B3, B[3], Op);
   xor(B4, B[4], Op);
   xor(B5, B[5], Op);
   xor(B6, B[6], Op);
   xor(B7, B[7], Op);
   xor(B8, B[8], Op);
   xor(B9, B[9], Op);
   xor(B10, B[10], Op);
   xor(B11, B[11], Op);
   xor(B12, B[12], Op);
   xor(B13, B[13], Op);
   xor(B14, B[14], Op);
   xor(B15, B[15], Op);
   xor(B16, B[16], Op);
   xor(B17, B[17], Op);
   xor(B18, B[18], Op);
   xor(B19, B[19], Op);
   xor(B20, B[20], Op);
   xor(B21, B[21], Op);
   xor(B22, B[22], Op);
   xor(B23, B[23], Op);
   xor(B24, B[24], Op);
   xor(B25, B[25], Op);
   xor(B26, B[26], Op);
   xor(B27, B[27], Op);
   xor(B28, B[28], Op);
   xor(B29, B[29], Op);
   xor(B30, B[30], Op);
   xor(B31, B[31], Op);
   
   // signed overflow check
   xor(C, C31, Op);     
   xor(V, C31, C30);     
   
   // unsigned overflow check
   assign Cout = C31;   
   
   full_adder fa0(S[0], C0, A[0], B0, Op);    // Least significant bit.
   full_adder fa1(S[1], C1, A[1], B1, C0);
   full_adder fa2(S[2], C2, A[2], B2, C1);
   full_adder fa3(S[3], C3, A[3], B3, C2);   
   full_adder fa4(S[4], C4, A[4], B4, C3);
   full_adder fa5(S[5], C5, A[5], B5, C4);
   full_adder fa6(S[6], C6, A[6], B6, C5);   
   full_adder fa7(S[7], C7, A[7], B7, C6);
   full_adder fa8(S[8], C8, A[8], B8, C7);
   full_adder fa9(S[9], C9, A[9], B9, C8);   
   full_adder fa10(S[10], C10, A[10], B10, C9);
   full_adder fa11(S[11], C11, A[11], B11, C10);
   full_adder fa12(S[12], C12, A[12], B12, C11);   
   full_adder fa13(S[13], C13, A[13], B13, C12);
   full_adder fa14(S[14], C14, A[14], B14, C13);
   full_adder fa15(S[15], C15, A[15], B15, C14);
   full_adder fa16(S[16], C16, A[16], B16, C15);
   full_adder fa17(S[17], C17, A[17], B17, C16);
   full_adder fa18(S[18], C18, A[18], B18, C17);   
   full_adder fa19(S[19], C19, A[19], B19, C18);
   full_adder fa20(S[20], C20, A[20], B20, C19);
   full_adder fa21(S[21], C21, A[21], B21, C20); 
   full_adder fa22(S[22], C22, A[22], B22, C21);
   full_adder fa23(S[23], C23, A[23], B23, C22);
   full_adder fa24(S[24], C24, A[24], B24, C23);   
   full_adder fa25(S[25], C25, A[25], B25, C24);
   full_adder fa26(S[26], C26, A[26], B26, C25);
   full_adder fa27(S[27], C27, A[27], B27, C26);    
   full_adder fa28(S[28], C28, A[28], B28, C27); 
   full_adder fa29(S[29], C29, A[29], B29, C28);
   full_adder fa30(S[30], C30, A[30], B30, C29);
   full_adder fa31(S[31], C31, A[31], B31, C30);   
   
endmodule // ripple_carry_adder_subtractor

/*                 FULL ADDER FOR 16 BIT Subtractor          */
module full_adder(S, Cout, A, B, Cin);
   output S;
   output Cout;
   input  A;
   input  B;
   input  Cin;
   
   wire   w1;
   wire   w2;
   wire   w3;
   wire   w4;
   
   xor(w1, A, B);
   xor(S, Cin, w1);
   and(w2, A, B);   
   and(w3, A, Cin);
   and(w4, B, Cin);   
   or(Cout, w2, w3, w4);
endmodule

/* ========================================================= */
/*                 4x16 BIT DECODER                          */
/* ========================================================= */
module Decoder4x16 (input [3:0] select, input enable, output reg [15:0] out);

always @(select, enable)
begin

    if(enable == 1'b0)
        out = 16'b0000000000000000;
    else if(enable == 1'b1)
        if(select == 4'b0000)
            out <= 16'b0000000000000001;
        else if(select == 4'b0001)
            out <= 16'b0000000000000010;
        else if(select == 4'b0010)
            out <= 16'b0000000000000100;
        else if(select == 4'b0011)
            out <= 16'b0000000000001000;
        else if(select == 4'b0100)
            out <= 16'b0000000000010000;
        else if(select == 4'b0101)
            out <= 16'b0000000000100000;
        else if(select == 4'b0110)
            out <= 16'b0000000001000000;
        else if(select == 4'b0111)
            out <= 16'b0000000010000000;
        else if(select == 4'b1000)
            out <= 16'b0000000100000000;
        else if(select == 4'b1001)
            out <= 16'b0000001000000000;
        else if(select == 4'b1010)
            out <= 16'b0000010000000000;
        else if(select == 4'b1011)
            out <= 16'b0000100000000000;
        else if(select == 4'b1100)
            out <= 16'b0001000000000000;
        else if(select == 4'b1101)
            out <= 16'b0010000000000000;
        else if(select == 4'b1110)
            out <= 16'b0100000000000000;
        else if(select == 4'b1111)
            out <= 16'b1000000000000000;
    end

endmodule

/* ========================================================= */
/*                 16x4 BIT ENCODER                          */
/* ========================================================= */
module Encoder16x4(Enable, EncIn, BinOut);
input Enable;
input [15:0] EncIn;
output [3:0] BinOut;
reg [3:0] BinOut;
always @(Enable or EncIn)
begin
    BinOut = 0;
    if(Enable) begin
        case(EncIn)
            16'h0002 : BinOut = 1;
            16'h0004 : BinOut = 2;
            16'h0008 : BinOut = 3;
            16'h0010 : BinOut = 4;
            16'h0020 : BinOut = 5;
            16'h0040 : BinOut = 6;
            16'h0080 : BinOut = 7;
            16'h0100 : BinOut = 8;
            16'h0200 : BinOut = 9;
            16'h0400 : BinOut = 10;
            16'h0800 : BinOut = 11;
            16'h1000 : BinOut = 12;
            16'h2000 : BinOut = 13;
            16'h4000 : BinOut = 14;
            16'h8000 : BinOut = 15;
        endcase
    end
end
endmodule

/* ========================================================= */
/*                   16 BIT MULTIPLEXER 	                 */
/* ========================================================= */
module Mux16(i15, i14, i13, i12, i11, i10, i9, i8, i7, i6, i5, i4, i3, i2, i1, i0, s, b);
   input [15:0] i15, i14, i13, i12, i11, i10, i9, i8, i7, i6, i5, i4, i3, i2, i1, i0;
   input [15:0]   s;
   output [15:0] b;
   assign b = (s[0]? i0 : (s[1]? i1 : (s[2]? i2 : (s[3]? i3 :
           (s[4]? i4 : (s[5]? i5 : (s[6]? i6 : (s[7]? i7 :
           (s[8]? i8 : (s[9]? i9 : (s[10]? i10 : (s[11]? i11 :
		   (s[12]? i12 :  (s[13]? i13 :(s[14]? i14 : i15)
		   ))))))))))))));
endmodule

/* ========================================================= */
/*                   32 BIT MULTIPLEXER 	                 */
/* ========================================================= */
module Mux32(i15, i14, i13, i12, i11, i10, i9, i8, i7, i6, i5, i4, i3, i2, i1, i0, s, b); 
   input [31:0] i15, i14, i13, i12, i11, i10, i9, i8, i7, i6, i5, i4, i3, i2, i1, i0;
   input [15:0]   s;
   output [31:0] b;
   assign b = (s[0]? i0 : (s[1]? i1 : (s[2]? i2 : (s[3]? i3 :
           (s[4]? i4 : (s[5]? i5 : (s[6]? i6 : (s[7]? i7 :
           (s[8]? i8 : (s[9]? i9 : (s[10]? i10 : (s[11]? i11 :
		   (s[12]? i12 :  (s[13]? i13 :(s[14]? i14 : i15)
		   ))))))))))))));
endmodule

/* ========================================================= */
/*                 32 BIT Divisor                            */
/* ========================================================= */
module Divisor32Bit(A, B, Q) ;
  input [31:0] A, B;    // inputs
  output reg [31:0] Q;      // quotient
  
  always @(*)
  begin
    if(B == 32'b00000000000000000000000000000000)
        begin
        Q <= 0;
        end
    else
        begin
        Q = A/B;
        end
  end
endmodule

/* ========================================================= */
/*                   	D Flip-Flop 			             */
/* ========================================================= */
module DFF(D,clk,Q);
input[31:0] D; // Data input 
input clk; // clock input 
output reg[31:0] Q; // output Q 
always @(posedge clk) 
begin
 Q <= D; 
end 
endmodule 

/* ========================================================= */
/*                   	Finite State Machine	             */
/* ========================================================= */
module FSM(CLOCK, InputOp, OutputOp, Error);
	input[3:0]	InputOp;
	input		CLOCK, Error;
	output reg[3:0]	OutputOp;
	reg[3:0] state;
	
	// Our Reset here is if OpCode = 1000
	// This machine will not allow continuous operations
	// The system default is to stay idle as much as possible unless commanded to do otherwise
	always @(posedge CLOCK)
	begin
	if(InputOp == 4'b1000)	// Reset
	begin					// Initializes State and Circuit (DFF)
		state <= 4'b0000;		// Go to Ready
		OutputOp <= 4'b1000;	// Reset
	end
	else
	begin	
		case(state)
		4'b0000:	// Ready
		begin
			if(InputOp == 4'b0000)	// No-Operation
			begin
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
			else if(InputOp == 4'b0001)	// Add
			begin
			state <= 4'b0001;		// Go to ADD
			OutputOp <= 4'b0001;	// Addition
			end
			else if(InputOp == 4'b0010)	// Sub
			begin
			state <= 4'b0010;		// Go to SUB
			OutputOp <= 4'b0010;	// Subtraction	
			end
			else if(InputOp == 4'b0011)	// And
			begin
			state <= 4'b0011;		// Go to AND
			OutputOp <= 4'b0011;	// And Op
			end
			else if(InputOp == 4'b0100)	// Or
			begin
			state <= 4'b0100;		// Go to OR
			OutputOp <= 4'b0100;	// Or Op
			end
			else if(InputOp == 4'b0101)	// Not
			begin
			state <= 4'b0101;		// Go to NOT
			OutputOp <= 4'b0101;	// Not Op
			end
			else if(InputOp == 4'b0110)	// Xor
			begin
			state <= 4'b0110;		// Go to XOR
			OutputOp <= 4'b0110;	// Xor Op
			end
			else if(InputOp == 4'b0111)	// Mult
			begin
			state <= 4'b0111;		// Go to MULT
			OutputOp <= 4'b0111;	// Multiply Op
			end
			else if(InputOp == 4'b1001)	// Div
			begin
			state <= 4'b1001;		// Go to DIV
			OutputOp <= 4'b1001;	// Divide Op
			end
			else					// If Not Valid Next State then we consider it a No-Operation
			begin					// This will simply return it to Ready and the system will be idle
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
		end
		4'b0001:	// ADD
		begin
			if(InputOp == 4'b0000)	// No-Operation
			begin
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
			else if(Error == 1)			// If there is an error
			begin
			state <= 4'b1000;		// Go to Error
			OutputOp <= 4'b1111;	// Safety Net for Error just stay idle
			end
			else					// If Not Valid Next State then we consider it a No-Operation
			begin					// This will simply return it to Ready and the system will be idle
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
		end
		4'b0010:	// SUB
		begin
			if(InputOp == 4'b0000)	// No-Operation
			begin
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
			else if(Error == 1)			// If there is an error
			begin
			state <= 4'b1000;		// Go to Error
			OutputOp <= 4'b1111;	// Safety Net for Error just stay idle
			end
			else					// If Not Valid Next State then we consider it a No-Operation
			begin					// This will simply return it to Ready and the system will be idle
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
		end
		4'b0011:	// AND
		begin
			if(InputOp == 4'b0000)	// No-Operation
			begin
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
			else					// If Not Valid Next State then we consider it a No-Operation
			begin					// This will simply return it to Ready and the system will be idle
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end	
		end
		4'b0100:	// OR
		begin
			if(InputOp == 4'b0000)	// No-Operation
			begin
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
			else					// If Not Valid Next State then we consider it a No-Operation
			begin					// This will simply return it to Ready and the system will be idle
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
		end
		4'b0101:	// NOT
		begin
			if(InputOp == 4'b0000)	// No-Operation
			begin
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
			else					// If Not Valid Next State then we consider it a No-Operation
			begin					// This will simply return it to Ready and the system will be idle
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
		end
		4'b0110:	// XOR
		begin
			if(InputOp == 4'b0000)	// No-Operation
			begin
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
			else					// If Not Valid Next State then we consider it a No-Operation
			begin					// This will simply return it to Ready and the system will be idle
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
		end
		4'b0111:	// MULT
		begin
			if(InputOp == 4'b0000)	// No-Operation
			begin
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
			else					// If Not Valid Next State then we consider it a No-Operation
			begin					// This will simply return it to Ready and the system will be idle
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
		end
		4'b1000:	// ERR
		begin
			// You have to Reset to get out of this state
			state <= 4'b0000;		// Go to Ready
			OutputOp <= 4'b1000;	// Reset
		end
		4'b1001:	// DIV
		begin
			if(InputOp == 4'b0000)	// No-Operation
			begin
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
			else					// If Not Valid Next State then we consider it a No-Operation
			begin					// This will simply return it to Ready and the system will be idle
			state <= 4'b0000;		// Stay at Ready
			OutputOp <= 4'b0000;	// Idle
			end
		end
		endcase
	end
	end
endmodule 

/* ========================================================= */
/*                     Error Checker			             */
/* ========================================================= */
module ErrorChecker(check, operation, error);
input check;
input[3:0] operation;
output reg error;

      always @(*)
	  begin
        if(operation == 4'b0001)		// If adding
        begin
            if(check == 1'b1)				// If carry-out equals 1 then there is an error
            begin							
            error = 1;
            end
            else
            begin
            error = 0;
            end
        end
        else if(operation == 4'b0010)	// If subtracting
        begin	
            if(check == 1'b0)				// If carry-out equals 0 then there is an error
            begin
            error = 1;
            end
            else
            begin
            error = 0;
            end
        end
		else if(operation == 4'b1111)
		begin
		error = 1;
		end
        else
        begin
        error = 0;
        end
    end

endmodule

/* ========================================================= */
/*                   	BREAD BOARD				             */
/* ========================================================= */

module BreadBoard(inputB, OpCode, CLK, enable);
	/* INPUTS */
	input[31:0] inputB;
	input[3:0] OpCode; // same as opcode
	input CLK;
	input enable;
	
	// Mutilplexor
	wire[31:0] ch15,ch9,ch8,ch7,ch6,ch5,ch4,ch3,ch2_ch1,ch0;
	wire[31:0] x14, x13, x12, x11, x10;	// substitute
	wire[31:0] result;	// goes with DFF
	wire[15:0] select;
    
	// Decoder
	wire[15:0] sout;
	
	// Encoder
	//wire[15:0] sin;
	wire[3:0] OpVal; // same as opcode

	// D Flip-Flop
	wire[31:0] valueout;

	/* Sum & Difference */
	/*
		Addition
		Wires In: inputB & valueout & Opcode[1]
		Wires Out: ch1 & overflow (V) & Carry
	*/
	/*
		Subtraction
		Wires In: inputB & valueout & Opcode[1]
		Wires Out: ch2 & overflow(V) & Carry
	*/
	wire v,c,cout,error;


	/* Logic Gates */
	/*
		AND Gate
		Wires In: inputB & valueout
		Wires Out: ch3
	*/
	/*
		OR Gate
		Wires In: inputB & valueout
		Wires Out: ch4
	*/
	/*
		NOT Gate
		Wires In: inputB & valueout
		Wires Out: ch5
	*/
	/*
		XOR Gate
		Wires In: inputB & valueout
		Wires Out: ch6
	*/

	/* Product */
	/*  
		Multiplication
		Wires In: Lower 16 of inputB & valueout
		Wires Out: ch7
	*/
	// Lower 16 bits for Multiplier
	wire [15:0] inB, valout;
	assign inB = inputB[15:0];
	assign valout = valueout[15:0];
	
	// 32 bit divisor
	wire [31:0] Q;
	
	/* Intialization */
	// Channel 2 and 1 should be the same connection
	Mux32 mux(ch15, x14, x13, x12, x11, x10, ch9, ch8, ch7, ch6, ch5, ch4, ch3, ch2_ch1, ch2_ch1, ch0, sout, result);
	Decoder4x16 decoder(OpCode, enable, sout);
	Encoder16x4 encoder(enable, sout, OpVal);
	// For Channel 0 No Change
	assign ch0 = valueout;
	AdderSub AddSubt(ch2_ch1, c, v, cout, valueout, inputB, OpCode[1]);
	And_32 AND_Gate(inputB, valueout, ch3);
	Or_32 OR_Gate(inputB, valueout, ch4);
	Not_32 NOT_Gate(valueout, ch5);
	Xor_32 XOR_Gate(inputB, valueout, ch6);
	Multiplier16Bit mult(inB, valout, ch7);
	// For Channel 8 Reset
	assign ch8 = 32'b00000000000000000000000000000000;
	Divisor32Bit Div(valueout,inputB,ch9);
	// For Channel 15 Error State
	assign ch15 = 32'b00000000000000000000000000000000;
	DFF Dflipflop(result, CLK, valueout);
	ErrorChecker EC(cout,OpCode,error); 
	
endmodule

module testbench();

	reg[31:0] inputB;
	reg[3:0] OpCode;
	wire[3:0] OutOp;
	reg CLK;
	reg enable;
	reg error;
	
	// FSM will take in clock, opcode & error; then spit out a valid outop
	FSM FSmach(CLK, OpCode, OutOp, bread.error);
	// BreadBoard will take in an inputB, the valid OpCode, a clock, and enable to run the circuit
	BreadBoard bread(inputB,OutOp,CLK,enable);
	
//---------------------------------------------
//Clock
//---------------------------------------------

	initial
	begin
		forever
			begin
			#5
			CLK = 0;
			#5
			CLK = 1;
		end
	end
	
//---------------------------------------------
//The Display Thread with Clock Control
//---------------------------------------------
	initial
	begin
	#1 // Offset the Square Wave
	$display("C|                                           |                                           |             |                                           |     ");
	$display("L|                                      Input|                                 ACC/Result|  Instruction|                                      Value|     ");
	$display("K|         #|                             BIN|         #|                             BIN|  CMD|   OpCd|         #|                             BIN|Error");
	$display("-|----------|--------------------------------|----------|--------------------------------|-----|-------|----------|--------------------------------|-----");
		forever
			begin
				#5
				case(OutOp)
				4'b0000:			// No-Operation
				begin
				$display("%1b|%d|%b|%d|%b|No-Op|   %b|%d|%b|%b",CLK,bread.inputB,bread.inputB,bread.valueout,bread.valueout,OutOp,bread.result,bread.result,bread.error);
				end
				4'b0001:			// Add
				begin
				$display("%1b|%d|%b|%d|%b|ADD  |   %b|%d|%b|%b",CLK,bread.inputB,bread.inputB,bread.valueout,bread.valueout,OutOp,bread.result,bread.result,bread.error);
				end
				4'b0010:			// Subtract
				begin
				$display("%1b|%d|%b|%d|%b|SUB  |   %b|%d|%b|%b",CLK,bread.inputB,bread.inputB,bread.valueout,bread.valueout,OutOp,bread.result,bread.result,bread.error);
				end
				4'b0011:			// AND
				begin
				$display("%1b|%d|%b|%d|%b|AND  |   %b|%d|%b|%b",CLK,bread.inputB,bread.inputB,bread.valueout,bread.valueout,OutOp,bread.result,bread.result,bread.error);
				end
				4'b0100:			// OR
				begin
				$display("%1b|%d|%b|%d|%b|OR   |   %b|%d|%b|%b",CLK,bread.inputB,bread.inputB,bread.valueout,bread.valueout,OutOp,bread.result,bread.result,bread.error);
				end
				4'b0101:			// NOT
				begin
				$display("%1b|%d|%b|%d|%b|NOT  |   %b|%d|%b|%b",CLK,bread.inputB,bread.inputB,bread.valueout,bread.valueout,OutOp,bread.result,bread.result,bread.error);
				end
				4'b0110:			// XOR
				begin
				$display("%1b|%d|%b|%d|%b|XOR  |   %b|%d|%b|%b",CLK,bread.inputB,bread.inputB,bread.valueout,bread.valueout,OutOp,bread.result,bread.result,bread.error);
				end
				4'b0111:			// Multiply
				begin
				$display("%1b|%d|%b|%d|%b|MULT |   %b|%d|%b|%b",CLK,bread.inputB,bread.inputB,bread.valueout,bread.valueout,OutOp,bread.result,bread.result,bread.error);
				end
				4'b1000:			// Reset
				begin
				$display("%1b|%d|%b|%d|%b|Reset|   %b|%d|%b|%b",CLK,bread.inputB,bread.inputB,bread.valueout,bread.valueout,OutOp,bread.result,bread.result,bread.error);
				end
				4'b1001:			// Divide
				begin
				$display("%1b|%d|%b|%d|%b|DIV  |   %b|%d|%b|%b",CLK,bread.inputB,bread.inputB,bread.valueout,bread.valueout,OutOp,bread.result,bread.result,bread.error);
				end
				4'b1111:			// Error
				begin
				$display("%1b|%d|%b|%d|%b|ERROR|   %b|%d|%b|%b",CLK,bread.inputB,bread.inputB,bread.valueout,bread.valueout,OutOp,bread.result,bread.result,bread.error);
				end
				endcase
			end
	end
	
//---------------------------------------------
//Input Stimulus
//---------------------------------------------
	initial 
	begin
assign enable = 1'b1;
		#2 // Offset the Square Wave
		// Print out Errors
		inputB = 32'b11111111111111111111111111111111;
		OpCode = 4'b1000;	// Initialize with Reset always
		#20
		OpCode = 4'b0001;	// Addition
		#20
	    inputB = 32'b00000000000000000000000000000001;
		#20
	    inputB = 32'b00000000000000000000000000000001;
	    OpCode = 4'b0010;	// Subtraction
		#50
		// Print out All Operations
		inputB = 32'b00000000000000000000000000000010;
		OpCode = 4'b1000;	// Initialize with Reset always
		#20
		OpCode = 4'b0001;	// Addition
		#20
		inputB = 32'b00000000000000000000000000000100;
		OpCode = 4'b0111;	// Multiply
		#20
		inputB = 32'b00000000000000000000000000000100;
		OpCode = 4'b0010;	// Subtract
		#20
		inputB = 32'b00000000011100000000000000000100;
		OpCode = 4'b0011;	// AND
		#20
		inputB = 32'b00000000000000000000000011110100;
		OpCode = 4'b0100;	// OR
		#20
		OpCode = 4'b0101;	// NOT
		#20
		inputB = 32'b00000000000000000000000011110100;
		OpCode = 4'b0110;	// XOR
		#20
		OpCode = 4'b0000;	// No-Operation
		#20
		OpCode = 4'b1000;	// Reset
		#20
		inputB = 32'b00000000000000000000000000010101;
		OpCode = 4'b0001;	// Addition
		#20
		inputB = 32'b00000000000000000000000000000011;
		OpCode = 4'b1001;	// Divide
		#20
		OpCode = 4'b1000;	// Reset
		#20
		$finish;
	end

endmodule