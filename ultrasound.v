module ultrasound(clk_in, clk_out
, increment, transmit, receive
, clk_5M
, z_on, markers
, reset_in, reset_out
, on_in, on_out
, receive_inverse
, encoderPinA, encoderPinB
, sin, cos
, VGA_HS, VGA_VS, VGA_BLANK, VGA_SYNC,
test
);
    input clk_in, reset_in, on_in, encoderPinA, encoderPinB;
    output clk_out, reset_out, on_out;
    output increment, transmit, receive
	 
    , clk_5M
    , z_on, markers
    , receive_inverse
	 , VGA_HS, VGA_VS, VGA_BLANK, VGA_SYNC
    ;
	 
	 output test;
	 assign test = encoderPinA;
	 
    
    output[7:0] sin, cos; //sin on red VGA pin, cos on green
    
    // debug inputs
    assign clk_out = clk_in;
    assign reset_out = reset_in;
    assign on_out = on_in;
    
    clk_div MHz5(clk_in, 1'b0, clk_5M);
    FSMv3 mvp(clk_5M, on_in, ~reset_in, increment, transmit, receive, z_on, markers);
    
    assign receive_inverse = ~receive;
    //assign cos = angle*2;
	 
    // Ramps
//    vonRamp ramp(clk_5M, encoderPinA, encoderPinB, reset_in, sin,cos);
	 
    // VGA
	 wire [31:0] X, Y;
	 wire display;
	 VGA jokes(clk_5M, 1'b0, VGA_HS, VGA_VS, VGA_BLANK, VGA_SYNC, X, Y, display);
	 
	 wire [7:0] angle;
	 getAngle(increment, angle, encoderPinA, encoderPinB);//clock,angle,start,reverse
	 angles(angle, sin, cos);
	 
endmodule 
module vonRamp(clk_5M,encoderPinA, encoderPinB, reset_in, sin,cos);

    input clk_5M,encoderPinA, encoderPinB, reset_in;

    output [7:0] sin;
	 output [7:0] cos;

    

    reg quadA, quadB, quadA_delayed, quadB_delayed;

    always @(posedge clk_5M) quadA <= encoderPinA;

    always @(posedge clk_5M) quadB <= encoderPinB;

    always @(posedge clk_5M) quadA_delayed <= quadA;

    always @(posedge clk_5M) quadB_delayed <= quadB;

    wire count_enable = quadA ^ quadA_delayed ^ quadB ^ quadB_delayed;

    wire count_direction = quadA ^ quadB_delayed;

    reg [7:0] count = 8'h80;

    always @(posedge clk_5M)

        begin

            if(count_enable)

            begin

                if(count_direction) count<=count+1; else count<=count-1;

            end

        end

    

    wire [7:0] angle = count;

    

//    reg [7:0] sin_reg,cos_reg;

//    integer a = 32'h00000080;

//    assign i_encoderPinA = ~encoderPinA;

//    reg [31:0] angle = 32'h00000080;  //128 in decimal, setting to 45 degrees.

//    always @(posedge encoderPinA or posedge i_encoderPinA or posedge reset_in)

//        begin

//          if (reset_in)

//              begin

//                 sin_reg <= 8'b00000000;

//                 cos_reg <= 8'b00000000;

//              end

//          else if(~(encoderPinA == encoderPinB))

//                begin

//                    a = a-1;

//                end

//          else if (encoderPinA == encoderPinB)

//                begin

//                    a = a+1;

//                end

//        end

//    always @(negedge encoderPinA or posedge reset_in)

//     

//        begin

//          if (reset_in)

//              begin

//                 sin_reg <= 8'b00000000;

//                 cos_reg <= 8'b00000000;

//              end

//          else if(~(encoderPinA == encoderPinB))

//                begin

//                    a = a-1;

//                end

//          else if (encoderPinA == encoderPinB)

//                begin

//                    a = a+1;

//                end

//        end

    //send angle, sin, and cos through sine and cosine modules    

    angles ang(angle, sin, cos);

endmodule

module angles(angle, sin, cos

, a1, a2

);

    input [7:0] angle;

    output [7:0] sin;
	 output [7:0] cos;

  output [31:0] a1, a2;

    

    wire [63:0] angle_1 = angle * 200;     //convert to angle * 35

    wire [63:0] angle_2 = 9000 - angle_1;





  //wire [63:0] a = 18000-angle_1;

 //wire [63:0] b = 4000*angle_1*a;

  //wire [63:0] c = 405000000-angle_1*a;

  //wire [63:0] sine = (256*b)/c;





    wire [63:0] sine = (256*((4000*angle_1*(18000-angle_1))/(405000000-angle_1*(18000-angle_1))))/1000;   //modified Bhaskara's to use 100*angle

    wire [63:0] cosine = (256*((4000*angle_2*(18000-angle_2))/(405000000-angle_2*(18000-angle_2))))/1000;

    

//    real angle_in = a;

//    real angle_1 = angle_in * factor;

//    real angle_2 = (angle_in * factor) + 90;

//    real factor = 0.35;

//    real normalize = 256;

//    real a = 4;

//    real b = 180;

//    real c = 40500;

//

//    real sine = ((a*angle_1*(b-angle_1))/(c-angle_1*(b-angle_1)))*normalize; //Bhaskara's approx.

//    real cosine = ((a*angle_2*(b-angle_2))/(c-angle_2*(b-angle_2)))*normalize;

//    

//

//    assign sin = sine;

//    assign cos = cosine;

    assign sin = sine[7:0];

    assign cos = cosine[7:0];

  assign a1 = angle_1;

  assign a2 = angle_2;

endmodule

module FSMv3(clockIn, on, reset, increment, transmit, receive, z_on, markers);
    input clockIn, on, reset;
    output increment, transmit, receive, z_on, markers;
    
     wire S1, S0, resetCounter, resetCounterNextCycle, NS1, NS0;
    wire counterIs0, delayIncrement, delayTransmit, delayReceive;  // will be 1 when true
    wire [31:0] count;
     wire increment_intermediate;
    
    // set up counter
    counterBME counter(clock, resetCounter | reset, count);
    
     // only run clock when machine is on
     wire clock;
     assign clock = clockIn & on;
     
    // set up DFFs for S0 and S1
    dffe S1_dff(.d(NS1), .clk(clock), .clrn(~reset), .prn(1'b1), .ena(1'b1), .q(S1));
    dffe S0_dff(.d(NS0), .clk(clock), .clrn(~reset), .prn(1'b1), .ena(1'b1), .q(S0));
    
    // set up reset clock DFF (delay 1 cycle)
    dffe resetCtr_dff(.d(resetCounterNextCycle), .clk(clock), .clrn(~reset), .prn(1'b1), .ena(1'b1), .q(resetCounter));
    
    // assign controls
    assign counterIs0 = (count == 0);
    assign delayIncrement = (count == 20000);
    assign delayTransmit = (count == 50); // 10us delay
    assign delayReceive = (count == 2000);
    
    // next state calculation    
    assign NS1 = (~S1 & S0 & delayTransmit) | (S1 & ~S0 & ~delayReceive);
    assign NS0 = (~S1 & ~S0 & delayIncrement) | (~S1 & S0 & ~delayTransmit);
    
    // reset clock calculation
    assign resetCounterNextCycle = (~S1 & ~S0 & delayIncrement) | (~S1 & S0 & delayTransmit) | (S1 & ~S0 & delayReceive);
    
    // output signals calculation
    assign increment_intermediate = ~S1 & ~S0 & (count == 0);
     dffe increment_dff(.d(increment_intermediate), .clk(clock), .clrn(~reset), .prn(1'b1), .ena(1'b1), .q(increment));
    assign transmit = ~S1 & S0 & (count == 0);
    assign receive = S1 & ~S0;
    assign z_on = S1 & ~S0 & (count >= 65); // 65 = time to move one cm
    assign markers = S1 & ~S0 & ((count % 65) == 0) & (count != 0);  // every 65 cycles (1 cm), 1 pulse    
endmodule 
module counterBME(clock, reset, out);
    input clock, reset;
    output [31:0] out;
    
    dffe counter_dff_initial(.d(~out[0]), .clk(clock), .clrn(!reset), .prn(1'b1), .ena(1'b1), .q(out[0]));
    
    genvar i;
    generate
        for (i=1; i<32; i=i+1) begin: generateCounter
            dffe counter_dff(.d(~out[i]), .clk(~out[i-1]), .clrn(!reset), .prn(1'b1), .ena(1'b1), .q(out[i]));
        end
    endgenerate    
endmodule
module clk_div 
#( 
parameter WIDTH = 3, // Width of the register required
parameter N = 5// We will divide by 12 for example in this case]
)
(clk,reset, clk_out);
    input clk;
    input reset;
    output clk_out;
     
    reg [WIDTH-1:0] r_reg;
    wire [WIDTH-1:0] r_nxt;
    reg clk_track;
     
    always @(posedge clk or posedge reset)
     
    begin
      if (reset)
          begin
              r_reg <= 0;
        clk_track <= 1'b0;
          end
     
      else if (r_nxt == N)
            begin
              r_reg <= 0;
              clk_track <= ~clk_track;
            end
     
      else 
            r_reg <= r_nxt;
    end
     
     assign r_nxt = r_reg+1;             
     assign clk_out = clk_track;
endmodule 

module VGA(clock, reset, VGA_HS, VGA_VS, VGA_BLANK, VGA_SYNC, X, Y, display);

	input clock, reset;
	output VGA_VS, VGA_HS, VGA_BLANK, VGA_SYNC, display;
	output[31:0] X, Y;

	// CONSTANTS
	wire[31:0] ha, hb, hc, hd, va, vb, vc, vd, hT, vT;
	wire hp, vp;
	
	// 1280x1024
	assign ha = 112; 
	assign hb = 248;
	assign hc = 1280;
	assign hd = 48;
	assign hp = 1'b1;
	assign va = 3;		
	assign vb = 38;
	assign vc = 1024; 
	assign vd = 1;
	assign vp = 1'b1;
	
	// 1440x900
//	assign ha = 152; 
//	assign hb = 232;
//	assign hc = 1440;
//	assign hd = 80;
//	assign hp = 1'b1;
//	assign va = 3;		
//	assign vb = 28;
//	assign vc = 900; 
//	assign vd = 1;
//	assign vp = 1'b1;
	
	
	assign hT = ha + hb + hc + hd;  
	assign vT = va + vb + vc + vd; 
	
	
	assign VGA_SYNC = 1'b0; 
	assign VGA_BLANK = 1'b1;
	assign VGA_HS = hs;
	assign VGA_VS = vs;
	
	reg[31:0] Xcount, X; 
	reg[31:0] Ycount, Y; 

	
	reg hs, vs, blank;
	reg display; 
	
	always @(posedge clock) begin
		if(reset) begin 
			Xcount = 0; 
			Ycount = 0; 
			hs = ~hp; 
			vs = ~vp; 
			display = 1'b0;
			X = 0; 
			Y = 0; 
		end
		
		else if(clock) begin
			
			if (Xcount < (hT-1)) begin
				Xcount = Xcount + 1;
			end else begin
				Xcount = 0;
				
				if(Ycount < (vT-1)) begin
					Ycount = Ycount + 1;
				end else begin
					Ycount = 0;
				end
				
			end
			
		end
		
		if ((Xcount < (hc + hd)) | (Xcount > (hc + hd + ha))) begin
			hs = ~hp; 
		end else begin 
			hs = hp; 
		end 
		
		if ((Ycount < (vc + vd)) | (Ycount > (vc + vd + va))) begin
			vs = ~vp; 
		end else begin 
			vs = vp; 
		end 
		

		if (Xcount < hc) begin 
			X = Xcount;				
		end
		if (Ycount < vc) begin 
			Y = Ycount;				
		end
		
		
		if ((Xcount < hc) & (Ycount < vc)) begin
			display = 1'b1;
		end else begin													  
			display = 1'b0;
		end
		
	end
endmodule 

module getAngle(clock, angle, start, reverse, startIncrement, startDecrement);

output [7:0] angle;
input start, reverse, clock;

reg [7:0] cnt; 

output startIncrement, startDecrement;

dffe incrementDFF(.d(start), .clk(start||reverse), .clrn(1'b1), .prn(1'b1), .ena(1'b1), .q(startIncrement));
dffe decrementDFF(.d(reverse), .clk(start||reverse), .clrn(1'b1), .prn(1'b1), .ena(1'b1), .q(startDecrement));

assign angle = cnt;
	
always @ (posedge clock)
begin
	if(start)
		cnt = 8'h00;
    else if(startIncrement && (cnt < 1011010))

        cnt = cnt + 1;

    else if((startDecrement || cnt == 1011010) && (cnt > 00000000))
        cnt = cnt - 1;
end

endmodule




