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
	else if(startIncrement && (cnt < 11111111))
		cnt = cnt + 1;
	else if((startDecrement || cnt == 1011010) && (cnt > 00000000))
		cnt = cnt - 1;
end

endmodule


