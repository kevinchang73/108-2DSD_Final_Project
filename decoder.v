module decoder(
    i_ins,
    o_br,
    o_imm
);

input i_ins;
output o_br;
output [31:0] o_imm;

wire [7:0] in;
wire beq;
wire bneq;

assign in = {i_ins[12], i_ins[6:0]};
assign beq  = ({in[7:6],in[2]}==3'b010)? 1'b1:1'b0;
assign bneq = ({in[7:6],in[2]}==3'b110)? 1'b1:1'b0;
assign o_br = (beq || bneq);
assign o_imm = (beq || bneq) ? {{20{i_ins[31]}},i_ins[7],i_ins[30:25],i_ins[11:8],1'b0} : 32'd4;

