module IF(
    i_clk,
    i_rst_n,
    i_pc,       // program counter
    i_ins,      // instruction
    i_stall,    // stall signal
    i_b_req,    // branch request
    o_jump,     // taking o_pc_br
    o_miss,     // miss prediction
    o_pred,     // prediction
    o_pc_br     // branch pc
);

input         i_clk;
input         i_rst_n;
input  [31:0] i_pc;
input  [31:0] i_ins;
input         i_stall;
input         i_b_req;
output        o_jump;
output        o_miss;
output [31:0] o_pc_br;

comparator comp(
    .i_clk(i_clk),
    .i_rst_n(i_rst_n),
    .i_br(branch),
    .i_pred(o_pred),
    .i_req(i_b_req),
    .o_correct(hit)
);

BPU predict(
    .clk(i_clk),
    .rst_n(i_rst_n),
    .branch(branch),
    .stall(i_stall),
    .miss(~hit),
    .pred(o_pred)
);

decoder dec(
    .i_ins(i_ins),
    .o_br(branch),
    .o_imm(imm),
    .o_jal(jal)
);

wire hit;                       // not missing
wire branch;                    // beq || bneq
wire jal;                       // jal operation
wire [31:0] imm;                // immediate
reg  [31:0] pc_nt_r, pc_nt_w;   // pc not-taken
reg  [31:0] pc_r, pc_w;         // previous pc

assign o_pc_br = (o_miss)  ? (pc_nt_r)                              // miss, stall or immediate
                 (i_stall) ? (pc_r) : (i_pc + imm);

assign o_miss  = ~(hit || i_stall)                                  // flush
assign o_jump  = ((branch && pred) || jal || o_miss);              // address != pc + 4
assign pc_nt_w = (o_jump) ? (i_pc + 32'd4) : (i_pc + imm);          // pc not taken
assign pc_w    = i_pc;                                              // current pc

always@(posedge i_clk) begin
    if (~i_rst_n) begin
        pc_nt_r <= 32'b0; 
        pc_r    <= 32'b0;
    end

    else begin
        pc_nt_r <= pc_nt_w;
        pc_r    <= pc_w;
    end
end

endmodule