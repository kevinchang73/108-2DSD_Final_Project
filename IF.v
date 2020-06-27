module IF(
    i_clk,
    i_rst_n,
    i_ins,      // instruction
    i_stall,    // stall signal
    i_b_req,    // branch request
    o_flush,    // flush because of prediction
    o_pc        // program_counter
);
comparator comp(
    .i_clk(i_clk),
    i_rst_n(i_rst_n),
    i_br(branch),
    i_pred(pred),
    i_req(i_b_req),
    o_correct(~o_flush)
);

BPU predict(
    .clk(i_clk),
    .rst_n(i_rst_n),
    .branch(branch),
    .stall(i_stall),
    .miss(o_flush),
    .pred(pred)
);

decoder dec(
    .i_ins(i_ins),
    .o_br(branch),
    .o_imm(imm)
);

wire branch;            // branch start
wire pred;
wire [31:0] imm;        // immediate
reg  [31:0] pc_r, pc_w; // program counter

assign pc_w = (pred == 1'b1) ? (pc_r + 32'd4) : (pc_r + imm);
assign o_pc = pc_r;

always @(posedge i_clk) begin
    if (~i_rst_n) begin
        pc_r <= 32'b0;
    end

    else begin
        pc_r <= pc_w;
    end
end

endmodule