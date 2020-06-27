module comparator(
    i_clk,
    i_rst_n,
    i_br,
    i_pred,
    i_req,
    o_correct
);

input i_clk;
input i_rst_n;
input i_br;
input i_pred;
input i_req;
output o_correct;

reg pred_r, pred_w;
reg state_r, state_w;
reg correct;

localparam S_IDLE = 1'b0;
localparam S_COMP = 1'b1;

assign o_correct = correct;

always@(*) begin
    pred_w = i_pred;
    state_w = ((i_br == 1'b1) && (correct == 1'b1)) ? S_COMP : S_IDLE;
    case(state_r)
        S_IDLE : correct = 1'b1;
        S_COMP : correct = (pred_r == i_req);
    endcase
end

always@(posedge i_clk) begin
    if (~i_rst_n) begin
        state_w <= S_IDLE;
        pred_r <= 1'b0;
    end

    else begin
        state_r = state_w;
        pred_r = pred_w;
    end
end

endmodule

