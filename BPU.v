module BPU(
    clk,
    rst_n,
    branch,
    stall,
    miss,
    pred
);

input clk;
input rst_n;
input branch;
input stall;
input miss;
output pred;

reg [1:0] state_r, state_w;

parameter NB    = 2'd0;
parameter TNB   = 2'd1;
parameter TBR   = 2'd2;
parameter BR    = 2'd3;

assign pred = ((state_r == NB) || (state_r == TNB)) ? 1'b0 : 1'b1;

always@(*) begin
    state_w = state_r;
    if ((branch == 1'b1) && (stall == 1'b0)) begin
        case(state_r)
            NB:begin
                case(miss)
                    1'b0:state_w = NB;
                    1'b1:state_w = TNB;
                endcase
            end
            TNB:begin
                case(miss)
                    1'b0:state_w = NB;
                    1'b1:state_w = BR;
                endcase
            end
            TBR:begin
                case(miss)
                    1'b0:state_w = BR;
                    1'b1:state_w = NB;
                endcase
            end
            BR:begin
                case(miss)
                    1'b0:state_w = BR;
                    1'b1:state_w = TBR;
                endcase
            end
        endcase
    end
end

always@(posedge clk) begin
    if (~rst_n) begin
        state_r <= NB;
    end

    else begin
        state_r <= state_w;
    end
end

endmodule