// Top module of your design, you cannot modify this module!!
module CHIP (	clk,
				rst_n,
//----------for slow_memD------------
				mem_read_D,
				mem_write_D,
				mem_addr_D,
				mem_wdata_D,
				mem_rdata_D,
				mem_ready_D,
//----------for slow_memI------------
				mem_read_I,
				mem_write_I,
				mem_addr_I,
				mem_wdata_I,
				mem_rdata_I,
				mem_ready_I,
//----------for TestBed--------------				
				DCACHE_addr, 
				DCACHE_wdata,
				DCACHE_wen   
			);
input			clk, rst_n;
//--------------------------

output			mem_read_D;
output			mem_write_D;
output	[31:4]	mem_addr_D;
output	[127:0]	mem_wdata_D;
input	[127:0]	mem_rdata_D;
input			mem_ready_D;
//--------------------------
output			mem_read_I;
output			mem_write_I;
output	[31:4]	mem_addr_I;
output	[127:0]	mem_wdata_I;
input	[127:0]	mem_rdata_I;
input			mem_ready_I;
//----------for TestBed--------------
output	[29:0]	DCACHE_addr;
output	[31:0]	DCACHE_wdata;
output			DCACHE_wen;
//--------------------------

// wire declaration
wire        ICACHE_ren;
wire        ICACHE_wen;
wire [29:0] ICACHE_addr;
wire [31:0] ICACHE_wdata;
wire        ICACHE_stall;
wire [31:0] ICACHE_rdata;

wire        DCACHE_ren;
wire        DCACHE_wen;
wire [29:0] DCACHE_addr;
wire [31:0] DCACHE_wdata;
wire        DCACHE_stall;
wire [31:0] DCACHE_rdata;


//------------------------------
	// Note that the overall design of your RISCV includes:
	// 1. pipelined RISCV processor
	// 2. data cache
	// 3. instruction cache


	RISCV_Pipeline i_RISCV(
		// control interface
		.clk            (clk)           , 
		.rst_n          (rst_n)         ,
//----------I cache interface-------		
		.ICACHE_ren     (ICACHE_ren)    ,//o =1
		.ICACHE_wen     (ICACHE_wen)    ,//o =0
		.ICACHE_addr    (ICACHE_addr)   ,//o 6~31 bit =0
		.ICACHE_wdata   (ICACHE_wdata)  ,//0 =0
		.ICACHE_stall   (ICACHE_stall)  ,//i
		.ICACHE_rdata   (ICACHE_rdata)  ,//i
//----------D cache interface-------
		.DCACHE_ren     (DCACHE_ren)    ,//o
		.DCACHE_wen     (DCACHE_wen)    ,//o
		.DCACHE_addr    (DCACHE_addr)   ,//o
		.DCACHE_wdata   (DCACHE_wdata)  ,//o
		.DCACHE_stall   (DCACHE_stall)  ,//i
		.DCACHE_rdata   (DCACHE_rdata)//i
	);
	

	cache D_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (DCACHE_ren)  ,
        .proc_write (DCACHE_wen)  ,
        .proc_addr  (DCACHE_addr) ,
        .proc_rdata (DCACHE_rdata),
        .proc_wdata (DCACHE_wdata),
        .proc_stall (DCACHE_stall),
        .mem_read   (mem_read_D)  ,
        .mem_write  (mem_write_D) ,
        .mem_addr   (mem_addr_D)  ,
        .mem_wdata  (mem_wdata_D) ,
        .mem_rdata  (mem_rdata_D) ,
        .mem_ready  (mem_ready_D)
	);

	cache I_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (ICACHE_ren)  ,
        .proc_write (ICACHE_wen)  ,
        .proc_addr  (ICACHE_addr) ,
        .proc_rdata (ICACHE_rdata),
        .proc_wdata (ICACHE_wdata),
        .proc_stall (ICACHE_stall),
        .mem_read   (mem_read_I)  ,
        .mem_write  (mem_write_I) ,
        .mem_addr   (mem_addr_I)  ,
        .mem_wdata  (mem_wdata_I) ,
        .mem_rdata  (mem_rdata_I) ,
        .mem_ready  (mem_ready_I)
	);
endmodule

module RISCV_Pipeline(
	// control interface
		clk, 
		rst_n,
//----------I cache interface-------		
		ICACHE_ren,
		ICACHE_wen,
		ICACHE_addr,
		ICACHE_wdata,
		ICACHE_stall,
		ICACHE_rdata,
//----------D cache interface-------
		DCACHE_ren,
		DCACHE_wen,
		DCACHE_addr,
		DCACHE_wdata,
		DCACHE_stall,
		DCACHE_rdata
);
	input clk,rst_n;
	input ICACHE_stall,DCACHE_stall;
	input [31:0] ICACHE_rdata,DCACHE_rdata;
	output reg ICACHE_ren,ICACHE_wen,DCACHE_ren,DCACHE_wen;
	output reg [31:0] ICACHE_wdata,DCACHE_wdata;
	output reg [29:0] ICACHE_addr,DCACHE_addr;
	localparam Cmp=2'd1;
	localparam WB=2'd2;
	integer i;
	//-----WB control---------//
	reg [31:0] WB_cmp_result;
	reg [31:0] WB_data;
	reg regwrite_cmp,regwrite_WB,regwrite_ID;
	reg jalr_WB,jalr_cmp,jalr_ID; 
	reg jal_WB,jal_cmp,jal_ID;
	reg mem_read_WB,mem_write_WB,mem_read_cmp,mem_write_cmp,mem_read_ID,mem_write_ID;
	//-----Computation control--------//
	reg [7:0] Mem_addr_cmp,Mem_addr_ID;
	reg Alusrc_ID,Alusrc_cmp;
	//------ID control---------------//
	reg beq,bneq;
	//-----instrction and pc---------//
	reg [31:0] Ins;
	reg [31:0] Ins_nxt;
	reg [4:0] Rd[0:2];
	reg [5:0] PC,PC_nxt,PC_ID,PC_ID_nxt,PC_cmp; //8bit -2'b00(4byte)=6bit PC
	reg [5:0] PC_add4[0:2],PC_add4_nxt; //fliflop
	reg [5:0] PC_plus4; //result after addtion of PC+4
	reg [5:0] PC_fwd_cmp,PC_fwd_WB;
	//-----Hazard detection---//
	reg [1:0] pc_mux,Ins_mux;
	reg control_mux;
	reg equality;
	//-----register-----------//
	reg [4:0] rs1_addr_cmp,rs2_addr_cmp;
	reg [31:0] rs1_fwd_data,rs2_fwd_data,rs1_cmp_data,rs2_cmp_data;
	wire [31:0] rs1_reg_data,rs2_reg_data;
	//-----ALU and cache D----------------//
	reg [31:0] alu_in_x,alu_in_y,alu_result_WB;
	wire [31:0] alu_result_cmp;
	//-----Imm Gen------------------------//
	reg [31:0] imm_output_reg,imm_output_cmp;
	reg t1,t2;
	//-----ALu control------------------//
	reg [3:0] alu_ctrl_output_reg,alu_ctrl_output_cmp;
	//-----Forward Unit-----------------//
	reg mux_A_cmp,mux_A_ID,mux_B_cmp,mux_B_ID;
	//----cache D-----------------------//
	reg [31:0] cacheD_result_WB;
	//-----cache D combinational--------//

	always@(*) begin
		if(mux_A_cmp) DCACHE_addr={24'd0,PC_fwd_WB};
		else DCACHE_addr={22'd0,Mem_addr_cmp};
		if(mux_B_cmp) DCACHE_wdata={WB_cmp_result[7:0],WB_cmp_result[15:8],WB_cmp_result[23:16],WB_cmp_result[31:24]};
		else DCACHE_wdata={rs2_cmp_data[7:0],rs2_cmp_data[15:8],rs2_cmp_data[23:16],rs2_cmp_data[31:24]};
		DCACHE_ren=mem_read_cmp;
		DCACHE_wen=mem_write_cmp;
	end
	//-----cache I combinational-------//
	always@(*) begin
		ICACHE_addr={24'd0,PC[5:0]};//since prcessor has to achieve one word(4 byte) from cache each time.
		//ICACHE_wdata=32'hffffffff;
		ICACHE_ren=1'b1;
		ICACHE_wen=1'b0;
	end
	//-----Forward Unit Combinational---//
	
	//-----Alu combinational------------//
	Alu alu(
		.ctrl(alu_ctrl_output_cmp),
		.x(alu_in_x),
		.y(alu_in_y),
		.result(alu_result_cmp)
	);
	always@(*) begin
		if(mux_A_cmp) alu_in_x=WB_cmp_result;
		else alu_in_x=rs1_cmp_data;
		if(Alusrc_cmp) alu_in_y=imm_output_cmp;                //////have some problem
		else alu_in_y=(mux_B_cmp)? WB_cmp_result:rs2_cmp_data;
	end

	//-----Alu control combinational----//
	always@(*) begin
		alu_ctrl_output_reg={(Ins[30]&Ins[4]&(Ins[12]|Ins[5])),Ins[14],(Ins[13]&Ins[4]),Ins[12]}; //Ins[30],Ins[14:12],Ins[5:4] 
	//out={(in[5]&in[0]&(in[2]|in[1])),in[4],(in[3]&in[0]),in[2]};
	end

	//-----imm gen combinational--------//
	always@(*) begin
    	t1=((!Ins[5])&(!Ins[3]))|Ins[2];
    	t2=((!Ins[6])|Ins[2])&(!Ins[3]);
        case({t1,t2})
            2'b00: imm_output_reg={{20{Ins[31]}},Ins[7],Ins[30:25],Ins[11:8],1'b0};//beq,bneq
            2'b01: imm_output_reg={{20{Ins[31]}},Ins[31:25],Ins[11:7]};//sw
            2'b10: imm_output_reg={{12{Ins[31]}},Ins[19:12],Ins[20],Ins[30:21],1'b0};//jal
            2'b11: imm_output_reg=(Ins[13:12]==2'b01)? {{27{Ins[25]}},Ins[24:20]}:{{20{Ins[31]}},Ins[31:20]};//jalr,lw, R type immediate, use funct 3to seperate slli,srai,srli 
        endcase
    end
	
	//-----regsiter combinational------//
	register_file register(
		.Clk(clk),
    	.rst_n(rst_n),
	    .WEN(regwrite_WB),
	    .RW(Rd[WB]),
	    .busW(WB_data),
	    .RX(Ins[19:15]),
	    .RY(Ins[24:20]),
	    .busX(rs1_reg_data),
	    .busY(rs2_reg_data)
	);
	always@(*) begin

		if(mem_read_WB) WB_cmp_result=cacheD_result_WB;
		else WB_cmp_result=alu_result_WB;
		if(jal_WB||jalr_WB) WB_data={24'd0,PC_add4[WB],2'd0};
		else WB_data=WB_cmp_result;

		if (regwrite_WB&&(Rd[WB]==Ins[19:15])) rs1_fwd_data=WB_cmp_result;
		else rs1_fwd_data=rs1_reg_data;
		if (regwrite_WB&&(Rd[WB]==Ins[24:20])) rs2_fwd_data=WB_cmp_result;
		else rs2_fwd_data=rs2_reg_data;
		equality=(rs1_fwd_data[7:0]==rs2_fwd_data[7:0])? 1'b1:1'b0; //compare 8 bit for convenient
	end

	//-----Hazard Combinational--------//
	always@(*) begin
		mux_A_ID=regwrite_cmp&(Rd[Cmp]==Ins[19:15]);
		mux_B_ID=regwrite_cmp&(Rd[Cmp]==Ins[24:20]);

		if((mux_A_ID|mux_B_ID)&(beq||bneq)) begin
			pc_mux=2'b01; //pc-->pc
			Ins_mux=2'b01; //hold IF
			control_mux=1;	//load use hazzard	
		end
		else if(jalr_ID) begin
			pc_mux=2'b11; //jalr_address-->pc
			Ins_mux=2'b10; //flush IF
			control_mux=0;
		end
		else if (jal_ID) begin
			pc_mux=2'b10; //branch or jump address-->pc
			Ins_mux=2'b10; //flush IF
			control_mux=0;
		end
		else if ((beq&equality)||(bneq&(~equality))) begin
			pc_mux=2'b10; //branch or jump address-->pc
			Ins_mux=2'b10; //flush IF
			control_mux=0;
		end
		else begin
			pc_mux=2'b00; //pc+4-->pc
			Ins_mux=2'b00; //IF go to next address
			control_mux=0;
		end
	end
	//-----Instruction Combinational---//
	reg [31:0] flag;
	always @(*) begin
		case(Ins_mux)
			2'b00: Ins_nxt={ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]};
			2'b01: Ins_nxt=Ins;
			2'b10: Ins_nxt=32'd0;
			default: Ins_nxt=32'd0;
		endcase
		Mem_addr_ID=rs1_fwd_data[9:2]+imm_output_reg[9:2]; 
		PC_fwd_cmp=alu_result_cmp[7:2]+imm_output_reg[7:2];
		PC_plus4=PC[5:0]+1'b1;  //reduce adder area
		Rd[0]=Ins[11:7];
		flag={24'd0,PC_ID,2'd0};
		case(pc_mux)
			2'b00: begin //normal
				PC_nxt=PC_plus4;
				PC_ID_nxt=PC;
				PC_add4_nxt=PC_plus4;
			end
			2'b01: begin //stall (hold)
				PC_nxt=PC_ID;
				PC_ID_nxt=PC_ID;
				PC_add4_nxt=PC_add4[0];
			end
			2'b10: begin //beq 
				PC_nxt=PC_ID[5:0]+imm_output_reg[7:2];// reduce adder area origin:PC_ID+imm_result_0; 
				PC_ID_nxt=6'b111111;
				PC_add4_nxt=6'b111111;
			end
			2'b11: begin //jalr
				PC_nxt=Mem_addr_ID[5:0];       //pre-calculate jalr ,reduce adder area origin:rs1_0+imm_result_0;  only need 6bit for jumping
				PC_ID_nxt=6'b111111;
				PC_add4_nxt=6'b111111;
			end
		endcase
	end
	//-----control combinational--//
	always@(*)begin //.in({Ins[12],Ins[6:0]}),
    	beq=({Ins[12],Ins[6],Ins[2]}==3'b010)? 1'b1:1'b0;
    	bneq=({Ins[12],Ins[6],Ins[2]}==3'b110)? 1'b1:1'b0;
        regwrite_ID=(({Ins[5:4],Ins[2]}==3'b100)|(!Ins[11:7]))? 1'b0:1'b1; //Ins[11:7]=Rd[0];
        jalr_ID=(Ins[3:2]==2'b01)? 1'b1:1'b0;
        jal_ID=(Ins[3:2]==2'b11)? 1'b1:1'b0;
        mem_read_ID=({Ins[5:4],Ins[0]}==3'b001)? 1'b1:1'b0; //add in[0] to prevent when flush IF using 32'd0
        mem_write_ID=(Ins[6:4]==3'b010)? 1'b1:1'b0;
        Alusrc_ID=({{Ins[6]|Ins[4]},Ins[5],Ins[2]}==3'b110)? 1'b0:1'b1;
    end

	
	//------alu control,alu,imm,cacheD,register  sequential---------//
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			alu_ctrl_output_cmp<=6'd0;
			alu_result_WB<=32'd0;
			imm_output_cmp<=32'd0;
			rs1_cmp_data<=32'd0;
			rs2_cmp_data<=32'd0;
			rs1_addr_cmp<=5'd0;
			rs2_addr_cmp<=5'd0;
			cacheD_result_WB<=32'd0;
		end
		else begin
			if((~DCACHE_stall)&&(~ICACHE_stall)) begin
				alu_ctrl_output_cmp<=alu_ctrl_output_reg;
				alu_result_WB<=alu_result_cmp;
				imm_output_cmp<=imm_output_reg;
				rs1_cmp_data<=rs1_fwd_data;
				rs2_cmp_data<=rs2_fwd_data;
				rs1_addr_cmp<=Ins[19:15];
				rs2_addr_cmp<=Ins[24:20];
				cacheD_result_WB<={DCACHE_rdata[7:0],DCACHE_rdata[15:8],DCACHE_rdata[23:16],DCACHE_rdata[31:24]};
			end 
			else begin
				alu_ctrl_output_cmp<=alu_ctrl_output_cmp;
				alu_result_WB<=alu_result_WB;
				imm_output_cmp<=imm_output_cmp;
				rs1_cmp_data<=rs1_cmp_data;
				rs2_cmp_data<=rs2_cmp_data;
				rs1_addr_cmp<=rs1_addr_cmp;
				rs2_addr_cmp<=rs2_addr_cmp;
				cacheD_result_WB<=cacheD_result_WB;
			end
		end
	end
	
	//-----Instruction and pc sequential--//
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			Ins<=32'h00_00_00_00;
			for(i=1;i<=2;i=i+1) begin
				Rd[i]<=5'd0;
				PC_add4[i]<=6'd0;
			end
			PC_add4[0]<=6'd0;
			PC_ID<=6'd0;
			PC_cmp<=6'd0;
			PC<=6'd0;
			Mem_addr_cmp<=6'd0;
			PC_fwd_WB<=6'd0;
		end
		else begin
			if((~DCACHE_stall)&&(~ICACHE_stall)) begin
				Ins<=Ins_nxt;
				for(i=2;i>0;i=i-1) begin
					Rd[i]<=Rd[i-1];
					PC_add4[i]<=PC_add4[i-1];
				end
				PC_add4[0]<=PC_add4_nxt;
				PC_ID<=PC_ID_nxt;
				PC_cmp<=PC_ID;
				PC<=PC_nxt;
				Mem_addr_cmp<=Mem_addr_ID;
				PC_fwd_WB<=PC_fwd_cmp;
			end
			else begin
				Ins<=Ins;
				for(i=2;i>0;i=i-1) begin
					Rd[i]<=Rd[i];
					PC_add4[i]<=PC_add4[i];
				end
				PC_add4[0]<=PC_add4[0];
				PC_ID<=PC_ID;
				PC_cmp<=PC_cmp;
				PC<=PC;
				Mem_addr_cmp<=Mem_addr_cmp;
				PC_fwd_WB<=PC_fwd_WB;
			end
		end
	end

	//-----control,forward sequential------//
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			jalr_WB<=1'b0;
			jalr_cmp<=1'b0;
			jal_WB<=1'b0;
			jal_cmp<=1'b0;
			regwrite_WB<=1'b0;
			regwrite_cmp<=1'b0;
			mem_write_WB<=1'b0;
			mem_write_cmp<=1'b0;
			mem_read_WB<=1'b0;
			mem_read_cmp<=1'b0;
			Alusrc_cmp<=1'b0;
			mux_A_cmp<=1'b0;
			mux_B_cmp<=1'b0;
		end
		else begin
			if((~DCACHE_stall)&&(~ICACHE_stall)) begin
				jalr_WB<=jalr_cmp;
				jalr_cmp<=jalr_ID;
				jal_WB<=jal_cmp;
				jal_cmp<=jal_ID;
				regwrite_WB<=regwrite_cmp;
				mem_write_WB<=mem_write_cmp;
				mem_read_WB<=mem_read_cmp;
				Alusrc_cmp<=Alusrc_ID;
				if(control_mux) begin
					regwrite_cmp<=1'b0;
					mem_write_cmp<=1'b0;
					mem_read_cmp<=1'b0;
				end
				else begin
					regwrite_cmp<=regwrite_ID;
					mem_write_cmp<=mem_write_ID;
					mem_read_cmp<=mem_read_ID;
				end
				mux_A_cmp<=mux_A_ID;
				mux_B_cmp<=mux_B_ID;
			end
			else begin
				jalr_WB<=jalr_WB;
				jalr_cmp<=jalr_cmp;
				jal_WB<=jal_WB;
				jal_cmp<=jal_cmp;
				regwrite_WB<=regwrite_WB;
				regwrite_cmp<=regwrite_cmp;
				mem_write_WB<=mem_write_WB;
				mem_write_cmp<=mem_write_cmp;
				mem_read_WB<=mem_read_WB;
				mem_read_cmp<=mem_read_cmp;
				Alusrc_cmp<=Alusrc_cmp;
				mux_A_cmp<=mux_A_cmp;
				mux_B_cmp<=mux_B_cmp;
			end
		end
	end
endmodule


module Alu(ctrl,x,y,result);
	input  [3:0] ctrl;
	input  signed [31:0] x;
    input  signed [31:0] y;
	output reg [31:0] result;
	always@(*)
	begin
		case(ctrl[2:0])
			3'b000: result=(ctrl[3]==1'b0)? x+y:x-y;
			3'b111: result=x&y;
			3'b110: result=x|y;
			3'b100: result=x^y;
			3'b001: result=x<<y;
			3'b101: result=(ctrl[3]==1'b1)? x>>>y:x>>y;
			3'b010: result=(x<y)? 32'd1:32'd0;
			default: result=32'd0;
		endcase
	end
endmodule

module register_file(
    Clk  ,
    rst_n ,
    WEN  ,
    RW   ,
    busW ,
    RX   ,
    RY   ,
    busX ,
    busY
);
input        Clk, WEN,rst_n;
input  [4:0] RW, RX, RY;
input  [31:0] busW;
output reg [31:0] busX, busY;

reg [31:0] r[0:31];


always@(*) begin
        busX=r[RX];
        busY=r[RY];
end

integer i;
always@(posedge Clk) begin
        if(rst_n==1'b0) begin
            for (i=0;i<32;i=i+1) begin
                r[i]<=32'd0;
            end
        end
        else if (WEN&&RW) begin
            r[RW]<=busW;
        end
        else begin
        	for (i=0;i<32;i=i+1) begin
                r[i]<=r[i];
            end
        end
end

endmodule

module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output  reg       proc_stall;
    output  reg [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output  reg       mem_read, mem_write;
    output  reg [27:0] mem_addr;
    output reg [127:0] mem_wdata;
    
//==== wire/reg definition ================================
    localparam state_ready=2'b00;//initial state
    localparam state_update=2'b01;//update memory state
    localparam state_read=2'b10;//read data from memory state
    reg [1:0] cache_tag [0:7];
    reg [31:0] cache_data [0:7][0:3]; //7block,each has 4 words
    reg [1:0] state;
    reg [1:0] miss; //00:no miss 01:regular miss 10: initial miss(i.e. read memory initially)
    reg [2:0] target;//which block is reading/writing
    reg [1:0] word_region;//which word(1~4) in the target block is reading/writing 

//==== combinational circuit ==============================
always@(*) begin
    word_region=proc_addr[1:0];
    target=proc_addr[4:2];
    if(cache_tag[target]==proc_addr[6:5]) begin
    	miss=2'b00;
        proc_stall=1'b0;
    end
    else if (cache_tag[target][1]==1'b1) begin //initial miss
    	miss=2'b10;
        proc_stall=proc_read|proc_write;
    end
    else begin //regular miss
        miss=2'b01;
        proc_stall=proc_read|proc_write;
    end
    case(state)
        state_ready: begin
            proc_rdata=(proc_read)? cache_data[target][word_region]:32'hffffffff;
            mem_addr=28'd0;
            mem_wdata=128'd0;
            mem_write=0;
            mem_read=0;
        end
        state_update: begin
        	proc_rdata=32'hffffffff;
            mem_addr={cache_tag[target],target};
            mem_wdata={cache_data[target][3],cache_data[target][2],cache_data[target][1],cache_data[target][0]};
            mem_write=1;
            mem_read=0;
        end
        state_read: begin
        	proc_rdata=32'hffffffff;
            mem_addr=proc_addr[29:2];
            mem_wdata=128'd0;
            mem_write=0;
            mem_read=1;
        end
        default: begin
        	proc_rdata=32'hffffffff;
            mem_addr=28'd0;
            mem_wdata=128'd0;
            mem_write=0;
            mem_read=0;
        end
    endcase
end
//==== sequential circuit =================================
integer i;
always@( posedge clk ) begin
    if( proc_reset ) begin
        state<=state_ready;
        for(i=0;i<8;i=i+1) begin
            cache_tag[i]<=2'b11;
        end
    end
    else begin
        case(state)
            state_ready: begin
            	if((miss==2'b10)&&(proc_read||proc_write)) begin //initail miss
                    state<=state_read;
                end
            	else if((miss==2'b01)&&(proc_read||proc_write)) begin //regular miss
                    state<=state_update;
                end
                else begin //no miss
                    state<=state_ready;
                    if(proc_write==1) begin
                        cache_data[target][word_region]<=proc_wdata;
                    end
                    else begin
                        cache_data[target][word_region]<=cache_data[target][word_region];
                    end
                end
                
            end
            state_update: begin
                if(mem_ready==0) begin
                    state<=state_update;
                end
                else begin
                    state<=state_read;
                end
            end
            state_read: begin
                if(mem_ready==0) begin
                    state<=state_read;
                end
                else begin
                    state<=state_ready;
                    cache_data[target][3]<=mem_rdata[127:96];
                    cache_data[target][2]<=mem_rdata[95:64];
                    cache_data[target][1]<=mem_rdata[63:32];
                    cache_data[target][0]<=mem_rdata[31:0];
                    cache_tag[target]<={1'b0,proc_addr[5]};
                end
            end
            default: begin
            	state<=state_ready;
                if(proc_write==1) begin
                    cache_data[target][word_region]<=proc_wdata;
                end
                else begin
                    cache_data[target][word_region]<=cache_data[target][word_region];
                end
            end
        endcase
    end
end

endmodule