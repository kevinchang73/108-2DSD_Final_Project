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
	localparam Ex=2'd1;
	localparam Mem=2'd2;
	localparam WB=2'd3;
	integer i;
	
	//-----WB control---------//
	reg RegWrite[0:3]; 
	reg [1:0] DataToReg[0:3]; 
	reg Jalr[0:3]; 
	reg Jal[0:3];
	//-----Mem control--------//
	reg MemRead[0:2];
	reg MemWrite[0:2];
	//-----EX control---------//
	reg Alusrc[0:1];
	//-----tempt used control--//
	reg beq,bneq;
	reg control_mux;
	reg t1,t2;
	//-----instrction and pc---------//
	reg [31:0] Ins;
	reg [31:0] Ins_nxt;
	reg [4:0] Rd[0:3];
	reg [5:0] PC,PC_nxt,PC_ID,PC_ID_nxt; //8bit -2'b00(4byte)=6bit PC
	reg [5:0] PC_add4[0:3]; //fliflop
	reg [5:0] PC_plus4; //result after addtion of PC+4
	reg [5:0] PC_add4_nxt;
	//-----Hazard detection---//
	reg [1:0] pc_mux,Ins_mux;
	//-----register-----------//
	reg equal;
	reg [31:0] reg_w_data;
	wire [31:0] reg_data1,reg_data2;
	reg [31:0] rs1_0,rs2_0,rs1_Ex,rs2_Ex,rs2_Mem;
	reg [4:0] read1[0:1],read2[0:1];
	//-----ALU and cache D----------------//
	wire [31:0] alu_result_Ex;
	reg [31:0] alu_result_Mem,alu_result_WB;
	reg [31:0] cacheD_result_WB;
	//-----Imm Gen------------------------//
	reg [31:0] imm_result_0;
	reg [31:0] imm_result_Ex;
	//-----ALu control------------------//
	reg [3:0] alu_control_result_Ex,alu_control_result_reg;
	reg [31:0] alu_rs1,alu_rs2;
	//-----Forward Unit-----------------//
	reg [1:0] FwA_Ex,FwB_Ex,FwA_ID,FwB_ID;

	// testing
	wire [7:0] PC_prototype;
	assign PC_prototype = PC*4;

	//-----cache D combinational--------//
	always@(*) begin
		DCACHE_addr=alu_result_Mem[31:2];//since prcessor has to achieve one word(4 byte) from cache each time.
		DCACHE_wdata={rs2_Mem[7:0],rs2_Mem[15:8],rs2_Mem[23:16],rs2_Mem[31:24]};
		DCACHE_ren=MemRead[Mem];
		DCACHE_wen=MemWrite[Mem];
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
		.ctrl(alu_control_result_Ex),
		.x(alu_rs1),
		.y(alu_rs2),
		.result(alu_result_Ex)
	);
	always@(*) begin
		case(FwA_Ex)
			2'b01: alu_rs1=reg_w_data;
			2'b10: alu_rs1=alu_result_Mem;
			2'b00: alu_rs1=rs1_Ex;
			default: alu_rs1=rs1_Ex;
		endcase
		case(FwB_Ex)
			2'b01: alu_rs2=reg_w_data;
			2'b10: alu_rs2=alu_result_Mem;
			2'b00: alu_rs2=(Alusrc[Ex])? imm_result_Ex:rs2_Ex; 
			default: alu_rs2=(Alusrc[Ex])? imm_result_Ex:rs2_Ex;
		endcase
	end

	//-----Alu control combinational----//
	always@(*) begin
		alu_control_result_reg={(Ins[30]&Ins[4]&(Ins[12]|Ins[5])),Ins[14],(Ins[13]&Ins[4]),Ins[12]}; //Ins[30],Ins[14:12],Ins[5:4] 
	//out={(in[5]&in[0]&(in[2]|in[1])),in[4],(in[3]&in[0]),in[2]};
	end

	//-----imm gen combinational--------//
	always@(*) begin
    	t1=((!Ins[5])&(!Ins[3]))|Ins[2];
    	t2=((!Ins[6])|Ins[2])&(!Ins[3]);
        case({t1,t2})
            2'b00: imm_result_0={{20{Ins[31]}},Ins[7],Ins[30:25],Ins[11:8],1'b0};//beq,bneq
            2'b01: imm_result_0={{20{Ins[31]}},Ins[31:25],Ins[11:7]};//sw
            2'b10: imm_result_0={{12{Ins[31]}},Ins[19:12],Ins[20],Ins[30:21],1'b0};//jal
            2'b11: imm_result_0=(Ins[13:12]==2'b01)? {{27{Ins[25]}},Ins[24:20]}:{{20{Ins[31]}},Ins[31:20]};//jalr,lw, R type immediate, use funct 3to seperate slli,srai,srli 
        endcase
    end

	//-----regsiter combinational------//
	register_file register(
		.Clk(clk),
    	.rst_n(rst_n),
	    .WEN(RegWrite[WB]),
	    .RW(Rd[WB]),
	    .busW(reg_w_data),
	    .RX(read1[0]),
	    .RY(read2[0]),
	    .busX(reg_data1) ,
	    .busY(reg_data2)
	);
	always@(*) begin
		case(DataToReg[WB])
			2'b00: reg_w_data=alu_result_WB;
			2'b01: reg_w_data=cacheD_result_WB;
			2'b10: reg_w_data={24'd0,PC_add4[WB],2'd0};
			default: reg_w_data=32'hffffffff;
		endcase
		FwA_ID[1]=RegWrite[Ex]& (Rd[Ex]==read1[0]);
		FwA_ID[0]=RegWrite[Mem]& (Rd[Mem]==read1[0]);
		FwB_ID[1]=RegWrite[Ex]& (Rd[Ex]==read2[0]);
		FwB_ID[0]=RegWrite[Mem]& (Rd[Mem]==read2[0]);

		if (FwA_ID[0]) rs1_0=alu_result_Mem;
		else if ((RegWrite[WB])&&(Rd[WB]==read1[0])) rs1_0=reg_w_data;
		else rs1_0=reg_data1;
		if (FwB_ID[0]) rs2_0=alu_result_Mem;
		else if ((RegWrite[WB])&&(Rd[WB]==read2[0])) rs2_0=reg_w_data;
		else rs2_0=reg_data2;

		equal=(rs1_0[7:0]==rs2_0[7:0])? 1'b1:1'b0; //compare 8 bit for convenient
		read1[0]=Ins[19:15];
		read2[0]=Ins[24:20];
	end
	//-----Hazard Combinational--------//
	always@(*) begin
		if(FwA_ID[1]|FwB_ID[1]) begin
			if(MemRead[Ex]||beq||bneq) begin
				pc_mux=2'b01; //pc-->pc
				Ins_mux=2'b01; //hold IF
				control_mux=1;	//load use hazzard	
			end
			//--else: use other forward to compensate, no need stall---//
			else begin
				pc_mux=2'b00; //pc+4-->pc        
				Ins_mux=2'b00; //IF go to next address
				control_mux=0;
			end
		end
		else if(Jalr[0]) begin
			pc_mux=2'b11; //jalr_address-->pc
			Ins_mux=2'b10; //flush IF
			control_mux=0;
		end
		else if (Jal[0]) begin
			pc_mux=2'b10; //branch or jump address-->pc
			Ins_mux=2'b10; //flush IF
			control_mux=0;
		end
		else if ((beq&equal)||(bneq&(~equal))) begin
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
		flag={24'd0,PC_ID,2'd0};
		case(Ins_mux)
			2'b00: Ins_nxt={ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]};
			2'b01: Ins_nxt=Ins;
			2'b10: Ins_nxt=32'd0;
			default: Ins_nxt=32'd0;
		endcase
		PC_plus4=PC[5:0]+1'b1;  //reduce adder area
		Rd[0]=Ins[11:7];
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
				PC_nxt=PC_ID[5:0]+imm_result_0[7:2];// reduce adder area origin:PC_ID+imm_result_0; 
				PC_ID_nxt=6'b111111;
				PC_add4_nxt=6'b111111;
			end
			2'b11: begin //jalr
				PC_nxt=rs1_0[7:2]+imm_result_0[7:2];       //pre-calculate jalr ,reduce adder area origin:rs1_0+imm_result_0; 
				PC_ID_nxt=6'b111111;
				PC_add4_nxt=6'b111111;
			end
		endcase
	end
	//-----control combinational--//
	always@(*)begin //.in({Ins[12],Ins[6:0]}),
    	beq=({Ins[12],Ins[6],Ins[2]}==3'b010)? 1'b1:1'b0;
    	bneq=({Ins[12],Ins[6],Ins[2]}==3'b110)? 1'b1:1'b0;
        RegWrite[0]=(({Ins[5:4],Ins[2]}==3'b100)|(!Ins[11:7]))? 1'b0:1'b1; //Ins[11:7]=Rd[0];
        Jalr[0]=(Ins[3:2]==2'b01)? 1'b1:1'b0;
        Jal[0]=(Ins[3:2]==2'b11)? 1'b1:1'b0;
        MemRead[0]=({Ins[5:4],Ins[0]}==3'b001)? 1'b1:1'b0; //add in[0] to prevent when flush IF using 32'd0
        MemWrite[0]=(Ins[6:4]==3'b010)? 1'b1:1'b0;
        Alusrc[0]=({{Ins[6]|Ins[4]},Ins[5],Ins[2]}==3'b110)? 1'b0:1'b1;
        DataToReg[0]=(Ins[2]==1)? 2'b10:
        		  ({Ins[4],Ins[2]}==2'b00)? 2'b01:2'b00;
    end

	//------cache D sequential -------------//
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			cacheD_result_WB<=32'd0;
		end
		else begin
			if((~DCACHE_stall)&&(~ICACHE_stall)) cacheD_result_WB<={DCACHE_rdata[7:0],DCACHE_rdata[15:8],DCACHE_rdata[23:16],DCACHE_rdata[31:24]};
			else cacheD_result_WB<=cacheD_result_WB;
		end
	end
	//------register ,fwd sequential -----------//
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			rs1_Ex<=32'd0;
			rs2_Ex<=32'd0;
			rs2_Mem<=32'd0;
			read1[Ex]<=5'd0;
			read2[Ex]<=5'd0;
			FwA_Ex<=1'b0;
			FwB_Ex<=1'b0;
		end
		else begin
			if((~DCACHE_stall)&&(~ICACHE_stall)) begin
				rs1_Ex<=rs1_0;
				rs2_Ex<=rs2_0;
				rs2_Mem<=rs2_Ex;
				read1[Ex]<=read1[0];
				read2[Ex]<=read2[0];
				FwA_Ex<=FwA_ID;
				FwB_Ex<=FwB_ID;
			end
			else begin
				rs1_Ex<=rs1_Ex;
				rs2_Ex<=rs2_Ex;
				rs2_Mem<=rs2_Mem;
				read1[Ex]<=read1[Ex];
				read2[Ex]<=read2[Ex];
				FwA_Ex<=FwA_Ex;
				FwB_Ex<=FwB_Ex;
			end	
		end
	end
	//------alu control sequential---------//
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			alu_control_result_Ex<=4'd0;
		end
		else begin
			if((~DCACHE_stall)&&(~ICACHE_stall)) alu_control_result_Ex<=alu_control_result_reg;
			else alu_control_result_Ex<=alu_control_result_Ex;
		end
	end
	//------alu sequential-----------------//
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			alu_result_Mem<=32'd0;
			alu_result_WB<=32'd0;
		end
		else begin
			if((~DCACHE_stall)&&(~ICACHE_stall)) begin
				alu_result_Mem<=alu_result_Ex;
				alu_result_WB<=alu_result_Mem;
			end
			else begin
				alu_result_Mem<=alu_result_Mem;
				alu_result_WB<=alu_result_WB;
			end
			
		end
	end
	//------imm sequential-----------------//
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			imm_result_Ex<=32'd0;
		end
		else begin
			if((~DCACHE_stall)&&(~ICACHE_stall)) imm_result_Ex<=imm_result_0;
			else imm_result_Ex<=imm_result_Ex;
		end
	end
	//-----Instruction and pc sequential--//
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			Ins<=32'h00_00_00_00;
			for(i=1;i<=3;i=i+1) begin
				Rd[i]<=5'd0;
				PC_add4[i]<=6'd0;
			end
			PC_add4[0]<=6'd0;
			PC_ID<=6'd0;
			PC<=6'd0;

		end
		else begin
			if((~DCACHE_stall)&&(~ICACHE_stall)) begin
				Ins<=Ins_nxt;
				for(i=3;i>0;i=i-1) begin
					Rd[i]<=Rd[i-1];
					PC_add4[i]<=PC_add4[i-1];
				end
				PC_add4[0]<=PC_add4_nxt;
				PC_ID<=PC_ID_nxt;
				PC<=PC_nxt;
			end
			else begin
				Ins<=Ins;
				for(i=3;i>0;i=i-1) begin
					Rd[i]<=Rd[i];
					PC_add4[i]<=PC_add4[i];
				end
				PC_add4[0]<=PC_add4[0];
				PC_ID<=PC_ID;
				PC<=PC;
			end
		end
	end
	//-----control sequential------//
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			for(i=1;i<=3;i=i+1) begin
				DataToReg[i]<=2'b00;
				Jalr[i]<=1'b0;
				Jal[i]<=1'b0;
				RegWrite[i]<=1'b0;
			end
			for(i=1;i<=2;i=i+1) begin
				MemRead[i]<=1'b0;
				MemWrite[i]<=1'b0;
			end
			Alusrc[1]<=1'b0;
		end
		else begin
			if((~DCACHE_stall)&&(~ICACHE_stall)) begin
				for(i=3;i>=1;i=i-1) begin
					DataToReg[i]<=DataToReg[i-1];
					Jalr[i]<=Jalr[i-1];
					Jal[i]<=Jal[i-1];
				end
				for(i=3;i>=2;i=i-1) begin
					RegWrite[i]<=RegWrite[i-1];
				end
				MemRead[2]<=MemRead[1];
				MemWrite[2]<=MemWrite[1];
				Alusrc[1]<=Alusrc[0];
				if(control_mux) begin //flush control
					RegWrite[1]<=1'b0;
					MemRead[1]<=1'b0;
					MemWrite[1]<=1'b0;
				end
				else begin
					RegWrite[1]<=RegWrite[0];
					MemRead[1]<=MemRead[0];
					MemWrite[1]<=MemWrite[0];
				end
				
			end
			else begin
				for(i=1;i<=3;i=i+1) begin
					DataToReg[i]<=DataToReg[i];
					Jalr[i]<=Jalr[i];
					Jal[i]<=Jal[i];
					RegWrite[i]<=RegWrite[i];
				end
				for(i=1;i<=2;i=i+1) begin
					MemRead[i]<=MemRead[i];
					MemWrite[i]<=MemWrite[i];
				end
				Alusrc[1]<=Alusrc[1];
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