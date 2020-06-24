// Top module of your design, you cannot modify this module!!
module CHIP (		clk,
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
			.ICACHE_ren     (ICACHE_ren)    ,
			.ICACHE_wen     (ICACHE_wen)    ,
			.ICACHE_addr    (ICACHE_addr)   ,
			.ICACHE_wdata   (ICACHE_wdata)  ,
			.ICACHE_stall   (ICACHE_stall)  ,
			.ICACHE_rdata   (ICACHE_rdata)  ,
	//----------D cache interface-------
			.DCACHE_ren     (DCACHE_ren)    ,
			.DCACHE_wen     (DCACHE_wen)    ,
			.DCACHE_addr    (DCACHE_addr)   ,
			.DCACHE_wdata   (DCACHE_wdata)  ,
			.DCACHE_stall   (DCACHE_stall)  ,
			.DCACHE_rdata   (DCACHE_rdata)
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
	wire WB_regwrite;
	wire [1:0] WB_DataToReg;
	wire WB_Jalr;
	wire WB_Jal;
	reg RegWrite[0:3]; 
	reg [1:0] DataToReg[0:3]; 
	reg Jalr[0:3]; 
	reg Jal[0:3];
	//-----Mem control--------//
	wire Mem_memread;
	wire Mem_memwrite;
	reg MemRead[0:2];
	reg MemWrite[0:2];
	//-----EX control---------//
	wire Ex_Alusrc;
	reg Alusrc[0:1];
	//-----tempt used control--//
	wire beq,bneq;
	wire control_mux;
	//-----instrction and pc---------//
	reg [31:0] Ins;
	reg [31:0] Ins_nxt;
	reg [4:0] Rd[0:3];
	reg [31:0] PC,PC_nxt,PC_ID,PC_ID_nxt;
	reg [31:0] PC_add4[0:3]; //fliflop
	reg [31:0] PC_plus4; //result after addtion of PC+4
	reg [31:0] PC_add4_nxt;
	reg [15:0] compressed_Ins;
	//-----Hazard detection---//
	wire [1:0] pc_mux,Ins_mux;
	//-----register-----------//
	reg equal;
	reg [31:0] reg_w_data;
	wire [31:0] reg_data1,reg_data2;
	reg [31:0] rs1_0,rs2_0,rs1_Ex,rs2_Ex,rs2_Ex_forward,rs2_Mem;
	reg [4:0] read1[0:1],read2[0:1];
	//-----ALU and cache D----------------//
	wire [31:0] alu_result_Ex;
	reg [31:0] alu_result_Mem,alu_result_WB;
	reg [31:0] cacheD_result_WB;
	//-----Imm Gen------------------------//
	wire [31:0] imm_result_0;
	reg [31:0] imm_result_Ex;
	//-----ALu control------------------//
	reg [5:0] alu_control_Ex;
	wire [3:0] alu_control_result_Ex;
	reg [31:0] alu_rs1,alu_rs2;
	//-----Forward Unit-----------------//
	wire [1:0] FwA,FwB;


	//-----cache D combinational--------//
	always@(*) begin
		DCACHE_addr={alu_result_Mem[30:2],1'b0};//since prcessor has to achieve one word(4 byte) from cache each time.
		DCACHE_wdata={rs2_Mem[7:0],rs2_Mem[15:8],rs2_Mem[23:16],rs2_Mem[31:24]};
		DCACHE_ren=MemRead[Mem];
		DCACHE_wen=MemWrite[Mem];
	end
	//-----cache I combinational-------//
	always@(*) begin
		ICACHE_addr=PC[30:1];//since prcessor has to achieve one word(4 byte) from cache each time.
		//ICACHE_wdata=32'hffffffff;
		ICACHE_ren=1'b1;
		ICACHE_wen=1'b0;
	end
	//-----Forward Unit Combinational---//
	Forward_Unit forward(
		.regwrite_1(RegWrite[Mem]),
		.rd_1(Rd[Mem]),
		.regwrite_2(RegWrite[WB]),
		.rd_2(Rd[WB]),
		.rs1(read1[Ex]),
		.rs2(read2[Ex]),
		.muxA(FwA),
		.muxB(FwB)
	);
	
	//-----Alu combinational------------//
	Alu alu(
		.ctrl(alu_control_result_Ex),
		.x(alu_rs1),
		.y(alu_rs2),
		.result(alu_result_Ex)
	);
	always@(*) begin
		case(FwB)
			2'b01: rs2_Ex_forward=reg_w_data;
			2'b10: rs2_Ex_forward=alu_result_Mem;
			2'b00: rs2_Ex_forward=rs2_Ex;
			default: rs2_Ex_forward=32'hffffffff;
		endcase
		case(FwA)
			2'b01: alu_rs1=reg_w_data;
			2'b10: alu_rs1=alu_result_Mem;
			2'b00: alu_rs1=rs1_Ex;
			default: alu_rs1=32'hffffffff;
		endcase
		alu_rs2=(Alusrc[Ex])? imm_result_Ex:rs2_Ex_forward;
	end

	//-----Alu control combinational----//
	AluCtrol_Unit Alu_control(
		.in(alu_control_Ex),
		.out(alu_control_result_Ex)
	);
	//-----imm gen combinational--------//
	ImmGen imm(
		.in(Ins[31:0]),
		.out(imm_result_0)
	);
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
			2'b10: reg_w_data=PC_add4[WB];
			default: reg_w_data=32'hffffffff;
		endcase
		if ((RegWrite[Mem])&&(Rd[Mem]==read1[0])) rs1_0=alu_result_Mem;
		else if ((RegWrite[WB])&&(Rd[WB]==read1[0])) rs1_0=reg_w_data;
		else rs1_0=reg_data1;
		if ((RegWrite[Mem])&&(Rd[Mem]==read2[0])) rs2_0=alu_result_Mem;
		else if ((RegWrite[WB])&&(Rd[WB]==read2[0])) rs2_0=reg_w_data;
		else rs2_0=reg_data2;
		equal=(rs1_0==rs2_0)? 1'b1:1'b0;
		read1[0]=Ins[19:15];
		read2[0]=Ins[24:20];
	end
	//-----Hazard Combinational--------//
	Hazard_dection_Unit Hazard(
		.rs1(read1[0]),
		.rs2(read2[0]),
		.equal(equal),
		.rd_Ex(Rd[Ex]),
		.reg_write_Ex(RegWrite[Ex]),
		.jal(Jal[0]),
		.jalr(Jalr[0]),
		.beq(beq),
		.bneq(bneq),
		.mem_read_(MemRead[Ex]),
		.pc_mux(pc_mux),
		.IF_mux(Ins_mux),
		.control_mux(control_mux)
	);
	//-----Instruction Combinational---//
	always @(*) begin
		compressed_Ins = {ICACHE_rdata[23:16],ICACHE_rdata[31:24]};
		case(Ins_mux)
			2'b00: Ins_nxt= (compressed_Ins[1:0] == 2'b11)? //not compressed
								{ICACHE_rdata[7:0],ICACHE_rdata[15:8],ICACHE_rdata[23:16],ICACHE_rdata[31:24]} : 
							(compressed_Ins[1:0] == 2'b00 && compressed_Ins[15:13] == 3'b010)? //c.lw
								{5'b0, compressed_Ins[5], compressed_Ins[12:10], compressed_Ins[6], 2'b0, 2'b01, compressed_Ins[9:7], 3'b010, 2'b01, compressed_Ins[4:2], 7'b0000011} :
							(compressed_Ins[1:0] == 2'b00 && compressed_Ins[15:13] == 3'b110)? //c.sw
								{5'b0, compressed_Ins[5], compressed_Ins[12], 2'b01, compressed_Ins[4:2], 2'b01, compressed_Ins[9:7], 3'b010, compressed_Ins[11:10], compressed_Ins[6], 2'b0, 7'b0100011} :
							(compressed_Ins[1:0] == 2'b01 && compressed_Ins[15:13] == 3'b000)? //c.addi, c.nop
								{{7{compressed_Ins[12]}}, compressed_Ins[6:2], compressed_Ins[11:7], 3'b000, compressed_Ins[11:7], 7'b0010011} :
							(compressed_Ins[1:0] == 2'b01 && compressed_Ins[15:13] == 3'b001)? //c.jal
								{compressed_Ins[12], compressed_Ins[8], compressed_Ins[10:9], compressed_Ins[6], compressed_Ins[7], compressed_Ins[2], compressed_Ins[11], compressed_Ins[5:3], {9{compressed_Ins[12]}}, 5'b1, 7'b1101111} :
							(compressed_Ins[1:0] == 2'b01 && compressed_Ins[15:13] == 3'b100 && compressed_Ins[11:10] == 2'b10)? //c.andi
								{{7{compressed_Ins[12]}}, compressed_Ins[6:2], 2'b01, compressed_Ins[9:7], 3'b111, 2'b01, compressed_Ins[9:7], 7'b0010011} :
							(compressed_Ins[1:0] == 2'b01 && compressed_Ins[15:13] == 3'b100 && compressed_Ins[11:10] == 2'b00)? //c.srli
								{7'b0000000, compressed_Ins[6:2], 2'b01, compressed_Ins[9:7], 3'b101, 2'b01, compressed_Ins[9:7], 7'b0010011} :
							(compressed_Ins[1:0] == 2'b01 && compressed_Ins[15:13] == 3'b100 && compressed_Ins[11:10] == 2'b01)? //c.srai
								{7'b0100000, compressed_Ins[6:2], 2'b01, compressed_Ins[9:7], 3'b101, 2'b01, compressed_Ins[9:7], 7'b0010011} :
							(compressed_Ins[1:0] == 2'b01 && compressed_Ins[15:13] == 3'b101)? //c.j
								{compressed_Ins[12], compressed_Ins[8], compressed_Ins[10:9], compressed_Ins[6], compressed_Ins[7], compressed_Ins[2], compressed_Ins[11], compressed_Ins[5:3], {9{compressed_Ins[12]}}, 5'b0, 7'b1101111} :
							(compressed_Ins[1:0] == 2'b01 && compressed_Ins[15:13] == 3'b110)? //c.beqz
								{{4{compressed_Ins[12]}}, compressed_Ins[6:5], compressed_Ins[2], 5'b0, 2'b01, compressed_Ins[9:7], 3'b000, compressed_Ins[11:10], compressed_Ins[4:3], compressed_Ins[12], 7'b1100011} :
							(compressed_Ins[1:0] == 2'b01 && compressed_Ins[15:13] == 3'b111)? //c.bnez
								{{4{compressed_Ins[12]}}, compressed_Ins[6:5], compressed_Ins[2], 5'b0, 2'b01, compressed_Ins[9:7], 3'b001, compressed_Ins[11:10], compressed_Ins[4:3], compressed_Ins[12], 7'b1100011} :
							(compressed_Ins[1:0] == 2'b10 && compressed_Ins[15:13] == 3'b000)? //c.slli
								{7'b0, compressed_Ins[6:2], compressed_Ins[11:7], 3'b001, compressed_Ins[11:7], 7'b0010011} :
							(compressed_Ins[1:0] == 2'b10 && compressed_Ins[15:13] == 3'b100 && compressed_Ins[6:2] == 5'b00000 && compressed_Ins[12] == 1'b0)? //c.jr
								{12'b0, compressed_Ins[11:7], 3'b0, 5'b00000, 7'b1100111} :
							(compressed_Ins[1:0] == 2'b10 && compressed_Ins[15:13] == 3'b100 && compressed_Ins[6:2] == 5'b00000 && compressed_Ins[12] == 1'b1)? //c.jalr
								{12'b0, compressed_Ins[11:7], 3'b0, 5'b00001, 7'b1100111} :
							(compressed_Ins[1:0] == 2'b10 && compressed_Ins[15:13] == 3'b100 && compressed_Ins[12] == 1'b0)? //c.mv
								{7'b0, compressed_Ins[6:2], 5'b0, 3'b0, compressed_Ins[11:7], 7'b0110011} :
							(compressed_Ins[1:0] == 2'b10 && compressed_Ins[15:13] == 3'b100 && compressed_Ins[12] == 1'b1)? //c.add
								{7'b0, compressed_Ins[6:2], compressed_Ins[11:7], 3'b0, compressed_Ins[11:7], 7'b0110011} :	{25'b0, 7'b0010011};
			2'b01: Ins_nxt=Ins;
			2'b10: Ins_nxt=32'd0;
			default: Ins_nxt=32'd0;
		endcase
		PC_plus4 = (compressed_Ins[1:0] == 2'b11)? {24'd0,(PC[7:0]+8'd4)} : {24'd0,(PC[7:0]+8'd2)};  //reduce adder area //compressed:pc+2;else:pc+4
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
				PC_nxt={24'd0,(PC_ID[7:0]+imm_result_0[7:0])};// reduce adder area origin:PC_ID+imm_result_0; 
				PC_ID_nxt=32'hffffffff;
				PC_add4_nxt=32'hffffffff;
			end
			2'b11: begin //jalr
				PC_nxt={24'd0,(rs1_0[7:0]+imm_result_0[7:0])};       //pre-calculate jalr ,reduce adder area origin:rs1_0+imm_result_0; 
				PC_ID_nxt=32'hffffffff;
				PC_add4_nxt=32'hffffffff;
			end
		endcase
	end
	//-----control combinational--//
	Control_Unit Control(
		.in({Ins[12],Ins[6:0]}),
		.rd(Ins[11:7]),
		.RegWrite(WB_regwrite),
		.DataToReg(WB_DataToReg),
		.jal(WB_Jal),
		.jalr(WB_Jalr),
		.MemRead(Mem_memread),
		.MemWrite(Mem_memwrite),
		.Alusrc(Ex_Alusrc),
		.beq(beq),
		.bneq(bneq)
	);
	always@(*) begin
		RegWrite[0]=WB_regwrite;
		MemRead[0]=Mem_memread;
		MemWrite[0]=Mem_memwrite;
		DataToReg[0]=WB_DataToReg;
		Jal[0]=WB_Jal;
		Jalr[0]=WB_Jalr;
		Alusrc[0]=Ex_Alusrc;
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
	//------register sequential -----------//
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			rs1_Ex<=32'd0;
			rs2_Ex<=32'd0;
			rs2_Mem<=32'd0;
			read1[Ex]<=5'd0;
			read2[Ex]<=5'd0;
		end
		else begin
			if((~DCACHE_stall)&&(~ICACHE_stall)) begin
				rs1_Ex<=rs1_0;
				rs2_Ex<=rs2_0;
				rs2_Mem<=rs2_Ex_forward;
				read1[Ex]<=read1[0];
				read2[Ex]<=read2[0];
			end
			else begin
				rs1_Ex<=rs1_Ex;
				rs2_Ex<=rs2_Ex;
				rs2_Mem<=rs2_Mem;
				read1[Ex]<=read1[Ex];
				read2[Ex]<=read2[Ex];
			end	
		end
	end
	//------alu control sequential---------//
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			alu_control_Ex<=6'd0;
		end
		else begin
			if((~DCACHE_stall)&&(~ICACHE_stall)) alu_control_Ex<={Ins[30],Ins[14:12],Ins[5:4]};
			else alu_control_Ex<=alu_control_Ex;
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
				PC_add4[i]<=32'd0;
			end
			PC_add4[0]<=32'd0;
			PC_ID<=32'd0;
			PC<=32'd0;

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

module Hazard_dection_Unit(rs1,rs2,equal,rd_Ex,reg_write_Ex,jal,jalr,beq,bneq,mem_read_,pc_mux,IF_mux,control_mux); //variable ending with '_' means next stage value
	input [4:0] rs1,rs2,rd_Ex;
	input equal,jal,jalr,beq,bneq,mem_read_,reg_write_Ex;
	output reg [1:0] pc_mux,IF_mux;
	output reg control_mux;
	always@(*) begin
		if(((rd_Ex==rs1)||(rd_Ex==rs2))&reg_write_Ex) begin
			if(mem_read_||beq||bneq||jalr) begin
				pc_mux=2'b01; //pc-->pc
				IF_mux=2'b01; //hold IF
				control_mux=1;	//load use hazzard	
			end
			//--else: use other forward to compensate, no need stall---//
			else begin
				pc_mux=2'b00; //pc+4-->pc        
				IF_mux=2'b00; //IF go to next address
				control_mux=0;
			end
		end
		else if(jalr) begin
			pc_mux=2'b11; //jalr_address-->pc
			IF_mux=2'b10; //flush IF
			control_mux=0;
		end
		else if (jal) begin
			pc_mux=2'b10; //branch or jump address-->pc
			IF_mux=2'b10; //flush IF
			control_mux=0;
		end
		else if ((beq&equal)||(bneq&(~equal))) begin
			pc_mux=2'b10; //branch or jump address-->pc
			IF_mux=2'b10; //flush IF
			control_mux=0;
		end
		else begin
			pc_mux=2'b00; //pc+4-->pc
			IF_mux=2'b00; //IF go to next address
			control_mux=0;
		end
	end
endmodule

module Forward_Unit(regwrite_1,rd_1,regwrite_2,rd_2,rs1,rs2,muxA,muxB); //EX|1(MEM)|2(WB)
	input regwrite_1,regwrite_2;
	input [4:0] rs1,rs2,rd_1,rd_2;
	output reg [1:0] muxA,muxB;
	always@(*) begin
		if((regwrite_1) && (rd_1==rs1)) muxA=2'b10;
		else if((regwrite_2)  && (rd_2==rs1)) muxA=2'b01;
		else muxA=2'b00;
		if((regwrite_1)  && (rd_1==rs2)) muxB=2'b10;
		else if((regwrite_2)  && (rd_2==rs2)) muxB=2'b01;
		else muxB=2'b00;
	end 
endmodule

module Control_Unit(in,rd,RegWrite,DataToReg,jal,jalr,MemRead,MemWrite,Alusrc,beq,bneq);//
    input  [7:0] in;
    input [4:0] rd;
    output reg RegWrite,jal,jalr,MemRead,MemWrite,Alusrc,beq,bneq;
    output reg [1:0] DataToReg;
    always@(*)begin
    	beq=({in[7:6],in[2]}==3'b010)? 1'b1:1'b0;
    	bneq=({in[7:6],in[2]}==3'b110)? 1'b1:1'b0;
        RegWrite=(({in[5:4],in[2]}==3'b100)|(!rd))? 1'b0:1'b1;
        DataToReg=(in[2]==1)? 2'b10:
        		  ({in[4],in[2]}==2'b00)? 2'b01:2'b00;
        jalr=(in[3:2]==2'b01)? 1'b1:1'b0;
        jal=(in[3:2]==2'b11)? 1'b1:1'b0;
        MemRead=({in[5:4],in[0]}==3'b001)? 1'b1:1'b0; //add in[0] to prevent when flush IF using 32'd0
        MemWrite=(in[6:4]==3'b010)? 1'b1:1'b0;
        Alusrc=({{in[6]|in[4]},in[5],in[2]}==3'b110)? 1'b0:1'b1;
    end
endmodule

module ImmGen(in,out);
    input [31:0] in;
    output reg [31:0] out;
    reg t1,t2,garbage;
    always@(*) begin
    	t1=((!in[5])&(!in[3]))|in[2];
    	t2=((!in[6])|in[2])&(!in[3]);
        case({t1,t2})
            2'b00: out={{20{in[31]}},in[7],in[30:25],in[11:8],1'b0};//beq,bneq
            2'b01: out={{20{in[31]}},in[31:25],in[11:7]};//sw
            2'b10: out={{12{in[31]}},in[19:12],in[20],in[30:21],1'b0};//jal
            2'b11: out=(in[13:12]==2'b01)? {{27{in[25]}},in[24:20]}:{{20{in[31]}},in[31:20]};//jalr,lw, R type immediate, use funct 3to seperate slli,srai,srli 
            default:out=31'd0;
        endcase
    end
endmodule

module AluCtrol_Unit(in,out); //in=IF[30|14:12|4]
	input [5:0] in;
	output reg [3:0] out;
	reg tempt;
	always@(*) begin
		out={(in[5]&in[0]&(in[2]|in[1])),in[4],(in[3]&in[0]),in[2]};
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
			default: result=32'hffff1111;
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
    localparam state_ready  = 2'b00;//initial state
    localparam state_update = 2'b01;//update memory state
    localparam state_read   = 2'b10;//read data from memory state
	localparam state_read_2 = 2'b11; //read next block's data (for cross boundary instruction)
    reg [25:0] cache_tag [0:3][0:1];//tag width is differnt from dm!!!
    reg [31:0] cache_data [0:3][0:1][0:3]; //4block,each has 2 tag, each tag has 4 words
    reg [1:0] state;
    reg [1:0] miss; //00:no miss 01:regular miss 10: initial miss(i.e. read memory initially) 11:cross_boundary miss
    reg [1:0] target;//which block is reading/writing
    reg  tag_region[0:3];//which tag (left/right) is using in each block
    reg [1:0] word_region;//which word(1~4) in the target block is reading/writing 
	reg [1:0] nxt_word; // = word_region + 1
    reg current_tag;//=tag_region[target] i.e. to simplify the code
	reg nxt_tag; //next block's tag (for cross boundary instruction)
	reg [26:0] nxt_addr; //next block's address
    //reg desire_tag; //signal for the tag_region to know which tag is the older one 

//==== combinational circuit ==============================
integer i;
always@(*) begin
    word_region = proc_addr[2:1];
    target 		= proc_addr[4:3];
	nxt_word	= proc_addr[2:1] + 1;
	nxt_addr 	= proc_addr[29:3] + 1;
	nxt_tag		= 0;
    if (cache_tag[target][0][3:0] == proc_addr[8:5]) begin //hit left//compare only 4 bit for convenient
		if (proc_addr[2:0] == 3'b111) begin
			if (cache_tag[nxt_addr[1:0]][0][3:0] == nxt_addr[5:2]) begin
				miss = 2'b00;
				proc_stall = 1'b0;
				current_tag = 0;
				nxt_tag = 0;
			end
			else if (cache_tag[nxt_addr[1:0]][1][3:0] == nxt_addr[5:2]) begin
				miss = 2'b00;
				proc_stall = 1'b0;
				current_tag = 0;
				nxt_tag = 1;
			end
			else begin
				miss = 2'b11;
				proc_stall = 1'b1;
				current_tag = 0;
				nxt_tag = tag_region[nxt_addr[1:0]];
			end
		end
		else begin
			miss = 2'b00;
			proc_stall = 1'b0;
			current_tag = 0;
			nxt_tag = tag_region[nxt_addr[1:0]];
			//desire_tag=0;
		end
    end
    else if (cache_tag[target][1][3:0] == proc_addr[8:5]) begin //hit right ////compare only 4 bit for convenient
		if (proc_addr[2:0] == 3'b111) begin
			if (cache_tag[nxt_addr[1:0]][0][3:0] == nxt_addr[5:2]) begin
				miss = 2'b00;
				proc_stall = 1'b0;
				current_tag = 1;
				nxt_tag = 0;
			end
			else if (cache_tag[nxt_addr[1:0]][1][3:0] == nxt_addr[5:2]) begin
				miss = 2'b00;
				proc_stall = 1'b0;
				current_tag = 1;
				nxt_tag = 1;
			end
			else begin
				miss = 2'b11;
				proc_stall = 1'b1;
				current_tag = 1;
				nxt_tag = tag_region[nxt_addr[1:0]];
			end
		end
		else begin
			miss=2'b00;
			proc_stall=1'b0;
			current_tag=1;
			nxt_tag = tag_region[nxt_addr[1:0]];
			//desire_tag=1;
		end
    end
    else if (cache_tag[target][0][25:24]==2'b11) begin //left tag is still in reset state //initail miss
        miss=2'b10;
        if((proc_read||proc_write)) begin
            proc_stall = 1'b1;
        end
        else begin
            proc_stall=1'b0;
        end
        current_tag=0;
        //desire_tag=0;
    end
    else if (cache_tag[target][1][25:24]==2'b11) begin//left tag is still in reset state //initail miss
        miss=2'b10;
        if((proc_read||proc_write)) begin
            proc_stall=1'b1;
        end
        else begin
            proc_stall=1'b0;
        end
        current_tag=1;
        //desire_tag=1;
    end
    else begin //regular miss
        miss=2'b01;
        if((proc_read||proc_write)) begin
            proc_stall=1'b1;
        end
        else begin
            proc_stall=1'b0;
        end
        current_tag = tag_region[target];
		nxt_tag = tag_region[nxt_addr[1:0]];
        //desire_tag=1;//no use
    end

    case(state)
        state_ready: begin
            proc_rdata	= (proc_read)? 
						  (proc_addr[0] == 1'b1) ? 
						  (proc_addr[2:0] == 3'b111)? {cache_data[target][current_tag][word_region][15:0], cache_data[nxt_addr[1:0]][nxt_tag][0][31:16]} :
						  							  {cache_data[target][current_tag][word_region][15:0], cache_data[target][current_tag][nxt_word][31:16]} :
													  cache_data[target][current_tag][word_region] :
													  32'hffffffff;
            mem_addr	= 28'd0;
            mem_wdata	= 128'd0;
            mem_write	= 0;
            mem_read	= 0;
        end
        state_update: begin
            proc_rdata	= 32'hffffffff;
            mem_addr	= {cache_tag[target][current_tag],target};
            mem_wdata	= {cache_data[target][current_tag][3],cache_data[target][current_tag][2],cache_data[target][current_tag][1],cache_data[target][current_tag][0]};
            mem_write	= 1;
            mem_read	= 0;
        end
        state_read: begin
            proc_rdata	= 32'hffffffff;
            mem_addr	= {1'b0, proc_addr[29:3]};
            mem_wdata	= 128'd0;
            mem_write	= 0;
            mem_read	= 1;
        end
		state_read_2: begin
			proc_rdata 	= 32'hffffffff;
			mem_addr   	= {1'b0, nxt_addr};
			mem_wdata	= 128'd0;
			mem_write	= 0;
			mem_read	= 1;
		end
        default: begin
            proc_rdata	= 32'hffffffff;
            mem_addr	= 28'd0;
            mem_wdata	= 128'd0;
            mem_write	= 0;
            mem_read	= 0;
        end
    endcase
end

//==== sequential circuit =================================
always@( posedge clk ) begin
    if( proc_reset ) begin
        state<=state_ready;
        for(i=0;i<4;i=i+1) begin
            cache_tag[i][0]<=26'b11111111111111111111111111;
            cache_tag[i][1]<=26'b11111111111111111111111111;
            tag_region[i]<=0;
        end
    end
    else begin
        case(state)
            state_ready: begin
                if (miss == 2'b10 && (proc_read||proc_write)) begin //initail miss
                    tag_region[target] <= current_tag;
                    state <= state_read;
                end
                else if (miss == 2'b01 && (proc_read||proc_write)) begin //regular miss
                    tag_region[target] <= ~tag_region[target];
                    state <= state_update;
                end
				else if (miss == 2'b11 && (proc_read||proc_write)) begin //current hit but next addr. miss (for cross boundary instructions)
					tag_region[target] <= current_tag;
					state <= state_read_2;
				end
                else begin
                    tag_region[target] <= current_tag;
                    state <= state_ready;
                    if(proc_write == 1) begin
                        cache_data[target][current_tag][word_region] <= proc_wdata;
                    end
                    else begin
                        cache_data[target][current_tag][word_region] <= cache_data[target][current_tag][word_region];
                    end
                end
            end
            state_update: begin
                tag_region[target] <= tag_region[target];
                if(mem_ready == 0) begin
                    state <= state_update;
                end
                else begin
                    state <= state_read;
                end
            end
            state_read: begin
                if(mem_ready == 0) begin
                    state <= state_read;
                    tag_region[target] <= tag_region[target];
                end
                else begin
					if((proc_addr[2:0] == 3'b111) && (cache_tag[nxt_addr[1:0]][0][3:0] != nxt_addr[5:2]) && (cache_tag[nxt_addr[1:0]][1][3:0] != nxt_addr[5:2])) begin
						state <= state_read_2;
						tag_region[target] <= tag_region[target];
						cache_data[target][current_tag][3] <= cache_data[target][current_tag][3];
						cache_data[target][current_tag][2] <= cache_data[target][current_tag][2];
						cache_data[target][current_tag][1] <= cache_data[target][current_tag][1];
						cache_data[target][current_tag][0] <= cache_data[target][current_tag][0];
						cache_tag [target][current_tag]	   <= cache_tag [target][current_tag];
					end
					else begin
						state <= state_ready;
						tag_region[target] <= tag_region[target];
						cache_data[target][current_tag][3] <= mem_rdata[127:96];
						cache_data[target][current_tag][2] <= mem_rdata[95:64];
						cache_data[target][current_tag][1] <= mem_rdata[63:32];
						cache_data[target][current_tag][0] <= mem_rdata[31:0];
						cache_tag [target][current_tag]	   <= {1'b0, proc_addr[29:5]};
					end
                end
            end
			state_read_2: begin
				if (mem_ready == 0)begin
					state <= state_read_2;
				end
				else begin
					state <= state_ready;
					cache_data[nxt_addr[1:0]][tag_region[nxt_addr[1:0]]][3] <= mem_rdata[127:96];
					cache_data[nxt_addr[1:0]][tag_region[nxt_addr[1:0]]][2] <= mem_rdata[95:64];
					cache_data[nxt_addr[1:0]][tag_region[nxt_addr[1:0]]][1] <= mem_rdata[63:32];
					cache_data[nxt_addr[1:0]][tag_region[nxt_addr[1:0]]][0] <= mem_rdata[31:0];
					cache_tag [nxt_addr[1:0]][tag_region[nxt_addr[1:0]]]    <= {1'b0, nxt_addr[26:2]};
				end
			end
            default: begin //regular
            	tag_region[target] <= current_tag;
                state <= state_read;
            end
        endcase
    end
end

endmodule
