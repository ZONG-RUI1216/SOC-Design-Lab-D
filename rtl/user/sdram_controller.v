module sdram_controller (
        input   clk,
        input   rst,

        // these signals go directly to the IO pins
        // output  sdram_clk,
        output  sdram_cle,
        output  sdram_cs,
        output  sdram_cas,
        output  sdram_ras,
        output  sdram_we,
        output  sdram_dqm,
        output  [1:0]  sdram_ba,
        output  [12:0] sdram_a,
        // Jiin: split dq into dqi (input) dqo (output)
        // inout [7:0] sdram_dq,
        input   [31:0] sdram_dqi,
        output  [31:0] sdram_dqo,

        // User interface
        // Note: we want to remap addr (see below)
        // input [22:0] addr,       // address to read/write
        input   [22:0] user_addr,   // the address will be remap to addr later
        
        input   rw,                 // 1 = write, 0 = read
        input   [31:0] data_in,     // data from a read
        output  [31:0] data_out,    // data for a write
        output  busy,               // controller is busy when high
        input   in_valid,           // pulse high to initiate a read/write
        output  out_valid           // pulses high when data from read is valid
    );

    // Jiin: SDRAM Timing  3-3-3, i.e. CASL=3, PRE=3, ACT=3
    localparam tCASL            = 13'd2;       // 3T actually
    localparam tPRE             = 13'd2;       // 3T PRECHARGE
    localparam tACT             = 13'd2;       // 3T ACTIVATE
    localparam tREF             = 13'd6;       // 7T 
	localparam tIns				= 13'd6;
	localparam tdat				= 13'd2;
    localparam tRef_Counter     = 10'd15;     // Refresh cycle 750T

    // MA Map
    // BA (Bank Address) - 9:8
    // RA (Row Address)  - 22:10
    // CA (Col Address)  - 2'b0, 1'b0, <7:0>, 2'b0
    `define BA      9:8
    `define RA      22:10
    `define CA      7:0

    // Address Remap
    //   - remap user address to addr to create more offpage/onpage cases
    // 
    wire [22:0] addr;
    wire [12:0] Mapped_RA;
    wire [1:0]  Mapped_BA;
    wire [7:0]  Mapped_CA;
    assign Mapped_RA = user_addr[22:10];
    assign Mapped_BA = user_addr[9:8];
    assign Mapped_CA = user_addr[7:0];
    assign addr = {Mapped_RA, Mapped_BA, Mapped_CA};

    // Commands for the SDRAM    CS RAS CAS WE
    localparam CMD_UNSELECTED    = 4'b1000; //8
    localparam CMD_NOP           = 4'b0111; //7
    localparam CMD_ACTIVE        = 4'b0011; //3
    localparam CMD_READ          = 4'b0101; //5
    localparam CMD_WRITE         = 4'b0100; //4
    localparam CMD_TERMINATE     = 4'b0110; //6
    localparam CMD_PRECHARGE     = 4'b0010; //2
    localparam CMD_REFRESH       = 4'b0001; //1
    localparam CMD_LOAD_MODE_REG = 4'b0000; //0
    
    localparam STATE_SIZE = 4;
    localparam INIT = 4'd0,
               WAIT = 4'd1,           // initial is replace by this
               PRECHARGE_INIT = 4'd2, // not use in this Lab
               REFRESH_INIT_1 = 4'd3, // not use in this Lab
               REFRESH_INIT_2 = 4'd4, // not use in this Lab
               LOAD_MODE_REG = 4'd5,  // not use in this Lab
               IDLE = 4'd6,
               REFRESH = 4'd7,
               ACTIVATE = 4'd8,
               READ = 4'd9,
               READ_RES = 4'd10,
               WRITE = 4'd11,
               PRECHARGE = 4'd12,
			   WAIT_READ = 4'd13;
    
    // registers for SDRAM signals
    reg cle_d, cle_q;  //constant 1
    reg dqm_q, dqm_d;  
    reg [3:0] cmd_d, cmd_q;
    reg [1:0] ba_d, ba_q;    // bank select registers?
    reg [12:0] a_d, a_q;     // bram address
    reg [31:0] dq_d, dq_q;   // write data to bram
    reg [31:0] dqi_d, dqi_q; // read data from bram
    reg dq_en_d, dq_en_q;

    // Output assignments
    assign sdram_cle = cle_q;
    assign sdram_cs = cmd_q[3];
    assign sdram_ras = cmd_q[2];
    assign sdram_cas = cmd_q[1];
    assign sdram_we = cmd_q[0];
    assign sdram_dqm = dqm_q;
    assign sdram_ba = ba_q;
    assign sdram_a = a_q;
    // assign sdram_dqi = dq_en_q ? dq_q : 8'hZZ; // only drive when dq_en_q is 1
    assign sdram_dqo = dq_en_q ? dq_q : 32'hZZZZZZZZ;

    reg [STATE_SIZE-1:0] state_d, state_q;
    reg [STATE_SIZE-1:0] next_state_d, next_state_q;  // store that when delay time is complete what is the next state

    reg [22:0] addr_d, addr_q;
    reg [31:0] data_d, data_q;
    reg out_valid_d, out_valid_q;

    reg [15:0] delay_ctr_d, delay_ctr_q;

    reg [9:0] refresh_ctr_d, refresh_ctr_q;
    reg refresh_flag_d, refresh_flag_q;

    reg ready_d, ready_q;
    reg saved_rw_d, saved_rw_q;
    reg [22:0] saved_addr_d, saved_addr_q;
    reg [31:0] saved_data_d, saved_data_q;

    reg rw_op_d, rw_op_q;

    reg [3:0] row_open_d, row_open_q;  // it will be 1, when ROW is activate
    reg [12:0] row_addr_d[3:0], row_addr_q[3:0];
	wire [12:0] row_addr_d0, row_addr_d1, row_addr_d2, row_addr_d3;
	assign row_addr_d0 = row_addr_q[0];
	assign row_addr_d1 = row_addr_q[1];
	assign row_addr_d2 = row_addr_q[2];
	assign row_addr_d3 = row_addr_q[3];
    reg [2:0] precharge_bank_d, precharge_bank_q; // when precharge_bank_q[2] == 1 mean all banks have to refresh
    integer i;

    assign data_out = data_q;
    assign busy = !ready_q;
    assign out_valid = out_valid_q;
    
	// prefetch buffer scheme
	wire [12:0] fetch_addrI0, fetch_addrI1, fetch_addrI2, fetch_addrI3, fetch_addrI4, fetch_addrI5, fetch_addrI6, fetch_addrI7;
	assign fetch_addrI0 = fetch_addrI[0];
	assign fetch_addrI1 = fetch_addrI[1];
	assign fetch_addrI2 = fetch_addrI[2];
	assign fetch_addrI3 = fetch_addrI[3];
	assign fetch_addrI4 = fetch_addrI[4];
	assign fetch_addrI5 = fetch_addrI[5];
	assign fetch_addrI6 = fetch_addrI[6];
	assign fetch_addrI7 = fetch_addrI[7];
	wire [31:0] fetch_bufI0, fetch_bufI1, fetch_bufI2, fetch_bufI3, fetch_bufI4, fetch_bufI5, fetch_bufI6, fetch_bufI7;
	assign fetch_bufI0 = fetch_bufI[0];
	assign fetch_bufI1 = fetch_bufI[1];
	assign fetch_bufI2 = fetch_bufI[2];
	assign fetch_bufI3 = fetch_bufI[3];
	assign fetch_bufI4 = fetch_bufI[4];
	assign fetch_bufI5 = fetch_bufI[5];
	assign fetch_bufI6 = fetch_bufI[6];
	assign fetch_bufI7 = fetch_bufI[7];
	genvar j;
	localparam BUF_SIZE_D = 12;
	localparam BUF_SIZE_I = 16;
	reg [31:0] fetch_bufI[BUF_SIZE_I-1:0]; //separate into instructure
	reg [31:0] fetch_bufI_n[BUF_SIZE_I-1:0]; 
	reg [10:0]  fetch_addrI[BUF_SIZE_I-1:0];
	reg [10:0]  fetch_addrI_n[BUF_SIZE_I-1:0];
	reg i_his, i_his_n; //history of insturcture
	
	reg [31:0] fetch_bufD[BUF_SIZE_D-1:0]; //separate into data
	reg [31:0] fetch_bufD_n[BUF_SIZE_D-1:0];
	reg [10:0]  fetch_addrD[BUF_SIZE_D-1:0];
	reg [10:0]  fetch_addrD_n[BUF_SIZE_D-1:0];
	reg [BUF_SIZE_I-1:0] cache_validI, cache_validI_n;
	reg [BUF_SIZE_D-1:0] cache_validD, cache_validD_n;
	wire [BUF_SIZE_I-1:0] i_flag;
	wire [BUF_SIZE_D-1:0] d_flag;
	reg [3:0] i_idx;
	reg [3:0] d_idx;
	reg [3:0] exe_cache;
	reg [3:0] fetch_ctr, fetch_ctr_n;
	//reg [31:0] fetch_buf2[BUF_SIZE-1:0];
	//reg [31:0] fetch_buf3[BUF_SIZE-1:0];
	//--------------prefetch cache-------------------------------------
	
	always @(posedge clk) begin
        if(rst) begin
			for (i = 0; i < BUF_SIZE_I; i = i + 1)begin
				fetch_bufI[i]  <= 0;
				fetch_addrI[i] <= 0;
			end
			for (i = 0; i < BUF_SIZE_D; i = i + 1)begin
				fetch_bufD[i]  <= 0;
				fetch_addrD[i] <= 0;
			end
		end else begin
			for (i = 0; i < BUF_SIZE_I; i = i + 1)begin
				fetch_bufI[i] <= fetch_bufI_n[i];
				fetch_addrI[i] <= fetch_addrI_n[i];
			end
			for (i = 0; i < BUF_SIZE_D; i = i + 1)begin
				fetch_bufD[i] <= fetch_bufD_n[i];
				fetch_addrD[i] <= fetch_addrD_n[i];
			end
		end
		i_his <= i_his_n;
		cache_validI <= cache_validI_n;
		cache_validD <= cache_validD_n;
		fetch_ctr <= fetch_ctr_n;
	end	
	generate 
		for(j =0; j < BUF_SIZE_I; j = j + 1 )begin
			assign i_flag[j] =({saved_addr_q[10:0]} == fetch_addrI[j])? cache_validI[j] : 1'b0;
		end
		
		for(j =0; j < BUF_SIZE_D; j = j + 1 )begin
			assign d_flag[j] =({saved_addr_q[10:0]} == fetch_addrD[j])? cache_validD[j] : 1'b0;
		end
	endgenerate 
	
	// one hot encode
	always@(*) begin
		case(i_flag)
		16'h1: 	  i_idx = 0;
		16'h2:	  i_idx = 1;
		16'h4:    i_idx = 2;
		16'h8:    i_idx = 3;
		16'h10:   i_idx = 4;
		16'h20:   i_idx = 5;
		16'h40:   i_idx = 6;
		16'h80:   i_idx = 7;
		16'h100:  i_idx = 8;
		16'h200:  i_idx = 9;
		16'h400:  i_idx = 10;
		16'h800:  i_idx = 11;
		16'h1000: i_idx = 12;
		16'h2000: i_idx = 13;
		16'h4000: i_idx = 14;
		16'h8000: i_idx = 15;
		default:  i_idx = 0;
		endcase
	end
	
	always@(*) begin
		case(d_flag)
		16'h1: 	  d_idx = 0;
		16'h2:	  d_idx = 1;
		16'h4:    d_idx = 2;
		16'h8:    d_idx = 3;
		16'h10:   d_idx = 4;
		16'h20:   d_idx = 5;
		16'h40:   d_idx = 6;
		16'h80:   d_idx = 7;
		16'h100:  d_idx = 8;
		16'h200:  d_idx = 9;
		16'h400:  d_idx = 10;
		16'h800:  d_idx = 11;
		default:  d_idx = 0;
		endcase
	end
	
	always @(*) begin
		// Default values
		// cache
		cache_validI_n = cache_validI;
		cache_validD_n = cache_validD;/*
		for (i = 0; i < BUF_SIZE_I; i = i + 1)begin // I can't use for ..... the run-sim will stuck 
			fetch_bufI_n[i] = fetch_bufI[i];
			fetch_addrI_n[i] = fetch_addrI[i];
		end
		for (i = 0; i < BUF_SIZE_D; i = i + 1)begin
			fetch_bufD_n[i] = fetch_bufD[i];
			fetch_addrD_n[i] = fetch_addrD[i];
		end*/
		fetch_bufI_n[0] = fetch_bufI[0];
		fetch_bufI_n[1] = fetch_bufI[1];
		fetch_bufI_n[2] = fetch_bufI[2];
		fetch_bufI_n[3] = fetch_bufI[3];
		fetch_bufI_n[4] = fetch_bufI[4];
		fetch_bufI_n[5] = fetch_bufI[5];
		fetch_bufI_n[6] = fetch_bufI[6];
		fetch_bufI_n[7] = fetch_bufI[7];
		fetch_bufI_n[8] = fetch_bufI[8];
		fetch_bufI_n[9] = fetch_bufI[9];
		fetch_bufI_n[10] = fetch_bufI[10];
		fetch_bufI_n[11] = fetch_bufI[11];
		fetch_bufI_n[12] = fetch_bufI[12];
		fetch_bufI_n[13] = fetch_bufI[13];
		fetch_bufI_n[14] = fetch_bufI[14];
		fetch_bufI_n[15] = fetch_bufI[15];
		
		fetch_addrI_n[0]  = fetch_addrI[0];
		fetch_addrI_n[1]  = fetch_addrI[1];
		fetch_addrI_n[2]  = fetch_addrI[2];
		fetch_addrI_n[3]  = fetch_addrI[3];
		fetch_addrI_n[4]  = fetch_addrI[4];
		fetch_addrI_n[5]  = fetch_addrI[5];
		fetch_addrI_n[6]  = fetch_addrI[6];
		fetch_addrI_n[7]  = fetch_addrI[7];
		fetch_addrI_n[8]  = fetch_addrI[8];
		fetch_addrI_n[9]  = fetch_addrI[9];
		fetch_addrI_n[10] = fetch_addrI[10];
		fetch_addrI_n[11] = fetch_addrI[11];
		fetch_addrI_n[12] = fetch_addrI[12];
		fetch_addrI_n[13] = fetch_addrI[13];
		fetch_addrI_n[14] = fetch_addrI[14];
		fetch_addrI_n[15] = fetch_addrI[15];
		
		
		fetch_bufD_n[0] = fetch_bufD[0];
		fetch_bufD_n[1] = fetch_bufD[1];
		fetch_bufD_n[2] = fetch_bufD[2];
		fetch_bufD_n[3] = fetch_bufD[3];
		fetch_bufD_n[4] = fetch_bufD[4];
		fetch_bufD_n[5] = fetch_bufD[5];
		fetch_bufD_n[6] = fetch_bufD[6];
		fetch_bufD_n[7] = fetch_bufD[7];
		fetch_bufD_n[8] = fetch_bufD[8];
		fetch_bufD_n[9] = fetch_bufD[9];
		fetch_bufD_n[10] = fetch_bufD[10];
		fetch_bufD_n[11] = fetch_bufD[11];
		
		fetch_addrD_n[0]  = fetch_addrD[0];
		fetch_addrD_n[1]  = fetch_addrD[1];
		fetch_addrD_n[2]  = fetch_addrD[2];
		fetch_addrD_n[3]  = fetch_addrD[3];
		fetch_addrD_n[4]  = fetch_addrD[4];
		fetch_addrD_n[5]  = fetch_addrD[5];
		fetch_addrD_n[6]  = fetch_addrD[6];
		fetch_addrD_n[7]  = fetch_addrD[7];
		fetch_addrD_n[8]  = fetch_addrD[8];
		fetch_addrD_n[9]  = fetch_addrD[9];
		fetch_addrD_n[10] = fetch_addrD[10];
		fetch_addrD_n[11] = fetch_addrD[11];
		i_his_n = i_his;
		
        case (state_q)
			INIT: begin
				cache_validD_n = 0;
				cache_validI_n = 0;
				
				i_his_n = 0;
			end
			IDLE: begin
			
			end
			READ: begin
				if(exe_cache[3]) // result
					cache_validD_n = 12'h0ff & cache_validD;
				else if(exe_cache[2]) // B
					cache_validD_n = 12'hf0f & cache_validD;
				else if(exe_cache[1])
					cache_validD_n = 12'hff0 & cache_validD;
				else if(i_his)
					cache_validI_n = 16'h00ff & cache_validI;
				else
					cache_validI_n = 16'hff00 & cache_validI;
				
				if(exe_cache[3]) //result
					fetch_addrD_n[8] = addr_q[10:0];
				if(exe_cache[2]) //B
					fetch_addrD_n[4] = addr_q[10:0];
				if(exe_cache[1]) //A
					fetch_addrD_n[0] = addr_q[10:0];
				if(exe_cache[0]) //result
					if(i_his)
						fetch_addrI_n[8] = addr_q[10:0];
					else
						fetch_addrI_n[0] = addr_q[10:0];
			end
			WAIT_READ: begin
				if(exe_cache[3]) //result
					fetch_addrD_n[8+fetch_ctr] = {addr_q[10:8],a_d[9:2]};
				if(exe_cache[2]) //B
					fetch_addrD_n[4+fetch_ctr] = {addr_q[10:8],a_d[9:2]};
				if(exe_cache[1]) //A
					fetch_addrD_n[0+fetch_ctr] = {addr_q[10:8],a_d[9:2]};
				if(exe_cache[0]) //result
					if(i_his)
						fetch_addrI_n[8+fetch_ctr] = {addr_q[10:8],a_d[9:2]};
					else
						fetch_addrI_n[0+fetch_ctr] = {addr_q[10:8],a_d[9:2]};
			end
			READ_RES: begin
				
			end
		endcase
		if(fetch_ctr == tIns + 4)
			i_his_n = ~i_his;
			
		if(fetch_ctr >='d3) begin
			if(exe_cache[3])begin //result
				fetch_bufD_n[8+fetch_ctr - 3] = dqi_d;
				cache_validD_n[8+fetch_ctr - 3] = 1;
			end
			if(exe_cache[2])begin //B
				fetch_bufD_n[4+fetch_ctr - 3] = dqi_d;
				cache_validD_n[4+fetch_ctr - 3] = 1;
			end
			if(exe_cache[1])begin //A
				fetch_bufD_n[0+fetch_ctr - 3] = dqi_d;
				cache_validD_n[0+fetch_ctr - 3] = 1;
			end
			if(exe_cache[0]) //result
				if(i_his)begin
					fetch_bufI_n[8+fetch_ctr - 3] = dqi_d;
					cache_validI_n[8+fetch_ctr - 3] = 1;
				end else begin
					fetch_bufI_n[0+fetch_ctr - 3] = dqi_d;
					cache_validI_n[0+fetch_ctr - 3] = 1;
				end
		end
	end
	//------------------------------------------------------
	always @(*) begin
		if(addr_q[9])begin // data cache miss
			if(addr_q[8]) // result
				exe_cache = 4'b1000;
			else if(addr_q[7:0] < 8'h40) // A
				exe_cache = 4'b0010;
			else //B
				exe_cache = 4'b0100;
		end else begin // insturcture cache miss
				exe_cache = 4'b0001;
			//i_his_n = ~i_his;
		end
	end
	
    always @(*) begin
        fetch_ctr_n = fetch_ctr;/*
		for (i = 0; i < BUF_SIZE_I; i = i + 1)begin
			fetch_bufI_n[i] = fetch_bufI[i];
			fetch_addrI_n[i] = fetch_addrI[i];
		end
		for (i = 0; i < BUF_SIZE_D; i = i + 1)begin
			fetch_bufD_n[i] = fetch_bufD[i];
			fetch_addrD_n[i] = fetch_addrD[i];
		end*/
		
		
        dq_d = dq_q;
        dqi_d = sdram_dqi;
        dq_en_d = 1'b0; // normally keep the bus in high-Z
        cle_d = cle_q;
        cmd_d = CMD_NOP; // default to NOP
        dqm_d = 1'b0;
        ba_d = 2'd0;
        a_d = 13'd0;
        state_d = state_q;
        next_state_d = next_state_q; 
        delay_ctr_d = delay_ctr_q;
        addr_d = addr_q;
        data_d = data_q;
        out_valid_d = 1'b0;
        precharge_bank_d = precharge_bank_q;
        rw_op_d = rw_op_q;

        row_open_d = row_open_q;

        // row_addr is a 2d array and must be coppied this way
        for (i = 0; i < 4; i = i + 1)
            row_addr_d[i] = row_addr_q[i];

        // The data in the SDRAM must be refreshed periodically.
        // This conter ensures that the data remains intact.
        refresh_flag_d = refresh_flag_q;
        refresh_ctr_d = refresh_ctr_q + 1'b1;
        // Jiin : refresh_counter tRef_Counter
        // if (refresh_ctr_q > 10'd750) begin
        if (refresh_ctr_q > tRef_Counter) begin
            refresh_ctr_d = 10'd0;
            refresh_flag_d = 1'b1;
        end

        // This is a queue of 1 for read/write operations.
        // When the queue is empty we aren't busy and can
        // accept another request.
        saved_rw_d = saved_rw_q;
        saved_data_d = saved_data_q;
        saved_addr_d = saved_addr_q;
        ready_d = ready_q;
        if (ready_q && in_valid) begin // 當不是busy、且in_valid =1時才會吃值
            saved_rw_d = rw;
            saved_data_d = data_in;
            saved_addr_d = addr;
            ready_d = 1'b0;
        end 

        case (state_q)
            ///// INITALIZATION /////
            INIT: begin
                row_open_d = 4'b0;
                out_valid_d = 1'b0;
                // a_d = 13'b0;
                // Reserved, Burst Access, Standard Op, CAS = 2, Sequential, Burst = 4
                a_d = {3'b000, 1'b0, 2'b00, 3'b010, 1'b0, 3'b010}; //010
                ba_d = 2'b0;
                cle_d = 1'b1;
                state_d = WAIT;
                // Note: Jiin - We can skip the power-up sequence & LMR
                // directly jump to IDLE state
                // Power-up Sequence
                // 1. wait for power-up sequence, cmd - NOP or INHIBIT
                // 2. precharge all
                // 3. 2 x Auto-refresh

                // delay_ctr_d = 16'd10100; // wait for 101us
                // next_state_d = PRECHARGE_INIT;
				fetch_ctr_n = 0;
                delay_ctr_d = 16'd0;
                next_state_d = IDLE;
                refresh_flag_d = 1'b0;
                refresh_ctr_d = 10'b1;
                ready_d = 1'b1;

                dq_en_d = 1'b0;
            end
            WAIT: begin
                delay_ctr_d = delay_ctr_q - 1'b1;  // according to different action will have differnnt delay time
                if (delay_ctr_q == 13'd0) begin
                    state_d = next_state_q;
                    // if (next_state_q == WRITE) begin
                    //     dq_en_d = 1'b1; // enable the bus early
                    //     dq_d = data_q[7:0];
                    // end
                end
            end
            ///// IDLE STATE /////
            IDLE: begin
				fetch_ctr_n = 0;
                if (refresh_flag_q) begin // we need to do a refresh
                    state_d = PRECHARGE;
                    next_state_d = REFRESH;
                    precharge_bank_d = 3'b100; // all banks
                    refresh_flag_d = 1'b0; // clear the refresh flag
                end else if (!ready_q) begin // operation waiting
                    ready_d = 1'b1; // clear the queue
                    rw_op_d = saved_rw_q; // save the values we'll need later
                    addr_d = saved_addr_q;

                    if (saved_rw_q) // Write
                        data_d = saved_data_q;

                    
					if((saved_addr_q[9] ==1 && |d_flag) || (saved_addr_q[9] ==0 && |i_flag) )begin // cache hit
						state_d =  IDLE;
						//out_valid_d = 1'b1;
						//if(saved_addr_q[9])
						//	data_d = fetch_bufD[d_idx];
						//else
						//	data_d = fetch_bufI[i_idx];
					end 
					else begin 	// cache miss
						if (row_open_q[saved_addr_q[9:8]]) begin // if the row is open we don't have to activate it
							if (row_addr_q[saved_addr_q[9:8]] == saved_addr_q[22:10]) begin
								// Row is already open
								if (saved_rw_q)
									state_d = WRITE;
								else
									state_d = READ;
							end else begin
								// A different row in the bank is open
								state_d = PRECHARGE; // precharge open row
								precharge_bank_d = {1'b0, saved_addr_q[9:8]};
								next_state_d = ACTIVATE; // open current row
							end
						end else begin
							// no rows open
							state_d = ACTIVATE; // open the row
						end
					end
                end
            end

            ///// REFRESH /////
            REFRESH: begin
                cmd_d = CMD_REFRESH;
                state_d = WAIT;

                // Jiin
                // delay_ctr_d = 13'd6; // gotta wait 7 clocks (66ns)
                delay_ctr_d = tREF;

                next_state_d = IDLE;
            end

            ///// ACTIVATE /////
            ACTIVATE: begin
                cmd_d = CMD_ACTIVE;
                a_d = addr_q[22:10]; //activate 時 address是吃row address
                ba_d = addr_q[9:8];

                // Jiin:
                //      delay_ctr_d = 13'd0;
                delay_ctr_d = tACT;

                state_d = WAIT;

                if (rw_op_q)
                    next_state_d = WRITE;
                else
                    next_state_d = READ;

                row_open_d[addr_q[9:8]] = 1'b1; // row is now open
                row_addr_d[addr_q[9:8]] = addr_q[22:10];
            end

            ///// READ /////
            READ: begin
                cmd_d = CMD_READ;
                a_d = {2'b0, 1'b0, addr_q[7:0], 2'b0}; // Read / Write時 address是吃column address
                ba_d = addr_q[9:8];
				
                state_d = WAIT_READ;
				//state_d = WAIT;
                // Jiin
                // delay_ctr_d = 13'd2; 
				//delay_ctr_d = tCASL; // wait for the data to show up
				fetch_ctr_n = 1;

                next_state_d = READ_RES;

            end
			WAIT_READ:begin
				fetch_ctr_n = fetch_ctr + 1'b1;
				cmd_d = CMD_READ;
				if(ba_q == 2'd2 && addr_q[7:0] >= 8'h40) // B idx
					a_d = a_q + 'h40; 
				else
					a_d = a_q + 'h10; 
				if(a_d[10] == 1)
					a_d = {2'b0, 1'b0, 8'd1, 2'b0}; // addr = 1 never get the odd address from WB bus 
                ba_d = addr_q[9:8];
				if(exe_cache[0])
					if(fetch_ctr > tIns)
						state_d = READ_RES;
					else
						state_d = WAIT_READ;
				else
					if(fetch_ctr > tdat)
						state_d = READ_RES;
					else
						state_d = WAIT_READ;
			end
            READ_RES: begin	
				fetch_ctr_n = fetch_ctr + 1'b1;
				if(exe_cache[0])
					if(fetch_ctr == tIns + 3)
						state_d = IDLE;
					else
						state_d = READ_RES;
				else
					if(fetch_ctr == tdat + 3)
						state_d = IDLE;
					else
						state_d = READ_RES;
            end

            ///// WRITE /////
            WRITE: begin
                cmd_d = CMD_WRITE;

                dq_d = data_q;
                // data_d = data_q;
                dq_en_d = 1'b1; // enable out bus
                a_d = {2'b0, 1'b0, addr_q[7:0], 2'b00};
                ba_d = addr_q[9:8];

                //state_d = IDLE;
				
				state_d = WAIT;

                // Jiin
                // delay_ctr_d = 13'd6; // gotta wait 7 clocks (66ns)
                delay_ctr_d = 1;

                next_state_d = IDLE;
            end

            ///// PRECHARGE /////
            PRECHARGE: begin
                cmd_d = CMD_PRECHARGE;
                a_d[10] = precharge_bank_q[2]; // all banks
                ba_d = precharge_bank_q[1:0];
                state_d = WAIT;

                // Jiin
                // delay_ctr_d = 13'd0;
                delay_ctr_d = tPRE;

                if (precharge_bank_q[2]) begin
                    row_open_d = 4'b0000; // closed all rows
                end else begin
                    row_open_d[precharge_bank_q[1:0]] = 1'b0; // closed one row
                end
            end

            default: state_d = INIT;
        endcase
		if(!ready_q && !saved_rw_q) begin // is read 
			if((saved_addr_q[9] ==1 && |d_flag) || (saved_addr_q[9] ==0 && |i_flag) )begin // cache hit
				ready_d = 1;
				out_valid_d = 1'b1;
				if(saved_addr_q[9])
					data_d = fetch_bufD[d_idx];
				else
					data_d = fetch_bufI[i_idx];
			end 
		end
		if(fetch_ctr == 'd4) begin
			data_d = dqi_q; // data_d by pass
			out_valid_d = 1'b1;
			//state_d = IDLE;
		end
    end

    always @(posedge clk) begin
        if(rst) begin
            cle_q <= 1'b0;
            dq_en_q <= 1'b0;
            state_q <= INIT;
            ready_q <= 1'b0;
        end else begin
            cle_q <= cle_d;
            dq_en_q <= dq_en_d;
            state_q <= state_d;
            ready_q <= ready_d;
        end

        saved_rw_q <= saved_rw_d;
        saved_data_q <= saved_data_d;
        saved_addr_q <= saved_addr_d;

        cmd_q <= cmd_d;
        dqm_q <= dqm_d;
        ba_q <= ba_d;
        a_q <= a_d;
        dq_q <= dq_d;
        dqi_q <= dqi_d;

        next_state_q <= next_state_d;
        refresh_flag_q <= refresh_flag_d;
        refresh_ctr_q <= refresh_ctr_d;
        data_q <= data_d;
        addr_q <= addr_d;
        out_valid_q <= out_valid_d;
        row_open_q <= row_open_d;
        for (i = 0; i < 4; i = i + 1)
            row_addr_q[i] <= row_addr_d[i];
        precharge_bank_q <= precharge_bank_d;
        rw_op_q <= rw_op_d;
        delay_ctr_q <= delay_ctr_d;
    end

endmodule