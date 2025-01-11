`ifndef N
`define N              16
`endif
`define W               8
`define lgN     ($clog2(`N))
`define dbLgN (2*$clog2(`N))
`define lglgN   ($clog2(`lgN))

`define delayRedUnit    `lgN + 1
`define delayPE        `delayRedUnit + 2
`define Vector 0
`define Buffer 1

// prevent auto-inference for width
/* verilator lint_off WIDTHEXPAND */
/* verilator lint_off WIDTHTRUNC */

typedef struct packed { logic [`W-1:0] data; } data_t;

module add_(
    input   logic   clock,
    input   data_t  a,
    input   data_t  b,
    output  data_t  out
);
    always_ff @(posedge clock) begin
        out.data <= a.data + b.data;
    end
endmodule

module mul_(
    input   logic   clock,
    input   data_t  a,
    input   data_t  b,
    output  data_t out
);
    always_ff @(posedge clock) begin
        out.data <= a.data * b.data;
    end
endmodule

// shift register for delay
module delay_shift #(parameter W = `N, parameter DELAY_CYCLES = 1)(
    input logic clock, reset,
    input logic[W-1:0] in,
    output logic[W-1:0] out
);
    logic [W-1:0] shift_reg [DELAY_CYCLES-1:0];
    always_ff @(posedge clock) begin
        for(int i = 1; i < DELAY_CYCLES; i++) begin
            if(reset) shift_reg[i] <= 0;
            else shift_reg[i] <= shift_reg[i-1];
        end
    end
    always_ff @(posedge clock) begin
        if(reset) shift_reg[0] <= 0;
        else shift_reg[0] <= in;
    end

    assign out = shift_reg[DELAY_CYCLES-1];
endmodule

// a naive adder tree
module AdderTree #(parameter LENGTH = `N)(
    input clock,
    input data_t add_ins[LENGTH-1:0],
    output data_t sum_out
);
    localparam LENGTH_LEFT = LENGTH / 2;
    localparam LENGTH_RIGHT = LENGTH - LENGTH_LEFT;
    generate 
        if (LENGTH == 1) begin
            assign sum_out = add_ins[0];
        end
        else begin
            // define left sub-tree and right sub-tree signals
            data_t sum_out_left, sum_out_right;
            data_t add_ins_left [LENGTH_LEFT-1:0];
            data_t add_ins_right [LENGTH_RIGHT-1:0];
            // input assignment
            genvar i;
            for(i = 0; i < LENGTH_LEFT; i++) begin
                assign add_ins_left[i] = add_ins[i + LENGTH_RIGHT];
            end
            for(i = 0; i < LENGTH_RIGHT; i++) begin
                assign add_ins_right[i] = add_ins[i];
            end
            // instatiating sub-modules
            AdderTree #(
                .LENGTH(LENGTH_LEFT)
            ) subtree_left (
                .clock(clock),
                .add_ins(add_ins_left),
                .sum_out(sum_out_left)
            );
            AdderTree #(
                .LENGTH(LENGTH_RIGHT)
            ) subtree_right (
                .clock(clock),
                .add_ins(add_ins_right),
                .sum_out(sum_out_right)
            );

            //assign sum_out = sum_out_left + sum_out_right;
            add_ REG_ADDER (.clock(clock), .a(sum_out_left), .b(sum_out_right), .out(sum_out.data));
        end
    endgenerate

endmodule

module RedUnit #(parameter UPPER_DELAY = 0)(
    input   logic               clock,
                                reset,
    input   data_t              data[`N-1:0],
    input   logic               split[`N-1:0],
    input   logic [`lgN-1:0]    out_idx[`N-1:0],
    output  data_t              out_data[`N-1:0],
    output  int                 delay,
    output  int                 num_el,
    output  data_t              halo_sum
);
    // num_el 总是赋值为 N
    assign num_el = `N;
    // delay 你需要自己为其赋值，表示电路的延迟
    assign delay = `delayRedUnit;

    // Prefix sum logic: Hillis-Steele Scan Algorithm
    data_t pfx_sum [`lgN:0][`N-1:0];  // depth is lgN+1, width is N
    data_t zero;
    assign zero.data = 0;

    generate
        // leaf nodes
        for(genvar i = 0; i < `N; i++)
            assign pfx_sum[0][i] = data[i];

        for(genvar i = 0; i < `lgN; i++) begin
            for(genvar j = 0; j < `N; j++) begin
                localparam two_power_i = 1 << i;
                localparam flag = j < two_power_i;
                add_ pfx_sum_adder(
                    .clock(clock),
                    .a(pfx_sum[i][j]), 
                    .b(flag ? zero : pfx_sum[i][j - two_power_i]),
                    .out(pfx_sum[i + 1][j]));
            end
        end
    endgenerate

    // delay the split and out_idx for output
    logic [`lgN-1:0] out_idx_reg [`N-1:0];
    logic split_reg [`N-1:0];
    generate
        for(genvar i = 0; i < `N; i++) begin
            // lgN for pfxsum
            delay_shift #(.W(1), .DELAY_CYCLES(`lgN + UPPER_DELAY)) split_delay_shift(.clock(clock), .reset(reset), .in(split[i]), .out(split_reg[i]));
            // 1 cycle for part_sum update 
            delay_shift #(.W(`lgN), .DELAY_CYCLES(`lgN + 1 + UPPER_DELAY)) out_idx_delay_shift(.clock(clock), .reset(reset), .in(out_idx[i]), .out(out_idx_reg[i])); 
            // 1 cycle for output update
            // total delay is lgN + 2
        end
    endgenerate

    // get partsum start idx
    data_t part_sum [`N-1:0];
    logic [`lgN-1:0] partsum_head_idx [`N-1:0];
    logic found[`N-1:0];
    always_comb begin
        for(int i = 0; i < `N; i++) begin
            partsum_head_idx[i] = 0;
            found[i] = 0;
            for(int j = i - 1; j >= 0; j--) begin
                if(split_reg[j] && ~found[i]) begin
                    partsum_head_idx[i] = j + 1; // [j+1, i] is the partsum range
                    found[i] = 1;
                end
                else partsum_head_idx[i] = partsum_head_idx[i]; // only select the last split
            end
        end
    end

    // get partsum
    always_ff @(posedge clock) begin
        for(int i = 0; i < `N; i++)
            if(reset) part_sum[i] <= 0;
            else if(split_reg[i])
                part_sum[i] <= partsum_head_idx[i] > 0 ? pfx_sum[`lgN][i] - pfx_sum[`lgN][partsum_head_idx[i] - 1] : pfx_sum[`lgN][i]; 
            else part_sum[i] <= 0;
    end
    
    // halo adder logic
    data_t halo_sum_reg;
    always_ff @(posedge clock) begin
        if(reset) halo_sum_reg <= 0;
        else if(split_reg[`N-1]) halo_sum_reg <= 0;
        else halo_sum_reg <= pfx_sum[`lgN][`N-1] - pfx_sum[`lgN][partsum_head_idx[`N-1] - 1];
    end

    // delay 1 cycle to wait for out_data update
    delay_shift #(.W(`W), .DELAY_CYCLES(1)) out_idx_delay_shift(.clock(clock), .reset(reset), .in(halo_sum_reg), .out(halo_sum)); 

    always_comb begin
        for(int i = 0; i < `N; i++) begin
            out_data[i] = part_sum[out_idx_reg[i]];
        end
    end

endmodule

module StartDetector #(parameter Type = `Vector)(
    input logic clock,
    input logic reset,
    input logic start,
    output logic en,
    output logic [`lgN-1:0] ctr_
);
    generate
        if(Type == `Buffer) begin
            if(`N == 4) begin
                assign en = start;
                assign ctr_ = 0;
            end
            else begin 
                logic started;
                logic [`lgN-3:0] ctr_next, ctr;
                assign en = start || started;
                assign ctr_next = en ? ctr + 1 : ctr;
                assign ctr_ = ctr;
                
                always_ff @(posedge clock) begin
                    if(reset) ctr <= 0;
                    else ctr <= ctr_next;
                end 
                always_ff @(posedge clock) begin
                    if(reset) started <= 0;
                    else started <= start || (ctr != (`N/4-1) && ctr != 0); // 0 decided by start signal
                end
            end
        end
        else if(Type == `Vector) begin
            logic started;
            logic [`lgN-1:0] ctr_next, ctr;
            assign en = start || started;
            assign ctr_next = en ? ctr + 1 : ctr;
            assign ctr_ = ctr;
            
            always_ff @(posedge clock) begin
                if(reset) ctr <= 0;
                else ctr <= ctr_next;
            end 
            always_ff @(posedge clock) begin
                if(reset) started <= 0;
                else started <= start || (ctr != (`N-1) && ctr != 0); // 0 decided by start signal
            end
        end
    endgenerate

endmodule

module PE(
    input   logic               clock,
                                reset,
    input   logic               lhs_start,
    input   logic [`dbLgN-1:0]  lhs_ptr [`N-1:0],
    input   logic [`lgN-1:0]    lhs_col [`N-1:0],
    input   data_t              lhs_data[`N-1:0],
    input   data_t              rhs[`N-1:0],
    output  data_t              out[`N-1:0],
    output  int                 delay,
    output  int                 num_el
    ,output logic out_idx_valid [`N-1:0] // used for SpMM output
);
    // num_el 总是赋值为 N
    assign num_el = `N;
    // delay 你需要自己为其赋值，表示电路的延迟
    //assign delay = `lgN + 2; // lgN+1 for RedUnit, 1 for inner product
    assign delay = `delayPE;
    
    // get the enable signal and counter
    logic lhs_en;
    logic [`lgN-1:0] lhs_ctr;
    StartDetector #(.Type(`Vector)) lhs_detector(.clock(clock), .reset(reset), .start(lhs_start), .en(lhs_en), .ctr_(lhs_ctr));

    // get the split vector
    logic split [`N-1:0]; 
    logic split_table [`N-1:0][`N-1:0]; // a split lookup table
    logic [`lgN-1:0] row_id [`N-1:0];
    logic [`lgN-1:0] col_id [`N-1:0];
    always_comb begin
        for(int i = 0; i < `N; i++) begin   
            row_id[i] = lhs_ptr[i] / `N;
            col_id[i] = lhs_ptr[i] % `N;
        end
    end
    always_comb begin
        for(int i = 0; i < `N; i++) begin
            for(int j = 0; j < `N; j++) begin
                split_table[i][j] = 0;
            end
        end
        for(int i = 0; i < `N; i++)
            split_table[row_id[i]][col_id[i]] = 1;
    end

    // deal with the output of zero lines in lhs
    logic [`lgN-1:0] last_non_zero_id [`N-1:0];
    logic [`lgN-1:0] zero_lines [`N-1:0]; // number of zero lines
    always_comb begin
        last_non_zero_id[0] = 0;
        zero_lines[0] = 0; // first line must be non-zero
        for(int i = 1; i < `N; i++) begin
            if(lhs_ptr[i] == lhs_ptr[i-1]) begin
                last_non_zero_id[i] = last_non_zero_id[i-1];
                zero_lines[last_non_zero_id[i]] += 1;
            end
            else begin
                last_non_zero_id[i] = i;
                zero_lines[i] = 0;
            end
        end
    end

    // update split according to lhs_ctr
    always_comb begin
        for(int i = 0; i < `N; i++)
            split[i] = lhs_en ? split_table[lhs_ctr][i] : 0;
    end

    // out_idx according to split vector
    logic [`lgN-1:0] split_row_id, done_row_ctr;
    logic split_row_en;
    // if the final elment is not splited, the row is splited
    assign split_row_en = ~split[`N-1];

    // count the done rows for the next row id
    always_ff @(posedge clock) begin
        if(reset) done_row_ctr <= 0;
        else if(!lhs_en) done_row_ctr <= 0;
        else done_row_ctr <= done_row_ctr + out_idx_ctr;
    end 

    // out_idx generation logic
    logic [`lgN-1:0] out_idx [`N-1:0];
    //logic out_idx_valid [`N-1:0]; stated as output port
    logic [`lgN-1:0] out_idx_ctr;
    always_comb begin
        out_idx_ctr = 0;
        for(int i = 0; i < `N; i++) begin
            out_idx[i] = 0;
            out_idx_valid[i] = 0;
        end
        for(int i = 0; i < `N; i++) begin
            if(split[i]) begin
                out_idx[done_row_ctr + out_idx_ctr] = i;
                out_idx_valid[done_row_ctr + out_idx_ctr] = 1;
                out_idx_ctr += (zero_lines[done_row_ctr + out_idx_ctr] + 1); // modified for zero lines
            end
        end
    end

    // Inner product of lhs_data and rhs_data
    data_t data [`N-1:0];
    generate
        for(genvar i = 0; i < `N; i++)begin
            mul_ DATA_MUL_UNIT(.clock(clock), .a(lhs_data[i]), .b(rhs[lhs_col[i]]), .out(data[i]));
        end
    endgenerate

    data_t halo_sum;
    logic [`lgN-1:0] halo_id;
    data_t red_out [`N-1:0];
    int delay_redu; // placeholder
    // instantiate RedUnit (1 additional delay from data's multiplication)
    RedUnit #(.UPPER_DELAY(1)) PE_REDUNIT(.clock(clock), .reset(reset),
         .data(data), .split(split), .out_idx(out_idx), .out_data(red_out), 
         .delay(delay_redu), .num_el(num_el), .halo_sum(halo_sum));

    // filter the invalid output
    logic out_idx_valid_reg [`N-1:0];
    generate
        for(genvar i = 0; i < `N; i++) begin
            delay_shift #(.W(1), .DELAY_CYCLES(`delayPE - 1)) out_idx_valid_delay_shift(.clock(clock), .reset(reset), .in(out_idx_valid[i]), .out(out_idx_valid_reg[i])); 
        end
    endgenerate

    // find the first partsum id
    logic flag;
    always_comb begin
        flag = 0;
        halo_id = 0;
        for(int i = 0; i < `N; i++) begin
            if(out_idx_valid_reg[i] && ~flag) begin
                flag = 1;
                halo_id = i;
            end
        end
    end

    always_ff @(posedge clock) begin
        for(int i = 0; i < `N; i++) begin
            if(reset) out[i] <= 0;
            else if(i == halo_id) out[i] <= red_out[i] + halo_sum;
            else out[i] <= out_idx_valid_reg[i] ? red_out[i] : 0;
        end
    end

endmodule

module rhs_dbbuf(
    input   logic               clock,
                                reset,
    input   logic               wr_en,
    input   logic               rd_en,
    input   data_t              rhs_data [3:0][`N-1:0],
    input   logic               rd_ws, // weight-stationary read, not discard
    output  data_t              data [`N-1:0][`N-1:0],
    output  logic               wr_valid,
    output  logic               rd_valid
);

    data_t rhs_buffer [1:0][`N-1:0][`N-1:0]; // two arrays
    logic wr_sel, rd_sel;
    logic [1:0] rd_ready;
    logic [1:0] wr_ready;
    logic [`lgN-1:0] wr_ptr;
    assign rd_sel = ~wr_sel;

    // write logic
    always_ff @(posedge clock) begin
        for(int i = 0; i < 4; i++) begin
            for(int j = 0; j < `N; j++) begin
                if(wr_en && wr_ready[wr_sel])
                    rhs_buffer[wr_sel][j][wr_ptr * 4 + i] <= rhs_data[i][j]; // store RHS_T in the buffer
                else
                    rhs_buffer[wr_sel][i][j] <= rhs_buffer[wr_sel][i][j];
            end
        end
    end

    always_ff @(posedge clock) begin
        if(reset) wr_ptr <= 0;
        else if(wr_en && wr_ptr != `N/4-1)
            wr_ptr <= wr_ptr + 1;
        else wr_ptr <= 0;
    end

    always_ff @(posedge clock) begin
        if(reset) begin
            wr_ready[wr_sel] <= 1;
            rd_ready[wr_sel] <= 0;
        end
        else begin
            if(wr_en && wr_ptr == `N/4-1) begin
                wr_ready[wr_sel] <= 0;
                rd_ready[wr_sel] <= 1;
            end
            else if(wr_en && wr_ptr != `N/4-1) begin
                wr_ready[wr_sel] <= 1;
                rd_ready[wr_sel] <= 0;
            end
            else begin
                wr_ready[wr_sel] <= 1;
                rd_ready[wr_sel] <= rd_ready[wr_sel];
            end
        end
    end

    // read logic
    always_comb begin
        for(int i = 0; i < `N; i++) begin
            for(int j = 0; j < `N; j++) begin
                if(rd_en && rd_ready[rd_sel])
                    data[i][j] = rhs_buffer[rd_sel][i][j];
                else
                    data[i][j] = 0;
            end
        end
    end

    always_ff @(posedge clock) begin
        if(reset) begin
            wr_ready[rd_sel] <= 1;
            rd_ready[rd_sel] <= 0;
        end
        else if(rd_en && rd_ready[rd_sel]) begin
            wr_ready[rd_sel] <= rd_ws ? 0 : 1; // if rd_ws, not writable, still readable
            rd_ready[rd_sel] <= rd_ws ? 1 : 0;
        end
    end
    
    // switch
    always_ff @(posedge clock) begin
        if(reset) wr_sel <= 0;
        else if(!rd_en)
            wr_sel <= wr_ready[0] ? 0 : wr_ready[1] ? 1 :0;
        else wr_sel <= wr_sel;
    end

    assign wr_valid = | wr_ready;
    assign rd_valid = | rd_ready;

endmodule

// print array task macro
`define PRINT_ARRAY(TASK_NAME, ROW_MAX, COL_MAX, ARRAY_NAME) \
    task TASK_NAME; \
        integer i, j; \
        begin \
            for (i = 0; i < ROW_MAX; i = i + 1) begin \
                for (j = 0; j < COL_MAX; j = j + 1) begin \
                    $fwrite(file, "%d ", ARRAY_NAME[i][j]); \
                end \
                $fwrite(file, "\n"); \
            end \
        end \
    endtask

module SpMM(
    input   logic               clock,
                                reset,
    /* 输入在各种情况下是否 ready */
    output  logic               lhs_ready_ns,
                                lhs_ready_ws,
                                lhs_ready_os,
                                lhs_ready_wos,
    input   logic               lhs_start,
    /* 如果是 weight-stationary, 这次使用的 rhs 将保留到下一次 */
                                lhs_ws,
    /* 如果是 output-stationary, 将这次的结果加到上次的 output 里 */
                                lhs_os,
    input   logic [`dbLgN-1:0]  lhs_ptr [`N-1:0],
    input   logic [`lgN-1:0]    lhs_col [`N-1:0],
    input   data_t              lhs_data[`N-1:0],
    output  logic               rhs_ready,
    input   logic               rhs_start,
    input   data_t              rhs_data [3:0][`N-1:0],
    output  logic               out_ready,
    input   logic               out_start,
    output  data_t              out_data [3:0][`N-1:0],
    output  int                 num_el
);
    // num_el 总是赋值为 N
    assign num_el = `N;

    //assign lhs_ready_ns = 0;
    //assign lhs_ready_ws = 0;
    //assign lhs_ready_os = 0;
    //assign lhs_ready_wos = 0;
    //assign rhs_ready = 0;
    //assign out_ready = 0;

    // detect the start signal
    logic rhs_en, out_en, lhs_en;
    logic [`lgN-1:0] rhs_ctr, out_ctr, lhs_ctr;
    StartDetector #(.Type(`Buffer)) rhs_buffer_detector (.clock(clock), .reset(reset), .start(rhs_start), .en(rhs_en), .ctr_(rhs_ctr));
    StartDetector #(.Type(`Buffer)) out_buffer_detector (.clock(clock), .reset(reset), .start(out_start), .en(out_en), .ctr_(out_ctr));
    StartDetector #(.Type(`Vector)) lhs_data_detector (.clock(clock), .reset(reset), .start(lhs_start), .en(lhs_en), .ctr_(lhs_ctr));
    // ws and os signal
    logic lhs_ws_en;
    logic [`lgN-1:0] lhs_ws_ctr;
    StartDetector #(.Type(`Vector)) lhs_ws_data_detector (.clock(clock), .reset(reset), .start(lhs_ws), .en(lhs_ws_en), .ctr_(lhs_ws_ctr));
    logic lhs_os_en;
    logic [`lgN-1:0] lhs_os_ctr;
    StartDetector #(.Type(`Vector)) lhs_os_data_detector (.clock(clock), .reset(reset), .start(lhs_os), .en(lhs_os_en), .ctr_(lhs_os_ctr));
    assign lhs_ready_wos = lhs_ready_ws &&  lhs_ready_os;

    //----------------------rhs_buffer logic--------------------------------
    data_t rhs_buffer [1:0][`N-1:0][`N-1:0];
    data_t rhs_out [`N-1:0][`N-1:0];
    logic [`lgN-1:0] rhs_wr_ptr;

    // 0 for empty or discard, 1 for writing, 2 for ready to read
    localparam EMPTY = 0, BUSY_WRITE = 1, READY_READ = 2, BUSY_READ = 3;
    logic [1:0] rhs_buffer_state [1:0]; 
    logic [1:0] rhs_buffer_next_state [1:0];
    
    always_ff @(posedge clock) begin
        for(int i = 0; i < 2; i++) begin
            if(reset)
                rhs_buffer_state[i] <= EMPTY;
            else 
                rhs_buffer_state[i] <= rhs_buffer_next_state[i];
        end
    end

    always_comb begin
        case(rhs_buffer_state[1])
            EMPTY : rhs_buffer_next_state[1] = rhs_start ? BUSY_WRITE : EMPTY;
            BUSY_WRITE : rhs_buffer_next_state[1] = rhs_wr_ptr == `N/4-1 ? READY_READ : BUSY_WRITE; 
            READY_READ : rhs_buffer_next_state[1] = rhs_update ? EMPTY : READY_READ; // if updated, allow next input
        endcase
        case(rhs_buffer_state[0])
            EMPTY : rhs_buffer_next_state[0] = rhs_update ? READY_READ : EMPTY;
            //READY_READ : rhs_buffer_next_state[0] = lhs_ctr == `N-1 ? EMPTY : READY_READ; // if read out, discard
            READY_READ : rhs_buffer_next_state[0] = lhs_ctr == `N-1 ? (lhs_ws_en ? READY_READ : EMPTY) : READY_READ; // add ws logic
        endcase
    end

    always_ff @(posedge clock) begin
        if(reset) rhs_ready <= 0;
        else rhs_ready <= rhs_buffer_state[1] == EMPTY && !rhs_start;
    end

    logic rhs_update;
    assign rhs_update = rhs_buffer_state[1] == READY_READ && rhs_buffer_state[0] != READY_READ;

    logic  rhs_wr_en, rhs_rd_en;
    assign rhs_wr_en = rhs_en;
    assign rhs_rd_en = lhs_en;
    // read logic
    always_ff @(posedge clock) begin
        for(int i = 0; i < `N; i++) begin
            for(int j = 0; j < `N; j++)
                if(reset)
                    rhs_buffer[0][i][j] <= 0;
                else if(rhs_update) // read done, discard the matrix and update
                    rhs_buffer[0][i][j] <= rhs_buffer[1][i][j];
                else 
                    rhs_buffer[0][i][j] <= rhs_buffer[0][i][j];
        end
    end
    always_ff @(posedge clock) begin
        for(int i = 0; i < `N; i++) begin
            for(int j = 0; j < `N; j++) begin
                if(reset) rhs_out[i][j] <= 0;
                else if(rhs_buffer_state[0] == READY_READ && rhs_rd_en) rhs_out[i][j] <= rhs_buffer[0][i][j];
                else rhs_out[i][j] <= rhs_out[i][j];
            end
        end
    end

    // write logic
    always_ff @(posedge clock) begin
        for(int i = 0; i < `N; i++) begin
            for(int j = 0; j < `N; j++) begin
                if(reset)
                    rhs_buffer[1][i][j] <= 0;
            end
        end
    end
    always_ff @(posedge clock) begin
        for(int i = 0; i < 4; i++) begin
            for(int j = 0; j < `N; j++) begin
                if(rhs_wr_en)
                    rhs_buffer[1][j][rhs_wr_ptr * 4 + i] <= rhs_data[i][j]; // store RHS_T in the buffer
                else
                    rhs_buffer[1][i][j] <= rhs_buffer[1][i][j];
            end
        end
    end

    always_ff @(posedge clock) begin
        if(reset) rhs_wr_ptr <= 0;
        else if(rhs_wr_en && rhs_wr_ptr != `N/4-1)
            rhs_wr_ptr <= rhs_wr_ptr + 1;
        else rhs_wr_ptr <= 0;
    end

    always_ff @(posedge clock) begin
        if(reset) lhs_ready_ns <= 0;
        //else lhs_ready_ns <= rhs_buffer_state[0] == READY_READ && !lhs_start && !lhs_en;
        // only when the result is outputed, lhs_ns can be high
        else lhs_ready_ns <= rhs_buffer_state[0] == READY_READ && !lhs_start && !lhs_en && out_buffer_state[0] != READY_OUTPUT;
    end

    // ws ready logic
    always_ff @(posedge clock) begin
        if(reset) lhs_ready_ws <= 0;
        //else lhs_ready_ws <= rhs_buffer_state[0] == READY_READ && !lhs_start && !lhs_en; 
        else lhs_ready_ws <= rhs_buffer_state[0] == READY_READ && !lhs_start && !lhs_en && out_buffer_state[0] != READY_OUTPUT;
    end
    // os ready logic
    always_ff @(posedge clock) begin
        if(reset) lhs_ready_os <= 0;
        else lhs_ready_os <= rhs_buffer_state[0] != EMPTY && !lhs_start && !lhs_en;
    end
    //----------------------rhs_buffer logic--------------------------------

    // Instantiate N PEs in parallel
    data_t pe_out_cols [`N-1:0][`N-1:0];
    logic pe_out_cols_valid [`N-1:0];
    logic pe_out_cols_valid_delayed [`N-1:0]; // delay the PE valid flags for output

    int delay; // placeholder vals 
    generate
        for(genvar i = 0; i < `N; i++) begin
            delay_shift #(.W(1), .DELAY_CYCLES(`delayPE)) pe_out_cols_valid_delay_shift(
            .clock(clock), .reset(reset), .in(pe_out_cols_valid[i]), .out(pe_out_cols_valid_delayed[i])
        );
        end
        for(genvar i = 0; i < `N; i++) begin
            PE PE_UNIT(
                .clock(clock),
                .reset(reset),
                .lhs_start(lhs_start),
                .lhs_ptr(lhs_ptr),
                .lhs_col(lhs_col),
                .lhs_data(lhs_data),
                //.rhs(rhs_buffer[i]),
                .rhs(rhs_buffer[0][i]),
                .out(pe_out_cols[i]), // output column vectors
                .delay(delay),
                .num_el(num_el),
                .out_idx_valid(pe_out_cols_valid) // only output the valid value to buffer
            );
        end
    endgenerate

    // generate load columns ready signal by PE delay
    logic out_col_start, out_col_en;
    logic [`lgN-1:0] out_col_ctr;
    delay_shift #(.W(1), .DELAY_CYCLES(`delayPE)) 
        delay_lhs_start_to_out_ready(.clock(clock), .reset(reset), .in(lhs_start), .out(out_col_start));
    StartDetector #(.Type(`Vector)) out_col_detector(.clock(clock), .reset(reset), 
        .start(out_col_start), .en(out_col_en), .ctr_(out_col_ctr));

    /*// generate out_ready signal after loading
    always_ff @(posedge clock) begin
        if(reset) out_ready <= 0;
        else out_ready <= out_col_ctr == `N-1; // last column loaded
    end*/
    //----------------------out_dbbuf logic--------------------------------

    data_t out_buffer [1:0][`N-1:0][`N-1:0];

    localparam BUSY_SENDOUT = 1, READY_OUTPUT = 2, BUSY_OFFLOAD = 3;
    logic [1:0] out_buffer_state [1:0];
    logic [1:0] out_buffer_next_state [1:0];

    // FSM definition
    always_ff @(posedge clock) begin
        for(int i = 0; i < 2; i++) begin
            if(reset)
                out_buffer_state[i] <= EMPTY;
            else
                out_buffer_state[i] <= out_buffer_next_state[i];
        end
    end

    logic out_os_start, out_os_en;
    logic [`lgN-1:0] out_os_ctr;
    delay_shift #(.W(1), .DELAY_CYCLES(`delayPE))
        delay_lhs_os_to_out_os(.clock(clock), .reset(reset), .in(lhs_os && lhs_start), .out(out_os_start));
    StartDetector #(.Type(`Vector)) out_os_detector(.clock(clock), .reset(reset), 
        .start(out_os_start), .en(out_os_en), .ctr_(out_os_ctr));

    localparam NONE_OS = 0, GET_OS = 1, BUSY_OS = 2;
    logic [1:0] os_state, os_next_state;
    logic os_id;
    always_ff @(posedge clock) begin
        if(reset) os_state <= NONE_OS;
        else os_state <= os_next_state;
    end
    always_comb begin
        case(os_state)
            NONE_OS: os_next_state = lhs_os && lhs_start ? GET_OS : NONE_OS;
            GET_OS: os_next_state = out_col_start ? BUSY_OS : GET_OS;
            BUSY_OS: os_next_state = lhs_os && lhs_start ? GET_OS : (out_col_ctr == `N-1 ? NONE_OS : BUSY_OS);
        endcase
    end
    always_ff @(posedge clock) begin
        if(reset) os_id <= 1;
        else if(lhs_os && lhs_start) begin
            if(!(out_buffer_state[0] == READY_OUTPUT && out_buffer_state[1] == READY_OUTPUT))
                os_id <= 0;
            else os_id <= 1;
        end
    end

    always_comb begin
        case(out_buffer_state[0])
            EMPTY : out_buffer_next_state[0] = out_dbbuf_update ? READY_OUTPUT : EMPTY;
            READY_OUTPUT : out_buffer_next_state[0] = out_start ? BUSY_SENDOUT : (out_os_start && out_buffer_state[1] == EMPTY ? BUSY_OFFLOAD : READY_OUTPUT);
            BUSY_OFFLOAD : out_buffer_next_state[0] = out_col_ctr == `N-1 ? READY_OUTPUT : BUSY_OFFLOAD;
            BUSY_SENDOUT : out_buffer_next_state[0] = out_ptr == `N/4-1 ? EMPTY : BUSY_SENDOUT;
        endcase
        case(out_buffer_state[1])
            EMPTY : out_buffer_next_state[1] = (out_col_start && !out_os_start)? BUSY_OFFLOAD : EMPTY;
            BUSY_OFFLOAD : out_buffer_next_state[1] = out_col_ctr == `N-1 ? READY_OUTPUT : BUSY_OFFLOAD;
            READY_OUTPUT : out_buffer_next_state[1] = out_dbbuf_update ? EMPTY : (out_col_start ? BUSY_OFFLOAD : READY_OUTPUT);
        endcase
    end  

    logic out_dbbuf_update;
    assign out_dbbuf_update = out_buffer_state[1] == READY_OUTPUT && out_buffer_state[0] == EMPTY;

    logic  out_wr_en, out_rd_en;
    logic  out_wr_os_en;

    assign out_wr_en = out_col_en;
    assign out_rd_en = out_en;    

    // load PE result columns(write) logic
    always_ff @(posedge clock) begin
        for(int j = 0; j < `N; j++) begin
            for(int i = 0; i < `N; i++) begin
                if(reset) out_buffer[1][i][j] <= 0;
                else if(out_wr_en && pe_out_cols_valid_delayed[i])
                    out_buffer[1][i][j] <= pe_out_cols[j][i];
                else if(out_os_en && os_id == 1 && pe_out_cols_valid_delayed[i])
                    out_buffer[1][i][j] <= out_buffer[1][i][j] + pe_out_cols[j][i];
                else out_buffer[1][i][j] <= out_buffer[1][i][j];
            end
        end
    end

    // sendout out_buffer[0](read) logic
    always_ff @(posedge clock) begin
        for(int i = 0; i < `N; i++) begin
            for(int j = 0; j < `N; j++) begin
                if(reset)
                    out_buffer[0][i][j] <= 0;
                else if(out_dbbuf_update)
                    out_buffer[0][i][j] <= out_buffer[1][i][j];
                else if(out_os_en && os_id == 0 && pe_out_cols_valid_delayed[i])
                    out_buffer[0][i][j] <= out_buffer[0][i][j] + pe_out_cols[j][i];
                else
                    out_buffer[0][i][j] <= out_buffer[0][i][j];
            end
        end
    end
    /*always_ff @(posedge clock) begin
        for(int i = 0; i < 4; i++) begin
            for(int j = 0; j < `N; j++) begin
                if(out_rd_en)
                    out_data[i][j] <= out_buffer[0][i + out_ptr * 4][j];
                else out_data[i][j] <= 0;
            end
        end
    end*/
    always_comb begin
        for(int i = 0; i < 4; i++) begin
            for(int j = 0; j < `N; j++) begin
                out_data[i][j] = out_rd_en ? out_buffer[0][i + out_ptr * 4][j] : 0;
            end
        end
    end

    logic [`lgN-1:0] out_ptr;
    assign out_ptr = out_ctr;

    always_ff @(posedge clock) begin
        if(reset) out_ready <= 0;
        else out_ready <= out_buffer_state[0] == READY_OUTPUT && !out_start && os_state == NONE_OS;
    end

    //----------------------out_dbbuf logic--------------------------------


endmodule
