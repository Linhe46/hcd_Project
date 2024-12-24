`ifndef N
`define N              16
`endif
`define W               8
`define lgN     ($clog2(`N))
`define dbLgN (2*$clog2(`N))
`define N2              `N*`N
`define delayRedUnit   `lgN + 1
`define delayPE        `lgN + 2

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

module RedUnit(
    input   logic               clock,
                                reset,
    input   data_t              data[`N-1:0],
    input   logic               split[`N-1:0],
    input   logic [`lgN-1:0]    out_idx[`N-1:0],
    output  data_t              out_data[`N-1:0],
    output  int                 delay,
    output  int                 num_el
);
    // num_el 总是赋值为 N
    assign num_el = `N;
    // delay 你需要自己为其赋值，表示电路的延迟
    //assign delay = `lgN + 1; // delay is log_2(N) for an adder tree, 1 for read out
    assign delay = `delayRedUnit;

    // 60 points assumption: only read one single line (split === 0)
    // implement an adder tree
    data_t partial_sum [`N-1:0];
    AdderTree #(.LENGTH(`N)) add_tree(
        .clock(clock),
        .add_ins(data),
        .sum_out(partial_sum[`N-1]) // for 60p cases, the checker's output_idx
    );

    //assign out_data[`N-1] = partial_sum;
    always_ff @(posedge clock) begin
        for(int i=0;i<`N;i++) begin
            out_data[i] <= partial_sum[out_idx[i]];
        end
    end
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
);
    // num_el 总是赋值为 N
    assign num_el = `N;
    // delay 你需要自己为其赋值，表示电路的延迟
    //assign delay = `N;
    //assign delay = `lgN + 2; // lgN+1 for RedUnit, 1 for inner product
    assign delay = `delayPE;
    
    logic valid;
    logic [`N-1:0] cnt;
    logic [1:0] state, next_state;
    localparam IDLE = 0, READING = 1;

    always_ff @(posedge clock) begin
        if(reset)
            cnt <= 0;
        else begin
            case(state)
                READING: cnt <= cnt + 1;
                default: cnt <= cnt;
            endcase
        end
    end

    always_ff @(posedge clock) begin
        if(reset)
            state <= IDLE;
        else 
            state <= next_state;
    end

    always_comb begin
        next_state = state;
        case(state)
            IDLE: begin
                if (lhs_start)
                    next_state = READING;
            end
            READING: begin
                if (cnt == `N-1)
                    next_state = IDLE;
            end
        endcase
    end

    assign valid = state == READING || lhs_start;

    // convert the CSR format matrix into readable form 
    // split and output_idx definition


    logic split [`N-1:0];
    data_t data [`N-1:0];
    data_t out_reg [`N-1:0];
    logic [`lgN-1:0] out_idx [`N-1:0];
    
/*
    logic [`N-1:0] split_norm [`N-1:0];
    
    for (integer i=0;i<`N;i++) begin
        always_ff @(posedge clock) begin
            if(reset) begin
                split[i] <= 0;
                out_idx[i] <= 0;
            end
            else if(valid) begin
                split
        end
*/
    // 60 pt out_idx
    always_comb begin
        for(int i = 0; i < `N; i++) begin
            out_idx[i] = `N-1;
        end
    end
    
    // Inner product of lhs_data and rhs_data
    generate
        for(genvar i = 0; i < `N; i++)begin
            mul_ DATA_MUL_UNIT(.clock(clock), .a(lhs_data[i]), .b(rhs[lhs_col[i]]), .out(data[i]));
        end
    endgenerate

    RedUnit PE_REDUNIT(
        .clock(clock),
        .reset(reset),
        .data(data),
        .split(split),
        .out_idx(out_idx),
        .out_data(out_reg)
    );

    always_comb begin
        for(int i=0; i<`N;i++) begin
            out[i] = out_reg[i];
        end
    end

endmodule

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
    assign lhs_ready_ws = 0;
    assign lhs_ready_os = 0;
    assign lhs_ready_wos = 0;
    //assign rhs_ready = 0;
    //assign out_ready = 0;

    // input/output buffer
    data_t rhs_buffer [`N-1:0][`N-1:0];
    data_t out_buffer [`N-1:0][`N-1:0];

    localparam IDLE = 0, READING_RHS = 1, DONE_RHS = 2, OUTPUT_RESULT = 3, DONE_OUTPUT = 4, CALCULATING = 5;
    logic [2:0] state, next_state;

    logic rhs_input_valid;
    //logic output_valid;
    assign rhs_input_valid = state == READING_RHS || rhs_start;
    //assign output_valid = state == OUTPUT_RESULT || out_start;

    always_ff @(posedge clock or posedge rhs_start ) begin
        if(reset)
            state <= IDLE;
        else if(rhs_start)
            state <= READING_RHS;
        else
            state <= next_state;
    end
    
    always_comb begin
        next_state = state;
        case(state)
            IDLE: begin
                if(rhs_start) next_state = READING_RHS;
            end
            READING_RHS: begin
                if(rhs_input_cnt == `N/4-1) next_state = DONE_RHS;
            end
            DONE_RHS: begin
                next_state = OUTPUT_RESULT;
            end
            OUTPUT_RESULT: begin
                if(output_cnt == `N/4-1) next_state = DONE_OUTPUT;
            end
            DONE_OUTPUT: next_state = IDLE;
            default: next_state = state;
        endcase
    end

    logic [`N/4-1:0] rhs_input_cnt;
    //logic [`N/4-1:0] output_cnt;

    //assign rhs_ready = rhs_input_cnt == `N/4 - 1 && rhs_input_valid;

    // ready signal detector
    // for starting input, calculating and output
    logic rhs_ready_next;
    //logic out_ready_next;
    logic lhs_ready_ns_next;
    always_ff @(posedge clock) begin
        if(reset)
            rhs_ready <= 0;
        else
            rhs_ready <= rhs_ready_next;
    end
    always_comb begin
        rhs_ready_next = state == IDLE && ~rhs_ready;
    end

    always_ff @(posedge clock) begin
        if(reset)
            lhs_ready_ns <= 0;
        else
            lhs_ready_ns <= lhs_ready_ns_next;
    end
    always_comb begin
        lhs_ready_ns_next = state == DONE_RHS;
    end


    always_ff @(posedge clock) begin
        if(reset)
            rhs_input_cnt <= 0;
        else begin
            case(state)
                READING_RHS: rhs_input_cnt <= rhs_input_cnt + 1;
                default: rhs_input_cnt <= rhs_input_cnt;
            endcase
        end
    end

    // output ctr
    logic out_started, out_en;
    logic [`N-1:0] out_ctr, out_ctr_next;  // lgN-2?
    assign out_en = out_start || out_started;
    assign out_ctr_next = out_en ? out_ctr + 1 : out_ctr;
    always_ff @(posedge clock) begin
        if(reset) out_started <= 0;
        else out_started <= out_start || (out_ctr != (`N/4-1));
    end

    // read the rhs_input
    always_ff @(posedge clock) begin
        for(int i = 0; i < 4; i++) begin
            for(int j = 0; j < `N; j++) begin
                if(rhs_input_valid)
                    //rhs_buffer[rhs_input_cnt * 4 + i][j] <= rhs_data[i][j];
                    rhs_buffer[j][rhs_input_cnt * 4 + i] <= rhs_data[i][j]; // store RHS_T in the buffer
            end
        end
    end
                
    // instantiate `N PEs
    generate
        for(genvar i = 0; i < `N; i++) begin
            PE PE_UNIT(
                .clock(clock),
                .reset(reset),
                .lhs_start(lhs_start),
                .lhs_ptr(lhs_ptr),
                .lhs_col(lhs_col),
                .lhs_data(lhs_data),
                .rhs(rhs_buffer[i]),
                .out(out_buffer[i]),
                .delay(),
                .num_el()
            );
        end
    endgenerate

    // output the result
    always_ff @(posedge clock) begin
        for(int i = 0; i < 4; i++) begin
            for(int j = 0; j< `N; j++) begin
                if(out_en)
                    out_data[i][j] <= out_buffer[output_cnt * 4 + i][j];
            end
        end
    end


    // debug print in a text file
    integer file = $fopen("output.txt", "w");
    `PRINT_ARRAY(print_rhs, 4, `N, rhs_data)
    `PRINT_ARRAY(print_rhs_buffer, `N, `N, rhs_buffer)
    `PRINT_ARRAY(print_out_buffer, `N, `N, out_buffer)

    always @(posedge clock or negedge clock) begin
        $fdisplay(file, "clk = %b, rhs_start = %b, rhs_ready = %b, rhs_cnt=%d, rhs_valid = %b, state = %s", clock, rhs_start, rhs_ready, rhs_input_cnt, rhs_input_valid, 
        state == 0 ? "IDLE" : state == 1 ? "READING RHS" : state == 2 ? "RHS_DONE" : state == 3 ? "OUTPUT_RES" : "DONE_OUTPUT");
        $fdisplay(file, "rhs input: "); print_rhs();
        $fdisplay(file, "rhs_buffer: "); print_rhs_buffer();
        $fdisplay(file, "lhs_start = %b, lhs_ready_ns = %b", lhs_start, lhs_ready_ns);
        $fdisplay(file, "out_buffer: "); print_out_buffer();
    end

endmodule
