`timescale 1ns / 1ps
//==============================================================================
// tb_ip_axi_softmax - Testbench for Row-wise Softmax IP
//
// Mirrors tb_ip_axi_linear.sv structure (same VIP usage pattern: AXI-Lite
// master VIP for control, 1x AXI-Stream master VIP for S input, 1x
// AXI-Stream slave VIP for softmax-weight output), adapted for softmax's
// single-input-stream / single-output-stream interface (no K/Q dual stream,
// no TDEST routing - softmax.sv has only one S_AXIS port).
//
// LƯU Ý QUAN TRỌNG (giống hệt cảnh báo trong tb_ip_axi_linear.sv):
//   `u_dut` bên dưới instantiate `ip_axi_softmax_0` - IP đã đóng gói trong
//   Vivado, các tham số (D_HEAD/SEQ_LEN/DATA_WIDTH/EXP_WIDTH/DIV_LATENCY)
//   của DUT bị khóa cứng lúc customize IP. Sửa localparam trong TB này chỉ
//   đổi số TB dùng để tính IN_DEPTH/OUT_DEPTH và log - KHÔNG tự đổi DUT.
//   Phải re-customize `ip_axi_softmax_0` trong IP Catalog với cùng tham số
//   rồi mới chạy sim, nếu không TB sẽ pass giả hoặc treo do lệch kích cỡ.
//
// GOLDEN DATA - điểm cần bạn tự chuẩn bị trước khi chạy:
//   golden_model.py (mục Q x K^T) chỉ xuất `golden_score.mem` (attention
//   score S, input của softmax) và `exp_rom.mem`/`exp_rom.coe` (ROM LUT).
//   Nó KHÔNG xuất sẵn golden softmax weights (Q1.15). Bạn cần tự thêm một
//   hàm compute_softmax_golden() vào golden_model.py (đã có sẵn hàm
//   compute_softmax() dùng nội bộ để in report - chỉ cần export nó ra
//   file, ví dụ ghi weights_int ra "golden_softmax.mem", 32-bit/word,
//   row-major SEQ_LEN x D_HEAD, cùng định dạng %08h như golden_score.mem)
//   trước khi trỏ MEM_GOLDEN bên dưới vào file đó.
//==============================================================================

import axi_vip_pkg::*;
import axi4stream_vip_pkg::*;

import axi_vip_0_pkg::*;
import axi4stream_vip_0_pkg::*;
import axi4stream_vip_1_pkg::*;

module tb_ip_axi_softmax;

    // ------------------------------------------------------------------
    // >>> SỬA SỐ NÀY để đổi test case <<<
    // Phải khớp với cấu hình thực tế của ip_axi_softmax_0 (re-customize IP
    // trong Vivado nếu đổi các số này) - xem cảnh báo ở đầu file.
    // ------------------------------------------------------------------
    localparam int D_HEAD      = 16;
    localparam int SEQ_LEN     = 16;
    localparam int DATA_WIDTH  = 16;
    localparam int EXP_WIDTH   = 16;

    // synthesis translate_off
    initial begin
        $display("[TB CONFIG] D_HEAD=%0d SEQ_LEN=%0d DATA_WIDTH=%0d EXP_WIDTH=%0d",
                  D_HEAD, SEQ_LEN, DATA_WIDTH, EXP_WIDTH);
    end
    // synthesis translate_on

    localparam int IN_DEPTH  = SEQ_LEN * D_HEAD;   // S input words  (= 256 for 16x16)
    localparam int OUT_DEPTH = SEQ_LEN * D_HEAD;   // softmax output words, same shape

    localparam logic [31:0] S00_CTRL   = 32'h00;
    localparam logic [31:0] S00_STATUS = 32'h04;
    localparam int  TIMEOUT_CYCLES     = 500_000;

    // golden_score.mem tái dùng từ ip_axi_linear golden model (làm input S
    // cho softmax); golden_softmax.mem cần bạn tự thêm export - xem note
    // ở đầu file.
    localparam string MEM_S       = "E:/DOWNLOAD/HCMUT/TTKS/src/mem files/golden model/golden_score.mem";
    localparam string MEM_GOLDEN  = "E:/DOWNLOAD/HCMUT/TTKS/src/mem files/golden model/golden_softmax.mem";

    // Output directory for RTL simulation results
    localparam string RTL_OUT_DIR = "E:/DOWNLOAD/HCMUT/TTKS/src/mem files/rtl/";

    logic clk;
    logic resetn;

    // ================================================================
    //  Phase timing variables (unit: ns, from $time with timescale 1ns/1ps)
    // ================================================================
    longint t_sim_start;
    longint t_reset_done;
    longint t_start_cmd;
    longint t_s_stream_start;
    longint t_s_stream_done;
    longint t_capture_done;

    // ================================================================
    //  PASS/FAIL tracking (programmatic - not dependent on grepping log)
    // ================================================================
    int  x_err_cnt;         // count of TVALID=1 & TDATA=X events on M_AXIS
    int  compare_err_cnt;   // mismatches vs golden, set by compare_output()
    bit  sim_done;          // guards watchdog after normal completion
    int  cyc_cnt;           // free-running cycle counter for watchdog

    // ================================================================
    //  Signal declarations
    // ================================================================

    // ── AXI Lite ─────────────────────────────
    logic [3:0]  s00_axi_awaddr;
    logic [2:0]  s00_axi_awprot;
    logic        s00_axi_awvalid;
    logic        s00_axi_awready;
    logic [31:0] s00_axi_wdata;
    logic [3:0]  s00_axi_wstrb;
    logic        s00_axi_wvalid;
    logic        s00_axi_wready;
    logic [1:0]  s00_axi_bresp;
    logic        s00_axi_bvalid;
    logic        s00_axi_bready;
    logic [3:0]  s00_axi_araddr;
    logic [2:0]  s00_axi_arprot;
    logic        s00_axi_arvalid;
    logic        s00_axi_arready;
    logic [31:0] s00_axi_rdata;
    logic [1:0]  s00_axi_rresp;
    logic        s00_axi_rvalid;
    logic        s00_axi_rready;

    // ── AXIS Master (VIP -> IP), drives S rows ────
    logic [31:0] s00_axis_tdata;
    logic        s00_axis_tvalid;
    logic        s00_axis_tready;
    logic        s00_axis_tlast;

    // ── AXIS Slave (IP -> VIP), captures softmax weights ──
    logic [31:0] m00_axis_tdata;
    logic        m00_axis_tvalid;
    logic        m00_axis_tready;
    logic        m00_axis_tlast;

    // ================================================================
    //  Arrays (Global to avoid pass-by-reference errors)
    // ================================================================
    logic [31:0] mem_s        [0:IN_DEPTH-1];
    logic [31:0] captured_out [0:OUT_DEPTH-1];
    logic [31:0] golden_out   [0:OUT_DEPTH-1];

    // ================================================================
    //  VIP Agents
    // ================================================================
    axi_vip_0_mst_t        mst_lite;
    axi4stream_vip_0_mst_t mst_axis;
    axi4stream_vip_1_slv_t slv_axis;

    // ================================================================
    //  Instantiations
    // ================================================================
    axi_vip_0 u_axilite_mst (
        .aclk            (clk),
        .aresetn         (resetn),
        .m_axi_awaddr    (s00_axi_awaddr),
        .m_axi_awprot    (s00_axi_awprot),
        .m_axi_awvalid   (s00_axi_awvalid),
        .m_axi_awready   (s00_axi_awready),
        .m_axi_wdata     (s00_axi_wdata),
        .m_axi_wstrb     (s00_axi_wstrb),
        .m_axi_wvalid    (s00_axi_wvalid),
        .m_axi_wready    (s00_axi_wready),
        .m_axi_bresp     (s00_axi_bresp),
        .m_axi_bvalid    (s00_axi_bvalid),
        .m_axi_bready    (s00_axi_bready),
        .m_axi_araddr    (s00_axi_araddr),
        .m_axi_arprot    (s00_axi_arprot),
        .m_axi_arvalid   (s00_axi_arvalid),
        .m_axi_arready   (s00_axi_arready),
        .m_axi_rdata     (s00_axi_rdata),
        .m_axi_rresp     (s00_axi_rresp),
        .m_axi_rvalid    (s00_axi_rvalid),
        .m_axi_rready    (s00_axi_rready)
    );

    axi4stream_vip_0 u_axis_mst (
        .aclk            (clk),
        .aresetn         (resetn),
        .m_axis_tdata    (s00_axis_tdata),
        .m_axis_tvalid   (s00_axis_tvalid),
        .m_axis_tready   (s00_axis_tready),
        .m_axis_tlast    (s00_axis_tlast)
    );

    axi4stream_vip_1 u_axis_slv (
        .aclk            (clk),
        .aresetn         (resetn),
        .s_axis_tdata    (m00_axis_tdata),
        .s_axis_tvalid   (m00_axis_tvalid),
        .s_axis_tready   (m00_axis_tready),
        .s_axis_tlast    (m00_axis_tlast)
    );

    ip_axi_softmax_0 u_dut (
        .s00_axi_aclk    (clk),
        .s00_axi_aresetn (resetn),
        .s00_axi_awaddr  (s00_axi_awaddr),
        .s00_axi_awprot  (s00_axi_awprot),
        .s00_axi_awvalid (s00_axi_awvalid),
        .s00_axi_awready (s00_axi_awready),
        .s00_axi_wdata   (s00_axi_wdata),
        .s00_axi_wstrb   (s00_axi_wstrb),
        .s00_axi_wvalid  (s00_axi_wvalid),
        .s00_axi_wready  (s00_axi_wready),
        .s00_axi_bresp   (s00_axi_bresp),
        .s00_axi_bvalid  (s00_axi_bvalid),
        .s00_axi_bready  (s00_axi_bready),
        .s00_axi_araddr  (s00_axi_araddr),
        .s00_axi_arprot  (s00_axi_arprot),
        .s00_axi_arvalid (s00_axi_arvalid),
        .s00_axi_arready (s00_axi_arready),
        .s00_axi_rdata   (s00_axi_rdata),
        .s00_axi_rresp   (s00_axi_rresp),
        .s00_axi_rvalid  (s00_axi_rvalid),
        .s00_axi_rready  (s00_axi_rready),

        .s00_axis_tdata  (s00_axis_tdata),
        .s00_axis_tvalid (s00_axis_tvalid),
        .s00_axis_tready (s00_axis_tready),
        .s00_axis_tlast  (s00_axis_tlast),

        .m00_axis_tdata  (m00_axis_tdata),
        .m00_axis_tvalid (m00_axis_tvalid),
        .m00_axis_tready (m00_axis_tready),
        .m00_axis_tlast  (m00_axis_tlast)
    );

    // ================================================================
    //  Explicit M_AXIS TDATA-X monitor
    //  (giống tb_ip_axi_linear.sv - kiểm tra độc lập với VIP protocol
    //  checker, cho phép ra PASS/FAIL bằng code thay vì đọc log tay)
    // ================================================================
    always @(posedge clk) begin
        if (resetn && m00_axis_tvalid) begin
            if ($isunknown(m00_axis_tdata)) begin
                x_err_cnt++;
                $display("[ERROR][TB-XCHECK] t=%0t  M_AXIS TVALID=1 but TDATA contains X (0x%08h)",
                          $time, m00_axis_tdata);
            end
        end
    end

    // ================================================================
    //  Watchdog - catches deadlock (e.g. div_gen latency mismatch causing
    //  div_out_idx to never reach D_HEAD-1, or a tready/tvalid handshake
    //  bug that never asserts)
    // ================================================================
    always @(posedge clk) begin
        if (!resetn) begin
            cyc_cnt <= 0;
        end else begin
            cyc_cnt <= cyc_cnt + 1;
            if (!sim_done && (cyc_cnt > TIMEOUT_CYCLES)) begin
                $display("[FATAL] t=%0t  WATCHDOG TIMEOUT: exceeded %0d cycles without completing capture (possible deadlock)",
                          $time, TIMEOUT_CYCLES);
                $fatal(1, "[TB] watchdog timeout - simulation did not complete");
            end
        end
    end

    // ================================================================
    //  Display Helpers
    // ================================================================

    // Print a repeated-character separator line
    task automatic print_sep(input string ch, input int width);
        string line;
        line = "";
        for (int i = 0; i < width; i++) line = {line, ch};
        $display("%s", line);
    endtask

    // Format nanosecond value: auto-scale to ns / us / ms
    function automatic string fmt_ns(input longint t_ns);
        real v;
        if (t_ns == 0)
            return "0 ns";
        else if (t_ns < 1_000) begin
            return $sformatf("%0d ns", t_ns);
        end else if (t_ns < 1_000_000) begin
            v = real'(t_ns) / 1_000.0;
            return $sformatf("%.3f us  (%0d ns)", v, t_ns);
        end else begin
            v = real'(t_ns) / 1_000_000.0;
            return $sformatf("%.3f ms  (%0d ns)", v, t_ns);
        end
    endfunction

    // ================================================================
    //  Phase Timing Report
    // ================================================================
    task automatic print_timing_report();
        longint ph_reset, ph_s, ph_total_hw, ph_sim_total;

        ph_reset     = t_reset_done    - t_sim_start;
        ph_s         = t_s_stream_done - t_s_stream_start;
        // Hardware total = from START command to last output word captured
        ph_total_hw  = t_capture_done  - t_start_cmd;
        ph_sim_total = t_capture_done  - t_sim_start;

        $display("");
        print_sep("=", 62);
        $display("  PHASE TIMING REPORT");
        print_sep("=", 62);
        $display("  %-30s : %s", "[1] Reset / Init",             fmt_ns(ph_reset));
        $display("  %-30s : %s", "[2] S Stream (ST_LOAD_ROW..)",  fmt_ns(ph_s));
        $display("  %-30s : %s", "[3] Output Capture",            fmt_ns(t_capture_done - t_s_stream_start));
        print_sep("-", 62);
        $display("  %-30s : %s", "HW Total (START -> Done)",  fmt_ns(ph_total_hw));
        $display("  %-30s : %s", "Simulation Total",          fmt_ns(ph_sim_total));
        print_sep("=", 62);
        $display("");
    endtask

    // ================================================================
    //  Tasks: AXI-Lite
    // ================================================================
    task automatic axi_write(input logic [31:0] addr, input logic [31:0] data);
        xil_axi_resp_t resp;
        mst_lite.AXI4LITE_WRITE_BURST(addr, 3'b000, data, resp);
        if (resp !== XIL_AXI_RESP_OKAY)
            $display("[WARN][AXILITE] Write addr=0x%0h  resp=%0d", addr, resp);
    endtask

    task automatic axi_read(input logic [31:0] addr, output logic [31:0] rdata);
        xil_axi_resp_t resp;
        mst_lite.AXI4LITE_READ_BURST(addr, 3'b000, rdata, resp);
        if (resp !== XIL_AXI_RESP_OKAY)
            $display("[WARN][AXILITE] Read  addr=0x%0h  resp=%0d", addr, resp);
    endtask

    // ================================================================
    //  Tasks: AXI-Stream
    //  softmax.sv chỉ có 1 stream vào (S) - không có TDEST routing như
    //  linear (K/Q). tlast được đặt ở phần tử cuối MỖI ROW (D_HEAD-1),
    //  khớp đúng cách ST_LOAD_ROW của softmax.sv nhận diện hết 1 row
    //  (row_load_done khi i_s_axis_tlast hoặc đủ D_HEAD beat).
    // ================================================================
    task automatic stream_s();
        axi4stream_transaction trans;
        for (int i = 0; i < IN_DEPTH; i++) begin
            trans = mst_axis.driver.create_transaction("s_trans");
            trans.set_data_beat(mem_s[i]);
            trans.set_last(((i % D_HEAD) == D_HEAD - 1) ? 1 : 0);
            mst_axis.driver.send(trans);
        end
    endtask

    task automatic set_axis_slave_ready();
        axi4stream_ready_gen ready_gen;
        ready_gen = slv_axis.driver.create_ready("ready_gen");
        ready_gen.set_ready_policy(XIL_AXI4STREAM_READY_GEN_NO_BACKPRESSURE);
        slv_axis.driver.send_tready(ready_gen);
    endtask

    task automatic capture_output();
        axi4stream_monitor_transaction trans;
        int word_idx = 0;
        while (word_idx < OUT_DEPTH) begin
            slv_axis.monitor.item_collected_port.get(trans);
            captured_out[word_idx] = trans.get_data_beat();
            word_idx++;
        end
    endtask

    // ================================================================
    //  compare_output - formatted verification table
    //  So sánh Q1.15 unsigned - mismatch được báo tuyệt đối (delta theo
    //  LSB), không dùng $signed vì output softmax luôn unsigned [0,1).
    // ================================================================
    localparam int FAIL_SHOW_MAX = 15;   // max detail rows shown on mismatch

    task automatic compare_output();
        int err_cnt   = 0;
        int shown_cnt = 0;
        int show_more = 0;

        // ── Header ──────────────────────────────────────────────────
        $display("");
        print_sep("=", 70);
        $display("  VERIFICATION RESULTS   [OUT_DEPTH = %0d]", OUT_DEPTH);
        print_sep("=", 70);

        // ── Per-entry check ─────────────────────────────────────────
        for (int i = 0; i < OUT_DEPTH; i++) begin
            if (captured_out[i] !== golden_out[i]) begin
                err_cnt++;
                if (shown_cnt < FAIL_SHOW_MAX) begin
                    $display("  [FAIL] idx=%4d  |  expected=0x%08h  |  got=0x%08h  |  delta=%0d",
                             i,
                             golden_out[i],
                             captured_out[i],
                             int'(captured_out[i]) - int'(golden_out[i]));
                    shown_cnt++;
                end else if (shown_cnt == FAIL_SHOW_MAX) begin
                    show_more = 1;
                    shown_cnt++;   // advance past threshold so this branch runs once
                end
            end
        end

        if (show_more)
            $display("  ... (only first %0d mismatches shown above)", FAIL_SHOW_MAX);

        // ── Summary ─────────────────────────────────────────────────
        print_sep("-", 70);
        if (err_cnt == 0) begin
            $display("  STATUS  :  *** PASS ***");
            $display("  RESULT  :  All %0d outputs match golden model.", OUT_DEPTH);
        end else begin
            $display("  STATUS  :  *** FAIL ***");
            $display("  ERRORS  :  %0d / %0d mismatches  (%.2f%%)",
                     err_cnt, OUT_DEPTH,
                     (real'(err_cnt) / real'(OUT_DEPTH)) * 100.0);
            $display("  CORRECT :  %0d / %0d outputs OK  (%.2f%%)",
                     OUT_DEPTH - err_cnt, OUT_DEPTH,
                     (real'(OUT_DEPTH - err_cnt) / real'(OUT_DEPTH)) * 100.0);
        end
        print_sep("=", 70);
        $display("");

        compare_err_cnt = err_cnt;
    endtask

    // ================================================================
    //  write_rtl_mem_files
    //  Writes two files to RTL_OUT_DIR:
    //    output_rtl.mem   - raw hex values of every captured output word
    //    compare_rtl.mem  - side-by-side RTL vs golden with PASS/FAIL tag
    // ================================================================
    task automatic write_rtl_mem_files();
        int  fd;
        int  err_cnt = 0;
        string fpath;

        // ── 1. output_rtl.mem ───────────────────────────────────────
        fpath = {RTL_OUT_DIR, "output_rtl_softmax.mem"};
        fd = $fopen(fpath, "w");
        if (fd == 0) begin
            $display("[WARN] Cannot create file: %s", fpath);
            $display("[WARN] Ensure directory exists: %s", RTL_OUT_DIR);
        end else begin
            $fdisplay(fd, "// RTL Simulation Output");
            $fdisplay(fd, "// Design : ip_axi_softmax  (row-wise softmax)");
            $fdisplay(fd, "// Params : D_HEAD=%0d  SEQ_LEN=%0d  DATA_WIDTH=%0d  EXP_WIDTH=%0d",
                      D_HEAD, SEQ_LEN, DATA_WIDTH, EXP_WIDTH);
            $fdisplay(fd, "// Words  : %0d  (SEQ_LEN x D_HEAD)", OUT_DEPTH);
            $fdisplay(fd, "// Time   : sim=%0t", $time);
            $fdisplay(fd, "//");
            for (int i = 0; i < OUT_DEPTH; i++)
                $fdisplay(fd, "%08h", captured_out[i]);
            $fclose(fd);
            $display("[FILE] output_rtl_softmax.mem -> %s", fpath);
        end

        // ── 2. compare_rtl.mem ──────────────────────────────────────
        fpath = {RTL_OUT_DIR, "compare_rtl_softmax.mem"};
        fd = $fopen(fpath, "w");
        if (fd == 0) begin
            $display("[WARN] Cannot create file: %s", fpath);
        end else begin
            $fdisplay(fd, "// RTL vs Golden Comparison");
            $fdisplay(fd, "// Columns: idx | rtl_output | golden | status");
            $fdisplay(fd, "// Params : D_HEAD=%0d  SEQ_LEN=%0d  DATA_WIDTH=%0d  EXP_WIDTH=%0d",
                      D_HEAD, SEQ_LEN, DATA_WIDTH, EXP_WIDTH);
            $fdisplay(fd, "// Time   : sim=%0t", $time);
            $fdisplay(fd, "//");
            $fdisplay(fd, "// idx    rtl_out  golden   status");
            $fdisplay(fd, "// ----  --------  --------  ------");
            for (int i = 0; i < OUT_DEPTH; i++) begin
                if (captured_out[i] === golden_out[i]) begin
                    $fdisplay(fd, "  %04d  %08h  %08h  PASS",
                              i, captured_out[i], golden_out[i]);
                end else begin
                    $fdisplay(fd, "  %04d  %08h  %08h  FAIL  (delta=%0d)",
                              i, captured_out[i], golden_out[i],
                              int'(captured_out[i]) - int'(golden_out[i]));
                    err_cnt++;
                end
            end
            $fdisplay(fd, "//");
            $fdisplay(fd, "// Summary: %0d PASS  /  %0d FAIL  /  %0d total",
                      OUT_DEPTH - err_cnt, err_cnt, OUT_DEPTH);
            $fclose(fd);
            $display("[FILE] compare_rtl_softmax.mem -> %s", fpath);
        end
    endtask

    // ================================================================
    //  Main Execution
    // ================================================================
    initial begin
        t_sim_start = $time;

        clk    = 1'b0;
        resetn = 1'b0;

        $readmemh(MEM_S,      mem_s);
        $readmemh(MEM_GOLDEN, golden_out);

        @(posedge clk);

        mst_lite = new("mst_lite", tb_ip_axi_softmax.u_axilite_mst.inst.IF);
        mst_axis = new("mst_axis", tb_ip_axi_softmax.u_axis_mst.inst.IF);
        slv_axis = new("slv_axis", tb_ip_axi_softmax.u_axis_slv.inst.IF);

        mst_lite.start_master();
        mst_axis.start_master();
        slv_axis.start_slave();

        #250;
        resetn = 1'b1;
        #50;
        t_reset_done = $time;

        set_axis_slave_ready();

        // Launch output capture in background (parallel with streaming)
        fork
            capture_output();
        join_none

        // ── Phase: write START ─────────────────────────────────────
        $display("[INFO] t=%0t  Writing START to S00_CTRL", $time);
        axi_write(S00_CTRL, 32'h1);
        t_start_cmd = $time;

        // ── Phase: S Stream ───────────────────────────────────────
        $display("[INFO] t=%0t  Streaming S  (%0d words, %0d x %0d)",
                 $time, IN_DEPTH, SEQ_LEN, D_HEAD);
        t_s_stream_start = $time;
        stream_s();
        t_s_stream_done  = $time;
        $display("[INFO] t=%0t  S stream complete", $time);

        // ── Wait for all OUT_DEPTH output words ───────────────────
        wait fork;
        t_capture_done = $time;
        $display("[INFO] t=%0t  Output capture complete (%0d words)", $time, OUT_DEPTH);

        // ── Reports ───────────────────────────────────────────────
        print_timing_report();
        compare_output();
        write_rtl_mem_files();

        // ── Final verdict (programmatic, matches HANDOFF pass criteria) ──
        sim_done = 1'b1;
        $display("");
        print_sep("#", 70);
        $display("  TEST CASE : D_HEAD=%0d SEQ_LEN=%0d DATA_WIDTH=%0d EXP_WIDTH=%0d",
                  D_HEAD, SEQ_LEN, DATA_WIDTH, EXP_WIDTH);
        $display("  Condition 1 (no AXI4STREAM_ERRM_TDATA_X on M_AXIS) : %s  (x_err_cnt=%0d)",
                  (x_err_cnt == 0) ? "PASS" : "FAIL", x_err_cnt);
        $display("  Condition 2 (%0d/%0d output words match golden)    : %s  (mismatches=%0d)",
                  OUT_DEPTH, OUT_DEPTH, (compare_err_cnt == 0) ? "PASS" : "FAIL", compare_err_cnt);
        if ((x_err_cnt == 0) && (compare_err_cnt == 0))
            $display("  OVERALL   : *** TEST PASSED ***");
        else
            $display("  OVERALL   : *** TEST FAILED ***");
        print_sep("#", 70);
        $display("");

        $finish;
    end
    
    // synthesis translate_on
    always #5 clk = ~clk;

endmodule