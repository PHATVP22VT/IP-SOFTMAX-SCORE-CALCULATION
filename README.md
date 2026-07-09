# IP `ip_axi_softmax` — Row-wise Softmax Accelerator

## Mục lục

1. [Tổng quan](#1-tổng-quan)
2. [Tham số (Customize IP)](#2-tham-số-customize-ip)
3. [Register map](#3-register-map-axi4-lite-c_s_axi_addr_width--4-tức-4-thanh-ghi-32-bit)
4. [Interface](#4-interface)
5. [Datapath — FSM 8 state](#5-datapath--fsm-8-state)
6. [Phép chia — `reciprocal_divider.sv`](#6-phép-chia--reciprocal_dividersv)
7. [ROM phụ trợ (`.coe`)](#7-rom-phụ-trợ-coe)
8. [Kiểm chứng](#8-kiểm-chứng-tb_ip_axi_softmaxsv)
9. [Bug đã sửa](#9-bug-đã-sửa-ghi-chú-lịch-sử-tránh-lặp-lại)
10. [Cấu trúc file](#10-cấu-trúc-file)
11. [Flow tính toán — sơ đồ luồng dữ liệu tổng thể](#11-flow-tính-toán--sơ-đồ-luồng-dữ-liệu-tổng-thể)
12. [FSM — trình tự state và điều kiện chuyển](#12-fsm--trình-tự-state-và-điều-kiện-chuyển)
13. [Chi tiết từng giai đoạn (per-cycle behavior)](#13-chi-tiết-từng-giai-đoạn-per-cycle-behavior)
14. [Tổng thời gian tính toán 1 hàng](#14-tổng-thời-gian-tính-toán-1-hàng-không-tính-stall-do-back-pressure)
15. [Bảng dependency giữa submodule](#15-bảng-dependency-giữa-submodule)
16. [Điểm cần lưu ý khi thay đổi tham số](#16-điểm-cần-lưu-ý-khi-thay-đổi-tham-số)

---

## 1. Tổng quan

Tính `Softmax(S)` theo từng hàng cho ma trận `S : [SEQ_LEN x D_HEAD]`, dùng trong pipeline attention (đầu vào là attention score, đầu ra là attention weight).

- Input: `SEQ_LEN` hàng, mỗi hàng `D_HEAD` phần tử, đưa vào qua AXI4-Stream slave, tuần tự từng beat.
- Output: cùng shape, mỗi phần tử là **Q1.15 unsigned fraction** (giá trị trong `[0, 1)`), đưa ra qua AXI4-Stream master.
- Điều khiển qua AXI4-Lite (start/status).

Kiến trúc gồm 2 module RTL:
- `softmax.sv` — FSM điều khiển + datapath (row buffer, tìm max, tính exp, cộng dồn, chia).
- `reciprocal_divider.sv` — phép chia bằng reciprocal-LUT + nhân, thay cho Xilinx Divider Generator (`div_gen`).

Đóng gói IP: `ip_axi_softmax.v` (top wrapper) + `ip_axi_softmax_slave_lite_v1_0_S00_AXI.v` (AXI4-Lite slave, sinh từ template chuẩn Vivado).

## 2. Tham số (Customize IP)

| Parameter | Mặc định | Ý nghĩa |
|---|---|---|
| `D_HEAD` | 64 | Số phần tử mỗi hàng (chiều rộng softmax) |
| `SEQ_LEN` | 64 | Số hàng |
| `DATA_WIDTH` | 16 | Bề rộng input `S` (signed) |
| `EXP_WIDTH` | 16 | Bề rộng output `exp_rom` / kết quả cuối (unsigned) |
| `RECIP_ADDR_W` | 12 | Bề rộng địa chỉ ROM reciprocal (mantissa bits). **Không phụ thuộc `D_HEAD`/`SEQ_LEN`**, không cần đổi khi resize bài toán |
| `RECIP_OUT_W` | 19 | Bề rộng output ROM reciprocal (Q0.RECIP_OUT_W unsigned) |

**Đã test pass khi thay đổi `SEQ_LEN` và `D_HEAD`** (kiểm chứng qua `tb_ip_axi_softmax.sv`, golden model so khớp 100%).

Ràng buộc bắt buộc giữa tham số (assertion tại `initial` block, synthesis translate_off — chỉ check trong sim):
- `D_HEAD >= 1`, `SEQ_LEN >= 1`
- `RECIP_OUT_W > EXP_WIDTH`

## 3. Register map (AXI4-Lite, `C_S_AXI_ADDR_WIDTH = 4`, tức 4 thanh ghi 32-bit)

| Offset | Tên | R/W | Mô tả |
|---|---|---|---|
| `0x00` | CTRL (slv_reg0) | R/W | Bit[0] = `start_softmax`. Ghi `1` để trigger. Tự clear về `0` bằng phần cứng khi `busy` chuyển `0→1` (`busy_rising`), miễn là không có write AXI đang xảy ra cùng cycle (ưu tiên write AXI) |
| `0x04` | STATUS (read-only, không có storage riêng) | R | Bit[0] = `latched_done` (set khi `i_softmax_done` pulse, tự clear khi ghi lại `start`), Bit[1] = `i_busy` (tổ hợp trực tiếp từ IP, không latch) |
| `0x08` | slv_reg2 | R/W | Không dùng (dự phòng) |
| `0x0C` | slv_reg3 | R/W | Không dùng (dự phòng) |

**Trình tự điều khiển từ software:**
1. Ghi `0x1` vào offset `0x00` để start.
2. Stream `SEQ_LEN x D_HEAD` word input `S` qua DMA vào `S00_AXIS` (source: MM2S DMA channel).
3. Đợi output đủ `SEQ_LEN x D_HEAD` word qua `M00_AXIS` (destination: S2MM DMA channel), hoặc poll bit DONE tại offset `0x04`.
4. Đọc `STATUS` để xác nhận `done=1` trước khi coi phép tính đã hoàn tất.

## 4. Interface

### AXI4-Lite Slave (`S00_AXI`)
Chuẩn, `C_S_AXI_DATA_WIDTH=32`, `C_S_AXI_ADDR_WIDTH=4`. FSM write/read theo đúng template mẫu Vivado, không có customization đặc biệt.

### AXI4-Stream Slave (`S00_AXIS`) — input
`tdata[31:0]`, `tvalid`, `tlast`, `tready`. `tready` chỉ được assert trong state `ST_LOAD_ROW` — các state xử lý nội bộ khác (`FIND_MAX`/`EXP_SUM`/`DIV_*`) đều `tready=0`, tạo back-pressure lên DMA nguồn.

### AXI4-Stream Master (`M00_AXIS`) — output
`tdata[31:0]` (Q1.15 unsigned trong 16 LSB, phần cao = 0), `tvalid`, `tlast`, `tready` (input từ DMA đích).

## 5. Datapath — FSM 8 state

```
IDLE → LOAD_ROW → FIND_MAX → EXP_SUM → DIV_ISSUE → DIV_DRAIN → SERIALIZE → (loop LOAD_ROW hoặc DONE)
```

| State | Chức năng |
|---|---|
| `ST_LOAD_ROW` | Nhận `D_HEAD` beat từ `S00_AXIS`, nạp vào `s_row_buf` (register array, distributed RAM) |
| `ST_FIND_MAX` | Duyệt tuần tự, mỗi cycle 1 so sánh, tìm `max_val` của hàng (dùng cho ổn định số học của softmax, `z = x - max <= 0`) |
| `ST_EXP_SUM` | Pipeline 2 tầng: tính địa chỉ `exp_rom` tổ hợp từ `z_val`, đọc `exp_rom` (1-cycle latency), cộng dồn vào `sum_acc` |
| `ST_DIV_ISSUE` | Đẩy lần lượt `D_HEAD` cặp (dividend, divisor) vào `reciprocal_divider` |
| `ST_DIV_DRAIN` | Đợi các kết quả pipeline còn lại trả về (latency cố định 3 cycle, in-order) |
| `ST_SERIALIZE` | Xuất `out_row_buf` ra `M00_AXIS`, lặp lại `LOAD_ROW` cho hàng kế hoặc sang `DONE` nếu là hàng cuối |

**`SUM_WIDTH`** tổng quát hóa theo `D_HEAD`: `SUM_WIDTH = clog2(D_HEAD) + EXP_WIDTH` — không hardcode 24-bit như bản gốc (chỉ đúng cho `SEQ_LEN=3`).

## 6. Phép chia — `reciprocal_divider.sv`

Thay Xilinx `div_gen` bằng phương pháp **range-reduction reciprocal**:

1. Tìm vị trí bit MSB=1 cao nhất của divisor (`sum`) bằng priority-encoder tổ hợp → `divisor = mantissa × 2^msb_pos`, mantissa ∈ [1.0, 2.0).
2. Lấy `ADDR_W` bit ngay dưới MSB làm địa chỉ ROM.
3. ROM (`recip_rom`, Block Memory Generator, nội dung từ `recip_rom.coe`) trả về `reciprocal(mantissa)` dạng Q0.OUT_W.
4. `weight = (dividend × recip_lut[addr]) >> (OUT_W + msb_pos - 15)`.

**Latency: 3 cycle cố định** (find-MSB → ROM read → multiply+shift), so với 55 cycle của `div_gen` cũ. Không cần FIFO in-order riêng vì latency ngắn và trả kết quả tuần tự.

**Độ chính xác:** sai số tối đa đo được là **1 LSB trên thang Q1.15**, verify bằng `recip_lut_check.py` trên 200k+ mẫu ngẫu nhiên, `D_HEAD ∈ {16, 64, 128}`.

## 7. ROM phụ trợ (`.coe`)

| ROM | File nguồn | Kích thước | Nội dung | Sinh bởi |
|---|---|---|---|---|
| `exp_rom` | `exp_rom.coe` | 2048 entry × `EXP_WIDTH` bit | LUT của `exp(-z)`, `z = x - max >= 0` (đã đảo dấu), địa chỉ = `(-(x-max)) & 0x7FF` | `golden_model.py` (tái sử dụng từ `attention_top.sv` cũ) |
| `recip_rom` | `recip_rom.coe` | 4096 entry (`2^RECIP_ADDR_W`) × `RECIP_OUT_W` bit | LUT của `reciprocal(mantissa)`, Q0.OUT_W unsigned | `golden_model.py`, hàm `generate_recip_lut()` / `write_coe_generic()` |

Cả hai ROM đều là **Vivado Block Memory Generator IP** (không phải RTL hành vi), **synchronous read, 1-cycle latency** (Port A: Primitives Output Register ON, Core Output Register OFF — phải xác nhận trong Summary tab của IP customize dialog trước khi generate).

**Nếu đổi `RECIP_ADDR_W` hoặc `RECIP_OUT_W`:** phải re-customize `recip_rom` (Width/Depth) và chạy lại `golden_model.py` để sinh lại `recip_rom.coe` khớp kích thước mới. `RECIP_ADDR_W`/`RECIP_OUT_W` **không phụ thuộc** `D_HEAD`/`SEQ_LEN`, nên không cần đổi khi chỉ resize bài toán.

## 8. Kiểm chứng (`tb_ip_axi_softmax.sv`)

- Cấu trúc giống `tb_ip_axi_linear.sv`: 1 AXI-Lite master VIP (control) + 1 AXI4-Stream master VIP (input `S`) + 1 AXI4-Stream slave VIP (capture output).
- So khớp từng word với golden model (`golden_softmax.mem`, xuất từ `golden_model.py`, hàm `compute_softmax()` tự viết thêm export ra file, 32-bit/word, row-major, cùng định dạng `%08h` với `golden_score.mem`).
- 2 tiêu chí PASS độc lập, kiểm tra bằng biến đếm chương trình (không phụ thuộc grep log):
  1. Không có sự kiện `TVALID=1 & TDATA=X` trên `M_AXIS` (`x_err_cnt == 0`).
  2. 100% output word khớp golden (`compare_err_cnt == 0`).
- Xuất file debug: `output_rtl_softmax.mem` (raw output) và `compare_rtl_softmax.mem` (so sánh từng dòng RTL vs golden, có tag PASS/FAIL).
- **Cảnh báo quan trọng trong TB:** `localparam` trong testbench (D_HEAD/SEQ_LEN/DATA_WIDTH/EXP_WIDTH) chỉ dùng để TB tự tính `IN_DEPTH`/`OUT_DEPTH` và log — **không** tự động đổi tham số của DUT (`ip_axi_softmax_0` đã đóng gói IP, tham số khóa cứng lúc Customize IP). Phải re-customize IP trong Vivado IP Catalog cho khớp trước khi chạy sim, nếu không TB sẽ pass giả hoặc treo do lệch kích cỡ.

**Test case đã pass:** `SEQ_LEN`, `D_HEAD` đã thay đổi và chạy lại — verify OK, không cần sửa thêm gì trong RTL.

## 9. Bug đã sửa (ghi chú lịch sử, tránh lặp lại)

- **Bug shift trong `reciprocal_divider`:** ban đầu trừ nhầm thêm 15 lần nữa dù `dividend` đầu vào đã pre-shift (`exp_val << 15`), khiến kết quả lớn hơn đúng `2^15` lần. Công thức đúng:
  ```
  weight = i_dividend * recip_q >> (OUT_W + msb_pos)
  ```
  (không trừ 15, vì phần `-15` đã tự triệt tiêu với `<<15` có sẵn trong `i_dividend`). Phát hiện qua `tb_recip_isolated.sv` (test cô lập riêng module `reciprocal_divider`, không qua toàn bộ `softmax`).

## 10. Cấu trúc file

```
softmax.sv                                     - datapath + FSM (module softmax + reciprocal_divider)
ip_axi_softmax.v                                - top wrapper IP (ghép AXI-Lite slave + softmax datapath)
ip_axi_softmax_slave_lite_v1_0_S00_AXI.v        - AXI4-Lite slave (template chuẩn Vivado)
tb_ip_axi_softmax.sv                            - testbench, dùng AXI VIP
exp_rom.coe                                     - init file cho exp_rom (Block Memory Generator)
recip_rom.coe                                   - init file cho recip_rom (Block Memory Generator)
golden_model.py                                 - sinh golden data + .coe (không đính kèm trong bộ file này)
```

---

## 11. Flow tính toán — sơ đồ luồng dữ liệu tổng thể

```
                         ┌─────────────────────────────────────────────────────┐
                         │                    softmax.sv                       │
                         │                                                     │
 S00_AXIS ──tdata[31:0]──▶ ST_LOAD_ROW                                         │
 (DMA MM2S)   tvalid,tlast │  s_row_buf[0..D_HEAD-1]  (register array)         │
                         │      │                                              │
                         │      ▼                                              │
                         │  ST_FIND_MAX ──▶ max_val (1 so sánh / cycle)        │
                         │      │                                              │
                         │      ▼                                              │
                         │  ST_EXP_SUM                                         │
                         │    z_val = s_row_buf[i] - max_val  (tổ hợp)         │
                         │      │                                              │
                         │      ▼                                              │
                         │   ┌────────────┐                                    │
                         │   │  exp_rom   │ ◀── addr = (-z_val) & 0x7FF        │
                         │   │ (BRAM IP,  │      (1-cycle sync read)           │
                         │   │ 2048 entry)│                                    │
                         │   └─────┬──────┘                                    │
                         │         ▼                                           │
                         │  exp_row_buf[i] = exp_rom_data                      │
                         │  sum_acc += exp_rom_data  ──▶ sum_latched           │
                         │      │                                              │
                         │      ▼                                              │
                         │  ST_DIV_ISSUE / ST_DIV_DRAIN                        │
                         │   ┌─────────────────────┐                           │
                         │   │  reciprocal_divider  │                          │
                         │   │  (3-cycle latency)   │                          │
                         │   │                      │                          │
                         │   │  i_dividend = exp_row_buf[i] << 15              │
                         │   │  i_divisor  = sum_latched (chung cho cả hàng)   │
                         │   │       │                                         │
                         │   │       ▼  Stage 0 (comb): tìm MSB(divisor)       │
                         │   │       ▼  Stage 1 (reg):  đọc recip_rom          │
                         │   │       ▼  Stage 2 (reg):  nhân + dịch phải       │
                         │   │  o_result = Q1.15 unsigned                     │
                         │   └──────────┬───────────┘                          │
                         │              ▼                                      │
                         │   out_row_buf[i] = div_result                       │
                         │      │                                              │
                         │      ▼                                              │
                         │  ST_SERIALIZE                                       │
                         │   xuất out_row_buf tuần tự ra M00_AXIS               │
                         └─────────────┬───────────────────────────────────────┘
                                       │ tdata[31:0] (Q1.15, 16 LSB), tvalid, tlast
                                       ▼
                              M00_AXIS (DMA S2MM)
```

## 12. FSM — trình tự state và điều kiện chuyển

```
        i_start_softmax
IDLE ─────────────────────▶ LOAD_ROW
                               │ row_load_done (đã nhận đủ D_HEAD beat)
                               ▼
                            FIND_MAX
                               │ findmax_done (đã duyệt hết D_HEAD phần tử)
                               ▼
                            EXP_SUM
                               │ expsum_done (ROM đã trả hết D_HEAD giá trị,
                               │              sum_acc đã cộng dồn xong)
                               ▼
                            DIV_ISSUE
                               │ div_in_idx == D_HEAD-1 (đã issue hết D_HEAD cặp)
                               ▼
                            DIV_DRAIN
                               │ div_all_returned (pipeline 3-cycle đã trả hết)
                               ▼
                            SERIALIZE
                               │ row_serialize_done
                        ┌──────┴───────┐
             hàng chưa   │              │  hàng cuối (ser_row_i == SEQ_LEN-1)
             phải cuối   ▼              ▼
                      LOAD_ROW        DONE ──▶ IDLE (1 cycle)
                   (lặp hàng kế)
```

Toàn bộ `SEQ_LEN` hàng xử lý **tuần tự**, không pipeline chồng giữa các hàng (hàng sau chỉ bắt đầu `LOAD_ROW` sau khi hàng trước xong `SERIALIZE`).

## 13. Chi tiết từng giai đoạn (per-cycle behavior)

### 3.1 `ST_LOAD_ROW`
- `o_s_axis_tready = 1` (chỉ state này mới nhận stream input).
- Mỗi cycle có `tvalid`: `s_row_buf[load_col] <= i_s_axis_tdata[DATA_WIDTH-1:0]`, `load_col++`.
- `row_load_done` khi `load_col == D_HEAD-1` và có `tvalid` (hoặc theo `tlast`, xem RTL để chính xác byte cuối).
- Thời gian: tối thiểu `D_HEAD` cycle (phụ thuộc `tvalid` liên tục từ DMA nguồn; nếu DMA có bubble, kéo dài hơn).

### 3.2 `ST_FIND_MAX`
- Mỗi cycle so sánh `s_row_buf[findmax_idx]` với `max_val` hiện tại, cập nhật nếu lớn hơn, `findmax_idx++`.
- Thuần tổ hợp so sánh + reg cập nhật — 1 phần tử/cycle.
- Thời gian cố định: `D_HEAD` cycle.
- Mục đích: ổn định số học — trừ `max_val` trước khi `exp()` để tránh tràn số khi tính `exp(x)` với `x` lớn.

### 3.3 `ST_EXP_SUM` — pipeline 2 tầng
```
Cycle n:    scan_idx = i          (tổ hợp)
            z_val = s_row_buf[i] - max_val
            exp_rom_addr = (z_val <= 0) ? (-z_val) & 0x7FF : 0

Cycle n+1:  exp_rom trả về exp_rom_data (ứng với addr của cycle n)
            rom_idx_q = i  (latch lại index để biết data này của phần tử nào)
            exp_row_buf[i] <= exp_rom_data
            sum_acc <= sum_acc + exp_rom_data
```
- Do `z = x - max <= 0` luôn đúng theo construction, `exp_rom_addr` luôn hợp lệ (không tràn âm).
- `exp_rom`: **Block Memory Generator, đồng bộ, 1-cycle latency**. Vì vậy `scan_idx` (địa chỉ đang gửi) và `rom_idx_q` (dữ liệu đang nhận) luôn lệch nhau đúng 1 cycle — đây là lý do cần 2 biến index riêng, không dùng chung 1 index cho cả gửi và nhận.
- `sum_acc` tích lũy `D_HEAD` giá trị `exp_rom_data`, độ rộng `SUM_WIDTH = clog2(D_HEAD) + EXP_WIDTH` để không tràn (worst case toàn bộ phần tử đều là `exp_rom` max value).
- Thời gian: `D_HEAD + 1` cycle (1 cycle bù cho độ trễ ROM ở đầu pipeline).
- Khi `expsum_done`: `sum_latched <= sum_acc` (chốt giá trị dùng chung cho toàn bộ phép chia của hàng này).

### 3.4 `ST_DIV_ISSUE` / `ST_DIV_DRAIN`
- `ST_DIV_ISSUE`: mỗi cycle issue 1 cặp `(dividend, divisor)` vào `reciprocal_divider`:
  ```
  div_dividend = exp_row_buf[div_in_idx] << 15   (Q1.15 numerator)
  div_divisor  = sum_latched                     (giống nhau cho cả D_HEAD lần issue)
  ```
  `div_in_idx` tăng dần 0 → D_HEAD-1, `i_tvalid` (= `div_issue_active`) = 1 mỗi cycle.
- Vì `reciprocal_divider` có latency cố định 3-cycle, **in-order** (không cần FIFO), kết quả trả về theo đúng thứ tự issue → `div_out_idx` tăng song song, lệch `div_in_idx` đúng 3 cycle.
- `ST_DIV_DRAIN`: sau khi issue xong `D_HEAD` cặp (chuyển state), đợi 3 cycle cuối cùng của pipeline trả hết kết quả (`div_all_returned`).
- Thời gian tổng: `D_HEAD + 3` cycle.
- Kết quả mỗi lần trả về: `out_row_buf[div_out_idx] <= div_result`.

**Chi tiết pipeline nội bộ `reciprocal_divider` (3 cycle):**
```
Stage 0 (comb, cùng cycle với issue):
    priority-encoder quét divisor[DIVISOR_WIDTH-1:0] từ MSB xuống,
    tìm msb_pos_c = vị trí bit '1' cao nhất.
    recip_addr_c = ADDR_W bit ngay dưới MSB (mantissa).

Stage 1 (reg, +1 cycle):
    latch s1_dividend, s1_msb_pos.
    đọc recip_rom[recip_addr_c] → recip_rom_data (1-cycle sync read, giống exp_rom).

Stage 2 (reg, +1 cycle nữa = tổng 3 cycle kể từ issue):
    prod_c = s1_dividend * recip_rom_data
    shift_total = OUT_W + s1_msb_pos
    shifted_c = prod_c >> shift_total
    o_result <= (shifted_c > MAX_Q15) ? MAX_Q15 : shifted_c[EXP_WIDTH-1:0]
```

### 3.5 `ST_SERIALIZE`
- Xuất `out_row_buf[0..D_HEAD-1]` ra `M00_AXIS` tuần tự, mỗi cycle 1 beat (nếu `i_m_axis_tready=1`; nếu không, `axis_out_stall` giữ nguyên state chờ).
- `tlast` assert ở beat cuối của **mỗi hàng** (`ser_col_j == D_HEAD-1`), không phải chỉ ở cuối toàn bộ frame — cần xác nhận DMA S2MM phía nhận xử lý đúng nhiều `tlast` liên tiếp (mỗi hàng 1 lần) nếu dùng packet mode, hoặc bỏ qua nếu dùng chế độ liên tục.
- Thời gian: `D_HEAD` cycle tối thiểu (dài hơn nếu `tready` phía nhận có bubble).

## 14. Tổng thời gian tính toán 1 hàng (không tính stall do back-pressure)

| Giai đoạn | Cycle |
|---|---|
| LOAD_ROW | D_HEAD |
| FIND_MAX | D_HEAD |
| EXP_SUM | D_HEAD + 1 |
| DIV_ISSUE + DIV_DRAIN | D_HEAD + 3 |
| SERIALIZE | D_HEAD |
| **Tổng / hàng** | **5·D_HEAD + 4** |
| **Tổng toàn bộ (SEQ_LEN hàng, tuần tự)** | **SEQ_LEN · (5·D_HEAD + 4)** |

Ví dụ `D_HEAD=SEQ_LEN=16`: ~16 × 84 = 1344 cycle (chưa tính stall AXI-Stream thực tế).

## 15. Bảng dependency giữa submodule

| Submodule | Vai trò trong flow | Latency | Vị trí trong state |
|---|---|---|---|
| `exp_rom` (BRAM IP) | LUT tính `exp(-z)` | 1 cycle | ST_EXP_SUM |
| `reciprocal_divider` | Tính `1/sum` rồi nhân ra kết quả chia | 3 cycle | ST_DIV_ISSUE/DRAIN |
| ├─ `recip_rom` (BRAM IP) | LUT tính `reciprocal(mantissa)` bên trong `reciprocal_divider` | 1 cycle (nằm trong 3-cycle tổng) | Stage 1 của `reciprocal_divider` |

## 16. Điểm cần lưu ý khi thay đổi tham số

- Đổi `D_HEAD`: ảnh hưởng trực tiếp `SUM_WIDTH`, số cycle mỗi giai đoạn, kích thước `s_row_buf`/`exp_row_buf`/`out_row_buf`. Không cần đổi `RECIP_ADDR_W`/`RECIP_OUT_W`.
- Đổi `EXP_WIDTH`: ảnh hưởng `DIVIDEND_WIDTH = EXP_WIDTH + 15`, `SUM_WIDTH`, và ràng buộc `RECIP_OUT_W > EXP_WIDTH` phải giữ đúng (có assertion check trong sim).
- Đổi `SEQ_LEN`: chỉ ảnh hưởng số vòng lặp `LOAD_ROW→SERIALIZE`, không ảnh hưởng logic bên trong 1 hàng.
