// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 06.10.2017
// Description: Performance counters


module perf_counters
  import ariane_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg  = config_pkg::cva6_cfg_empty,
    parameter int unsigned           NumPorts = 3                            // number of miss ports
) (
    input logic clk_i,
    input logic rst_ni,
    input logic debug_mode_i,  // debug mode
    // SRAM like interface
    input logic [11:0] addr_i,  // read/write address (up to 6 counters possible)
    input logic we_i,  // write enable
    input riscv::xlen_t data_i,  // data to write
    output riscv::xlen_t data_o,  // data to read
    // from commit stage
    input  scoreboard_entry_t [CVA6Cfg.NrCommitPorts-1:0] commit_instr_i,     // the instruction we want to commit
    input  logic [CVA6Cfg.NrCommitPorts-1:0]              commit_ack_i,       // acknowledge that we are indeed committing
    // from L1 caches
    input logic l1_icache_miss_i,
    input logic l1_dcache_miss_i,
    // from MMU
    input logic itlb_miss_i,
    input logic dtlb_miss_i,
    // from issue stage
    input logic sb_full_i,
    // from frontend
    input logic if_empty_i,
    // from PC Gen
    input exception_t ex_i,
    input logic eret_i,
    input bp_resolve_t resolved_branch_i,
    // for newly added events
    input exception_t branch_exceptions_i,  //Branch exceptions->execute unit-> branch_exception_o
    input icache_dreq_t l1_icache_access_i,
    input dcache_req_i_t [2:0] l1_dcache_access_i,
    input  logic [NumPorts-1:0][DCACHE_SET_ASSOC-1:0]miss_vld_bits_i,  //For Cache eviction (3ports-LOAD,STORE,PTW)
    input logic i_tlb_flush_i,
    input logic stall_issue_i,  //stall-read operands
    input  logic [63:0] cycle_count_i,
    input  logic [63:0] instr_count_i,
    input logic [31:0] mcountinhibit_i,
    input  logic [riscv::VLEN-1:0] pc_i,
    output logic ebs_store_req_o,
    input logic ebs_store_ack_i,
    output wt_cache_pkg::dcache_req_t ebs_store_data_o,
    output logic [3:0][4:0] ebs_regfile_addr_o,
    input logic [3:0][riscv::XLEN-1:0] ebs_regfile_data_i,
    output riscv::xlen_t ebs_addr_o,
    input riscv::xlen_t pc_dcache_miss_perf_i
);

  logic [63:0] generic_counter_d[6:1];
  logic [63:0] generic_counter_q[6:1];

  //internal signal to keep track of exception
  logic read_access_exception, update_access_exception;

  logic events[6:1];
  logic [63:0] events_pc_d[8:0], events_pc_q[8:0];
  //internal signal for  MUX select line input
  logic [4:0] mhpmevent_d[6:1];
  logic [4:0] mhpmevent_q[6:1];

  // ----------------------
  // Perf Event-Based Sampling Internal Signals
  // ----------------------

  // registers
  logic [63:0] threshold_d [8:0], threshold_q[8:0];
  logic [63:0] count_offset_d[8:0], count_offset_q[8:0];
  logic [63:0] maddr_addr_d, maddr_addr_q, maddr_offset_d, maddr_offset_q;
  riscv::ebs_sample_cfg_t    ebs_sample_cfg_d, ebs_sample_cfg_q;

  // state-machine control
  riscv::ebs_state_e ebs_state_d, ebs_state_q;
  logic [5:0] ebs_sample_index_d, ebs_sample_index_q;
  logic ebs_sample_trigger;

  // sample data
  assign ebs_regfile_addr_o = {ebs_sample_cfg_q.reg_addr3, ebs_sample_cfg_q.reg_addr2, ebs_sample_cfg_q.reg_addr1, ebs_sample_cfg_q.reg_addr0};

  logic [63:0] ebs_counter_data[8:0];
  logic [63:0] ebs_regfile_data[3:0];

  assign ebs_counter_data = {generic_counter_q[6], generic_counter_q[5], generic_counter_q[4], generic_counter_q[3], generic_counter_q[2], generic_counter_q[1], instr_count_i, 64'b0, cycle_count_i};
  assign ebs_regfile_data = {ebs_regfile_data_i[3], ebs_regfile_data_i[2], ebs_regfile_data_i[1], ebs_regfile_data_i[0]};

  logic [63:0] ebs_pc_sample_d, ebs_pc_sample_q;
  logic [63:0] ebs_counter_data_sample_d[8:0], ebs_counter_data_sample_q[8:0];
  logic [63:0] ebs_regfile_data_sample_d[3:0], ebs_regfile_data_sample_q[3:0];

  assign ebs_addr_o = maddr_addr_q;

  // sampled data encoding to cache
  assign ebs_store_data_o.rtype = wt_cache_pkg::DCACHE_STORE_REQ;
  assign ebs_store_data_o.tid = 1;
  assign ebs_store_data_o.nc = 1'b0;
  assign ebs_store_data_o.way = 'b0;
  assign ebs_store_data_o.data = (ebs_state_q == riscv::IDLE) ? 64'b0 : (ebs_sample_index_q >= 6'd32) ? ebs_regfile_data_sample_q[ebs_sample_index_q - 6'd32] : (ebs_sample_index_q > 6'd9) ? 64'b0 : (ebs_sample_index_q > 6'd0) ? ebs_counter_data_sample_q[ebs_sample_index_q - 6'd1] : ebs_pc_sample_q;
  assign ebs_store_data_o.paddr = maddr_addr_q + maddr_offset_q;
  assign ebs_store_data_o.user = 'b0;
  assign ebs_store_data_o.size = 3'b011;
  assign ebs_store_data_o.amo_op = ariane_pkg::AMO_NONE;
// ------------------------

  //Multiplexer
  always_comb begin : Mux
    events[6:1] = '{default: 0};
    events_pc_d = events_pc_q;

    for (int unsigned i = 1; i <= 6; i++) begin
      case (mhpmevent_q[i])
        5'b00000: events[i] = 0;
        5'b00001: events[i] = l1_icache_miss_i;  //L1 I-Cache misses
        5'b00010: begin events[i] = l1_dcache_miss_i; events_pc_d[i+2] = pc_dcache_miss_perf_i; end  //L1 D-Cache misses
        5'b00011: events[i] = itlb_miss_i;  //ITLB misses
        5'b00100: events[i] = dtlb_miss_i;  //DTLB misses
        5'b00101:
        for (int unsigned j = 0; j < CVA6Cfg.NrCommitPorts; j++)
        if (commit_ack_i[j]) events[i] = commit_instr_i[j].fu == LOAD;  //Load accesses
        5'b00110:
        for (int unsigned j = 0; j < CVA6Cfg.NrCommitPorts; j++)
        if (commit_ack_i[j] && (commit_instr_i[j].fu == STORE)) begin events[i] = 1'b1; events_pc_d[i+2] = commit_instr_i[j].pc; end //Store accesses
        5'b00111: events[i] = ex_i.valid;  //Exceptions
        5'b01000: events[i] = eret_i;  //Exception handler returns
        5'b01001:
        for (int unsigned j = 0; j < CVA6Cfg.NrCommitPorts; j++)
        if (commit_ack_i[j]) events[i] = commit_instr_i[j].fu == CTRL_FLOW;  //Branch instructions
        5'b01010:
        events[i] = resolved_branch_i.valid && resolved_branch_i.is_mispredict;//Branch mispredicts
        5'b01011: events[i] = branch_exceptions_i.valid;  //Branch exceptions
        // The standard software calling convention uses register x1 to hold the return address on a call
        // the unconditional jump is decoded as ADD op
        5'b01100:
        for (int unsigned j = 0; j < CVA6Cfg.NrCommitPorts; j++)
        if (commit_ack_i[j])
          events[i] = commit_instr_i[j].fu == CTRL_FLOW && (commit_instr_i[j].op == ADD || commit_instr_i[j].op == JALR) && (commit_instr_i[j].rd == 'd1 || commit_instr_i[j].rd == 'd5);//Call
        5'b01101:
        for (int unsigned j = 0; j < CVA6Cfg.NrCommitPorts; j++)
        if (commit_ack_i[j])
          events[i] = commit_instr_i[j].op == JALR && commit_instr_i[j].rd == 'd0;  //Return
        5'b01110: events[i] = sb_full_i;  //MSB Full
        5'b01111: events[i] = if_empty_i;  //Instruction fetch Empty
        5'b10000: events[i] = l1_icache_access_i.req;  //L1 I-Cache accesses
        5'b10001:
        events[i] = l1_dcache_access_i[0].data_req || l1_dcache_access_i[1].data_req || l1_dcache_access_i[2].data_req;//L1 D-Cache accesses
        5'b10010:
        events[i] = (l1_dcache_miss_i && miss_vld_bits_i[0] == 8'hFF) || (l1_dcache_miss_i && miss_vld_bits_i[1] == 8'hFF) || (l1_dcache_miss_i && miss_vld_bits_i[2] == 8'hFF);//eviction
        5'b10011: events[i] = i_tlb_flush_i;  //I-TLB flush
        5'b10100:
        for (int unsigned j = 0; j < CVA6Cfg.NrCommitPorts; j++)
        if (commit_ack_i[j])
          events[i] = commit_instr_i[j].fu == ALU || commit_instr_i[j].fu == MULT;//Integer instructions
        5'b10101:
        for (int unsigned j = 0; j < CVA6Cfg.NrCommitPorts; j++)
        if (commit_ack_i[j])
          events[i] = commit_instr_i[j].fu == FPU || commit_instr_i[j].fu == FPU_VEC;//Floating Point Instructions
        5'b10110: events[i] = stall_issue_i;  //Pipeline bubbles
        default: events[i] = 0;
      endcase
    end

  end

  always_comb begin : generic_counter
    generic_counter_d = generic_counter_q;
    data_o = 'b0;
    mhpmevent_d = mhpmevent_q;
    threshold_d = threshold_q;
    maddr_addr_d = maddr_addr_q;
    ebs_sample_cfg_d = ebs_sample_cfg_q;
    read_access_exception = 1'b0;
    update_access_exception = 1'b0;

    // Increment the non-inhibited counters with active events
    for (int unsigned i = 1; i <= 6; i++) begin
      if ((!debug_mode_i) && (!we_i)) begin
        if ((events[i]) == 1 && (!mcountinhibit_i[i+2])) begin
          generic_counter_d[i] = generic_counter_q[i] + 1'b1;
        end
      end
    end

    //Read
    unique case (addr_i)
      riscv::CSR_MHPM_COUNTER_3,
            riscv::CSR_MHPM_COUNTER_4,
            riscv::CSR_MHPM_COUNTER_5,
            riscv::CSR_MHPM_COUNTER_6,
            riscv::CSR_MHPM_COUNTER_7,
            riscv::CSR_MHPM_COUNTER_8  :begin
        if (riscv::XLEN == 32) data_o = generic_counter_q[addr_i-riscv::CSR_MHPM_COUNTER_3+1][31:0];
        else data_o = generic_counter_q[addr_i-riscv::CSR_MHPM_COUNTER_3+1];
      end
      riscv::CSR_MHPM_COUNTER_3H,
            riscv::CSR_MHPM_COUNTER_4H,
            riscv::CSR_MHPM_COUNTER_5H,
            riscv::CSR_MHPM_COUNTER_6H,
            riscv::CSR_MHPM_COUNTER_7H,
            riscv::CSR_MHPM_COUNTER_8H :begin
        if (riscv::XLEN == 32)
          data_o = generic_counter_q[addr_i-riscv::CSR_MHPM_COUNTER_3H+1][63:32];
        else read_access_exception = 1'b1;
      end
      riscv::CSR_MHPM_EVENT_3,
            riscv::CSR_MHPM_EVENT_4,
            riscv::CSR_MHPM_EVENT_5,
            riscv::CSR_MHPM_EVENT_6,
            riscv::CSR_MHPM_EVENT_7,
            riscv::CSR_MHPM_EVENT_8   :
      data_o = mhpmevent_q[addr_i-riscv::CSR_MHPM_EVENT_3+1];
      riscv::CSR_MHPM_THRESHOLD_CYC,
            riscv::CSR_MHPM_THRESHOLD_INSTRET,
            riscv::CSR_MHPM_THRESHOLD_3,
            riscv::CSR_MHPM_THRESHOLD_4,
            riscv::CSR_MHPM_THRESHOLD_5,
            riscv::CSR_MHPM_THRESHOLD_6,
            riscv::CSR_MHPM_THRESHOLD_7,
            riscv::CSR_MHPM_THRESHOLD_8 :begin
        if (riscv::XLEN == 32)
          data_o = threshold_q[addr_i-riscv::CSR_MHPM_THRESHOLD_CYC][31:0];
        else data_o = threshold_q[addr_i-riscv::CSR_MHPM_THRESHOLD_CYC];
      end
      riscv::CSR_MHPM_THRESHOLD_CYCH,
            riscv::CSR_MHPM_THRESHOLD_INSTRETH,
            riscv::CSR_MHPM_THRESHOLD_3H,
            riscv::CSR_MHPM_THRESHOLD_4H,
            riscv::CSR_MHPM_THRESHOLD_5H,
            riscv::CSR_MHPM_THRESHOLD_6H,
            riscv::CSR_MHPM_THRESHOLD_7H,
            riscv::CSR_MHPM_THRESHOLD_8H :begin
        if (riscv::XLEN == 32)
          data_o = threshold_q[addr_i-riscv::CSR_MHPM_THRESHOLD_CYCH][63:32];
        else read_access_exception = 1'b1;
      end
      riscv::CSR_HPM_COUNTER_3,
            riscv::CSR_HPM_COUNTER_4,
            riscv::CSR_HPM_COUNTER_5,
            riscv::CSR_HPM_COUNTER_6,
            riscv::CSR_HPM_COUNTER_7,
            riscv::CSR_HPM_COUNTER_8  :begin
        if (riscv::XLEN == 32) data_o = generic_counter_q[addr_i-riscv::CSR_HPM_COUNTER_3+1][31:0];
        else data_o = generic_counter_q[addr_i-riscv::CSR_HPM_COUNTER_3+1];
      end
      riscv::CSR_HPM_COUNTER_3H,
            riscv::CSR_HPM_COUNTER_4H,
            riscv::CSR_HPM_COUNTER_5H,
            riscv::CSR_HPM_COUNTER_6H,
            riscv::CSR_HPM_COUNTER_7H,
            riscv::CSR_HPM_COUNTER_8H :begin
        if (riscv::XLEN == 32)
          data_o = generic_counter_q[addr_i-riscv::CSR_HPM_COUNTER_3H+1][63:32];
        else read_access_exception = 1'b1;
      end
      riscv::CSR_MHPM_MADDR :
      data_o = maddr_addr_q;
      riscv::CSR_MHPM_EBS_CFG :
      data_o = ebs_sample_cfg_q;
      default: data_o = 'b0;
    endcase

    //Write
    if (we_i) begin
      unique case (addr_i)
        riscv::CSR_MHPM_COUNTER_3,
            riscv::CSR_MHPM_COUNTER_4,
            riscv::CSR_MHPM_COUNTER_5,
            riscv::CSR_MHPM_COUNTER_6,
            riscv::CSR_MHPM_COUNTER_7,
            riscv::CSR_MHPM_COUNTER_8  :begin
          if (riscv::XLEN == 32)
            generic_counter_d[addr_i-riscv::CSR_MHPM_COUNTER_3+1][31:0] = data_i;
          else generic_counter_d[addr_i-riscv::CSR_MHPM_COUNTER_3+1] = data_i;
        end
        riscv::CSR_MHPM_COUNTER_3H,
            riscv::CSR_MHPM_COUNTER_4H,
            riscv::CSR_MHPM_COUNTER_5H,
            riscv::CSR_MHPM_COUNTER_6H,
            riscv::CSR_MHPM_COUNTER_7H,
            riscv::CSR_MHPM_COUNTER_8H :begin
          if (riscv::XLEN == 32)
            generic_counter_d[addr_i-riscv::CSR_MHPM_COUNTER_3H+1][63:32] = data_i;
          else update_access_exception = 1'b1;
        end
        riscv::CSR_MHPM_EVENT_3,
            riscv::CSR_MHPM_EVENT_4,
            riscv::CSR_MHPM_EVENT_5,
            riscv::CSR_MHPM_EVENT_6,
            riscv::CSR_MHPM_EVENT_7,
            riscv::CSR_MHPM_EVENT_8   :
        mhpmevent_d[addr_i-riscv::CSR_MHPM_EVENT_3+1] = data_i;
        riscv::CSR_MHPM_THRESHOLD_CYC,
            riscv::CSR_MHPM_THRESHOLD_INSTRET,
            riscv::CSR_MHPM_THRESHOLD_3,
            riscv::CSR_MHPM_THRESHOLD_4,
            riscv::CSR_MHPM_THRESHOLD_5,
            riscv::CSR_MHPM_THRESHOLD_6,
            riscv::CSR_MHPM_THRESHOLD_7,
            riscv::CSR_MHPM_THRESHOLD_8 :begin
          if (riscv::XLEN == 32)
            threshold_d[addr_i-riscv::CSR_MHPM_THRESHOLD_CYC][31:0] = data_i;
          else threshold_d[addr_i-riscv::CSR_MHPM_THRESHOLD_CYC] = data_i;
        end
        riscv::CSR_MHPM_THRESHOLD_CYCH,
            riscv::CSR_MHPM_THRESHOLD_INSTRETH,
            riscv::CSR_MHPM_THRESHOLD_3H,
            riscv::CSR_MHPM_THRESHOLD_4H,
            riscv::CSR_MHPM_THRESHOLD_5H,
            riscv::CSR_MHPM_THRESHOLD_6H,
            riscv::CSR_MHPM_THRESHOLD_7H,
            riscv::CSR_MHPM_THRESHOLD_8H :begin
          if (riscv::XLEN == 32)
            threshold_d[addr_i-riscv::CSR_MHPM_THRESHOLD_CYCH][63:32] = data_i;
          else update_access_exception = 1'b1;
        end
        riscv::CSR_MHPM_MADDR :
        maddr_addr_d = data_i;
        riscv::CSR_MHPM_EBS_CFG :
        ebs_sample_cfg_d = data_i;
        default: update_access_exception = 1'b1;
      endcase
    end
  end

  // ----------------------
  // Perf Event-Based Sampling Control
  // ----------------------
  always_comb begin: ebs_state
    ebs_state_d = ebs_state_q;

    case(ebs_state_q)
      riscv::IDLE: begin
        ebs_state_d = ebs_sample_trigger ? riscv::SAMPLING : riscv::IDLE;
      end
      riscv::SAMPLING: begin
        ebs_state_d = (((!ebs_store_req_o) || (ebs_store_req_o && ebs_store_ack_i)) && (ebs_sample_index_q == 6'd35)) ? riscv::IDLE : riscv::SAMPLING;
      end
    endcase
  end

  always_comb begin: ebs
    ebs_sample_trigger = 1'b0;
    ebs_store_req_o = 1'b0;
    maddr_offset_d = maddr_offset_q;
    count_offset_d = count_offset_q;
    ebs_sample_index_d = ebs_sample_index_q;
    ebs_pc_sample_d = ebs_pc_sample_q;
    ebs_counter_data_sample_d = ebs_counter_data_sample_q;
    ebs_regfile_data_sample_d = ebs_regfile_data_sample_q;

    case(ebs_state_q)
      riscv::IDLE: begin
        for(int unsigned i = 0; i <= 8; i++) begin
          if ((ebs_counter_data[i] >= threshold_q[i] + count_offset_q[i]) && (threshold_q[i] > 64'b0)) begin
            count_offset_d[i] = ebs_counter_data[i];
            ebs_pc_sample_d = events_pc_q[i];
            ebs_counter_data_sample_d = ebs_counter_data;
            ebs_regfile_data_sample_d = ebs_regfile_data;
            ebs_sample_trigger = 1'b1;
          end
        end
      end
      riscv::SAMPLING: begin
        if ((ebs_sample_cfg_q[ebs_sample_index_q - 6'd1]) || (ebs_sample_index_q == 6'd0)) begin
          ebs_store_req_o = 1'b1;
        end
        if ((!ebs_store_req_o) || (ebs_store_req_o && ebs_store_ack_i)) begin
          ebs_sample_index_d = (ebs_sample_index_q == 6'd35) ? 6'd0 : (ebs_sample_index_q == 6'd9) ? 6'd32 : ebs_sample_index_q + 1;
          if (ebs_store_ack_i)
            maddr_offset_d = maddr_offset_q + 64'd8;
        end
      end
    endcase
  end

  //Registers
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      generic_counter_q <= '{default: 0};
      mhpmevent_q       <= '{default: 0};
      threshold_q       <= '{default:0};
      maddr_addr_q     <= '{default:0};
      maddr_offset_q     <= '{default:0};
      count_offset_q    <= '{default:0};
      ebs_sample_cfg_q      <= '{default:0};
      ebs_sample_index_q    <= '{default:0};
      events_pc_q               <= '{default:0};
      ebs_pc_sample_q               <= '{default:0};
      ebs_counter_data_sample_q     <= '{default:0};
      ebs_regfile_data_sample_q     <= '{default:0};
      ebs_state_q           <= riscv::IDLE;
    end else begin
      generic_counter_q <= generic_counter_d;
      mhpmevent_q       <= mhpmevent_d;
      threshold_q       <= threshold_d;
      maddr_addr_q     <= maddr_addr_d;
      maddr_offset_q   <= maddr_offset_d;
      count_offset_q    <= count_offset_d;
      ebs_sample_cfg_q      <= ebs_sample_cfg_d;
      ebs_sample_index_q    <= ebs_sample_index_d;
      events_pc_q             <= events_pc_d;
      ebs_pc_sample_q             <= ebs_pc_sample_d;
      ebs_counter_data_sample_q   <= ebs_counter_data_sample_d;
      ebs_regfile_data_sample_q   <= ebs_regfile_data_sample_d;
      ebs_state_q           <= ebs_state_d;
    end
  end

endmodule
