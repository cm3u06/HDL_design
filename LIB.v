
module shift_reg_fifo
#(
  parameter   DP_M1 = 7,
  parameter   DW_M1 = 8,
  parameter   AW_M1 = 3   // ceil(log2(DP))-1
)
(
  input                              clk ,
  input                            rst_n ,

  input                            wr ,
  input         [DW_M1:0]         w_data ,
  
  input                            rd ,
  output reg    [DW_M1:0]         r_data ,

  output                            full ,
  output                           empty ,
  output        [AW_M1:0]    size_remain
);



reg [DW_M1:0]   MEM[0:DP_M1];

reg [AW_M1+1:0]  fifo_level_m1;

assign  full  = fifo_level_m1 == DP_M1;
assign  empty = fifo_level_m1 == -1;
assign  size_remain = DP_M1 - fifo_level_m1;

wire    w_en  = wr & !full;
wire    r_en  = rd & !empty;

always@(posedge clk or negedge rst_n)
  if (!rst_n)
    fifo_level_m1 <= -1;
  else
    fifo_level_m1 <= fifo_level_m1 + w_en - r_en ;


// write
integer i;
always@(posedge clk)
begin
  MEM[0] <= w_en ? w_data : MEM[0] ;
  for (i = 1; i < DP_M1; i++)
    MEM[i] <= w_en ? MEM[i-1] : MEM[i] ;
end


// read
always@(posedge clk)
  r_data  <= r_en ? MEM[fifo_level_m1[AW_M1:0]] : r_data;
  
endmodule



module shift_reg_fifo_rdy_ack
#(
  parameter   DP_M1 = 7,
  parameter   DW_M1 = 8,
  parameter   AW_M1 = 3   // ceil(log2(DP))-1
)
(
  input                              clk ,
  input                            rst_n ,

  input                            i_rdy ,
  output                           i_ack ,
  input         [DW_M1:0]         i_data ,
  
  output                       o_rdy ,
  input                        o_ack ,
  output    [DW_M1:0]         o_data ,

  output                            full ,
  output                           empty ,
  output        [AW_M1:0]    size_remain
);


wire    wr = i_rdy & i_ack;
wire    i_ack = !full;


reg     r_vld   ;
wire    rd = !empty && !r_vld | o_ack;


shift_reg_fifo
#(
  .DP_M1 (DP_M1),
  .DW_M1 (DW_M1),
  .AW_M1 (AW_M1)   // ceil(log2(DP))+1-1
)
I_fifo
(
.          clk (        clk ),
.        rst_n (      rst_n ),
.              (            )
.        wr (      wr ),
.       w_data (     i_data ),
.              (            )
.        rd (      rd ),
.       r_data (     o_data ),
.              (            )
.         full (       full ),
.        empty (      empty ),
.  size_remain (size_remain )
);

wire  o_deal = o_rdy & o_ack;
wire  o_rdy = r_vld;

always @(posedge clk or negedge rst_n)
  if (!rst_n)
    r_vld   <=   0;
  else
    r_vld   <=   rd ? 1 : o_deal ? 0 : r_vld;




  
endmodule
