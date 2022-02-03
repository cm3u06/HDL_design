//`define WR_DEBUG
`define LIB_TB_1
//`define LIB_TB_2
`define WR_LIB_BEH_SRAM

module shift_reg_fifo
#(
  parameter   DP_M1 = 7,
  parameter   DW_M1 = 7,
  parameter   AW_M1 = 3   // ceil(log2(DP))
)
(
  input                       clk         ,
  input                       rst_n       ,

  input                       wr          ,
  input          [DW_M1:0]    w_data      ,
  
  input                       rd          ,
  output  reg    [DW_M1:0]    r_data      ,

  output                      full        ,
  output                      empty       ,
  output         [AW_M1:0]    size_remain
);



reg [DW_M1:0]   MEM[0:DP_M1];

reg signed [AW_M1:0]  fifo_level_m1;

assign  full        = fifo_level_m1 == DP_M1;
assign  empty       = fifo_level_m1 == -1;
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
  for (i = 1; i <= DP_M1; i++)
    MEM[i] <= w_en ? MEM[i-1] : MEM[i] ;
end


// read
always@(posedge clk)
  r_data  <= r_en ? MEM[fifo_level_m1[AW_M1-1:0]] : r_data;
  
endmodule



module shift_reg_fifo_rdy_ack
#(
  parameter  DP_M1  =    7 ,
  parameter  DW_M1  =    7 ,
  parameter  AW_M1  =    3 // ceil(log2(DP))
)
(
  input                  clk         ,
  input                  rst_n       ,

  input                  i_rdy       ,
  output                 i_ack       ,
  input     [DW_M1:0]    i_data      ,
  
  output                 o_rdy       ,
  input                  o_ack       ,
  output    [DW_M1:0]    o_data      ,

  output                 full        ,
  output                 empty       ,
  output    [AW_M1:0]    size_remain
);


wire    wr = i_rdy & i_ack;
wire    i_ack = !full;


reg     r_vld   ;
wire    rd = !empty && ( !r_vld | o_ack );


shift_reg_fifo
#(
  .DP_M1    ( DP_M1 ) ,
  .DW_M1    ( DW_M1 ) ,
  .AW_M1    ( AW_M1 ) // ceil(log2(DP))
)
I_fifo
(
  .clk            ( clk         ) ,
  .rst_n          ( rst_n       ) ,

  .wr             ( wr          ) ,
  .w_data         ( i_data      ) ,
  
  .rd             ( rd          ) ,
  .r_data         ( o_data      ) ,

  .full           ( full        ) ,
  .empty          ( empty       ) ,
  .size_remain    ( size_remain )
);

wire   o_deal = o_rdy & o_ack;
assign o_rdy = r_vld;

always @(posedge clk or negedge rst_n)
  if (!rst_n)
    r_vld   <=   0;
  else
    r_vld   <=   rd ? 1 : o_deal ? 0 : r_vld;




  
endmodule




module cdc_reg_fifo
#(
  parameter   DP_M1 = 7,
  parameter   DW_M1 = 7,
  parameter   AW_M1 = 3,  // ceil(log2(DP))
  parameter   SYNC_CLKI = 2,
  parameter   SYNC_CLKO = 2
)
(
  input                       i_clk   ,
  input                       i_rst_n ,
  input                       i_rdy   ,
  output                      i_ack   ,
  input          [DW_M1:0]    i_data  ,
  
  input                       o_clk   ,
  input                       o_rst_n ,
  output  reg                 o_rdy   ,
  input                       o_ack   ,
  output  reg    [DW_M1:0]    o_data

);

integer i;

reg [DW_M1:0]     MEM[0:DP_M1];

reg  [AW_M1:0]   clki_bin_addr;
reg  [AW_M1:0]   clki_gray_addr;
wire [AW_M1:0]   next_clki_bin_addr;
wire [AW_M1:0]   next_clki_gray_addr;
reg  [AW_M1:0]   clki_from_clko_gray_addr[1:SYNC_CLKI];
wire [AW_M1:0]   clki_from_clko_gray_addr_SYNC_CLKI = clki_from_clko_gray_addr[SYNC_CLKI];


reg  [AW_M1:0]   clko_bin_addr;
reg  [AW_M1:0]   clko_gray_addr;
wire [AW_M1:0]   next_clko_bin_addr;
wire [AW_M1:0]   next_clko_gray_addr;
reg  [AW_M1:0]   clko_from_clki_gray_addr[1:SYNC_CLKO];
wire [AW_M1:0]   clko_from_clki_gray_addr_SYNC_CLKO = clko_from_clki_gray_addr[SYNC_CLKO];

wire    i_deal = i_rdy & i_ack;
wire    o_deal = o_rdy & o_ack;


wire     w_enable =  !( clki_gray_addr [AW_M1:0]  == { ~clki_from_clko_gray_addr[SYNC_CLKI][AW_M1:AW_M1-1],  clki_from_clko_gray_addr[SYNC_CLKI][AW_M1-2:0] }  );
wire     r_enable =  !( clko_gray_addr [AW_M1:0]  ==  clko_from_clki_gray_addr[SYNC_CLKO][AW_M1:0]                                                             );

wire     w_deal   = w_enable & i_rdy;
assign   next_clki_bin_addr = w_deal ? clki_bin_addr+1 : clki_bin_addr;
assign   next_clki_gray_addr = {1'b0,next_clki_bin_addr[AW_M1:1]} ^ next_clki_bin_addr[AW_M1:0];

wire     r_deal   = r_enable & (!o_rdy | o_ack);
assign   next_clko_bin_addr = r_deal ? clko_bin_addr+1 : clko_bin_addr;
assign   next_clko_gray_addr = {1'b0,next_clko_bin_addr[AW_M1:1]} ^ next_clko_bin_addr[AW_M1:0];

assign    i_ack = w_enable;

// write

always @(posedge i_clk or negedge i_rst_n)
begin
  if (!i_rst_n) begin
    clki_bin_addr   <=  0;
    clki_gray_addr  <=  0;
    for (i = 1; i <= SYNC_CLKI; i++) begin
      clki_from_clko_gray_addr[i]  <=  0;
    end
  end
  else begin
    clki_bin_addr   <=  next_clki_bin_addr;
    clki_gray_addr  <=  next_clki_gray_addr;
    for (i = 1; i <= SYNC_CLKI; i++) begin
      if (i==1)
        clki_from_clko_gray_addr[i]  <=  clko_gray_addr;
      else
        clki_from_clko_gray_addr[i]  <=  clki_from_clko_gray_addr[i-1];
    end
  end
end


always @(posedge i_clk)
begin
  for (i = 0; i <= DP_M1; i++) begin
    MEM[i] <= i_deal && clki_bin_addr[AW_M1-1:0]==i  ?  i_data    :   MEM[i] ;
  end
end


// read
always @(posedge o_clk or negedge o_rst_n)
begin
  if (!o_rst_n) begin
    clko_bin_addr   <=  0;
    clko_gray_addr  <=  0;
    for (i = 1; i <= SYNC_CLKO; i++) begin
      clko_from_clki_gray_addr[i]  <=  0;
    end
  end
  else begin
    clko_bin_addr   <=  next_clko_bin_addr;
    clko_gray_addr  <=  next_clko_gray_addr;
    for (i = 1; i <= SYNC_CLKO; i++) begin
      if (i==1)
        clko_from_clki_gray_addr[i]  <=  clki_gray_addr;
      else
        clko_from_clki_gray_addr[i]  <=  clko_from_clki_gray_addr[i-1];
    end
  end
end

always @(posedge o_clk or negedge o_rst_n)
  if (!o_rst_n)
    o_rdy   <= 0;
  else 
    o_rdy   <=  r_deal ? 1 : o_deal ? 0 : o_rdy;

always @(posedge o_clk)
  o_data  <=  r_deal  ? MEM[clko_bin_addr[AW_M1-1:0]]   :   o_data;



endmodule





module sram_1p
#(
  parameter  DP_M1  =    7 ,
  parameter  DW_M1  =    7 ,
  parameter  AW_M1  =    2 // ceil(log2(DP))-1
)
(
  input                  clk         ,
  input                  rst_n       ,

  input                  sram_ce     ,
  input                  sram_we     ,
  input     [AW_M1:0]    sram_addr   ,
  input     [DW_M1:0]    sram_w_data ,
  output    [DW_M1:0]    sram_r_data
);

`ifndef WR_LIB_BEH_SRAM
`else
integer i;
reg [DW_M1:0]    mem[0:DP_M1];

// write
always @(posedge clk)
  for (i = 0; i <= DP_M1; i++) begin
    mem[i] <= ( sram_ce & sram_we ) && i==sram_addr ? sram_w_data : mem[i];
  end

// read
reg [AW_M1:0] sram_addr_latched;
always @(posedge clk)
  sram_addr_latched <= sram_ce&!sram_we ? sram_addr : sram_addr_latched; // non-volatile

assign  sram_r_data = mem[sram_addr_latched];
`endif

  
endmodule




module sram_fifo_rdy_ack
#(
  parameter  SRAM_DP_M1  =    31,
  parameter  DW_M1       =    7 ,
  parameter  SRAM_AW_M1  =    4 ,// ceil(log2(DP))-1
  parameter  READ_FIRST  =    0 // 0: write-first; 1:read-first
)
(
  input                         clk         ,
  input                         rst_n       ,

  input                         i_rdy       ,
  output                        i_ack       ,
  input            [DW_M1:0]    i_data      ,
  
  output  reg                   o_rdy       ,
  input                         o_ack       ,
  output           [DW_M1:0]    o_data      ,

  output                        full        ,
  output                        empty       ,
  output    [SRAM_AW_M1+1:0]    size_remain
);

parameter   DP_M1  = SRAM_DP_M1*2+1;

reg   [SRAM_AW_M1+2:0]     w_ptr;
reg   [SRAM_AW_M1+2:0]     r_ptr;
reg   [SRAM_AW_M1+2:0]     fifo_level;
wire  wr_enable ;
wire  rd_enable ;

assign  full =   ( w_ptr[SRAM_AW_M1+2] != r_ptr[SRAM_AW_M1+2] ) && ( w_ptr[SRAM_AW_M1+1:0] == r_ptr[SRAM_AW_M1+1:0] ) ;
assign  empty =  w_ptr == r_ptr;
assign  wr_enable = !full  && ( READ_FIRST ? !(rd_enable && w_ptr[0] == r_ptr[0]) : 1'b1 );
assign  rd_enable = !empty && ( READ_FIRST ? 1'b1 : !(wr_enable && w_ptr[0] == r_ptr[0]) );

assign  size_remain   =   DP_M1+1 - fifo_level;

assign  i_ack = wr_enable;
//
wire      w_deal = i_rdy & wr_enable;
wire      r_deal = rd_enable & (!o_rdy | o_ack);
wire      o_deal = o_rdy & o_ack;

always @(posedge clk or negedge rst_n)
  if (!rst_n) begin
    w_ptr     <=    0;
    r_ptr     <=    0;
    fifo_level <=   0;
  end
  else begin
    w_ptr     <=    w_deal ? w_ptr[SRAM_AW_M1+1:0]==DP_M1 ? { ~w_ptr[SRAM_AW_M1+2],{(SRAM_AW_M1+2){1'b0}} } : w_ptr+1 : w_ptr;
    r_ptr     <=    r_deal ? r_ptr[SRAM_AW_M1+1:0]==DP_M1 ? { ~r_ptr[SRAM_AW_M1+2],{(SRAM_AW_M1+2){1'b0}} } : r_ptr+1 : r_ptr;
    fifo_level <=   fifo_level + w_deal - r_deal;
  end



always @(posedge clk or negedge rst_n)
  if (!rst_n)
    o_rdy   <=   0;
  else
    o_rdy   <=   r_deal ? 1 : o_deal ? 0 : o_rdy;

reg sram_r_sel;
always @(posedge clk or negedge rst_n)
  if (!rst_n) 
    sram_r_sel <= 0;
  else
    sram_r_sel <= r_deal ? r_ptr[0] : sram_r_sel;


//////////////////////////////////////////////////////////////
// SRAM 1P x 2
//////////////////////////////////////////////////////////////
wire                      sram_ce_0     = ( wr_enable & !w_ptr[0] ) | (rd_enable & !r_ptr[0]);
wire                      sram_we_0     = ( wr_enable & !w_ptr[0] );
wire    [SRAM_AW_M1:0]    sram_addr_0   = wr_enable & !w_ptr[0] ? w_ptr[SRAM_AW_M1+1:1] : r_ptr[SRAM_AW_M1+1:1];
wire    [DW_M1:0]         sram_w_data_0 = i_data;
wire    [DW_M1:0]         sram_r_data_0 ;

wire                      sram_ce_1     = ( wr_enable & w_ptr[0] ) | (rd_enable & r_ptr[0]);
wire                      sram_we_1     = ( wr_enable & w_ptr[0] );
wire    [SRAM_AW_M1:0]    sram_addr_1   = wr_enable & w_ptr[0] ? w_ptr[SRAM_AW_M1+1:1] : r_ptr[SRAM_AW_M1+1:1];
wire    [DW_M1:0]         sram_w_data_1 = i_data;
wire    [DW_M1:0]         sram_r_data_1 ;

sram_1p
#(
  .DP_M1    ( SRAM_DP_M1 ) ,
  .DW_M1    ( DW_M1      ) ,
  .AW_M1    ( SRAM_AW_M1 ) // ceil(log2(DP))
)
I_sram_1p_0
(
  .clk            ( clk           ) ,
  .rst_n          ( rst_n         ) ,

  .sram_ce        ( sram_ce_0     ) ,
  .sram_we        ( sram_we_0     ) ,
  .sram_addr      ( sram_addr_0   ) ,
  .sram_w_data    ( sram_w_data_0 ) ,
  .sram_r_data    ( sram_r_data_0 )
);

sram_1p
#(
  .DP_M1    ( SRAM_DP_M1 ) ,
  .DW_M1    ( DW_M1      ) ,
  .AW_M1    ( SRAM_AW_M1 ) // ceil(log2(DP))
)
I_sram_1p_1
(
  .clk            ( clk           ) ,
  .rst_n          ( rst_n         ) ,

  .sram_ce        ( sram_ce_1     ) ,
  .sram_we        ( sram_we_1     ) ,
  .sram_addr      ( sram_addr_1   ) ,
  .sram_w_data    ( sram_w_data_1 ) ,
  .sram_r_data    ( sram_r_data_1 )
);


//////////////////////////////////////////////////////////////
//  output latch
//////////////////////////////////////////////////////////////

wire  [DW_M1:0]  sram_r_data  =   sram_r_sel  ? sram_r_data_1   : sram_r_data_0;
reg   [DW_M1:0]  sram_r_data_latched;
reg   r_deal_d1;

always @(posedge clk)
begin
  r_deal_d1   <=  r_deal;
  sram_r_data_latched   <=  r_deal_d1   ?   sram_r_data   : sram_r_data_latched;
end

assign  o_data  =   r_deal_d1  ? sram_r_data   : sram_r_data_latched;
  
endmodule



//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//    LIB TB
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////


 `ifdef LIB_TB_1
module LIB_TB_1 ();

parameter   DP_M1 = 31;
parameter   DW_M1 = 7;
parameter   AW_M1 = 4;
parameter   CYCLE = 10;
parameter   INITIAL_COUNT = 30;
parameter   COMPARE_COUNT = 1000;

reg                clk         ;
reg                rst_n       ;

reg                i_rdy       ;
wire               i_ack       ;
reg   [DW_M1:0]    i_data      ;

wire               o_rdy       ;
reg                o_ack       ;
wire  [DW_M1:0]    o_data      ;

wire               full        ;
wire               empty       ;
wire  [AW_M1:0]    size_remain ;

integer i;

///////////////////////////////////////////////////////////////
//   CLK, RST
///////////////////////////////////////////////////////////////

initial begin
  clk = 0;
  rst_n = 1;

  #CYCLE;
  rst_n = 0;
  #CYCLE;
  rst_n = 1;
end

always #(CYCLE/2) clk = ~clk;


///////////////////////////////////////////////////////////////
//   DUT
///////////////////////////////////////////////////////////////

//shift_reg_fifo_rdy_ack
//#(
//.DP_M1    ( DP_M1 ) ,
//.DW_M1    ( DW_M1 ) ,
//.AW_M1    ( AW_M1 ) // ceil(log2(DP))
//)
sram_fifo_rdy_ack
#(
.SRAM_DP_M1    ( DP_M1 ) ,
.DW_M1         ( DW_M1 ) ,
.SRAM_AW_M1    ( AW_M1 ) , // ceil(log2(DP))
.READ_FIRST    ( 1 ) 
)
I_DUT
(
.clk            ( clk         ) ,
.rst_n          ( rst_n       ) ,

.i_rdy          ( i_rdy       ) ,
.i_ack          ( i_ack       ) ,
.i_data         ( i_data      ) ,

.o_rdy          ( o_rdy       ) ,
.o_ack          ( o_ack       ) ,
.o_data         ( o_data      ) ,

.full           ( full        ) ,
.empty          ( empty       ) ,
.size_remain    ( size_remain )
);


///////////////////////////////////////////////////////////////
// stimulus
///////////////////////////////////////////////////////////////

// driver
reg rand_rdy;
integer wtime, rtime;
initial begin

i_rdy  = 0;
i_data = 'bx;
o_ack  = 0;

wait(!rst_n);
@(posedge rst_n);
#( CYCLE*10 );

fork
  // write
  begin
    @(posedge clk);
    i_rdy  <= 1;
    i_data <= 0;

    forever begin
      @(posedge clk);
      wtime = ( $time>>2 )*$random();
      rand_rdy = {$random(wtime)}%8 == 0;
      i_rdy <= rand_rdy;
      i_data <= rand_rdy ? i_data+1 : i_data;
    end
  end

  // read
  begin
    #( CYCLE*INITIAL_COUNT );
    @(posedge clk);
    o_ack <= 0;
    forever begin
      @(posedge clk);
      rtime = ( $time>>2 )*$random();
      o_ack <= {$random(rtime)}%2;
    end
  end
join

end

// capture
parameter MEM_DEPTH_M1 = COMPARE_COUNT+INITIAL_COUNT;
reg   [DW_M1:0]     MEM[0:MEM_DEPTH_M1];
integer   mem_w_ptr ;

always @(posedge clk or negedge rst_n)
if (!rst_n) begin
  for (i = 0; i <= MEM_DEPTH_M1; i++) begin
    MEM[i]<=0;
  end
  mem_w_ptr <= 0;
end
else begin
  for (i = 0; i <= MEM_DEPTH_M1; i++) begin
    MEM[i]<= i_rdy&i_ack && i==mem_w_ptr ? i_data : MEM[i];
  end
  mem_w_ptr <= i_rdy&i_ack ? mem_w_ptr+1 : mem_w_ptr;
end

// compare
integer   mem_r_ptr ;
reg[8*1000:0] s;
integer   pass_cnt;
always @(posedge clk or negedge rst_n)
  if (!rst_n) begin
    mem_r_ptr   <= 0;
    pass_cnt    <= 0;
  end
  else begin
    if (mem_r_ptr >= COMPARE_COUNT)
    begin
      $display("########################################################################################");
      $display("[%t:%m] comparison result : %s !!!!!!!!", $time, (pass_cnt==mem_r_ptr) ? "PASS" : "FAIL");
      $display("########################################################################################");
      $finish;
    end
    else begin
      if (o_rdy&o_ack)
      begin
        mem_r_ptr <= mem_r_ptr+1;
        $sformat(s,"[%t:%m] MEM[%08d] = %08d, o_data = %08d",$time, mem_r_ptr, MEM[mem_r_ptr], o_data);
        if (MEM[mem_r_ptr] == o_data)
        begin
          $display("%s => pass !", s);
          pass_cnt <= pass_cnt + 1;
        end
        else
          $display("%s => fail !", s);
      end
    end
  end

  initial begin
    $dumpfile("dump.vcd"); $dumpvars; 
  end

  endmodule
`endif


 `ifdef LIB_TB_2
module LIB_TB_2 ();

parameter   DP_M1 = 7;
parameter   DW_M1 = 7;
parameter   AW_M1 = 3;
parameter   CLKI_CYCLE = 5;
parameter   CLKO_CYCLE = 10;
parameter   INITIAL_COUNT = 30;
parameter   COMPARE_COUNT = 1000;

reg               i_clk   ;
reg               i_rst_n ;
reg               i_rdy   ;
wire              i_ack   ;
reg  [DW_M1:0]    i_data  ;

reg               o_clk   ;
reg               o_rst_n ;
wire              o_rdy   ;
reg               o_ack   ;
wire [DW_M1:0]    o_data  ;


integer i;

///////////////////////////////////////////////////////////////
//   CLK, RST
///////////////////////////////////////////////////////////////

initial begin
  i_clk = 0;
  i_rst_n = 1;

  #CLKI_CYCLE;
  i_rst_n = 0;
  #CLKI_CYCLE;
  i_rst_n = 1;
end

initial begin
  o_clk = 0;
  o_rst_n = 1;

  #CLKO_CYCLE;
  o_rst_n = 0;
  #CLKO_CYCLE;
  o_rst_n = 1;
end

always #(CLKI_CYCLE/2) i_clk = ~i_clk;
always #(CLKO_CYCLE/2) o_clk = ~o_clk;


///////////////////////////////////////////////////////////////
//   DUT
///////////////////////////////////////////////////////////////

cdc_reg_fifo
#(
.DP_M1  (  DP_M1 ) ,
.DW_M1  (  DW_M1 ) ,
.AW_M1  (  AW_M1 ) // ceil(log2(DP))
)
I_DUT
(
  .i_clk      ( i_clk   ) ,
  .i_rst_n    ( i_rst_n ) ,
  .i_rdy      ( i_rdy   ) ,
  .i_ack      ( i_ack   ) ,
  .i_data     ( i_data  ) ,
  
  .o_clk      ( o_clk   ) ,
  .o_rst_n    ( o_rst_n ) ,
  .o_rdy      ( o_rdy   ) ,
  .o_ack      ( o_ack   ) ,
  .o_data     ( o_data  )

);


///////////////////////////////////////////////////////////////
// stimulus
///////////////////////////////////////////////////////////////

// driver
reg rand_rdy;
integer wtime, rtime;
initial begin

  i_rdy  = 0;
  i_data = 'bx;

  wait(!i_rst_n);
  @(posedge i_rst_n);
  #( CLKI_CYCLE*10 );

  // write
  begin
    @(posedge i_clk);
    i_rdy  <= 1;
    i_data <= 0;

    forever begin
      @(posedge i_clk);
      wtime = ( $time>>2 )*$random();
      rand_rdy = ( {$random(wtime)}%5 ) == 1; // control input throughput
      i_rdy <= rand_rdy;
      i_data <= rand_rdy ? i_data+1 : i_data;
    end
  end

end

initial begin

  o_ack  = 0;

  wait(!o_rst_n);
  @(posedge o_rst_n);
  #( CLKO_CYCLE*INITIAL_COUNT );

  // read
  forever begin
    @(posedge o_clk);
    rtime = ( $time>>2 )*$random();
    o_ack <= {$random(rtime)}%2;
  end

end

// capture
parameter MEM_DEPTH_M1 = COMPARE_COUNT+INITIAL_COUNT;
reg   [DW_M1:0]     MEM[0:MEM_DEPTH_M1];
integer   mem_w_ptr ;

always @(posedge i_clk or negedge i_rst_n)
if (!i_rst_n) begin
  for (i = 0; i <= MEM_DEPTH_M1; i++) begin
    MEM[i]<=0;
  end
  mem_w_ptr <= 0;
end
else begin
  for (i = 0; i <= MEM_DEPTH_M1; i++) begin
    MEM[i]<= i_rdy&i_ack && i==mem_w_ptr ? i_data : MEM[i];
  end
  mem_w_ptr <= i_rdy&i_ack ? mem_w_ptr+1 : mem_w_ptr;
end

// compare
integer   mem_r_ptr ;
reg[8*1000:0] s;
integer   pass_cnt;
always @(posedge o_clk or negedge o_rst_n)
  if (!o_rst_n) begin
    mem_r_ptr   <= 0;
    pass_cnt    <= 0;
  end
  else begin
    if (mem_r_ptr >= COMPARE_COUNT)
    begin
      $display("########################################################################################");
      $display("[%t:%m] comparison result : %s !!!!!!!!", $time, (pass_cnt==mem_r_ptr) ? "PASS" : "FAIL");
      $display("########################################################################################");
      $finish;
    end
    else begin
      if (o_rdy&o_ack)
      begin
        mem_r_ptr <= mem_r_ptr+1;
        $sformat(s,"[%t:%m] MEM[%08d] = %08d, o_data = %08d",$time, mem_r_ptr, MEM[mem_r_ptr], o_data);
        if (MEM[mem_r_ptr] == o_data)
        begin
          $display("%s => pass !", s);
          pass_cnt <= pass_cnt + 1;
        end
        else
          $display("%s => fail !", s);
      end
    end
  end

  initial begin
    $dumpfile("dump.vcd"); $dumpvars; 
    //repeat(1000) @(posedge o_clk);
    //$finish;
  end

  endmodule
`endif

