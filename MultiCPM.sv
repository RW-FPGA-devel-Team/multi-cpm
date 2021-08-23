//============================================================================
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================





module MultiCPM
(       
        output        LED,                                              
        output        VGA_HS,
        output        VGA_VS,
        output        AUDIO_L,
        output        AUDIO_R, 
		  output [9:0]  DAC_L, 
		  output [9:0]  DAC_R, 
        input         TAPE_IN,
        input         UART_RX,
        output        UART_TX,
        input         SPI_SCK,
        output        SPI_DO,
        input         SPI_DI,
        input         SPI_SS2,
        input         SPI_SS3,
        input         CONF_DATA0,
        input         CLOCK_27,
        output  [5:0] VGA_R,
        output  [5:0] VGA_G,
        output  [5:0] VGA_B,

		  output [12:0] SDRAM_A,
		  inout  [15:0] SDRAM_DQ,
		  output        SDRAM_DQML,
        output        SDRAM_DQMH,
        output        SDRAM_nWE,
        output        SDRAM_nCAS,
        output        SDRAM_nRAS,
        output        SDRAM_nCS,
        output  [1:0] SDRAM_BA,
        output        SDRAM_CLK,
        output        SDRAM_CKE
);


 



`include "build_id.v" 
parameter CONF_STR = {
	"MultiCPM;;",
	"S,IMGVHD,Load VHD;",
	"O56,Screen Color,White,Green,Amber;",
	"T0,Reset;",
	"V,v",`BUILD_DATE
};

/////////////////  CLOCKS  ////////////////////////

wire clk_sys;
wire pll_locked;
wire cpuClock;
wire clk_ram;
wire clk_ram_ph;

pll pll
(
  .inclk0 (CLOCK_27),
  .c0 (clk_sys),
  .c1 (cpuClock),
  .c2 (clk_ram),
  .c3 (clk_ram_ph),
  .locked (pll_locked)
);
/////////////////  HPS  ///////////////////////////

wire [31:0] status;
wire  [1:0] buttons;


wire        ioctl_download;
wire  [7:0] ioctl_index;
wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire        forced_scandoubler;
wire        ypbpr;

wire        img_readonly;
wire        ioctl_wait = ~pll_locked;
wire  [1:0] img_mounted;
wire [31:0] img_size;

wire [31:0] sd_lba;
wire        sd_rd;
wire        sd_wr;
wire        sd_ack;
wire  [8:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din;
wire        sd_buff_wr;
wire        sd_ack_conf;
wire        PS2_CLK;
wire        PS2_DATA;

mist_io #(.STRLEN($size(CONF_STR)>>3)) mist_io
(
	.SPI_SCK   (SPI_SCK),
   .CONF_DATA0(CONF_DATA0),
   .SPI_SS2   (SPI_SS2),
   .SPI_DO    (SPI_DO),
   .SPI_DI    (SPI_DI),

	.clk_sys(clk_sys),
	.conf_str(CONF_STR),

	.buttons(buttons),
	.status(status),
	.scandoubler_disable(forced_scandoubler),
   .ypbpr     (ypbpr),
	
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_ce(1),

   .sd_sdhc(sdhc),
	.sd_conf(0),
  	.sd_lba(sd_lba),
	.sd_rd(sd_rd),
	.sd_wr(sd_wr),
	.sd_ack(sd_ack),
	.sd_ack_conf(sd_ack_conf),
	.sd_buff_addr(sd_buff_addr),
	.sd_buff_dout(sd_buff_dout),
	.sd_buff_din(sd_buff_din),
	.sd_buff_wr(sd_buff_wr),
	.img_mounted(img_mounted),
	.img_size(img_size),

	
	.ps2_kbd_clk(PS2_CLK),
	.ps2_kbd_data(PS2_DATA)
	

);

/////////////////  RESET  /////////////////////////

wire reset = status[0] | ~pll_locked;


//////////////// Computer /////////////////////////
wire [18:0] sramAddress;
wire [7:0]  sramDataIn;
wire [7:0]  sramDataOut;

wire n_sRamWE;
wire n_sRamOE;
wire n_sRam1CS;
wire n_sRam2CS;
wire RX,TX,RTS,CTS,RX2,TX2;
wire [1:0] R,G,B;


Microcomputer Microcomputer
(
  .n_reset (~reset),
  .clk     (clk_sys),
  .cpuClock(cpuClock),
  .cpuData (),
  
  .sramDataIn	(sramDataIn),
  .sramDataOut (sramDataOut),
  .sramAddress	(sramAddress),
  .n_sRamWE		(n_sRamWE),
  .n_sRamOE		(n_sRamOE),
  .n_sRam1CS	(n_sRam1CS),
  .n_sRam2CS	(n_sRam2CS),
  
  .rxd1    (RX),
  .txd1    (TX),
  .rts1    (1'b0),
  .cts1    (1'b0),
//
  .rxd2    (RX2),
  .txd2    (TX2),
  .rts2    (1'b0),
  .cts2    (1'b0),

  .videoR0  (R[0]),
  .videoG0  (G[0]),
  .videoB0	(B[0]),
  .videoR1	(R[1]),
  .videoG1	(G[1]),
  .videoB1	(B[1]),
  .hSync		(HSync),
  .vSync	   (VSync),
  .hBlank	(HBlank),
  .vBlank	(VBlank),
  .cepix  	(ce_pix),

  
  .sdCS    (sdss),
  .sdMOSI  (sdmosi),
  .sdMISO  (vsdmiso),
  .sdSCLK  (sdclk),
  .driveLED(LED),
  .ps2Data (PS2_DATA),
  .ps2Clk  (PS2_CLK)
);


assign SDRAM_CLK=clk_ram_ph;
ssdram ram
(
  .clock_i   (clk_ram),
  .reset_i   (~pll_locked),
  .refresh_i (1'b1),
  //
  .addr_i     (sramAddress),
  .data_i     (sramDataIn),
  .data_o     (sramDataOut),
  .cs_i       (~n_sRam1CS || ~n_sRam2CS),
  .oe_i       (~n_sRamOE),
  .we_i       (~n_sRamWE),
  //
  .mem_cke_o    (SDRAM_CKE),
  .mem_cs_n_o   (SDRAM_nCS),
  .mem_ras_n_o  (SDRAM_nRAS),
  .mem_cas_n_o  (SDRAM_nCAS),
  .mem_we_n_o   (SDRAM_nWE),
  .mem_udq_o    (SDRAM_DQMH),
  .mem_ldq_o    (SDRAM_DQML),
  .mem_ba_o     (SDRAM_BA),
  .mem_addr_o   (SDRAM_A),
  .mem_data_io  (SDRAM_DQ)
);


////////////////  Peripherals  ////////////////////////

wire [9:0] audio;

dac #(10) dac_l (
   .clk_i        (clk_sys),
   .res_n_i      (1      ),
   .dac_i        (audio),
   .dac_o        (AUDIO_L)
);

assign DAC_L=audio;
assign DAC_R=audio;
assign AUDIO_R=AUDIO_L;




/////////////////  VIDEO  /////////////////////////

wire  [1:0]  disp_color= status[6:5];
wire  [1:0]  colour; 
wire HSync,VSync,HBlank,VBlank;
assign colour=G;  //only one component to test... 
wire ce_pix;

logic [23:0] rgb_white;
logic [23:0] rgb_green;
logic [23:0] rgb_amber;

// Video colour processing
 always_comb begin
	  rgb_white = 24'hEFEFFEF;
	  if(colour==2'b00) rgb_white = 24'h0;
	  else if(colour==2'b11) rgb_white = 24'hFFFFFF;
 end

 always_comb begin
	  rgb_green = 24'h00E600;
	  if(colour==2'b00) rgb_green = 24'h0;
	  else if(colour==2'b11) rgb_green = 24'h00F600;;
 end

 always_comb begin
	  rgb_amber = 24'h4DE600;
	  if(colour==2'b00) rgb_amber = 24'h0;
	  else if(colour==2'b11) rgb_amber = 24'h5CF600;;
 end

 logic [23:0] mono_colour;
 always_comb begin
	  if(disp_color==2'b00) mono_colour = rgb_white;
	  else if(disp_color==2'b01) mono_colour = rgb_green;
	  else if(disp_color==2'b10) mono_colour= rgb_amber;
	  else mono_colour = rgb_white;
 end



wire [2:0] scale = status[9:7];


//assign VGA_HS = HSync;
//assign VGA_VS = VSync;
//assign VGA_R = mono_colour[23:19];
//assign VGA_G = mono_colour[15:9];
//assign VGA_B = mono_colour[7:3];

video_mixer #(280, 1) mixer
(
   //.*,
	.clk_sys(clk_sys),
	.ce_pix(ce_pix),
   .ce_pix_actual(ce_pix),
   
	.SPI_SCK     ( SPI_SCK    ),
	.SPI_SS3     ( SPI_SS3    ),
	.SPI_DI      ( SPI_DI     ),

	.scandoubler_disable(1'b1),
	.hq2x(1'b0),
	.mono(1'b0),
	.scanlines(2'b00),
   .ypbpr_full(1'b0),
   .line_start(0),

   .R(mono_colour[23:18]),
	.G(mono_colour[15:8]),
	.B(mono_colour[7:2]),
	
	.HSync  (HSync),
	.VSync  (VSync),
	
	.VGA_HS (VGA_HS),
	.VGA_VS (VGA_VS),
	.VGA_R  (VGA_R),
	.VGA_G  (VGA_G),
	.VGA_B  (VGA_B)


);


//////////////////   SD   ///////////////////

wire sdclk;
wire sdmosi;
wire sdmiso = vsdmiso ;
wire sdss;

reg vsd_sel = 0;
always @(posedge clk_sys) if(img_mounted) vsd_sel <= |img_size;

wire vsdmiso;
wire sdhc=1'b0;

sd_card sd_card
(
        .*,
        .clk_spi(clk_sys), 
        .sdhc(sdhc),
        .sck(sdclk),
        .ss(sdss),
        .mosi(sdmosi),
        .miso(vsdmiso)
);



endmodule