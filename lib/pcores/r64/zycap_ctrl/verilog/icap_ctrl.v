//----------------------------------------------------------------------------
// Filename:          icap_stream
// Version:           1.00.a
// Description:       ICAP hardmacro instantitation
// Author:            Vipin K
// 
//----------------------------------------------------------------------------

module icap_ctrl 
	(
		ACLK,
		ARESETN,
		S_AXIS_TREADY,
		S_AXIS_TDATA,
		S_AXIS_TLAST,
		S_AXIS_TVALID
	);

input                                     ACLK;
input                                     ARESETN;
output                                    S_AXIS_TREADY;
input      [31 : 0]                       S_AXIS_TDATA;
input                                     S_AXIS_TLAST;
input                                     S_AXIS_TVALID;

wire [31:0] icap_data;

assign S_AXIS_TREADY = 1'b1; 
assign icap_data = {S_AXIS_TDATA[24],S_AXIS_TDATA[25],S_AXIS_TDATA[26],S_AXIS_TDATA[27],S_AXIS_TDATA[28],S_AXIS_TDATA[29],S_AXIS_TDATA[30],S_AXIS_TDATA[31],S_AXIS_TDATA[16],S_AXIS_TDATA[17],S_AXIS_TDATA[18],S_AXIS_TDATA[19],S_AXIS_TDATA[20],S_AXIS_TDATA[21],S_AXIS_TDATA[22],S_AXIS_TDATA[23],S_AXIS_TDATA[8],S_AXIS_TDATA[9],S_AXIS_TDATA[10],S_AXIS_TDATA[11],S_AXIS_TDATA[12],S_AXIS_TDATA[13],S_AXIS_TDATA[14],S_AXIS_TDATA[15],S_AXIS_TDATA[0],S_AXIS_TDATA[1],S_AXIS_TDATA[2],S_AXIS_TDATA[3],S_AXIS_TDATA[4],S_AXIS_TDATA[5],S_AXIS_TDATA[6],S_AXIS_TDATA[7]};

   ICAPE2 #(
      .DEVICE_ID('h3651093),     // Specifies the pre-programmed Device ID value to be used for simulation
                                  // purposes.
      .ICAP_WIDTH("X32"),         // Specifies the input and output data width.
      .SIM_CFG_FILE_NAME("None")  // Specifies the Raw Bitstream (RBT) file to be parsed by the simulation
                                  // model.
   )
   ICAPE2_inst (
      .O(),           // 32-bit output: Configuration data output bus
      .CLK(ACLK),    // 1-bit input: Clock Input
      .CSIB(~S_AXIS_TVALID),   // 1-bit input: Active-Low ICAP Enable
      .I(icap_data),         // 32-bit input: Configuration data input bus
      .RDWRB(~S_AXIS_TVALID)  // 1-bit input: Read/Write Select input
   );

endmodule
