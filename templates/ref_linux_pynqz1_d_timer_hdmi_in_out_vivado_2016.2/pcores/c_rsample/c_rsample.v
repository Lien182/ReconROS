module c_rsample(
input    aclk,
input    aresetn,
//s_stream
input [23:0] s_axis_video_tdata,
input    s_axis_video_tlast,
output   s_axis_video_tready,
input    s_axis_video_tuser,
input    s_axis_video_tvalid,
//m_stream
output [15:0]  m_axis_video_tdata,
output   m_axis_video_tlast,
input    m_axis_video_tready,
output   m_axis_video_tuser,
output   m_axis_video_tvalid
);

reg toggle=0;

always @(posedge aclk)
begin
    if(s_axis_video_tvalid & s_axis_video_tready)
        toggle <= ~toggle;
end
 
assign m_axis_video_tdata = (toggle)? {s_axis_video_tdata[23:16],s_axis_video_tdata[7:0]} : s_axis_video_tdata[15:0];
assign m_axis_video_tlast = s_axis_video_tlast;
assign s_axis_video_tready = m_axis_video_tready;
assign m_axis_video_tuser = s_axis_video_tuser;
assign m_axis_video_tvalid = s_axis_video_tvalid;

endmodule