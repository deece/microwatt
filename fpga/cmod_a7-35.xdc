## Clock signal 12 MHz
set_property -dict { PACKAGE_PIN L17   IOSTANDARD LVCMOS33 } [get_ports { ext_clk }];
create_clock -add -name sys_clk_pin -period 83.33 -waveform {0 41.66} [get_ports {ext_clk}];

set_property -dict { PACKAGE_PIN J18   IOSTANDARD LVCMOS33 } [get_ports { uart0_txd }];
set_property -dict { PACKAGE_PIN J17   IOSTANDARD LVCMOS33 } [get_ports { uart0_rxd  }];

set_property -dict { PACKAGE_PIN A18   IOSTANDARD LVCMOS33 } [get_ports { ext_rst }];
