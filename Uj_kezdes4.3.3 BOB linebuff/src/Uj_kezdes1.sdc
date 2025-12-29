# Minimal, valid SDC just to shut up clock warnings

# 27 MHz board oscillator on port clk
create_clock -name {clk_board} -period 37.037 [get_ports {clk}]

# Optional: treat cam1_pclk (TVP5150) as a 27 MHz clock too
create_clock -name {cam1_pclk} -period 37.037 [get_ports {cam1_pclk}]