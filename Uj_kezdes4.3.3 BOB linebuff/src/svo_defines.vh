// svo_defines.vh - minimal stub just to keep svo_tmds happy

`ifndef SVO_DEFINES_VH
`define SVO_DEFINES_VH

// 24-bit RGB, no alpha
`define SVO_BITS_PER_RED     8
`define SVO_BITS_PER_GREEN   8
`define SVO_BITS_PER_BLUE    8
`define SVO_BITS_PER_ALPHA   0
`define SVO_BITS_PER_PIXEL   (`SVO_BITS_PER_RED + `SVO_BITS_PER_GREEN + `SVO_BITS_PER_BLUE)

// This macro is used in parameter lists in original SVO code.
// Here we define it empty so it won't break anything.
`define SVO_PASS_PARAMS

`endif
