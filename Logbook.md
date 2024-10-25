
# Sinusoid Generator

---
### Constant Frequency Sinusoid
#### counter.sv

```verilog
module counter #(
    parameter WIDTH = 8
)(
    input logic clk,
    input logic rst,
    input logic en,
    output logic [WIDTH-1:0] count
);  

always_ff @ (posedge clk)
    if (rst) count <= {WIDTH{1'b0}};                
    else count <= count + {{WIDTH-1{1'b0}}, en};
    
endmodule```

This is the counter implementation from Lab 1.

#### rom.sv

```verilog
module rom #(
    parameter ADDRESS_WIDTH = 8,
              DATA_WIDTH = 8
)(
    input logic clk,
    input logic [ADDRESS_WIDTH-1:0] addr,
    output logic [DATA_WIDTH-1:0] dout
);

logic [DATA_WIDTH-1:0] rom_array[2**ADDRESS_WIDTH-1:0];

initial begin
    $display("Loading rom.");
    $readmemh("sinerom.mem", rom_array);
end;

always_ff @(posedge clk)
    dout <= rom_array [addr];

endmodule
```

This is a synchronous ROM specified behaviourally. The ROM size is parametrised and the contents are configured in the `sinerom.mem` file. 
The `$readmemh(.)` allows the ROM to be loaded with the contents stored in the `sinerom.mem` file.

#### sinegen_tb.cpp

```verilog
#include "verilated.h"
#include "verilated_vcd_c.h"
#include "Vsinegen.h"
#include "vbuddy.cpp"

#define MAX_SIM_CYC 1000000
#define ADDRESS_WIDTH 8
#define ROM_SZ 256

int main(int argc, char **argv, char **env) {
    int scc;
    int edge;
    
    Verilated::commandArgs(argc, argv);
    Vsinegen* sinegen = new Vsinegen;

    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    sinegen->trace (tfp, 99);
    tfp->open ("sinegen.vcd");

    if (vbdOpen()!=1) return(-1);
    vbdHeader("Lab 2 SineGen");
    
    sinegen->clk = 1;
    sinegen->rst = 0;
    sinegen->en = 1;

    for (scc=0; scc<MAX_SIM_CYC; scc++){

        for (edge=0; edge<2; edge++) {
            tfp->dump (2*scc+edge);
            sinegen->clk = !sinegen->clk;
            sinegen->eval ();
        }
  
        sinegen->en = vbdFlag();
        vbdPlot(int(sinegen->dout), 0, 255);
        vbdCycle(scc);
        
        if ((Verilated::gotFinish()) || (vbdGetkey()=='q')){
            exit(0);
        }
    }
    vbdClose();
    tfp->close();
    exit(0);
}
```

This testbench uses the Vbuddy flag as an enable for the counter. When disabled the same ROM address is read repeatedly so the sinusoid flatlines. The value at the read ROM address is plotted by the Vbuddy plot function.

#### sinegen.py

```python
import math
import string

f = open("sinerom.mem","w")

for i in range(256):
    v = int(math.cos(2*3.1416*i/256)*127+127)
    if (i+1)%16 == 0:
        s = "{hex:2X}\n"
    else:
        s = "{hex:2X} "
    f.write(s.format(hex=v))

f.close()
```

This script generates the the configuration for the ROM. This uses the built in math library to calculate the values of a sinusoid, converts them into HEX, and formats them in the `sinerom.mem` file.

#### sinegen.sv

```verilog
module sinegen #(
  parameter WIDTH = 8
)(
  input  logic clk,
  input  logic rst,
  input  logic en,
  output logic [WIDTH-1:0] dout
);

  logic [WIDTH-1:0] address;

counter myCounter (
  .clk (clk),
  .rst (rst),
  .en (en),
  .count (address)
);

rom myRom (
  .clk (clk),
  .addr (address),
  .dout (dout)
);

endmodule
```

`sinegen.sv` is a top level module which contains the counter module and the ROM module.

#### Vbuddy Output

*Vbuddy Output*

---

### Variable Frequency Sinusoid
#### counter.sv

```verilog
module counter #(
    parameter WIDTH = 8
)(
    input logic clk,
    input logic rst,
    input logic en,
    output logic [WIDTH-1:0] count
);  

always_ff @ (posedge clk)
    if (rst) count <= {WIDTH{1'b0}};                
    else count <= (count + incr * {{WIDTH-1{1'b0}}, en});
    
endmodule```

An additional factor `incr` multiplies the increment of 1. This effectively increases the rate at which the counter counts.

#### rom.sv

The implementation of `rom.sv` is unchanged from the constant frequency sinusoid generator.

#### sinegen_tb.cpp

```verilog
#include "verilated.h"
#include "verilated_vcd_c.h"
#include "Vsinegen.h"
#include "vbuddy.cpp"

#define MAX_SIM_CYC 1000000
#define ADDRESS_WIDTH 8
#define ROM_SZ 256

int main(int argc, char **argv, char **env) {
    int scc;
    int edge;
    Verilated::commandArgs(argc, argv);
    Vsinegen* sinegen = new Vsinegen;

    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    sinegen->trace (tfp, 99);
    tfp->open ("sinegen.vcd");

    if (vbdOpen()!=1) return(-1);
    vbdHeader("Lab 2 SineGen");
    
    sinegen->clk = 1;
    sinegen->rst = 0;
    sinegen->en = 1;
    sinegen->incr = 1;

    for (scc=0; scc<MAX_SIM_CYC; scc++){

        for (edge=0; edge<2; edge++) {
            tfp->dump (2*scc+edge);
            sinegen->clk = !sinegen->clk;
            sinegen->eval ();
        }
        
        sinegen->incr = vbdValue();
        sinegen->en = vbdFlag();
        vbdPlot(int(sinegen->dout), 0, 255);
        vbdCycle(scc);
        
        if ((Verilated::gotFinish()) || (vbdGetkey()=='q')){
            exit(0);
        }
    }
    vbdClose();
    tfp->close();
    exit(0);
}
```

The value of the Vbuddy rotary is now used to vary a new logic datatype `incr`. This modifies the rate of counting which leads to a modified frequency. The rotary can be turned to adjust the frequency of the plotted sinusoid for any clock cycle.

#### sinegen.py

The implementation of `sinegen.py` is unchanged from the constant frequency sinusoid generator.

#### sinegen.sv

```verilog
module sinegen #(
  parameter WIDTH = 8
)(
  input  logic clk,
  input  logic rst,
  input  logic en,
  input logic incr,
  output logic [WIDTH-1:0] dout
);

  logic [WIDTH-1:0] address;

counter myCounter (
  .clk (clk),
  .rst (rst),
  .en (en),
  .incr (incr),
  .count (address)
);

rom myRom (
  .clk (clk),
  .addr (address),
  .dout (dout)
);

endmodule
```

`sinegen.sv` has been modified to accommodate the new logic signal `incr`. 

#### Vbuddy Output

*Vbuddy Output*

---
# Dual Sinusoid Generator

---
#### counter.sv

```verilog
module counter #(
    parameter WIDTH = 8
)(
    input logic clk,
    input logic rst,
    input logic diff,
    output logic [WIDTH-1:0] count1,
    output logic [WIDTH-1:0] count2s
);  

always_ff @ (posedge clk or posedge rst)begin
    if (rst) begin
        count1 <= {WIDTH{1'b0}};
        count2 <= count1 + diff;
    end
    else begin
        count1 <= count1 + {{WIDTH-1{1'b0}}, en};
        count2 <= count1 + diff;
    end
end
    
endmodule```

A new term `diff` is included to create a second count which is offset from the first.
Using `count1 + diff` ensures that the second count is still continuous and does not behave like the sinusoids with modified frequency.

#### dual_rom.sv

```verilog
module dual_rom #(
    parameter ADDRESS_WIDTH = 8,
              DATA_WIDTH = 8
)(
    input logic clk,
    input logic [ADDRESS_WIDTH-1:0] addr1,
    input logic [ADDRESS_WIDTH-1:0] addr2,
    output logic [DATA_WIDTH-1:0] dout1,
    output logic [DATA_WIDTH-1:0] dout2
);  

logic [DATA_WIDTH-1: 0] rom_array[2**ADDRESS_WIDTH-1:0];  

initial begin
    $display("Loading rom.");
    $readmemh("sinerom.mem", rom_array);
end  

always_ff @(posedge clk) begin
    dout1 <= rom_array [addr1];
    dout2 <= rom_array [addr2];
end  

endmodule
```

This is again a synchronous ROM specified behaviourally, and accesses the configured `.mem` file in the same way. However this is a dual-port ROM so two values can be read simultaneously from the ROM.

#### sinegen_tb.cpp

```verilog
#include "Vsinegen.h"
#include "verilated.h"
#include "verilated_vcd_c.h"
#include "vbuddy.cpp"

#define MAX_SIM_CYC 1000000
#define ADDRESS_WIDTH 8
#define ROM_SZ 256  

int main(int argc, char **argv, char **env) {
    int scc;
    int edge;
    
    Verilated::commandArgs(argc, argv);
    Vsinegen* sinegen = new Vsinegen;

    Verilated::traceEverOn(true);
    VerilatedVcdC* tfp = new VerilatedVcdC;
    sinegen->trace (tfp, 99);
    tfp->open ("sinegen.vcd");  

    if (vbdOpen()!=1) return(-1);
    vbdHeader("Lab 2 SineGen");

    sinegen->clk = 1;
    sinegen->rst = 0;
    sinegen->en = 1;
    sinegen->phase_diff = 1;  

    for (scc=0; scc<MAX_SIM_CYC; scc++){  

        for (edge=0; edge<2; edge++) {
            tfp->dump (2*scc+edge);
            sinegen->clk = !sinegen->clk;
            sinegen->eval ();
        }  

        sinegen->phase_diff = vbdValue();
        sinegen->en = vbdFlag();  

        vbdPlot(int(sinegen->dout1), 0, 255);
        vbdPlot(int(sinegen->dout2), 0, 255);
        vbdCycle(scc);
        if ((Verilated::gotFinish()) || (vbdGetkey()=='q')){
            exit(0);
        }
    }  
    vbdClose();
    tfp->close();
    exit(0);
}
```

This testbench uses the Vbuddy value to control the phase difference between the two sinusoids to be plotted. This can be modified using the rotary much like the variable frequency in the previous task. 

#### sinegen.py

The implementation of `sinegen.py` is unchanged from the constant frequency sinusoid generator.

#### sinegen.sv

```verilog
module sinegen #(
  parameter WIDTH = 8
)(
  input logic clk,
  input logic rst,
  input logic en,
  input logic phase_diff,
  output logic [WIDTH-1:0] dout1,
  output logic [WIDTH-1:0] dout2
); 

  logic [WIDTH-1:0] count1;
  logic [WIDTH-1:0] count2;  

counter myCounter (
  .clk (clk),
  .rst (rst),
  .en (en),
  .diff (phase_diff),
  .count1 (count1),
  .count2 (count2)
);

dual_rom myDualRom (
  .clk (clk),
  .addr1 (count1),
  .addr2 (count2),
  .dout1 (dout1),
  .dout2 (dout2)
);  

endmodule
```

`sinegen.sv` is a top level module which contains the counter module and the dual-port ROM module. The `phase_diff` logic signal is used by the counter to calculate the two offset counts.

#### Vbuddy Output

*Vbuddy Output*

---
# RAM Audio Signal Capture and Display

---

#### counter.sv

```verilog
module counter #(
    parameter WIDTH = 8
)(
    input logic clk,
    input logic rst,
    input logic en,
    input logic diff,
    output logic [WIDTH-1:0] count1,
    output logic [WIDTH-1:0] count2
);  

always_ff @ (posedge clk or posedge rst) begin
    if (rst)begin
        count1 <= {WIDTH{1'b0}};
        count2 <= count1 + diff;
    end

    else begin
        count1 <= count1 + {{WIDTH-1{1'b0}}, en};
        count2 <= count1 + diff;
    end
end  

endmodule
```

A new term `diff` is included to create a second count which is offset from the first.
An offset is needed so that the same port is not being simultaneously read from and written too.

#### dual_ram.sv

```verilog
module dual_ram #(
    parameter ADDRESS_WIDTH = 8,
              DATA_WIDTH = 8
)(
    input logic clk,
    input logic wr_en,
    input logic rd_en,
    input logic [ADDRESS_WIDTH-1:0] wr_addr,
    input logic [ADDRESS_WIDTH-1:0] rd_addr,
    input logic [DATA_WIDTH-1:0] din,
    output logic [DATA_WIDTH-1:0] dout
);  

logic [DATA_WIDTH-1:0] ram_array [2**ADDRESS_WIDTH-1:0];  

always_ff @(posedge clk) begin
    if (wr_en == 1'b1) ram_array[wr_addr] <= din;
    if (rd_en == 1'b1) dout <= ram_array [rd_addr];
end  

endmodule
```

This is a synchronous RAM specified behaviourally. This RAM has one read port and one write port. Each port has an enable signal and in addition the write port has a data input signal. Reading and writing occurs simultaneously but it is poor practice for both ports to use have the same address in any one clock cycle. 

Due to the non-blocking assignments all of the expressions on the right-hand side of the assignments are evaluated at the beginning of the clock edge and updated on the left-hand side at the end of the edge. In this case if the read and write addresses were the same the output data would be taken from before the write update.

#### sigdelay_tb.cpp

```verilog
#include "verilated.h"
#include "verilated_vcd_c.h"
#include "Vsigdelay.h"  

#include "vbuddy.cpp"    
#define MAX_SIM_CYC 1000000
#define ADDRESS_WIDTH 8
#define RAM_SZ pow(2,ADDRESS_WIDTH)
  
int main(int argc, char **argv, char **env) {
  int simcyc;    
  int tick;        

  Verilated::commandArgs(argc, argv);  

  Vsigdelay* top = new Vsigdelay;
  Verilated::traceEverOn(true);
  VerilatedVcdC* tfp = new VerilatedVcdC;
  top->trace (tfp, 99);
  tfp->open ("sigdelay.vcd");

  if (vbdOpen()!=1) return(-1);
  vbdHeader("Lab 2 Delay");
  //vbdSetMode(1);      

  top->clk = 1;
  top->rst = 0;
  top->wr = 1;
  top->rd = 1;
  top->offset = 64;
  vbdInitMicIn(RAM_SZ);  

  for (simcyc=0; simcyc<MAX_SIM_CYC; simcyc++) {
    for (tick=0; tick<2; tick++) {
      tfp->dump (2*simcyc+tick);
      top->clk = !top->clk;
      top->eval ();
    }
    top->mic_signal = vbdMicValue();
    top->offset = abs(vbdValue());  

    vbdPlot(int (top->mic_signal), 0, 255);
    vbdPlot(int (top->delayed_signal), 0, 255);
    vbdCycle(simcyc);  

    if ((Verilated::gotFinish()) || (vbdGetkey()=='q'))
      exit(0);
  }  

  vbdClose();
  tfp->close();
  printf("Exiting\n");
  exit(0);
}
```

This testbench uses the Vbuddy microphone value and value as the data input to the RAM and the offset value respectively. The testbench then plots the input data signal from the microphone and the output data signal being read from the RAM. 

#### sigdelay.sv

```verilog
module sinegen #(
  parameter WIDTH = 8
)(
  input logic clk,
  input logic rst,
  input logic wr,
  input logic rd,
  input logic offset,
  input logic mic_signal,
  output logic [WIDTH-1:0] delayed_signal
);  

  logic [WIDTH-1:0] count1;
  logic [WIDTH-1:0] count2;  

counter myCounter (
  .clk (clk),
  .rst (rst),
  .en (1),
  .diff (offset),
  .count1 (count1),
  .count2 (count2)
);  

dual_ram myDualRam (
  .clk (clk),
  .wr_en (wr),
  .rd_en (rd),
  .wr_addr (count2),
  .rd_addr (count1),
  .din (mic_signal),
  .dout (delayed_signal)
);

endmodule
```

`sigdelay.sv` is a top level module which contains the counter module and the dual-port RAM module. There are read and write enable signals for each port, the corresponding addresses, and the input and output data signals.

#### Vbuddy Output

*Vbuddy Output*