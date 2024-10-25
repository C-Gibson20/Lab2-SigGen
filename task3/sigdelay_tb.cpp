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
