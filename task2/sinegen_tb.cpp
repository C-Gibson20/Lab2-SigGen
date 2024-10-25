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