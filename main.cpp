#include "global_register.h"

int main(int argc, char** argv)
{
    GlobalDQReg g = GlobalDQReg();
    g.pairwiseRegister();
    g.runDQDiffusion();
}
