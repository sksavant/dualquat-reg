#include "global_register.h"

int main(int argc, char** argv)
{
    g = GlobalDQReg();
    g.pairwiseRegister();
    g.runDQDiffusion();
}
