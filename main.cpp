#include "global_register.h"

int main(int argc, char** argv)
{
    GlobalDQReg g = GlobalDQReg();
    g.saveAllKeyPoints("data/keypoints/");
    //g.pairwiseRegister();
    g.runDQDiffusion();
}
