#include "global_register.h"

int main(int argc, char** argv)
{
    GlobalDQReg g = GlobalDQReg();
    g.saveKeyPoints("data/keypoints/");
    //g.pairwiseRegister();
    g.runDQDiffusion();
}
