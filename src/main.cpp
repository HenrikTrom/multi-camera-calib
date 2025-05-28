#include "load_pattern.h"

using namespace Calibration;

int main(){

    Data data_;
    loadPattern(&data_);
    // detectPatterns(&data_);
    // calculateIntrinsics(&data_);
    calibrate(&data_);

    
    return 0;
}