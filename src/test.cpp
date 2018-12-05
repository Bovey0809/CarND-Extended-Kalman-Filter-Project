p#include <iostream>
#include "tools.h"


int main(int argc, char const *argv[])
{
    Tools tools;
    VectorXd x(4);
    VectorXd gd(4);

    x << 1, 2, 3, 4;
    gd << 2, 3, 4, 5;
    
    VectorXd result = tools.CalculateRMSE(x, gd);
    std::cout<<result;

    return 0;
}
