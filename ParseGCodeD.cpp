//Comment header
#include <iostream>
#include "Parse.h"
#include <fstream>
using namespace std;



int main()
{
    system("chdir");
    Parse DataIn("test5.txt");
    //char dummy;
    while (1) //Do not use endless loops in final product
    {
        DataIn.parseLine();
        cin.get();
    }

    return 0;
}



