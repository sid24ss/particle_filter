#include <iostream>
#include <string>
#include <pf/Map.h>

using namespace std;
using namespace pf;

int main () {
    Map wean_hall;
    if (wean_hall.loadFromFile(std::string("maps/wean.dat")))
        printf("Successfully loaded the map.\n");
    else
        printf("Oops. Map wasn't read.\n");
    return 0;
}