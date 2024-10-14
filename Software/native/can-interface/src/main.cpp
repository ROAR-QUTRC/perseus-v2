#include <hi_can_lib.hpp>
#include <iostream>

using namespace hi_can;
int main()
{
    std::cout << "testing" << std::endl;
    // test();

    Server server;
    Client client;
    server.handle();
    client.handle();
}