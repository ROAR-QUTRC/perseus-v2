#include "example_plugin.hpp"

int main(int argc, char** argv)
{
    try
    {
        ExportNodesToXML("tree_nodes_model.xml", argc, argv);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}