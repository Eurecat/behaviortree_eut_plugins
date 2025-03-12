#include <stdio.h>
#include <iostream>
#include <fstream>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/xml_parsing.h"

int main(int argc, char* argv[])
{

    if (argc != 3)
    {
        printf("Wrong number of command line arguments\nUsage: %s [filename_in.so] [filename_out.xml] \n", argv[0]);
        return 1;
    }

    std::string plugin_path = argv[1];

    BT::BehaviorTreeFactory factory;

    BT::SharedLibrary loader(plugin_path);
    if(loader.hasSymbol(BT::PLUGIN_SYMBOL))
    {
        typedef void (*Func)(BT::BehaviorTreeFactory&);
        auto func = (Func)loader.getSymbol(BT::PLUGIN_SYMBOL);
        func(factory);
    }
    // else if (loader.hasSymbol(BT::ROS_PLUGIN_SYMBOL))
    // {
    //     Func fn = (Func)loader.getSymbol(BT::ROS_PLUGIN_SYMBOL);
    //     fn(factory);
    // }
    else
    {
        printf("The file %s is not a valid BehaviorTree plugin\n", argv[1]);
        return 1;
    }

    std::string tree_xml = writeTreeNodesModelXML(factory,false);
 
    std::ofstream outfile;
    outfile.open(argv[2]);
    outfile << "<?xml version=\"1.0\"?>" << std::endl;
    outfile << tree_xml; 
    return 0;
}