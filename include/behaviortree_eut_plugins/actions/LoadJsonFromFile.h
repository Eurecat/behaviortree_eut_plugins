#ifndef LOAD_JSON_FROM_FILE_HPP
#define LOAD_JSON_FROM_FILE_HPP

#include <behaviortree_cpp/action_node.h>
#include <fstream>
#include <unistd.h>
// #include <sys/types.h>
// #include <pwd.h>


using namespace std;

namespace BT
{
class LoadJsonFromFile final : public BT::SyncActionNode
{
    public:
        using BT::SyncActionNode::SyncActionNode;
        ~LoadJsonFromFile() = default;

        static BT::PortsList providedPorts()
        {
            return { 
                BT::InputPort<std::string>("filename", "Name of the file"),
                BT::InputPort<std::string>("folder_path", "Prefix path of the file"),
                BT::OutputPort<nlohmann::json>("json_object", "Parameter to load as json")
            };
        }

        virtual BT::NodeStatus tick() override {

            const auto& filename  = getInput<std::string>("filename");
            if(!filename)  { throw BT::RuntimeError { name() + ": " + filename.error()  }; }
            
            const auto& prefix_path  = getInput<std::string>("folder_path");
            if(!prefix_path)  { throw BT::RuntimeError { name() + ": " + prefix_path.error()  }; }

            // const char *homedir;
            // if ((homedir = getenv("HOME")) == NULL) {
            //     homedir = getpwuid(getuid())->pw_dir;
            // }

            std::string filepath = prefix_path.value() + "/" + filename.value();

            std::ifstream file(filepath);
            if (  !file.is_open() ) 
            { 
                std::cerr << "Impossible to open json file at " << filepath << endl; 
                return BT::NodeStatus::FAILURE;
            } //Error: impossible to open the file

            try 
            {
                // parsing input with a syntax error
                nlohmann::json json_object = nlohmann::json::parse(file);
                setOutput("json_object", json_object);
                file.close();
                return BT::NodeStatus::SUCCESS;
            } //Success
            catch (const nlohmann::json::parse_error& e) 
            {
                // output exception information
                std::cerr << "Error while parsing json file at " << filepath 
                        << "; message: " << e.what() << '\n'
                        << "exception id: " << e.id << '\n'
                        << "byte position of error: " << e.byte << std::endl;
                file.close();
                return BT::NodeStatus::FAILURE;
            }//Error: exception reading the json file
        }
};
}

#endif
