#include "behaviortree_eut_plugins/eut_factory.h"


namespace BT
{

    void EutBehaviorTreeFactory::updateTypeInfoMap()
    {
        if(bt_factory_ptr_)
        {
            for(const auto& [nodemodel_name, manifest] : bt_factory_ptr_->manifests())
            {
                for(const auto& [port_name, portinfo] : manifest.ports)
                {
                    if(typeinfo_inf_map_.find(portinfo.typeName()) == typeinfo_inf_map_.end())
                    {
                        // new type to be added
                        typeinfo_inf_map_.insert(std::make_pair<std::string, BT::TypeInfo>(std::string{portinfo.typeName()}, static_cast<BT::TypeInfo>(portinfo)));
                    }
                }
            }
        }
    }

};