#ifndef FACTORY_UTIL_H
#define FACTORY_UTIL_H

#include "behaviortree_cpp/bt_factory.h"

namespace BT
{

    class EutBehaviorTreeFactory
    {
        public:
            typedef std::shared_ptr<EutBehaviorTreeFactory> Ptr;
            
            EutBehaviorTreeFactory(std::shared_ptr<BT::BehaviorTreeFactory> original_factory): 
                bt_factory_ptr_(original_factory)
            {}

            ~EutBehaviorTreeFactory() = default;

            BT::BehaviorTreeFactory& originalFactory(){return *bt_factory_ptr_;};

            void updateTypeInfoMap();

            BT::Expected<BT::TypeInfo> getTypeInfo(const std::string& type_name) const
            {
                const auto it = typeinfo_inf_map_.find(type_name);
                if(it != typeinfo_inf_map_.end())
                    return it->second;
                else
                    return nonstd::make_unexpected(BT::StrCat("TypeInfo for [" , type_name , "] is unknown"));
            }

        protected:

        private:
            std::shared_ptr<BT::BehaviorTreeFactory> bt_factory_ptr_;
            std::unordered_map<std::string, BT::TypeInfo> typeinfo_inf_map_;
    };
};

#endif