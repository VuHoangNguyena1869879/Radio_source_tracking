#ifndef RF_TAG_H_
#define RF_TAG_H_

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Sensor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/transport/Node.hh>
#include <gz/plugin/Register.hh>

#include <memory>

using namespace gz;
using namespace sim;
using namespace systems;

class RfTag : public System,
                public ISystemConfigure,
                public ISystemPreUpdate
{
public:
    RfTag();
    ~RfTag();

    void Configure(const Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   EntityComponentManager &ecm,
                   EventManager &eventMgr) override;

    void PreUpdate(const UpdateInfo &info,
                   EntityComponentManager &ecm) override;

private:
    void OnUpdate(const UpdateInfo &info, EntityComponentManager &ecm);

gz_sim::Entity entity;

    gz::sim::Entity parentSensor;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr rosPub;

    gz::common::ConnectionPtr updateConnection;
};

GZ_ADD_PLUGIN(RfTag,
              gz::sim::System,
              RfTag::ISystemConfigure,
              RfTag::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(RfTag, "gz::sim::systems::RfTag")

#endif  // RFID_TAG_H_
