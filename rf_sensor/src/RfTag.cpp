#include "RfTag.h"

using namespace gz;
using namespace sim;
using namespace systems;

RfTag::RfTag()
{
    // Initialize publisher
    _pub = _node.Advertise<gz::msgs::Pose>("/Rf/tag_pose");
}

RfTag::~RfTag()
{
}

void RfTag::Configure(const Entity &entity,
                        const std::shared_ptr<const sdf::Element> &sdf,
                        EntityComponentManager &ecm,
                        EventManager &eventMgr)
{
    _entity = entity;

    // Get the parent sensor
    parentSensor = std::dynamic_pointer_cast<gz::sim::Sensor>(gz::sim::Sensor(entity));

    if (!parentSensor)
    {
        gzerr << "RfTag requires a valid sensor.\n";
        return;
    }

    // Connect to the update event
    updateConnection = eventMgr.Connect<gz::sim::events::PreUpdate>(
        std::bind(&RfTag::OnUpdate, this, std::placeholders::_1, std::placeholders::_2));
}

void RfTag::PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm)
{
    OnUpdate(info, ecm);
}

void RfTag::OnUpdate(const UpdateInfo &info, EntityComponentManager &ecm)
{
    if (!parentSensor)
        return;

    // Create the message
    gz::msgs::Pose msg;
    auto poseComp = ecm.Component<components::Pose>(_entity);

    if (poseComp)
    {
        msgs::Set(&msg, poseComp->Data());
        _pub.Publish(msg);
    }
}
