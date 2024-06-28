#ifndef RF_RECEIVER_H_
#define RF_RECEIVER_H_

#include <string>
#include <vector>
#include <map>
#include <random>
#include <regex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Name.hh>
#include <gz/msgs.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>

#include <rf_sensor/msg/tag.hpp>
#include <rf_sensor/msg/tag_array.hpp>

using namespace gz;
using namespace sim;
using namespace systems;

class RfReceiver : public System,
                   public ISystemConfigure,
                   public ISystemPreUpdate
{
public:
    RfReceiver();
    ~RfReceiver();

    void Configure(const Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   EntityComponentManager &ecm,
                   EventManager &eventMgr) override;

    void PreUpdate(const UpdateInfo &info,
                   EntityComponentManager &ecm) override;

private:
    void OnUpdate(const UpdateInfo &info, EntityComponentManager &ecm);

    // New methods for antenna gain and occlusion handling
    double AntennaGain(double angle);
    bool IsOccluded(const gz::math::Vector3d &start, const gz::math::Vector3d &end, EntityComponentManager &ecm);

    Entity _entity;
    std::shared_ptr<gz::transport::Node> _node;
    rclcpp::Node::SharedPtr _node_handle;
    rclcpp::Publisher<rf_sensor::msg::TagArray>::SharedPtr _event_pub;

    std::map<std::string, Entity> _tags_map;

    double _frequency;
    double _noisephi;
    double _noiserssi;
    double _lambda;
    double _meterToPhi;
    double _range;
    double _communication_gain;
    std::string _optional_distribution;
    std::string _optional_distribution_params;
    std::vector<double> _params;

    int _number_of_tags;

    std::default_random_engine _generator;
    std::normal_distribution<double> _distribution;
    std::normal_distribution<double> _radius_distribution;

    std::uniform_real_distribution<double> _optional_dist1;
    std::exponential_distribution<double> _optional_dist2;
    std::gamma_distribution<double> _optional_dist3;
    std::weibull_distribution<double> _optional_dist4;
    std::normal_distribution<double> _optional_dist5;
    std::lognormal_distribution<double> _optional_dist6;
    std::chi_squared_distribution<double> _optional_dist7;
    std::cauchy_distribution<double> _optional_dist8;

    gz::transport::Node::Publisher _scanPub;
};

GZ_ADD_PLUGIN(RfReceiver,
              gz::sim::System,
              RfReceiver::ISystemConfigure,
              RfReceiver::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(RfReceiver, "gz::sim::systems::RfReceiver")

#endif  // RF_RECEIVER_H_
