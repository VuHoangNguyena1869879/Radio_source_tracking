#include "RfReceiver.h"
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/math/Vector3.hh>

using namespace gz;
using namespace sim;
using namespace systems;

/////////////////////////////////////////////////
RfReceiver::RfReceiver()
: _generator(std::random_device{}()),
  _distribution(0.0, 1.0),
  _radius_distribution(0.0, 1.0)
{
    // Initialize ROS node
    rclcpp::init(0, nullptr);
    _node_handle = rclcpp::Node::make_shared("rf_receiver_node");

    // Initialize ROS publisher
    _event_pub = _node_handle->create_publisher<rf_sensor::msg::TagArray>("rf_signal_data", 10);
}

RfReceiver::~RfReceiver()
{
    rclcpp::shutdown();
}

/////////////////////////////////////////////////
void RfReceiver::Configure(const Entity &entity,
                           const std::shared_ptr<const sdf::Element> &sdf,
                           EntityComponentManager &ecm,
                           EventManager &eventMgr)
{
    _entity = entity;

    // Parse SDF parameters
    if (sdf->HasElement("frequency"))
        _frequency = sdf->Get<double>("frequency");
    else
        _frequency = 865.7e6;

    if (sdf->HasElement("noisephi"))
        _noisephi = sdf->Get<double>("noisephi");
    else
        _noisephi = 0.1;

    if (sdf->HasElement("noiserssi"))
        _noiserssi = sdf->Get<double>("noiserssi");
    else
        _noiserssi = 0.005;

    if (sdf->HasElement("range"))
        _range = sdf->Get<double>("range");
    else
        _range = 5;

    if (sdf->HasElement("optional_distribution"))
        _optional_distribution = sdf->Get<std::string>("optional_distribution");
    else
        _optional_distribution = "";

    if (sdf->HasElement("optional_distribution_params"))
        _optional_distribution_params = sdf->Get<std::string>("optional_distribution_params");
    else
        _optional_distribution_params = "";

    if (!_optional_distribution.empty() && _optional_distribution != "none")
    {
        std::string temp;
        std::istringstream ss(_optional_distribution_params);
        while (ss >> temp)
        {
            if (std::regex_match(temp, std::regex("^(-?)(0|([1-9][0-9]*))(\\.[0-9]+)?$")))
                _params.push_back(std::stod(temp));
            else
            {
                gzerr << "Only doubles separated by space are supported" << std::endl;
                return;
            }
        }
    }

    if (_optional_distribution == "uniform_real_distribution")
        _optional_dist1 = std::uniform_real_distribution<double>(_params[0], _params[1]);
    else if (_optional_distribution == "exponential_distribution")
        _optional_dist2 = std::exponential_distribution<double>(_params[0]);
    else if (_optional_distribution == "gamma_distribution")
        _optional_dist3 = std::gamma_distribution<double>(_params[0], _params[1]);
    else if (_optional_distribution == "weibull_distribution")
        _optional_dist4 = std::weibull_distribution<double>(_params[0], _params[1]);
    else if (_optional_distribution == "normal_distribution")
        _optional_dist5 = std::normal_distribution<double>(_params[0], _params[1]);
    else if (_optional_distribution == "lognormal_distribution")
        _optional_dist6 = std::lognormal_distribution<double>(_params[0], _params[1]);
    else if (_optional_distribution == "chi_squared_distribution")
        _optional_dist7 = std::chi_squared_distribution<double>(_params[0]);
    else if (_optional_distribution == "cauchy_distribution")
        _optional_dist8 = std::cauchy_distribution<double>(_params[0], _params[1]);
    else if (!_optional_distribution.empty() && _optional_distribution != "none")
    {
        gzerr << "Distribution not supported" << std::endl;
        return;
    }

    if (sdf->HasElement("communication_gain"))
        _communication_gain = sdf->Get<double>("communication_gain");
    else
        _communication_gain = 250;

    _lambda = 299792458.0 / _frequency;
    _meterToPhi = (4 * M_PI) / _lambda;

    _event_pub = _node_handle->create_publisher<rf_sensor::msg::TagArray>("data", 10);

    gzmsg << "Antenna properties:\n"
          << "\tFrequency: " << _frequency << "\n"
          << "\tNoise phi (rad): " << _noisephi << "\n"
          << "\tNoise rssi: " << _noiserssi << "\n"
          << "\tOptional distribution: " << _optional_distribution << " with params: " << _optional_distribution_params << "\n"
          << "\tRange: " << _range << "\n"
          << "\tCommunication Gain: " << _communication_gain << "\n"
          << "\tLambda: " << _lambda << "\n";
}

/////////////////////////////////////////////////
void RfReceiver::PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm)
{
    OnUpdate(info, ecm);
}

/////////////////////////////////////////////////
void RfReceiver::OnUpdate(const UpdateInfo &info, EntityComponentManager &ecm)
{
    auto pose = ecm.Component<components::Pose>(_entity);
    if (pose == nullptr)
    {
        gzerr << "Pose component not found for entity " << _entity << std::endl;
        return;
    }

    gz::math::Pose3d world_antenna_position = pose->Data();

    // Iterate through the tags and calculate the required values
    std::vector<rf_sensor::msg::Tag> msgArray;

    for (const auto &tag_entry : _tags_map)
    {
        // Obtain the tag entity and its position
        auto tag_entity = ecm.EntityByComponents(components::Name(tag_entry.first));
        if (tag_entity == kNullEntity)
            continue;

        auto tag_pose = ecm.Component<components::Pose>(tag_entity);
        if (tag_pose == nullptr)
            continue;

        gz::math::Pose3d world_tag_position = tag_pose->Data();
        double dist = world_antenna_position.Pos().Distance(world_tag_position.Pos());

        if (dist > _range)
            continue;

        // Calculate the angle of incidence
        gz::math::Vector3d direction = (world_tag_position.Pos() - world_antenna_position.Pos()).Normalized();
        double angle = acos(world_antenna_position.Rot().RotateVector(direction).Dot(gz::math::Vector3d::UnitX));

        // Calculate gain
        double gain = AntennaGain(angle);

        // Check for occlusion
        bool occluded = IsOccluded(world_antenna_position.Pos(), world_tag_position.Pos(), ecm);

        if (!occluded)
        {
            double random_number = 0;
            if (!_optional_distribution.empty() && _optional_distribution != "none")
            {
                if (_optional_distribution == "uniform_real_distribution")
                    random_number = _optional_dist1(_generator);
                else if (_optional_distribution == "exponential_distribution")
                    random_number = _optional_dist2(_generator);
                else if (_optional_distribution == "gamma_distribution")
                    random_number = _optional_dist3(_generator);
                else if (_optional_distribution == "weibull_distribution")
                    random_number = _optional_dist4(_generator);
                else if (_optional_distribution == "normal_distribution")
                    random_number = _optional_dist5(_generator);
                else if (_optional_distribution == "lognormal_distribution")
                    random_number = _optional_dist6(_generator);
                else if (_optional_distribution == "chi_squared_distribution")
                    random_number = _optional_dist7(_generator);
                else if (_optional_distribution == "cauchy_distribution")
                    random_number = _optional_dist8(_generator);
            }

            // Calculate RSSI and phase here
            double rssi = _communication_gain - (20 * log10(dist) + random_number) * gain;
            double phi = _meterToPhi * dist + random_number;

            rf_sensor::msg::Tag tag_msg;
            tag_msg.name = tag_entry.first;  // Use the correct field name 'name'
            tag_msg.rssi = rssi;
            tag_msg.phi = phi;  // Use the correct field name 'phi'

            msgArray.push_back(tag_msg);
        }
    }

    rf_sensor::msg::TagArray tagArray_msg;
    tagArray_msg.tags = msgArray;

    _event_pub->publish(tagArray_msg);
}

/////////////////////////////////////////////////
double RfReceiver::AntennaGain(double angle)
{
    return 1.0; 
}

/////////////////////////////////////////////////
bool RfReceiver::IsOccluded(const gz::math::Vector3d &start, const gz::math::Vector3d &end, EntityComponentManager &ecm)
{
    return false;
}

