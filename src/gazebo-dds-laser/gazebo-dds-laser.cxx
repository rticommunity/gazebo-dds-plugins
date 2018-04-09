#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <math.h>
#include <sdf/sdf.hh>

#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <ignition/math/Vector3.hh>

#include <algorithm>

#include <dds/core/ddscore.hpp>
#include <dds/pub/ddspub.hpp>
#include <dds/sub/ddssub.hpp>
#include <rti/util/util.hpp>

#include "laserScanMsg.hpp"

namespace gazebo {
class GazeboDDSLaser : public RayPlugin {
public:
    GazeboDDSLaser()
            : participant_(226),
              topic_(participant_, "laserScan"),
              writer_(dds::core::null)
    {
        rti::core::policy::Property::Entry value(
                { "dds.data_writer.history.memory_manager.fast_pool.pool_"
                  "buffer_max_size",
                  "4096" });
        rti::core::policy::Property property;
        property.set(value);
        data_writer_qos_ << property;
        writer_ = dds::pub::DataWriter<laser_Scan_msg>(
                dds::pub::Publisher(participant_), topic_, data_writer_qos_);
    }

public:
    ~GazeboDDSLaser()
    {
    }

public:
    void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
    {
        // load plugin
        RayPlugin::Load(parent, sdf);

        // Store the pointer to the sensor
        sensor_ = parent;

        std::cout << "Laser plugin loaded" << std::endl;
        sensor_id_= sensor_->Id();
        std::cout << "ID:"<< sensor_id_ << std::endl;

        this->gazeboNode_
                = gazebo::transport::NodePtr(new gazebo::transport::Node());
        this->gazeboNode_->Init(parent->WorldName());

        this->laserSubscriber_ = this->gazeboNode_->Subscribe(
                this->sensor_->Topic(), &GazeboDDSLaser::OnScan, this);
    }

public:
    void OnScan(ConstLaserScanStampedPtr &msg)
    {
        laser_Scan_msg sample;

        Position position(
                msg->scan().world_pose().position().x(),
                msg->scan().world_pose().position().y(),
                msg->scan().world_pose().position().z());

        Orientation orientation(
                msg->scan().world_pose().orientation().x(),
                msg->scan().world_pose().orientation().y(),
                msg->scan().world_pose().orientation().z(),
                msg->scan().world_pose().orientation().w());

        sample.header(
                Header(Time(msg->time().sec(), msg->time().nsec()),
                       sensor_->ParentName()));

        sample.world_pose(World_Pose(position, orientation));
        sample.angle_min(msg->scan().angle_min());
        sample.angle_max(msg->scan().angle_max());
        sample.angle_step(msg->scan().angle_step());
        sample.range_min(msg->scan().range_min());
        sample.range_max(msg->scan().range_max());
        sample.count(msg->scan().count());
        sample.vertical_angle_min(msg->scan().vertical_angle_min());
        sample.vertical_angle_max(msg->scan().vertical_angle_max());
        sample.vertical_angle_step(msg->scan().vertical_angle_step());
        sample.vertical_count(msg->scan().vertical_count());

        sample.ranges().resize(msg->scan().ranges_size());
        std::copy(
                msg->scan().ranges().begin(),
                msg->scan().ranges().end(),
                sample.ranges().begin());

        sample.intensities().resize(msg->scan().intensities_size());
        std::copy(
                msg->scan().intensities().begin(),
                msg->scan().intensities().end(),
                sample.intensities().begin());

        writer_.write(sample);
    }

    
private:
    // Pointer to the world
    physics::WorldPtr world_;

    // Pointer to the sensor
    sensors::SensorPtr sensor_;

    dds::domain::DomainParticipant participant_;

    dds::topic::Topic<laser_Scan_msg> topic_;

    dds::pub::qos::DataWriterQos data_writer_qos_;

    dds::pub::DataWriter<laser_Scan_msg> writer_;

    gazebo::transport::NodePtr gazeboNode_;

    gazebo::transport::SubscriberPtr laserSubscriber_;

    unsigned int sensor_id_;
};

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboDDSLaser)
}  // namespace gazebo