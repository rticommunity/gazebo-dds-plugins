#ifndef GAZEBO_DDS_CXX
#define GAZEBO_DDS_CXX

#include "../include/gazebo-dds-laser.h"
#include "data-writer-listener.cxx"


namespace gazebo {
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboDDSLaser)

GazeboDDSLaser::GazeboDDSLaser()
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

    writer_.listener(
            new DataWriterListener<laser_Scan_msg>,
            dds::core::status::StatusMask::all());
}

GazeboDDSLaser::~GazeboDDSLaser()
{
}

void GazeboDDSLaser::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
{
    // load plugin
    RayPlugin::Load(parent, sdf);

    // Store the pointer to the sensor
    sensor_ = parent;

    this->gazeboNode_
            = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazeboNode_->Init(parent->WorldName());

    this->laserSubscriber_ = this->gazeboNode_->Subscribe(
            this->sensor_->Topic(), &GazeboDDSLaser::OnScan, this);
}

void GazeboDDSLaser::OnScan(ConstLaserScanStampedPtr &msg)
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


    sample.laser_id(sensor_->Id());
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
}  // namespace gazebo
#endif