#ifndef GAZEBO_DDS_LASER_CXX
#define GAZEBO_DDS_LASER_CXX

#include "../include/gazebo_dds_laser.h"
#include "data_writer_listener.cxx"


namespace gazebo {
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboDDSLaser)

GazeboDDSLaser::GazeboDDSLaser()
        : participant_(dds::core::null),
          topic_(dds::core::null),
          writer_(dds::core::null)
{
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

    laser_connect_count_ = 0;

    gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gazebo_node_->Init(parent->WorldName());

    int domain_id;
    if (!sdf->HasElement("domain_id"))
        domain_id = 0;
    else
        domain_id = sdf->Get<int>("domain_id");

    participant_ = dds::domain::find(domain_id);
    if (participant_ == dds::core::null) {
        participant_ = dds::domain::DomainParticipant(domain_id);
    }

    std::string topic_name= "laserScan";

    if (sdf->HasElement("topic_name"))
        topic_name=sdf->Get<std::string>("topic_name");
    
    topic_ = dds::topic::Topic<laser_Scan_msg>(participant_, topic_name);


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
            new DataWriterListener<laser_Scan_msg>(
                    std::bind(&GazeboDDSLaser::LaserConnect, this),
                    std::bind(&GazeboDDSLaser::LaserDisconnect, this)),
            dds::core::status::StatusMask::all());

    std::cout << "Starting Laser Plugin - Topic name: " << topic_name << std::endl;
}

void GazeboDDSLaser::LaserConnect()
{
    this->laser_connect_count_++;
    if (this->laser_connect_count_ == 1)
        this->laser_scan_sub_ = this->gazebo_node_->Subscribe(
                this->sensor_->Topic(), &GazeboDDSLaser::OnScan, this);
}

void GazeboDDSLaser::LaserDisconnect()
{
    this->laser_connect_count_--;
    if (this->laser_connect_count_ == 0)
        this->laser_scan_sub_.reset();
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