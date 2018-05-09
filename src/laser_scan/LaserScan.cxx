#ifndef DDS_LASER_SCAN_CXX
#define DDS_LASER_SCAN_CXX

#include "LaserScan.h"
#include "DataWriterListener.cxx"
#include "Properties.h"

namespace gazebo {

namespace dds {

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(LaserScan)

LaserScan::LaserScan()
        : participant_(::dds::core::null),
          topic_(::dds::core::null),
          writer_(::dds::core::null)
{
}

LaserScan::~LaserScan()
{
}

void LaserScan::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
{
    // load plugin
    RayPlugin::Load(parent, sdf);

    // Store the pointer to the sensor
    sensor_ = parent;

    laser_connect_count_ = 0;

    gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gazebo_node_->Init(parent->WorldName());

    // Obtain the domain id from loaded world
    int domain_id = 0;
    if (sdf->HasElement(DOMAIN_ID_PROPERTY_NAME)) {
        domain_id = sdf->Get<int>(DOMAIN_ID_PROPERTY_NAME);
    }

    participant_ = ::dds::domain::find(domain_id);
    if (participant_ == ::dds::core::null) {
        participant_ = ::dds::domain::DomainParticipant(domain_id);
    }

    // Obtain the topic name from loaded world
    std::string topic_name = "laserScan";
    if (sdf->HasElement(TOPIC_NAME_PROPERTY_NAME)) {
        topic_name = sdf->Get<std::string>(TOPIC_NAME_PROPERTY_NAME);
    }

    topic_ = ::dds::topic::Topic<LaserScanMsg>(participant_, topic_name);

    // Change the maximum size of the sequences
    rti::core::policy::Property::Entry value(
            { "dds.data_writer.history.memory_manager.fast_pool.pool_"
              "buffer_max_size",
              "4096" });

    rti::core::policy::Property property;
    property.set(value);
    data_writer_qos_ << property;
    writer_ = ::dds::pub::DataWriter<LaserScanMsg>(
            ::dds::pub::Publisher(participant_), topic_, data_writer_qos_);

    // Add a new listener to the DataWriter
    writer_.listener(
            new DataWriterListener<LaserScanMsg>(
                    std::bind(&LaserScan::LaserConnect, this),
                    std::bind(&LaserScan::LaserDisconnect, this)),
            ::dds::core::status::StatusMask::publication_matched());

    gzmsg << "Starting Laser Plugin - Topic name: " << topic_name << std::endl;
}

void LaserScan::LaserConnect()
{
    this->laser_connect_count_++;
    if (this->laser_connect_count_ == 1)
        this->laser_scan_sub_ = this->gazebo_node_->Subscribe(
                this->sensor_->Topic(), &LaserScan::OnScan, this);
}

void LaserScan::LaserDisconnect()
{
    this->laser_connect_count_--;
    if (this->laser_connect_count_ == 0)
        this->laser_scan_sub_.reset();
}

void LaserScan::OnScan(ConstLaserScanStampedPtr &msg)
{
    LaserScanMsg sample;

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

    sample.world_pose(WorldPose(position, orientation));
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

}  // namespace dds
}  // namespace gazebo

#endif  // DDS_LASER_SCAN_CXX
