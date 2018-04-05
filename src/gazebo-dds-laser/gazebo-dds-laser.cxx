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
            : participant(226),
              topic(participant, "laserScan"),
              writer(dds::pub::Publisher(participant), topic)
    {
    }

public:
    ~GazeboDDSLaser()
    {
    }

public:
    void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
        // load plugin
        RayPlugin::Load(_parent, _sdf);

        // Store the pointer to the sensor
        sensor = _parent;

        std::cout << "It was loaded" << std::endl;

        this->gazeboNode
                = gazebo::transport::NodePtr(new gazebo::transport::Node());
        this->gazeboNode->Init(_parent->WorldName());

        this->laserSubscriber = this->gazeboNode->Subscribe(
                this->sensor->Topic(), &GazeboDDSLaser::OnScan, this);
    }

public:
    void OnScan(ConstLaserScanStampedPtr &msg)
    {
        std::cout << "It was updated :" << msg->scan().angle_max() << std::endl;
        laser_Scan_msg sample;

        Time time;
        time.nsec(msg->time().sec());
        time.sec(msg->time().nsec());

        Header header;
        header.stamp(time);
        // header.frame_id();???

        sample.header(header);
        sample.angle_min(msg->scan().angle_min());
        sample.angle_max(msg->scan().angle_max());
        sample.angle_increment(msg->scan().angle_step());
        sample.time_increment(0);
        sample.scan_time(0);
        sample.range_min(msg->scan().range_min());
        sample.range_max(msg->scan().range_max());
        
        sample.ranges().resize(msg->scan().ranges_size());
        int i = 0;
        for(auto & range : msg->scan().ranges()){
            sample.ranges().at(i)=range;
            i++;
        }

        writer.write(sample);
    }

    // Pointer to the world
private:
    physics::WorldPtr world;

    // Pointer to the sensor
private:
    sensors::SensorPtr sensor;

    // Pointer to the update event connection
private:
    event::ConnectionPtr updateConnection;

private:
    dds::domain::DomainParticipant participant;

private:
    dds::topic::Topic<laser_Scan_msg> topic;

private:
    dds::pub::DataWriter<laser_Scan_msg> writer;

private:
    gazebo::transport::NodePtr gazeboNode;

private:
    gazebo::transport::SubscriberPtr laserSubscriber;

private:
    std::string frameName;
};

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboDDSLaser)
}  // namespace gazebo