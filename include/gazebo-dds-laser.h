#ifndef GAZEBO_DDS_H
#define GAZEBO_DDS_H

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

#include <rti/core/ListenerBinder.hpp>

#include "../build/generated/laserScanMsg.hpp"
#include "data-writer-listener.h"


namespace gazebo {

class GazeboDDSLaser : public RayPlugin {
public:
    GazeboDDSLaser();

    ~GazeboDDSLaser();

    void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf);

    void OnScan(ConstLaserScanStampedPtr &msg);

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
};

}  // namespace gazebo

#endif