#ifndef GAZEBO_DDS_LASER_H
#define GAZEBO_DDS_LASER_H

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
    /**
     * @brief Constructor
     */
    GazeboDDSLaser();

    /**
     * @brief Destructor
     */
    ~GazeboDDSLaser();

    /**
     * @brief Load the plugin inside Gazebo's system
     *
     * @param parent object of Gazebo's sensor
     * @param sdf object of Gazebo's world
     */
    void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf);

    /**
     * @brief Read the current sensor's information 
     *
     * @param msg current information of the sensor
     */
    void OnScan(ConstLaserScanStampedPtr &msg);

private:

    int laser_connect_count_;
    void LaserConnect();
    void LaserDisconnect();

    physics::WorldPtr world_;

    sensors::SensorPtr sensor_;

    dds::domain::DomainParticipant participant_;

    dds::topic::Topic<laser_Scan_msg> topic_;

    dds::pub::qos::DataWriterQos data_writer_qos_;

    dds::pub::DataWriter<laser_Scan_msg> writer_;

    gazebo::transport::NodePtr gazebo_node_;

    gazebo::transport::SubscriberPtr laser_scan_sub_;
};

}  // namespace gazebo

#endif