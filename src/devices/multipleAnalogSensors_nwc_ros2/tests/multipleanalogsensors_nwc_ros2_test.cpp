/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <math.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <random>

#include <catch2/catch_amalgamated.hpp>
#include <harness.h>

using namespace std::chrono_literals;


TEST_CASE("dev::MultipleAnalogSensors_nwc_ros2_test", "[yarp::dev]")
{
    YARP_REQUIRE_PLUGIN("fakeIMU", "device");
    YARP_REQUIRE_PLUGIN("imu_nws_ros2", "device");

//#if defined(DISABLE_FAILING_TESTS)
//    YARP_SKIP_TEST("Skipping failing tests")
//#endif

    yarp::os::Network::setLocalMode(true);

    SECTION("Test the nwc alone")
    {
        yarp::dev::PolyDriver nwc;

        yarp::os::Property pNwc;
        pNwc.put("device", "imu_nwc_ros2");
        pNwc.put("node_name", "imu_node");
        pNwc.put("topic_name", "/imu_topic");
        pNwc.put("sensor_name", "imu_sensor");
        REQUIRE(nwc.open(pNwc)); // multipleanalogsensors nwc open reported successful

        // Close devices
        nwc.close();
    }

    SECTION("Test topic data reception")
    {
        yarp::dev::IThreeAxisGyroscopes* iTestGyro;
        yarp::dev::IThreeAxisLinearAccelerometers* iTestAccel;
        yarp::dev::IOrientationSensors *iTestOrient;

        yarp::dev::PolyDriver nwc;

        yarp::os::Property pNwc;
        std::string node_name = "imu_node";
        std::string topic_name = "/imu_topic";
        std::string sensor_name = "imu_sensor";
        pNwc.put("device", "imu_nwc_ros2");
        pNwc.put("node_name", node_name);
        pNwc.put("topic_name", topic_name);
        pNwc.put("sensor_name", sensor_name);
        REQUIRE(nwc.open(pNwc)); // multipleanalogsensors nwc open reported successful

        // Check Interfaces
        REQUIRE(nwc.view(iTestGyro)); // IThreeAxisGyroscopes view reported successul
        REQUIRE(iTestGyro != nullptr);
        REQUIRE(nwc.view(iTestAccel)); // IThreeAxisLinearAccelerometers view reported successul
        REQUIRE(iTestAccel != nullptr);
        REQUIRE(nwc.view(iTestOrient)); // IOrientationSensors view reported successul
        REQUIRE(iTestOrient != nullptr);

        std::stringstream callStream;

        // Calculate time stamp
        uint64_t sec_part;
        uint64_t nsec_part;
        double yarpTime = yarp::os::Time::now();
        sec_part = int(yarpTime);
        nsec_part = (yarpTime - sec_part)*1000000000UL;

        // Example values
        std::string frame_name = "test_imu_device";
        double lower_bound = 0.0;
        double upper_bound = 45.0;
        std::uniform_real_distribution<double> unif_orient(lower_bound,upper_bound);
        std::default_random_engine re;
        double roll = unif_orient(re);
        double pitch = unif_orient(re);
        double yaw = unif_orient(re);
        double cr = cos((roll*M_PI/180.0) * 0.5);
        double sr = sin((roll*M_PI/180.0) * 0.5);
        double cp = cos((pitch*M_PI/180.0) * 0.5);
        double sp = sin((pitch*M_PI/180.0) * 0.5);
        double cy = cos((yaw*M_PI/180.0) * 0.5);
        double sy = sin((yaw*M_PI/180.0) * 0.5);

        double orientW = cr * cp * cy + sr * sp * sy;
        double orientX = sr * cp * cy - cr * sp * sy;
        double orientY = cr * sp * cy + sr * cp * sy;
        double orientZ = cr * cp * sy - sr * sp * cy;

        upper_bound = 5.0;
        std::uniform_real_distribution<double> unif_angVel(lower_bound,upper_bound);
        double angSpX = unif_angVel(re);
        double angSpY = unif_angVel(re);
        double angSpZ = unif_angVel(re);

        upper_bound = 9.8;
        std::uniform_real_distribution<double> unif_linAcc(lower_bound,upper_bound);
        double linAccX = unif_linAcc(re);
        double linAccY = unif_linAcc(re);
        double linAccZ = unif_linAcc(re);

        callStream << "ros2 topic pub --once " << topic_name << " sensor_msgs/msg/Imu \"{header: {stamp: {sec: ";
        callStream << sec_part << ", nanosec: " << nsec_part << "}, frame_id: '" << frame_name << "'}, orientation: {x: ";
        callStream << orientX << ", y: " << orientY << ", z: " << orientZ << ", w: " << orientW << "}, angular_velocity: ";
        callStream << "{x: " << angSpX*M_PI/180.0 << ", y: " << angSpY*M_PI/180.0 << ", z: " << angSpZ*M_PI/180.0 << "}, linear_acceleration: ";
        callStream << "{x: " << linAccX << ", y: " << linAccY << ", z: " << linAccZ << "}}\"";

        system(callStream.str().c_str());

        while(iTestGyro->getThreeAxisGyroscopeStatus(0) != yarp::dev::MAS_status::MAS_OK)
        {
            std::this_thread::sleep_for(250ms);
        }

        double timeStamp;
        std::string gotSensName;
        std::string gotFrameName;
        size_t gotSensNum;

        // Test IOrientationSensors
        yarp::sig::Vector orient(3);
        gotSensNum = iTestOrient->getNrOfOrientationSensors();
        REQUIRE(gotSensNum==1);
        REQUIRE(iTestOrient->getOrientationSensorName(0,gotSensName));
        REQUIRE(gotSensName==sensor_name);
        REQUIRE(iTestOrient->getOrientationSensorFrameName(0,gotFrameName));
        REQUIRE(gotFrameName==frame_name);
        REQUIRE(iTestOrient->getOrientationSensorMeasureAsRollPitchYaw(0,orient,timeStamp));
        REQUIRE(roll == Catch::Approx(orient[0]));
        REQUIRE(pitch == Catch::Approx(orient[1]));
        REQUIRE(yaw == Catch::Approx(orient[2]));

        // Test IThreeAxisGyroscopes
        yarp::sig::Vector angSpeed(3);
        gotSensNum = iTestGyro->getNrOfThreeAxisGyroscopes();
        REQUIRE(gotSensNum==1);
        REQUIRE(iTestGyro->getThreeAxisGyroscopeName(0,gotSensName));
        REQUIRE(gotSensName==sensor_name);
        REQUIRE(iTestGyro->getThreeAxisGyroscopeFrameName(0,gotFrameName));
        REQUIRE(gotFrameName==frame_name);
        REQUIRE(iTestGyro->getThreeAxisGyroscopeMeasure(0,angSpeed,timeStamp));
        REQUIRE(angSpX == Catch::Approx(angSpeed[0]));
        REQUIRE(angSpY == Catch::Approx(angSpeed[1]));
        REQUIRE(angSpZ == Catch::Approx(angSpeed[2]));

        // Test IThreeAxisLinearAccelerometers
        yarp::sig::Vector linAccel(3);
        gotSensNum = iTestAccel->getNrOfThreeAxisLinearAccelerometers();
        REQUIRE(gotSensNum==1);
        REQUIRE(iTestAccel->getThreeAxisLinearAccelerometerName(0,gotSensName));
        REQUIRE(gotSensName==sensor_name);
        REQUIRE(iTestAccel->getThreeAxisLinearAccelerometerFrameName(0,gotFrameName));
        REQUIRE(gotFrameName==frame_name);
        REQUIRE(iTestAccel->getThreeAxisLinearAccelerometerMeasure(0,linAccel,timeStamp));
        REQUIRE(linAccX == Catch::Approx(linAccel[0]));
        REQUIRE(linAccY == Catch::Approx(linAccel[1]));
        REQUIRE(linAccZ == Catch::Approx(linAccel[2]));

        // Close devices
        nwc.close();
    }

    yarp::os::Network::setLocalMode(false);
}
