/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/os/Network.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/WrapperSingle.h>
#include <yarp/dev/IRGBDSensor.h>
#include <yarp/dev/tests/IRGBDSensorTest.h>

#include <catch2/catch_amalgamated.hpp>
#include <harness.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

TEST_CASE("dev::Rangefinder2D_nws_ros2_test", "[yarp::dev]")
{
    YARP_REQUIRE_PLUGIN("fakeDepthCamera", "device");
    YARP_REQUIRE_PLUGIN("rgbdSensor_nws_ros2", "device");

    Network::setLocalMode(true);

    SECTION("Checking the nws alone")
    {
        PolyDriver ddnws;

        ////////"Checking opening nws"
        {
            Property pcfg;
            pcfg.put("device", "rgbdSensor_nws_ros2");
            pcfg.put("node_name", "rgbd_node");
            pcfg.put("depth_topic_name","/depth_topic");
            pcfg.put("color_topic_name","/rgbd_topic");
            pcfg.put("depth_frame_id","depthframe");
            pcfg.put("color_frame_id","colorframe");
            REQUIRE(ddnws.open(pcfg));
        }

        //"Close all polydrivers and check"
        {
            CHECK(ddnws.close());
        }
    }

    SECTION("Checking the nws attached to device")
    {
        PolyDriver ddnws;
        PolyDriver ddfake;
        yarp::dev::WrapperSingle* ww_nws = nullptr;

        ////////"Checking opening nws"
        {
            Property pcfg;
            pcfg.put("device", "rgbdSensor_nws_ros2");
            pcfg.put("node_name", "rgbd_node");
            pcfg.put("depth_topic_name","/depth_topic");
            pcfg.put("color_topic_name","/rgbd_topic");
            pcfg.put("depth_frame_id","depthframe");
            pcfg.put("color_frame_id","colorframe");
            REQUIRE(ddnws.open(pcfg));
        }

        ////////"Checking opening device"
        {
            Property pcfg_fake;
            pcfg_fake.put("device", "fakeDepthCamera");
            REQUIRE(ddfake.open(pcfg_fake));
        }

        //attach the nws to the fake device
        {
            ddnws.view(ww_nws);
            bool result_att = ww_nws->attach(&ddfake);
            REQUIRE(result_att);
        }

        //"Close all polydrivers and check"
        {
            CHECK(ddnws.close());
            CHECK(ddfake.close());
        }
    }

    Network::setLocalMode(false);
}
