/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/os/Network.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/WrapperSingle.h>

#include <catch2/catch_amalgamated.hpp>
#include <harness.h>

using namespace yarp::dev;
using namespace yarp::os;

TEST_CASE("dev::controlBoard_nws_ros2_test", "[yarp::dev]")
{
    YARP_REQUIRE_PLUGIN("controlBoard_nws_ros2", "device");
    YARP_REQUIRE_PLUGIN("fakeMotionControl", "device");

    Network::setLocalMode(true);

    SECTION("Checking nws alone")
    {
        PolyDriver ddnws;

        ////////"Checking opening nws"
        {
            Property pcfg;
            pcfg.put("device", "controlBoard_nws_ros2");
            pcfg.put("node_name", "controlboard_node");
            pcfg.put("topic_name","/controlBoard_nws_ros2/robot_part");
            REQUIRE(ddnws.open(pcfg));
        }

        //"Close all polydrivers and check"
        {
            CHECK(ddnws.close());
        }
    }

    SECTION("Checking nws attacehd to device")
    {
        PolyDriver ddnws;
        PolyDriver ddfake;
        yarp::dev::WrapperSingle* ww_nws = nullptr;

        ////////"Checking opening nws"
        {
            Property pcfg;
            pcfg.put("device", "controlBoard_nws_ros2");
            pcfg.put("node_name", "controlboard_node");
            pcfg.put("topic_name","/controlBoard_nws_ros2/robot_part");
            REQUIRE(ddnws.open(pcfg));
        }

        ////////"Checking opening device"
        {
            Property pcfg_fake;
            pcfg_fake.put("device", "fakeMotionControl");
            pcfg_fake.put("node_name", "controlboard_node");
            REQUIRE(ddfake.open(pcfg_fake));
        }

        //attach the nws to the fakelaser device
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
