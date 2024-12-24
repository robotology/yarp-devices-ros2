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

TEST_CASE("dev::Rangefinder2D_nws_ros2_test", "[yarp::dev]")
{
    YARP_REQUIRE_PLUGIN("rangefinder2D_nws_ros2", "device");
    YARP_REQUIRE_PLUGIN("fakeLaser", "device");

    Network::setLocalMode(true);

    SECTION("Checking the nws alone")
    {
        PolyDriver nws_driver;

        ////////"Checking opening polydriver"
        {
            Property nws_cfg;
            nws_cfg.put("device", "rangefinder2D_nws_ros2");
            nws_cfg.put("node_name", "lidar_node");
            nws_cfg.put("topic_name","/lidar");
            REQUIRE(nws_driver.open(nws_cfg));
        }

        //Close all polydrivers and check
        CHECK(nws_driver.close());
    }

    SECTION("Checking the nws attached to device")
    {
        PolyDriver ddnws;
        PolyDriver ddfake;
        yarp::dev::WrapperSingle* ww_nws = nullptr;

        ////////"Checking opening nws"
        {
            Property pcfg;
            pcfg.put("device", "rangefinder2D_nws_ros2");
            pcfg.put("node_name", "lidar_node");
            pcfg.put("topic_name","/lidar");
            REQUIRE(ddnws.open(pcfg));
        }

        ////////"Checking opening device"
        {
            Property pcfg_fake;
            pcfg_fake.put("device", "fakeLaser");
            pcfg_fake.put("test", "use_pattern");
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
