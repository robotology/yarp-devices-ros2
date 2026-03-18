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

TEST_CASE("dev::hapticDevice_nws_ros2_test", "[yarp::dev]")
{
    YARP_REQUIRE_PLUGIN("hapticDevice_nws_ros2", "device");

    Network::setLocalMode(true);

    SECTION("Checking the nws alone")
    {
        PolyDriver ddnws;

        // Checking opening nws
        {
            Property pcfg;
            pcfg.put("device",     "hapticDevice_nws_ros2");
            pcfg.put("node_name",  "haptic_node");
            pcfg.put("topic_name", "/hapticDevice_nws_ros2/haptic");
            REQUIRE(ddnws.open(pcfg));
        }

        // Close and check
        {
            CHECK(ddnws.close());
        }
    }

    SECTION("Checking attach with invalid device")
    {
        PolyDriver ddnws;
        yarp::dev::WrapperSingle* ww_nws = nullptr;

        // Open nws
        {
            Property pcfg;
            pcfg.put("device",    "hapticDevice_nws_ros2");
            pcfg.put("node_name", "haptic_node");
            pcfg.put("topic_name", "/hapticDevice_nws_ros2/haptic");
            REQUIRE(ddnws.open(pcfg));
        }

        // Attach with nullptr should fail
        {
            ddnws.view(ww_nws);
            REQUIRE(ww_nws != nullptr);
            bool result_att = ww_nws->attach(nullptr);
            CHECK_FALSE(result_att);
        }

        // Close and check
        {
            CHECK(ddnws.close());
        }
    }

    Network::setLocalMode(false);
}
