/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/WrapperSingle.h>

#include <catch2/catch_amalgamated.hpp>
#include <harness.h>

using namespace yarp::dev;
using namespace yarp::os;

TEST_CASE("dev::battery_nws_ros2_test", "[yarp::dev]")
{
    YARP_REQUIRE_PLUGIN("ros2PubTest", "device");

    Network::setLocalMode(true);

    SECTION("Checking the nws alone")
    {
        PolyDriver ddnws;

        ////////"Checking opening nws"
        {
            Property pcfg;
            pcfg.put("device", "ros2PubTest");
            REQUIRE(ddnws.open(pcfg));
        }

        //"Close all polydrivers and check"
        {
            CHECK(ddnws.close());
        }
    }
    Network::setLocalMode(false);
}
