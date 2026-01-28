/*
 * SPDX-FileCopyrightText: 2026 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/os/Network.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/WrapperSingle.h>

#include <catch2/catch_amalgamated.hpp>
#include <harness.h>

using namespace yarp::os;
using namespace yarp::dev;

TEST_CASE("dev::SimulatedWorld_nws_ros2_test", "[yarp::dev]")
{
    YARP_REQUIRE_PLUGIN("simulatedWorld_nws_ros2", "device");

    Network::setLocalMode(true);

    SECTION("Checking simulatedWorld_nws_ros2 device")
    {
        PolyDriver dd_nws;

        {
            Property p_nws;
            p_nws.put("device", "simulatedWorld_nws_ros2");
            REQUIRE(dd_nws.open(p_nws));
        }

        {
            CHECK(dd_nws.close());
        }
    }

    Network::setLocalMode(false);
}
