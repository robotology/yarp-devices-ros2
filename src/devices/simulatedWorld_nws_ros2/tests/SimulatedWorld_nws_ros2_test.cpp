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
    YARP_REQUIRE_PLUGIN("fakeSimulatedWorld", "device");

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

    SECTION("Checking simulatedWorld_nws_ros2 attached to fakeSimulatedWorld")
    {
        PolyDriver dd_nws;
        PolyDriver dd_fake;
        yarp::dev::WrapperSingle* ww_nws = nullptr;

        {
            Property p_nws;
            p_nws.put("device", "simulatedWorld_nws_ros2");
            p_nws.put("node_name", "simulatedworld_nws_ros2_test");
            REQUIRE(dd_nws.open(p_nws));
        }

        {
            Property p_fake;
            p_fake.put("device", "fakeSimulatedWorld");
            p_fake.put("period", 10);
            REQUIRE(dd_fake.open(p_fake));
        }

        // Attach the nws to the fake device
        {
            dd_nws.view(ww_nws);
            REQUIRE(ww_nws != nullptr);
            bool result_att = ww_nws->attach(&dd_fake);
            REQUIRE(result_att);
        }

        {
            CHECK(dd_nws.close());
            CHECK(dd_fake.close());
        }
    }

    Network::setLocalMode(false);
}
