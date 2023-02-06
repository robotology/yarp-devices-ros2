/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/dev/INavigation2D.h>
#include <yarp/dev/IMap2D.h>
#include <yarp/dev/Map2DLocation.h>
#include <yarp/dev/Map2DArea.h>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/WrapperSingle.h>
#include <yarp/dev/tests/IMap2DTest.h>

#include <catch2/catch_amalgamated.hpp>
#include <harness.h>

using namespace yarp::dev;
using namespace yarp::dev::Nav2D;
using namespace yarp::sig;
using namespace yarp::os;

TEST_CASE("dev::Map2D_nws_ros2_test", "[yarp::dev]")
{
    YARP_REQUIRE_PLUGIN("map2D_nws_ros2", "device");
    YARP_REQUIRE_PLUGIN("map2DStorage", "device");

    Network::setLocalMode(true);

    SECTION("Checking map2D_nws_ros2 device")
    {
        PolyDriver ddmapserver;

        ////////"Checking opening nws polydrivers"
        {
            Property pmapserver_cfg;
            pmapserver_cfg.put("device", "map2D_nws_ros2");
            REQUIRE(ddmapserver.open(pmapserver_cfg));
        }

        //"Close all polydrivers and check"
        {
            CHECK(ddmapserver.close());
        }
    }

    Network::setLocalMode(false);
}
