/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/dev/IFrameGrabberImage.h>
#include <yarp/dev/IRgbVisualParams.h>

#include <yarp/dev/tests/IFrameGrabberImageTest.h>
#include <yarp/dev/tests/IRgbVisualParamsTest.h>

#include <yarp/os/Network.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/dev/WrapperSingle.h>

#include <string>

#include <catch2/catch_amalgamated.hpp>
#include <harness.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


TEST_CASE("dev::frameGrabber_nws_ros2_test", "[yarp::dev]")
{
    YARP_REQUIRE_PLUGIN("fakeFrameGrabber", "device");
    YARP_REQUIRE_PLUGIN("frameGrabber_nws_ros2", "device");

    Network::setLocalMode(true);
    
    SECTION("Checking the nws alone")
    {
        PolyDriver ddnws;

        ////////"Checking opening nws"
        {
            Property pcfg;
            pcfg.put("device", "frameGrabber_nws_ros2");
            pcfg.put("node_name", "frameGrabber_node");
            pcfg.put("topic_name","/controlBoard_nws_ros2/robot_part");
            pcfg.put("frame_id","test_frame");
            REQUIRE(ddnws.open(pcfg));
        }

        //"Close all polydrivers and check"
        {
            CHECK(ddnws.close());
        }
    }

    SECTION("Checking the nws attached to device")
    {
        PolyDriver dd_fake;
        PolyDriver dd_nws;
        Property p_fake;
        Property p_nws;

        p_nws.put("device", "frameGrabber_nws_ros2");
        p_nws.put("node_name", "frameGrabber_node");
        p_nws.put("topic_name","/controlBoard_nws_ros2/robot_part");
        p_nws.put("frame_id","test_frame");
        
        p_fake.put("device", "fakeFrameGrabber");

        REQUIRE(dd_fake.open(p_fake));
        REQUIRE(dd_nws.open(p_nws));
        yarp::os::SystemClock::delaySystem(0.5);

        {yarp::dev::WrapperSingle* ww_nws; dd_nws.view(ww_nws);
        REQUIRE(ww_nws);
        bool result_att = ww_nws->attach(&dd_fake);
        REQUIRE(result_att); }

        yarp::os::SystemClock::delaySystem(0.5);

        CHECK(dd_nws.close());
        CHECK(dd_fake.close());
    }

    Network::setLocalMode(false);
}
