/*
 * SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/dev/tests/IOrientationSensorsTest.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>

#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>

#include <cmath>
#include <chrono>
#include <thread>

#include <catch2/catch_amalgamated.hpp>
#include <harness.h>

using namespace yarp::os;
using namespace yarp::dev;


TEST_CASE("dev::MultipleAnalogSensors_nwc_ros2_test", "[yarp::dev]")
{
    YARP_REQUIRE_PLUGIN("fakeIMU", "device");
    YARP_REQUIRE_PLUGIN("imu_nws_ros2", "device");

//#if defined(DISABLE_FAILING_TESTS)
//    YARP_SKIP_TEST("Skipping failing tests")
//#endif

    Network::setLocalMode(true);

    SECTION("Test the nwc alone")
    {
        PolyDriver nwc;

        Property pNwc;
        pNwc.put("device", "imu_nwc_ros2");
        pNwc.put("node_name", "imu_node");
        pNwc.put("topic_name", "/imu_topic");
        pNwc.put("sensor_name", "imu_sensor");
        REQUIRE(nwc.open(pNwc)); // multipleanalogsensors nwc open reported successful

        // Close devices
        nwc.close();
    }

    /* TODO: The test needs another section.
     *       This new section should:
     *          - Open a ROS2 publisher for the ROS" message managed by the nwc
     *          - Instantiate and open the nwc
     *          - Publish a test message and check if the data are correctly received by
     *            the nwc by calling all the methods of the interfaces exposed by the nwc
     */

    Network::setLocalMode(false);
}
