/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


// Generated by yarpDeviceParamParserGenerator (1.0)
// This is an automatically generated file. Please do not edit it.
// It will be re-generated if the cmake flag ALLOW_DEVICE_PARAM_PARSER_GERNERATION is ON.

// Generated on: Mon Aug 26 14:52:35 2024


#ifndef FRAMETRANSFORMGET_NWC_ROS2_PARAMSPARSER_H
#define FRAMETRANSFORMGET_NWC_ROS2_PARAMSPARSER_H

#include <yarp/os/Searchable.h>
#include <yarp/dev/IDeviceDriverParams.h>
#include <string>
#include <cmath>

/**
* This class is the parameters parser for class FrameTransformGet_nwc_ros2.
*
* These are the used parameters:
* | Group name | Parameter name   | Type   | Units   | Default Value | Required | Description                                                       | Notes |
* |:----------:|:----------------:|:------:|:-------:|:-------------:|:--------:|:-----------------------------------------------------------------:|:-----:|
* | GENERAL    | refresh_interval | double | seconds | 0.1           | 0        | The time interval outside which timed ft will be deleted          | -     |
* | ROS2       | ft_node          | string | -       | tfNodeGet     | 0        | The name of the ROS2 node                                         | -     |
* | ROS2       | ft_topic         | string | -       | /tf           | 0        | The name of the ROS2 topic from which fts will be received        | -     |
* | ROS2       | ft_topic_static  | string | -       | /tf_static    | 0        | The name of the ROS2 topic from which static fts will be received | -     |
*
* The device can be launched by yarpdev using one of the following examples (with and without all optional parameters):
* \code{.unparsed}
* yarpdev --device frameTransformGet_nwc_ros2 --GENERAL::refresh_interval 0.1 --ROS2::ft_node tfNodeGet --ROS2::ft_topic /tf --ROS2::ft_topic_static /tf_static
* \endcode
*
* \code{.unparsed}
* yarpdev --device frameTransformGet_nwc_ros2
* \endcode
*
*/

class FrameTransformGet_nwc_ros2_ParamsParser : public yarp::dev::IDeviceDriverParams
{
public:
    FrameTransformGet_nwc_ros2_ParamsParser();
    ~FrameTransformGet_nwc_ros2_ParamsParser() override = default;

public:
    const std::string m_device_classname = {"FrameTransformGet_nwc_ros2"};
    const std::string m_device_name = {"frameTransformGet_nwc_ros2"};
    bool m_parser_is_strict = false;
    struct parser_version_type
    {
         int major = 1;
         int minor = 0;
    };
    const parser_version_type m_parser_version = {};

    const std::string m_GENERAL_refresh_interval_defaultValue = {"0.1"};
    const std::string m_ROS2_ft_node_defaultValue = {"tfNodeGet"};
    const std::string m_ROS2_ft_topic_defaultValue = {"/tf"};
    const std::string m_ROS2_ft_topic_static_defaultValue = {"/tf_static"};

    double m_GENERAL_refresh_interval = {0.1};
    std::string m_ROS2_ft_node = {"tfNodeGet"};
    std::string m_ROS2_ft_topic = {"/tf"};
    std::string m_ROS2_ft_topic_static = {"/tf_static"};

    bool          parseParams(const yarp::os::Searchable & config) override;
    std::string   getDeviceClassName() const override { return m_device_classname; }
    std::string   getDeviceName() const override { return m_device_name; }
    std::string   getDocumentationOfDeviceParams() const override;
    std::vector<std::string> getListOfParams() const override;
};

#endif
