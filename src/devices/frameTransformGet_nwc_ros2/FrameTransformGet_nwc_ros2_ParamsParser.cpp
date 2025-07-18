/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


// Generated by yarpDeviceParamParserGenerator (2.0)
// This is an automatically generated file. Please do not edit it.
// It will be re-generated if the cmake flag ALLOW_DEVICE_PARAM_PARSER_GERNERATION is ON.

// Generated on: Mon May 26 22:00:40 2025


#include "FrameTransformGet_nwc_ros2_ParamsParser.h"
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

namespace {
    YARP_LOG_COMPONENT(FrameTransformGet_nwc_ros2ParamsCOMPONENT, "yarp.device.FrameTransformGet_nwc_ros2")
}


FrameTransformGet_nwc_ros2_ParamsParser::FrameTransformGet_nwc_ros2_ParamsParser()
{
}


std::vector<std::string> FrameTransformGet_nwc_ros2_ParamsParser::getListOfParams() const
{
    std::vector<std::string> params;
    params.push_back("GENERAL::refresh_interval");
    params.push_back("ROS2::ft_node");
    params.push_back("ROS2::ft_topic");
    params.push_back("ROS2::ft_topic_static");
    return params;
}


bool FrameTransformGet_nwc_ros2_ParamsParser::getParamValue(const std::string& paramName, std::string& paramValue) const
{
    if (paramName =="GENERAL::refresh_interval")
    {
        paramValue = std::to_string(m_GENERAL_refresh_interval);
        return true;
    }
    if (paramName =="ROS2::ft_node")
    {
        paramValue = m_ROS2_ft_node;
        return true;
    }
    if (paramName =="ROS2::ft_topic")
    {
        paramValue = m_ROS2_ft_topic;
        return true;
    }
    if (paramName =="ROS2::ft_topic_static")
    {
        paramValue = m_ROS2_ft_topic_static;
        return true;
    }

    yError() <<"parameter '" << paramName << "' was not found";
    return false;

}


std::string FrameTransformGet_nwc_ros2_ParamsParser::getConfiguration() const
{
    //This is a sub-optimal solution.
    //Ideally getConfiguration() should return all parameters but it is currently
    //returning only user provided parameters (excluding default values)
    //This behaviour will be fixed in the near future.
    std::string s_cfg = m_provided_configuration;
    return s_cfg;
}

bool      FrameTransformGet_nwc_ros2_ParamsParser::parseParams(const yarp::os::Searchable & config)
{
    //Check for --help option
    if (config.check("help"))
    {
        yCInfo(FrameTransformGet_nwc_ros2ParamsCOMPONENT) << getDocumentationOfDeviceParams();
    }

    m_provided_configuration = config.toString();
    yarp::os::Property prop_check(m_provided_configuration.c_str());
    //Parser of parameter GENERAL::refresh_interval
    {
        yarp::os::Bottle sectionp;
        sectionp = config.findGroup("GENERAL");
        if (sectionp.check("refresh_interval"))
        {
            m_GENERAL_refresh_interval = sectionp.find("refresh_interval").asFloat64();
            yCInfo(FrameTransformGet_nwc_ros2ParamsCOMPONENT) << "Parameter 'GENERAL::refresh_interval' using value:" << m_GENERAL_refresh_interval;
        }
        else
        {
            yCInfo(FrameTransformGet_nwc_ros2ParamsCOMPONENT) << "Parameter 'GENERAL::refresh_interval' using DEFAULT value:" << m_GENERAL_refresh_interval;
        }
        prop_check.unput("GENERAL::refresh_interval");
    }

    //Parser of parameter ROS2::ft_node
    {
        yarp::os::Bottle sectionp;
        sectionp = config.findGroup("ROS2");
        if (sectionp.check("ft_node"))
        {
            m_ROS2_ft_node = sectionp.find("ft_node").asString();
            yCInfo(FrameTransformGet_nwc_ros2ParamsCOMPONENT) << "Parameter 'ROS2::ft_node' using value:" << m_ROS2_ft_node;
        }
        else
        {
            yCInfo(FrameTransformGet_nwc_ros2ParamsCOMPONENT) << "Parameter 'ROS2::ft_node' using DEFAULT value:" << m_ROS2_ft_node;
        }
        prop_check.unput("ROS2::ft_node");
    }

    //Parser of parameter ROS2::ft_topic
    {
        yarp::os::Bottle sectionp;
        sectionp = config.findGroup("ROS2");
        if (sectionp.check("ft_topic"))
        {
            m_ROS2_ft_topic = sectionp.find("ft_topic").asString();
            yCInfo(FrameTransformGet_nwc_ros2ParamsCOMPONENT) << "Parameter 'ROS2::ft_topic' using value:" << m_ROS2_ft_topic;
        }
        else
        {
            yCInfo(FrameTransformGet_nwc_ros2ParamsCOMPONENT) << "Parameter 'ROS2::ft_topic' using DEFAULT value:" << m_ROS2_ft_topic;
        }
        prop_check.unput("ROS2::ft_topic");
    }

    //Parser of parameter ROS2::ft_topic_static
    {
        yarp::os::Bottle sectionp;
        sectionp = config.findGroup("ROS2");
        if (sectionp.check("ft_topic_static"))
        {
            m_ROS2_ft_topic_static = sectionp.find("ft_topic_static").asString();
            yCInfo(FrameTransformGet_nwc_ros2ParamsCOMPONENT) << "Parameter 'ROS2::ft_topic_static' using value:" << m_ROS2_ft_topic_static;
        }
        else
        {
            yCInfo(FrameTransformGet_nwc_ros2ParamsCOMPONENT) << "Parameter 'ROS2::ft_topic_static' using DEFAULT value:" << m_ROS2_ft_topic_static;
        }
        prop_check.unput("ROS2::ft_topic_static");
    }

    /*
    //This code check if the user set some parameter which are not check by the parser
    //If the parser is set in strict mode, this will generate an error
    if (prop_check.size() > 0)
    {
        bool extra_params_found = false;
        for (auto it=prop_check.begin(); it!=prop_check.end(); it++)
        {
            if (m_parser_is_strict)
            {
                yCError(FrameTransformGet_nwc_ros2ParamsCOMPONENT) << "User asking for parameter: "<<it->name <<" which is unknown to this parser!";
                extra_params_found = true;
            }
            else
            {
                yCWarning(FrameTransformGet_nwc_ros2ParamsCOMPONENT) << "User asking for parameter: "<< it->name <<" which is unknown to this parser!";
            }
        }

       if (m_parser_is_strict && extra_params_found)
       {
           return false;
       }
    }
    */
    return true;
}


std::string      FrameTransformGet_nwc_ros2_ParamsParser::getDocumentationOfDeviceParams() const
{
    std::string doc;
    doc = doc + std::string("\n=============================================\n");
    doc = doc + std::string("This is the help for device: FrameTransformGet_nwc_ros2\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("This is the list of the parameters accepted by the device:\n");
    doc = doc + std::string("'GENERAL::refresh_interval': The time interval outside which timed ft will be deleted\n");
    doc = doc + std::string("'ROS2::ft_node': The name of the ROS2 node\n");
    doc = doc + std::string("'ROS2::ft_topic': The name of the ROS2 topic from which fts will be received\n");
    doc = doc + std::string("'ROS2::ft_topic_static': The name of the ROS2 topic from which static fts will be received\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("Here are some examples of invocation command with yarpdev, with all params:\n");
    doc = doc + " yarpdev --device frameTransformGet_nwc_ros2 --GENERAL::refresh_interval 0.1 --ROS2::ft_node tfNodeGet --ROS2::ft_topic /tf --ROS2::ft_topic_static /tf_static\n";
    doc = doc + std::string("Using only mandatory params:\n");
    doc = doc + " yarpdev --device frameTransformGet_nwc_ros2\n";
    doc = doc + std::string("=============================================\n\n");    return doc;
}
