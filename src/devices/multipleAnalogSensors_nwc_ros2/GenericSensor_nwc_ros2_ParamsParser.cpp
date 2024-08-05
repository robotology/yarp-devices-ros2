/*
 * SPDX-FileCopyrightText: 2023-2023 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */


// Generated by yarpDeviceParamParserGenerator (1.0)
// This is an automatically generated file. Please do not edit it.
// It will be re-generated if the cmake flag ALLOW_DEVICE_PARAM_PARSER_GERNERATION is ON.

// Generated on: Mon Aug  5 16:25:52 2024


#include "GenericSensor_nwc_ros2_ParamsParser.h"
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>

namespace {
    YARP_LOG_COMPONENT(GenericSensor_nwc_ros2ParamsCOMPONENT, "yarp.device.GenericSensor_nwc_ros2")
}


GenericSensor_nwc_ros2_ParamsParser::GenericSensor_nwc_ros2_ParamsParser()
{
}


std::vector<std::string> GenericSensor_nwc_ros2_ParamsParser::getListOfParams() const
{
    std::vector<std::string> params;
    params.push_back("node_name");
    params.push_back("topic_name");
    params.push_back("sensor_name");
    return params;
}


bool      GenericSensor_nwc_ros2_ParamsParser::parseParams(const yarp::os::Searchable & config)
{
    //Check for --help option
    if (config.check("help"))
    {
        yCInfo(GenericSensor_nwc_ros2ParamsCOMPONENT) << getDocumentationOfDeviceParams();
    }

    std::string config_string = config.toString();
    yarp::os::Property prop_check(config_string.c_str());
    //Parser of parameter node_name
    {
        if (config.check("node_name"))
        {
            m_node_name = config.find("node_name").asString();
            yCInfo(GenericSensor_nwc_ros2ParamsCOMPONENT) << "Parameter 'node_name' using value:" << m_node_name;
        }
        else
        {
            yCError(GenericSensor_nwc_ros2ParamsCOMPONENT) << "Mandatory parameter 'node_name' not found!";
            yCError(GenericSensor_nwc_ros2ParamsCOMPONENT) << "Description of the parameter: name of the ros2 node";
            return false;
        }
        prop_check.unput("node_name");
    }

    //Parser of parameter topic_name
    {
        if (config.check("topic_name"))
        {
            m_topic_name = config.find("topic_name").asString();
            yCInfo(GenericSensor_nwc_ros2ParamsCOMPONENT) << "Parameter 'topic_name' using value:" << m_topic_name;
        }
        else
        {
            yCError(GenericSensor_nwc_ros2ParamsCOMPONENT) << "Mandatory parameter 'topic_name' not found!";
            yCError(GenericSensor_nwc_ros2ParamsCOMPONENT) << "Description of the parameter: name of the topic where the device must publish the data";
            return false;
        }
        prop_check.unput("topic_name");
    }

    //Parser of parameter sensor_name
    {
        if (config.check("sensor_name"))
        {
            m_sensor_name = config.find("sensor_name").asString();
            yCInfo(GenericSensor_nwc_ros2ParamsCOMPONENT) << "Parameter 'sensor_name' using value:" << m_sensor_name;
        }
        else
        {
            yCError(GenericSensor_nwc_ros2ParamsCOMPONENT) << "Mandatory parameter 'sensor_name' not found!";
            yCError(GenericSensor_nwc_ros2ParamsCOMPONENT) << "Description of the parameter: The name of the sensor the data are coming from";
            return false;
        }
        prop_check.unput("sensor_name");
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
                yCError(GenericSensor_nwc_ros2ParamsCOMPONENT) << "User asking for parameter: "<<it->name <<" which is unknown to this parser!";
                extra_params_found = true;
            }
            else
            {
                yCWarning(GenericSensor_nwc_ros2ParamsCOMPONENT) << "User asking for parameter: "<< it->name <<" which is unknown to this parser!";
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


std::string      GenericSensor_nwc_ros2_ParamsParser::getDocumentationOfDeviceParams() const
{
    std::string doc;
    doc = doc + std::string("\n=============================================\n");
    doc = doc + std::string("This is the help for device: GenericSensor_nwc_ros2\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("This is the list of the parameters accepted by the device:\n");
    doc = doc + std::string("'node_name': name of the ros2 node\n");
    doc = doc + std::string("'topic_name': name of the topic where the device must publish the data\n");
    doc = doc + std::string("'sensor_name': The name of the sensor the data are coming from\n");
    doc = doc + std::string("\n");
    doc = doc + std::string("Here are some examples of invocation command with yarpdev, with all params:\n");
    doc = doc + " yarpdev --device genericSensor_nwc_ros2 --node_name <mandatory_value> --topic_name <mandatory_value> --sensor_name <mandatory_value>\n";
    doc = doc + std::string("Using only mandatory params:\n");
    doc = doc + " yarpdev --device genericSensor_nwc_ros2 --node_name <mandatory_value> --topic_name <mandatory_value> --sensor_name <mandatory_value>\n";
    doc = doc + std::string("=============================================\n\n");    return doc;
}
