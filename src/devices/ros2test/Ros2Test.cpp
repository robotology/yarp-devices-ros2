/*
 * Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "Ros2Test.h"

bool open(yarp::os::Searchable& config)
{
    YARP_UNUSED(config);
    return true;
}

bool close()
{
    return true;
}
