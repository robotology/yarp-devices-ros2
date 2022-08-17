# Message used to send velocity commands to one or multiple joints
# names: it can contain the names of the joints to send the velocity command to. If left empty, a velocity value for all the joints available must be provided
# velocities: it must contain a velocity value for each name listed in the names vector or for every joint if the names vector is left empty
# ref_accelerations: it can contain the acceleration value for each velocities vector entry. It must contain the same number of elements as the velocity array or be empty. If empty the already set acceleration values will to be used
string[] names
float64[] velocities
float64[] ref_accelerations
