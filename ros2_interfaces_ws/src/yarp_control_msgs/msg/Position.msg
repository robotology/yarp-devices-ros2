# Message used to send position commands to one or multiple joints
# names: it can contain the names of the joints to send the position command to. If left empty, a position value for all the joints available must be provided
# positions: it must contain a position value for each name listed in the names vector or for every joint if the names vector is left empty
# ref_velocities: it can contain the velocity value for a specific position command. It must contain the same number of entries as the positions array or be empty. If empty the already set speed values will to be used
string[] names
float64[] positions
float64[] ref_velocities
