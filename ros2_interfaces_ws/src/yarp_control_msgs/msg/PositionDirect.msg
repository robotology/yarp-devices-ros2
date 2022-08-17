# Message used to send position direct commands to one or multiple joints
# names: it can contain the names of the joints to send the position command to. If left empty, a position value for all the joints available must be provided
# positions: it must contain a position value for each name listed in the names vector or for every joint if the names vector is left empty
string[] names
float64[] positions
