

##### External Forces ######################
function externalForcesNone(x, t, quadConsts)
    # Function that gives externally imposed forces as a function of time (in the earth frame)
    # This one applies no forces.
    return [0,0,0]
end
function externalForcesOnlyG(x, p, t)
    # Function that gives externally imposed forces as a function of time (in the earth frame)
    # The usual situation, with no forces other than gravity
    mass = p["quadConsts"].mass
    J = p["quadConsts"].J
    gravForce_E = [0, 0, -mass*g] # Earth Frame.  Gravity will always be vertical and downward in the earth frame.
    return gravForce_E
end
############################################
