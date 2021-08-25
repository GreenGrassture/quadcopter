# Functions for computing the behavior of the system



function simSys(x0, params, tSpan)
    quatCB = ManifoldProjection(quaternionError)
    prob = ODEProblem(outerDFun, x0, tSpan, params, callback=quatCB, save_everystep=false)
    sol = solve(prob)
    return sol
end

function outerDFun(x, params, t)
    # Function with correct type signature for ODE solver.  All of the parameters except the current 
    # state are packaged inside params
    quadConsts, controller, forceFunc, referenceSignal = params
    mass = quadConsts.mass
    J = quadConsts.J
    r = referenceSignal(x, t)
    u = controller(x, r, t)
    fExternal_E = forceFunc(x, t, quadConsts)
    dx = innerDFun(x, u, fExternal_E, mass, J, t)
    return dx
end

function innerDFun(x, u, fExternal_E, mass, J, t)
    # Has x and u as arguments to allow for taking derivatives wrt to each of them.  This will be wrapped in an outer function to make it compatable with the ODE solver
    # x: the current state vector
    # u: control input
    # fExternal_E: all of the external forces on the quadcopter, including gravity and possibly things like wind or contact forces
    # quadConsts: all of the ////physical constants that describe the quadcopter, i.e. mass, moments of intertia, lengths of rotor arms

    # State: [x1, x2, x3, v1, v2, v3, q0, q1, q2, q3, p, q, r]
    #        [1]         [4]          [7]            [11]   [13] 

    # Taking the opportunity to fix the mistake I made in the python version here: having tt after n's
    T0 = mass*g # Thrust required in hover to counteract gravity
    ΔT = u[1]   # Additional thrust commanded beyond T0
    n = u[2:4] # Moments about x, y, z axes (defined in the body frame)

    s = x[1:3] # position
    v = x[4:6] # velocity
    q = x[7:10] # quaternion representation of rotation
    ω = x[11:13] # angular velocity

    T_BE = quat2mat(q) # rotation matrix form of rotation q
    T_EB = T_BE' # T_EB is orthogonal so its inverse is equal to its transpose

    ds = sDot(v)
    #sDot = v # ds/dt = v

    # Compute accelerations, transforming them all into the earth frame E before combining
    motorForce_B = [0, 0, T0+ΔT] # Body frame.  Thrust will always be along the vertical axis of the quadcopter in the body frame.
    motorForce_E = T_EB*motorForce_B # Converted to earth frame
    dv = vDot((motorForce_E, fExternal_E), mass)
    #vDot = (motorForce_E + fExternal_E)/mass

    dq = qDot(q, ω)
    #qDot = 0.5*[-q[2:4]'
    #            q[1]*I(3)+skewMat(q[2:4])]*ω

    dω = ωDot(ω, n, J)
    #ωDot = inv(J)*(-skewMat(ω)*J*ω + n)

    dx = [ds; dv; dq; dω]
    return dx

    # In-place derivative assignment:
    #dx = zeros(Float64, 13)
    #dx[1:3] = sDot
    #dx[4:6] = vDot
    #dx[7:10] = qDot
    #dx[11:13] = omegaDot
end

@inline function qDot(q, ω)
    return 0.5*[-q[2:4]'; q[1]*I(3)+skewMat(q[2:4])]*ω
end

@inline function ωDot(ω, n, J)
    return inv(J)*(-skewMat(ω)*J*ω + n)
end

@inline function vDot(forces_E, mass)

    return sum(forces_E)/mass
end

@inline function sDot(v)
    return v
end


function quaternionError(resid, x, params, t)
    # Function that measures the error in the norm of the rotation quaternion.  Used according to:
    # https://diffeq.sciml.ai/stable/features/callback_library/#Manifold-Conservation-and-Projection
    quatNorm =  x[7]^2 + x[8]^2 + x[9]^2 + x[10]^2
    resid[1] = 1 - quatNorm
    resid[2:13] = zeros(12)
end

function motorModel(x, u, t, quadConsts)
    fMax = quadConsts.maxMotorForce

end
