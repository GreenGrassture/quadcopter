# Functions for computing the behavior of the system

function simSys(x0, p, tSpan, microCallback)
    # Handle callbacks
    quatCallback = ManifoldProjection(quaternionError)
    cb = CallbackSet(quatCallback, microCallback)
    prob = ODEProblem(dFun, x0, tSpan, p, callback=cb, save_everystep=false)
    sol = solve(prob)
    print("Simulation complete!\n")
    return sol
end

function dFun(x, p, t)
    # Function with correct type signature for ODE solver.  All of the parameters except the current 
    # state and the control input are packaged inside p.
    return innerDFun(x, p["u"], p, t)
end

function innerDFun(x, u, p, t)
    # Has x and u as arguments to allow for taking derivatives wrt to each of them.  Wrapped in an outer function to make it compatable with the ODE solver
    # x: the current state vector
    # u: control input

    # State: [x1, x2, x3, v1, v2, v3, q0, q1, q2, q3, p, q, r]
    #        [1]         [4]          [7]            [11]   [13] 
    quadConsts = p["quadConsts"]
    mass = quadConsts.mass
    J = quadConsts.J
    # Taking the opportunity to fix the mistake I made in the python version here: having tt after n's
    uLim = motorModel(u, p) # Limits the motor commands to the physically possible values
    tt = uLim[1]   # Total thrust
    n = uLim[2:4] # Moments about x, y, z axes (defined in the body frame)

    s = x[1:3] # position
    v = x[4:6] # velocity
    q = x[7:10] # quaternion representation of rotation
    ω = x[11:13] # angular velocity

    T_BE = quat2mat(q) # rotation matrix form of rotation q
    T_EB = T_BE' # T_EB is orthogonal so its inverse is equal to its transpose

    # Compute accelerations, transforming them all into the earth frame E before combining
    motorForce_B = [0, 0, tt] # Body frame.  Thrust will always be along the vertical axis of the quadcopter in the body frame.
    motorForce_E = T_EB*motorForce_B # Converted to earth frame
    externalForces_E = p["forces"](x, p, t)
  
    ds = sDot(v)
    #dx[1:3] = sDot(v)
    #setindex!(dx, sDot(v), [1,2,3])

    dv = vDot((motorForce_E, externalForces_E), mass)
    #dx[4:6] = vDot((motorForce_E, externalForces_E), mass)
    #setindex!(dx, vDot((motorForce_E, externalForces_E), mass), [4,5,6])

    dq = qDot(q, ω)
    #dx[7:10] = qDot(q, ω)
    #setindex!(dx, qDot(q, ω), [7,8,9,10])

    dω = ωDot(ω, n, J)
    #dx[11:13] = ωDot(ω, n, J)
    #setindex!(dx, ωDot(ω, n, J), [11,12,13])
    return [ds; dv; dq; dω]
    return nothing
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

function motorModel(u, p)
    fMax = p["quadConsts"].maxMotorForce
    motorForces = p["quadConsts"].unmix*u
    for i in 1:size(motorForces)[1]
        force = motorForces[i]
        if abs(force) > fMax
            motorForces[i] = sign(force)*fMax
        end
    end
    return p["quadConsts"].mix*motorForces
end

function speed2force(ω)
    # From values found in 136 lab
    return 4.254E-05*ω - 2.149E-02
end


function speed2PWM(ωDesired)
    # Convert a desired motor speed in radians/second into the necessary pwm command to achieve that speed 
    # (according to our linear model)
    a = -39.12526  # the zeroth order term
    b = 0.096503   # the first order term
    return Int64(round(a + b*ωDesired));
end

function force2speed(desiredForce_Newtons)
    # Convert a desired force (in a single propeller) into the motor speed in radians/second that will produce that force
    propConstant = 1.0e-08

    # We implement a safety check,
    # (no sqrtf for negative numbers)
    if (desiredForce_N <= 0) 
        return 0.0
    else
        return sqrtf(desiredForce_Newtons / propConstant)
    end
end