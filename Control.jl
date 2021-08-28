using ControlSystems

##### Controllers ##########################
function controllerNone()
    # x: state vector
    # p: ODE parameter dict
    # t: time
    # Just commands motors to do nothing (beyond nominal hover thrust).  Intended to be used as a dummy to get the model of the linear,
    # open-loop system from the nonlinear closed-loop version using AD.
    function nullController(x, p, t)
        mass = p["quadConsts"].mass
        hoverThrust = [mass*g 0 0 0]
        return vec([0 0 0 0] + hoverThrust)
    end
    return nullController
end

function makeLQR(f, xLin, p, tLin, Q, R, tSample=0.01)
    # It takes a nonlinear derivative function and linearizes it around xLin, then designs an optimal controller
    # based on Q and R
    uLin = p["controller"](xLin, p, tLin)
    A, B = linearize(f, xLin, uLin, p, tLin)
    # Our original A and B matrices are for the continuous-time system, so we have to
    # convert to the discrete time equivalent and use that for the next step.
    sys = c2d(ss(A, B, I(size(A)[1]), 0), tSample)
    #return makeLQRk(sys.A, sys.B, Q, R, p, tSample)
    K = tryLQR(sys.A, sys.B, Q, R, dlqr)
    function controller(x, p, t)
        mass = p["quadConsts"].mass
        r = p["reference"](x, p, t)
        hoverThrust = vec([mass*g 0 0 0])
        return -K*(x-r) + hoverThrust
    end
    return controller
end  

function tryLQR(A, B, Q, R, LQRfun)
    # Repeatedly try to apply LQRfun, removing the least controllable state until success or it runs out of states. Fortunately the test for 
    # controllability is the same for continuous and discrete case
    # LQRfun can be either lqr or dlqr depending on whether the problem is continous or discrete
    # If states were removed, then the K matrix that is returned will have 
    numStates = size(A)[1]
    ARed = A
    BRed = B
    QRed = Q
    RRed = R
    K = nothing
    removedStates = []
    while numStates > 0
        try
            print(string("Solving ARE with " , numStates, " states...\n"))
            K = LQRfun(ARed, BRed, QRed, RRed)
            print("Success\n")
            break
        catch e
            if isa(e, SingularException)
                print("Singular\n")
                removedState, ARed, BRed, QRed, RRed = removeLeastControllableState(ARed, BRed, QRed, RRed)
                print(string("Removed state #", removedState, "\n"))
                numStates = numStates - 1
                push!(removedStates, removedState)
            else
                throw(e)
            end
        end
    end
    insertRowsAndCols(K, removedStates)
    return K
end

function removeLeastControllableState(A, B, Q, R)
    # Determines the degree of controllability of each state, and removes the least controllable one
    U, Σ, Vt = svd(ctrb(A, B))
    worstVec = U[:,end]
    worstState = argmax(abs.(worstVec))
    ARed = deleteRowAndCol(A, worstState)
    BRed = deleteRow(B, worstState)
    QRed = deleteRowAndCol(Q, worstState)
    return (worstState, ARed, BRed, QRed, R)
end

############################################

##### Reference Signals ####################
function referenceSignalConstant(x, p, t)
    # Returns the reference signal as a function of time, and possibly the system state.
    # This version just returns the equilibrium position
    sRef = vec([0, 0, 0])
    vRef = vec([0, 0, 0])
    qRef = euler2quat(0, 0, 0)
    omegaRef = vec([0, 0, 0])
    xRef = [sRef; vRef; qRef; omegaRef]
    return xRef
end

function makeRamp(s0, sf, t0, tf)
    function referenceSignalRamp(x, p, t)
        # Returns the reference signal as a function of time, and possibly the system state.
        # This one ramps the desired position from 0 to sF in time T and holds it there
        if t < t0
            sRef = s0
        elseif t < tf
            # linearly interpolate between initial and final positions
            sRef = (tf - t)/(tf - t0)*s0 + (t - t0)/(tf - t0)*sf
        else
            sRef = sf
        end
        vRef = vec([0, 0, 0])
        qRef = euler2quat(0, 0, 0)
        omegaRef = vec([0, 0, 0])
        xRef = [sRef; vRef; qRef; omegaRef]
    end
    return referenceSignalRamp
end

rampRef = makeRamp([0, 0, 0], [1, 0, 0], 0, 1)


############################################

function linearize(nonLinDFun, x0, u0, p, t0)
    # Get the A and B matrices of the linearized system resulting from nonLinDFun
    dfdx = x -> jacobian(x -> nonLinDFun(x, u0, p, t0), x)[1] # df/dx(x, u0)
    dfdu = u -> jacobian(u -> nonLinDFun(x0, u, p, t0), u)[1] # df/du(x0, u)
    A = dfdx(x0)
    B = dfdu(u0)
    return (A, B)
end

function ctrbMat(A, B)
    # Computes the controllability for a linear system with x_dot = Ax + Bu

    # Interestingly, there is a connection with the Krylov subspace - the subspace spanned by
    # b, Ab, A^2b etc. for vector b
    # https://en.wikipedia.org/wiki/Krylov_subspace
    M, N = size(A)
    ctrlMat = B
    for m in 1:(M-1)
        ctrlMat = hcat(ctrlMat, A^m*B)
    end
    return ctrlMat
end

function isControllable(A, B)
    M, N = size(A)
    ctrl = ctrbMat(A, B)
    if rank(ctrl) == M
        return true
    else
        return false
    end
end

function uncontrolledStates(A, B)
    # Determine which of the states are not controllable

    # TODO: if the system turns out to be uncontrollable, maybe I should use SVD to transform it into a basis where that
    # uncontrollable subspace affects as few states as possible.  In the standard basis there's no guarantee that the 
    # subspace is orthogonal to the rest of the states but using SVD that should be possible
    M, N = size(A)
    ctrl = ctrbMat(A, B)
    U, Σ, Vt = svd(ctrl)
    # If the rank of the controllability matrix is less than N (the number of states/rows), then the SVD will have some singular values equal to zero.  In that case, we can trace those through the transformation to figure out which states they correspond to.
    # Count nonzero elements of Σ.  They will be in descending order, so we can assume the zero elements are at the end.
    rank = sum(Σ .!= 0) # I'm comparing against exactly zero here, but I could imagine needing to set a tolerance at some point
    if rank == M
        # If the controllability matrix has rank equal to the number of states, then all states are controllable
        return []
    else
        # Collect all of the vectors that correspond to zero-valued singular values, and check each state to see if it
        # can be contstructed from them.  This entails solving a system of linear equations of the form:
        # e_i = a_1v_1 + a_2v_2 + ... a_(M-rank)v_(M-rank) for i in 1:M.  This seems like an overly conservative
        # approach, since it might somehow be the case that the uncontrolled subspace just cuts across the i'th state
        # but I'll have to address that later

        # The singular values represent the degree of controllability along different directions
        # The columns of U are the directions corresponding to each singular value

        # Find all the directions that are not controllable (σ=0):
        zeroVecs = U[:,(rank+1):end]
        states = []
        nVecs = size(zeroVecs)[2]
        for i=1:nVecs
            for state in findall(v_i -> v_i != 0, zeroVecs[:,i])
                push!(states, state)
            end
        end
        return states
    end
end

function microcontroller!(integrator)
    # Encapsulates all the code that might be run on the microcontroller.  Responsible for updating values of controller signal.
    # Will eventually be responsible for running state estimation and motion planning as well.
    p = integrator.p
    x = integrator.u # The state variable is referred to as u by the integrator
    t = integrator.t
    controller = p["controller"]
    u = controller(x, p, t)
    integrator.p["u"] = u
    return nothing
end