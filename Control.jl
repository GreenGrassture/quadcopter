
##### Controllers ##########################
function controllerNone()
    # x: state vector
    # t: time
    # r: reference signal
    
    # Just commands motors to do nothing (beyond nominal hover thrust).  Intended to be used as a dummy to get the model of the linear, open-loop system from the closed loop version using AD.
    function nullController(x, r, t)
        return [0,0,0,0]
    end
    return nullController
end

function makeLQRt(nonLinDFun, xLin, linParams, tLin, Q, R)
    # Takes a nonlinear function and linearizes it around xLin, then designs an optimal controller
    # based on Q and R
    A, B = linearize(nonLinDFun, xLin, linParams, tLin)
    return makeLQRt(A, B, Q, R)
end

function makeLQRt(A, B, Q, R)
    # Takes a description of a linear system and quadratic cost function and returns a function that implements
    # the optimal controller for that problem

    # Check if the linearized form is controllable.  If not, we drop the uncontrollable states and find
    # a controller for the reduced state.  This is probably a bad idea if the states aren't stable (or bounded??)
    # but it should work fine for the intended purpose (quaternions, which are over-determined).
    if isControllable(A,B)
        K = lqr(A, B, Q, R)
    else
        # Find out which states are uncontrollable and delete the corresponding rows and columns from A and Q, and the corresponding rows from B
        deleteStates = uncontrolledStates(A, B) # If this branch is running, we expect deleteStates to not be empty
        # Create A, B, and Q matrices for our reduced state system
        AReduced = deleteRowsAndCols(A, deleteStates)
        BReduced = deleteRows(B, deleteStates)
        QReduced = deleteRowsAndCols(Q, deleteStates)
        # Find the optimal controller for that reduced system
        KReduced = lqr(AReduced, BReduced, QReduced, R)
        # Expand the K matrix out to the full state of the system, with zeros in the columns of the uncontrolled states
        K = insertCols(KReduced, deleteStates)
    end
    function controller(x, r, t)
        # the x that is passed from the integrator is a row vector so we take the transpose
        return -K*(x-r) 
    end
    return (controller, K)
end
############################################

##### Reference Signals ####################
function referenceSignalConstant(x, t)
    # Returns the reference signal as a function of time, and possibly the system state.
    # This version just returns the equilibrium position
    sRef = vec([0, 0, 0])
    vRef = vec([0, 0, 0])
    qRef = euler2quat(0, 0, 0)
    omegaRef = vec([0, 0, 0])
    xRef = [sRef; vRef; qRef; omegaRef]
    return xRef
end
############################################

function linearize(nonLinDFun, x0, odeParams, t0)
    # Get the A and B matrices of the linearized system resulting from nonLinDFun
    quadConsts, controller, externalForces, referenceSignal = odeParams
    mass = quadConsts.mass
    J = quadConsts.J
    r0 = referenceSignal(x0, t0)
    u0 = controller(x0, r0, t0)
    forces = externalForces(x0, t0, quadConsts)

    dfdx = x -> jacobian(x -> nonLinDFun(x, u0, forces, mass, J, t0), x)[1] # df/dx(x)
    dfdu = u -> jacobian(u -> nonLinDFun(x0, u, forces, mass, J, t0), u)[1] # df/du(u)

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
