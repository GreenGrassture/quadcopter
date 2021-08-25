using ControlSystems
using DifferentialEquations
using LinearAlgebra
using PlotlyJS
using Zygote

# My modules
#push!(LOAD_PATH, pwd())

include("Control.jl")
include("PlottingFuns.jl")
include("InsertDeleteRowsCols.jl")
include("SystemDynamics.jl")
include("RotationFuns.jl")
include("Forces.jl")
include("QuadConstants.jl")

# Notes:
# I'm running into a problem with the quaternion representation of the orientation - at hover conditions, the row of the A matrix corresponding to q0 is all zeros.
# This means that the state is not controllable, and it leads to a singularity exception when trying to solve the CARE for the purpose of LQR control.  Somehow I need to
# account for this issue.  My first thought is to delete q0 in the linear model when I'm using it for the CARE.  Then I can take the resulting K matrix and patch in an 
# additional row and column of zeros.  My biggest concern is that it might not be unique to q0.  Maybe the other elements of q become uncontrollable at other orientations.
# It seems like the underlying issue is that the quaternion representation is over-determined.  q0^2 ⧋ 1 - q1^2 + q2^2 + q3^2

# The nonlinearities are only in the orientation.  The position and velocity control could really be handled completely by a linear controller, with a nonlinear controller
# handling the leftover states.  Though velocity is still coupled to angle via mgsin(θ) etc.

# Steps:
# 1) Define nonlinear system along with a dummy controller to get open loop system
# 2) Use automatic differentiation to get linearized system (without control inputs)
# 3) Write LQR controller for linearized system
# 4) Plug LQR controller into nonlinear system

# Also, I need to implement a kalman filter and and a random noise generator that plays nicely with the ODE solver

##### Convenience Functions ################
function unit(d, idx)
    # Creates a unit vector in R^d with entry in position idx
    unitVec = zeros(d)
    unitVec[idx] = 1
    return unitVec
end

#############################################

# Define the nominal operating point and get the linearized model at that point
sLin = vec([0. 0. 0.])
vLin = vec([0. 0. 0.])
qLin = euler2quat(0,0,0)
ωLin = vec([0. 0. 0.])
xLin = vec([sLin; vLin; qLin; ωLin])
tLin = 0.
linParams = (CFConsts, controllerNone(), externalForcesOnlyG, referenceSignalConstant) 
A, B = linearize(innerDFun, xLin, linParams, tLin)
C = I(13)
D = 0
Q = I(13)
R = I(4)
controllerLQR, K = makeLQRt(A, B, Q, R)
#sys = ss(A,B,C,D)
# Create a controller for that operating point


# Define initial conditions for the ODE
s0 = vec([20. 0. 0.])
v0 = vec([0. 0. 0.])
q0 = euler2quat(0., 0, 0)
ω0 = vec([0. 0. 0.])
x0 = vec([s0; v0; q0; ω0])
t0 = 0.
tf = 10.
tSpan = (t0, tf)
# Set the problem parameters
ODEParams = (CFConsts, controllerLQR, externalForcesOnlyG, referenceSignalConstant) 
sol = simSys(x0, ODEParams, tSpan)
p = plotStates(sol)
#plotAandB(A, B)