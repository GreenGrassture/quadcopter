# Check if the initialization script has run.  If not, run it
if ! @isdefined initializedQuad
    include("initScript.jl")
end
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
# 4) Plug LQR controller into nonlineasr system

# Also, I need to implement a kalman filter and and a random noise generator that plays nicely with the ODE solver

# First define an array to hold all of the callbacks that might be created while initializing
# other things

# Define the nominal operating point and get the linearized model at that point
sLin = vec([0. 0. 0.])
vLin = vec([0. 0. 0.])
qLin = euler2quat(0,0,0)
ωLin = vec([0. 0. 0.])
xLin = vec([sLin; vLin; qLin; ωLin])
tLin = 0.
linParams = Dict("quadConsts"=>CFConsts,
                 "controller"=>controllerNone(),
                 "forces"=>externalForcesOnlyG,
                 "reference"=>referenceSignalConstant)
# We have to compute the initial control signal manually since the integrator hasn't run yet
uLin = linParams["controller"](xLin, linParams, tLin)
# Define the cost function J = Σ x'Qx + u'Ru
Q = 1.0*Matrix(I(13)) # I() will return a sparse matrix by default which breaks the ARE solver somehow
R = 1.0*Matrix(I(4))
tSample = 0.05
# Create a controller for that operating point
controllerLQR = makeLQR(innerDFun, xLin, linParams, tLin, Q, R, tSample)


microCallback = PeriodicCallback(microcontroller!, tSample, initial_affect=true)

# Define initial conditions for the ODE
s0 = vec([0. 0. 0.])
v0 = vec([0. 0. 0.])
q0 = euler2quat(0., 0, 0)
ω0 = vec([0. 0. 0.])
x0 = vec([s0; v0; q0; ω0])
t0 = 0.
tf = 10.
tSpan = (t0, tf)
# Set the problem parameters
ODEParams = Dict("quadConsts"=>CFConsts,
                 "controller"=>controllerLQR,
                 "forces"=>externalForcesOnlyG,
                 "reference"=>referenceSignalRamp) 
sol = simSys(x0, ODEParams, tSpan, microCallback)
p = plotStates(sol)
#plotAandB(A, B)