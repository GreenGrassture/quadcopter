# This script includes a bunch of the other files and libraries I need.  I split it off
# from the main script so the imports don't run over and over

using ControlSystems
using DifferentialEquations
using LinearAlgebra
using PlotlyJS
using Zygote

# My modules
include("Control.jl")
include("PlottingFuns.jl")
include("InsertDeleteRowsCols.jl")
include("SystemDynamics.jl")
include("RotationFuns.jl")
include("Forces.jl")
include("QuadConstants.jl")


##### Convenience Functions ################
function unit(d, idx)
    # Creates a unit vector in R^d with entry in position idx
    unitVec = zeros(d)
    unitVec[idx] = 1
    return unitVec
end
#############################################

#function initFun()
    # Run this to cause julia to compile/run all the important functions with the
    # proper arguments
    sInit = vec([0. 0. 0.])
    vInit = vec([0. 0. 0.])
    qInit = euler2quat(0, 0, 0)
    ωInit = vec([0. 0. 0.])
    xInit = vec([sInit; vInit; qInit; ωInit])
    tInit0 = 0.
    tInit1 = 5.
    tSample = 0.01
    initParams = Dict("quadConsts"=>CFConsts,
                      "controller"=>controllerNone(),
                      "forces"=>externalForcesOnlyG,
                      "reference"=>referenceSignalConstant) 
    uInit = initParams["controller"](xInit, initParams, tInit0)
    AInit, BInit = linearize(innerDFun, xInit, uInit, initParams, tInit0) 
    microCallback = PeriodicCallback(microcontroller!, tSample, initial_affect=true)
    dx0 = 1.0*zeros(13)
    sol = simSys(xInit, initParams, (tInit0, tInit1), microCallback);
#end
#initFun()

initializedQuad = true
print("Initialized Workspace")