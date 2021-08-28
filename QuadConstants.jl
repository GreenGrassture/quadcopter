
const g = 9.81 # m/s^2

JxxCF = 16e-6 # kg*m^2
JyyCF = 16e-6 # kg*m^2
JzzCF = 29e-6 # kg*m^2
JMatCF = [JxxCF 0 0
          0 JyyCF 0
          0 0 JzzCF]
massCF =  32e-3 # kg
LCF = 33e-3 # m
kappaCF = 0.01 #
maxMotorForceCF = 0.58 # N

#dFac = 1e3 # mm/m  distance factor - scales lengths internally to improve numerical stability.  MKS units seemed to be causing blowups in the python ODE solver
# converting m -> mm
#const g = gTrue*dFac # mm/s^2
#const Jxx = JxxTrue*dFac^2 # kg*mm^2
#const Jyy = JyyTrue*dFac^2 # kg*mm^2
#const Jzz= JzzTrue*dFac^2 # kg*mm^2
#const L = LTrue*dFac # mm
#const kappa = kappaTrue*dFac # mm
#const maxMotorForce = maxMotorForceTrue*dFac #kg*mm/s^2

###### Testing using normal units internally.  Seems to work so far.
# const g = gTrue # mm/s^2
# const Jxx = JxxTrue # kg*mm^2
# const Jyy = JyyTrue # kg*mm^2
# const Jzz= JzzTrue # kg*mm^2
# const J = [Jxx 0 0
#            0 Jyy 0
#            0 0 Jzz]
# const L = LTrue # mm
# const kappa = kappaTrue # mm
# const maxMotorForce = maxMotorForceTrue #kg*mm/s^2

struct QuadParams
    mass::Float64
    J::Matrix{Float64} # moment of inertia matrix
    L::Float64
    kappa::Float64
    maxMotorForce::Float64
    mix::Matrix{Float64}
    unmix::Matrix{Float64}
end


function mixAndUnmix(L, K)
    # mixer matrix converts motor forces into total thrust and three moments
    # unmixer does the reverse, taking the thrust and moments and returning motor forces
    unmixer = [  [ 1  1  1  1]
               L*[ 1 -1 -1  1]
               L*[-1 -1  1  1]
               K*[ 1 -1  1 -1]]
    mixer = inv(unmixer)
    return (mixer, unmixer)
end

mixCF, unmixCF = mixAndUnmix(LCF, kappaCF)

CFConsts = QuadParams(massCF,
                      JMatCF,
                      LCF,
                      kappaCF,
                      maxMotorForceCF,
                      mixCF,
                      unmixCF)


