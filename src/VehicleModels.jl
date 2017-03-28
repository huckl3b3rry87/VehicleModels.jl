__precompile__(false) # this module is NOT safe to precompile
module VehicleModels

using JuMP
using Parameters
using Interpolations

#function __init__()
  using NLOptControl
  using DifferentialEquations
#end

macro def(name, definition)
  return quote
    macro $name()
      esc($(Expr(:quote,definition)))
    end
  end
end

# funcitons in the VehicleModels.jl package
include("Three_DOF/Three_DOF.jl")
include("KinematicBicycle/KinematicBicycle.jl")
include("utils.jl")

export
  #########
  # Objects
  #########
  # Three DOF
  Vpara,

  # KinematicBicycle
  VparaKB,

  ###########
  # Functions
  ###########
  Linear_Spline,

  # Three DOF
  ThreeDOFv1,
  ThreeDOFv2,

  # KinematicBicycle
  KinematicBicycle,

  ###############################
  # Macros and support functions
  ###############################
  # Three DOF
  @F_YF,
  @F_YR,
  @FZ_RL,
  @FZ_RR,
  @Ax_min,
  @Ax_max,
  @unpack_Vpara,
  @pack_Vpara,

  # KinematicBicycle
  @unpack_VparaKB,
  @pack_VparaKB

  # REMOVE THE FINAL COMMA!
#############################
# types/functions/constants #
#############################

end # module
