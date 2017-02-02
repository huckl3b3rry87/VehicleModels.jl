module VehicleModels
using Media, DifferentialEquations, Dierckx, Atom, Plots, Parameters, Interpolations, JuMP, NLOptControl

macro def(name, definition)
  return quote
    macro $name()
      esc($(Expr(:quote,definition)))
    end
  end
end

# funcitons in the VehicleModels.jl package
include("Three_DOF/Three_DOF.jl")
include("utils.jl")

export
  # Objects
  Vpara,

  # Functions
  Linear_Spline,
  ThreeDOFv1,
  ThreeDOFv2,

  # Macros and support functions
  @F_YF,
  @F_YR,
  @FZ_RL,
  @FZ_RR,
  @Ax_min,
  @Ax_max,
  @unpack_Vpara,
  @pack_Vpara
  # REMOVE THE FINAL COMMA!

#############################
# types/functions/constants #
#############################

end # module
