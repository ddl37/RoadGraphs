### RoadGraphs.jl

"""
The following Julia code consists of separate files that are to be bundled together and installed as a module (`]pkg dev .` in REPL)

Project.toml
src/
    RoadGraphs.jl
    road_model.jl
    road_constraints.jl
    road_utils.jl
"""
module RoadGraphs

include("road_model.jl")

end
