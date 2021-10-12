### road_model.jl

using Dates
using Gadfly
using LightGraphs, MetaGraphs
using JuMP, BilevelJuMP
using Gurobi

struct ModelParams
    lambda
    sensor_cap_ratio
    force_sensor_bidirectional::Bool
    capacity_classes::Dict{String, Real}
end

struct SolveParams
    time_limit
    # TODO
    constraint_types
    # Sensors can be forced on, off, or left arbitrary (not in Dict)
    fixed_sensors::Dict{Tuple{Int, Int}, Bool}
end

# Default parameters
ModelParams(; lambda = 1) = ModelParams(
    lambda, 
    1.0, 
    false, 
    Dict(
        "motorway"  => 500, 
        "trunk"     => 400, 
        "primary"   => 300,
        "secondary" => 200,
        "tertiary"  => 100,
        "other"     => 50
    ))

SolveParams(; time_limit = 60) = SolveParams(time_limit, nothing, Dict())

export ModelParams, SolveParams

struct RoadModel
    m
    G
    constraints
    MP::ModelParams
    SP::SolveParams
    data::Dict{Symbol, Any}
end

include("road_utils.jl")
include("road_constraints.jl")

function RoadModel(G, MP::ModelParams, SP::SolveParams)::RoadModel
    # TODO: init errors if global const
    GRB_ENV = Gurobi.Env()
    m = BilevelModel(() -> Gurobi.Optimizer(GRB_ENV), mode = BilevelJuMP.StrongDualityMode(1e-5))
    set_optimizer_attribute(m, "TimeLimit", SP.time_limit)

    # clone the graph, as it will be mutated
    model_G = deepcopy(G)

    # constraints and arbitrary model data is stored here
    data = Dict()

    # Storing variables as properties in the graph is convenient
    for (u, v, p) ∈ edge_prop_triples(model_G)
        cap = get(MP.capacity_classes, p[:street_type], MP.capacity_classes["other"])

        fuv = @variable(Lower(m), base_name = "f_$(u),$(v)")
        suv = @variable(Upper(m), base_name = "s_$(u),$(v)")
        set_lower_bound(fuv, 0)
        set_upper_bound(fuv, cap)
        set_binary(suv)

        set_prop!(model_G, u, v, :flow_var, fuv)
        set_prop!(model_G, u, v, :sensor_var, suv)
        set_prop!(model_G, u, v, :capacity, cap)
    end

    RoadModel(m, model_G, [], MP, SP, data)
end

function setup!(M::RoadModel)
    edge_prop_map(prop) = edge_prop_triples(M.G) .|> ((_, _, p),) -> p[prop]

    # Various constraint passes
    cs_conserve_flow!(Lower(M.m), M)
    cs_inflow!(Lower(M.m), M)
    cs_sensor!(Lower(M.m), M)
    cs_anticycle_nonlocal!(Lower(M.m), M)

    # Objectives
    num_sensors = sum(edge_prop_map(:sensor_var))
    weighted_flows = sum(edge_prop_map(:length) .* edge_prop_map(:flow_var)) / 100
    λ = M.MP.lambda / length(edges(M.G))

    @objective(Lower(M.m), Max, weighted_flows)
    @objective(Upper(M.m), Min, num_sensors + λ * weighted_flows)
end

"""
Outputs sensor placements and flows as edge properties in model graph `M.G`.
"""
function solve!(M::RoadModel)
    optimize!(M.m)

    for (u, v, p) in edge_prop_triples(M.G)
        set_prop!(M.G, u, v, :sensor, value(p[:sensor_var]) ≈ 1)
        set_prop!(M.G, u, v, :flow, value(p[:flow_var]))
    end

    termination_status(M.m)
    # TODO: print summary
end

#
# Drawing utjilty functions
#

@resumable function position_edges(G)
    for (u, v, p) in edge_prop_triples(G)
        up, vp = props.(Ref(G), [u, v])
        @yield (
            up[:latitude], 
            up[:longitude], 
            vp[:latitude],
            vp[:longitude],
            p[:flow]
        )
    end
end

@resumable function position_sensors(G)
    for (u, v, p) in edge_prop_triples(G)
        if p[:sensor]
            @yield (
            (props(G, u)[:latitude] + props(G, v)[:latitude]) / 2,
            (props(G, u)[:longitude] + props(G, v)[:longitude]) / 2
        )
        end
    end
end

"""
Draws graph as SVG file using the outputs from `solve!`
"""
function draw(M::RoadModel, svg_name = "road_sensors.svg")
    # Compute latitude and longitude bounding box for plot
    lats = (1:length(vertices(M.G))) .|> i -> get_prop(M.G, i, :latitude)
    longs = (1:length(vertices(M.G))) .|> i -> get_prop(M.G, i, :longitude)

    p = plot(
		# plot sensors on top layer as points instead of lines
		layer(
			Geom.point;
			zip([:x, :y], zipv(position_sensors(M.G)))...
		),
		layer(
			Geom.segment(arrow=true, filled=true);
			zip([:x, :y, :xend, :yend, :color], zipv(position_edges(M.G)))...
		),
		Scale.x_continuous(minvalue=minimum(lats), maxvalue=maximum(lats)), 
		Scale.y_continuous(minvalue=minimum(longs), maxvalue=maximum(longs)),
		Theme(line_width = 0.1mm)
	)
    Gadfly.draw(SVGJS(svg_name), p)
    p
end

function save!(M::RoadModel, name="")
    # Not a particularly good way to serialise model - ids depends on latitude/longitude bounding box
    if !isempty(name)
        name = "-$(name)"
    end

    file_prefix = Dates.format(now(), Dates.ISODateTimeFormat)
    logs = Dict()
    logs["model_params"] = JSON.json(M.MP)
    logs["solve_params"] = JSON.json(M.SP)

    # JSON does not have tuples (hashable), so a Dict for edges won't work
    repr_edge((src, dst, prop_dict),) = let
        [(src, dst), Dict(
            "sensor" => prop_dict[:sensor],
            "flow" => prop_dict[:flow],
        )]
    end
    logs["edges"] = collect(It.map(repr_edge, edge_prop_triples(M.G)))

    mkpath("out")
    open("out/$(file_prefix)$(name).json", "w") do io
        write(io, JSON.json(logs))
    end

    draw(M, "out/$(file_prefix)$(name).svg")
end

function load(path)

end

export RoadModel, setup!, solve!, draw, save!, load
