### road_model.jl

using Dates
using Gadfly
using LightGraphs, MetaGraphs
using JuMP, BilevelJuMP
using Gurobi
using DataFrames

struct ModelParams
    lambda
    sensor_cap_ratio
    force_sensor_bidirectional::Bool
    junctions::Bool
    capacity_classes::Dict{String, Real}

    # Sensors can be forced on, off, or left arbitrary (not in Dict)
    # fixed_sensors::Dict{Tuple{Int, Int}, Bool}
end

struct SolveParams
    time_limit
    # TODO
    indicator_mode
    opt_attributes
end

# Default parameters
ModelParams(; lambda = 1, bidir = false, junctions = false) = ModelParams(
    lambda, 
    1.0, 
    bidir, 
    junctions,
    Dict(
        "motorway"  => 500, 
        "trunk"     => 400, 
        "primary"   => 300,
        "secondary" => 200,
        "tertiary"  => 100,
        "other"     => 50
    ))

SolveParams(; time_limit = 60, indicator_mode = false, opt_attributes = Dict("MIPFocus" => 3)) = SolveParams(time_limit, indicator_mode, opt_attributes)

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
    mode = SP.indicator_mode ? BilevelJuMP.IndicatorMode() : BilevelJuMP.StrongDualityMode(1e-5)
    m = BilevelModel(() -> Gurobi.Optimizer(GRB_ENV), mode = mode)
    set_optimizer_attribute(m, "TimeLimit", SP.time_limit)
    for (k, v) in pairs(SP.opt_attributes)
        set_optimizer_attribute(m, k, v)
    end

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

    # Mutually exclusive ways to prevent flow looping on 2-cycles
    if M.MP.junctions
        cs_junction!(Lower(M.m), M)
    else
        cs_conserve_flow!(Lower(M.m), M)
        cs_inflow!(Lower(M.m), M)
    end
    cs_sensor!(Lower(M.m), M)

    # Optional: Not implemented
    # cs_anticycle_local!(Lower(M.m), M)
    # cs_anticycle_nonlocal!(Lower(M.m), M)

    # Objectives

    sensor_map((u, v, p),) = let 
        # Sensors on both sides of the road should count as "one" sensor in bidirectional mode
        if M.MP.force_sensor_bidirectional && has_edge(M.G, v, u)
            p[:sensor_var] * 0.5
        else
            p[:sensor_var]
        end
    end
    num_sensors = 0
    # Type inference failure...
    for s in sensor_map.(edge_prop_triples(M.G))
        num_sensors += s
    end
    M.data[:num_sensors_var] = num_sensors

    weighted_flows = sum(edge_prop_map(:length) .* edge_prop_map(:flow_var)) / 100
    # try to normalise by "size" of graph
    λ = M.MP.lambda / length(edges(M.G))

    @objective(Lower(M.m), Max, weighted_flows)
    @objective(Upper(M.m), Min, num_sensors + λ * weighted_flows)
end

"""
Outputs sensor placements and flows as edge properties in model graph `M.G`.

Will throw error if solver fails to find a solution (seems to be picky about which parameters it likes)
"""
function solve!(M::RoadModel)
    optimize!(M.m)

    for (u, v, p) in edge_prop_triples(M.G)
        set_prop!(M.G, u, v, :sensor, value(p[:sensor_var]) ≈ 1)
        f = value(p[:flow_var])
        # Numerical errors...
        @assert f > -1e-5 "Got negative flow: $(f)"
        set_prop!(M.G, u, v, :flow, max(0, f))
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
function draw!(M::RoadModel, svg_name = "road_sensors.svg")
    # Compute latitude and longitude bounding box for plot
    lats = (1:length(vertices(M.G))) .|> i -> get_prop(M.G, i, :latitude)
    longs = (1:length(vertices(M.G))) .|> i -> get_prop(M.G, i, :longitude)

    ns = (Int ∘ round ∘ value)(M.data[:num_sensors_var])
    d = collect(zipv(position_edges(M.G)))
    df = DataFrame(Dict(:x => d[1], :y => d[2], :xend => d[3], :yend => d[4], :Flow => d[5]))

    p = plot(
		# plot sensors on top layer as points instead of lines
		layer(
			Geom.point;
			zip([:x, :y], zipv(position_sensors(M.G)))...
		),
		layer(
            df,
            x = :x,
            y = :y,
            xend = :xend,
            yend = :yend,
            color = :Flow,
			Geom.segment(arrow=true, filled=true)
		),
		Scale.x_continuous(minvalue=minimum(lats), maxvalue=maximum(lats)), 
		Scale.y_continuous(minvalue=minimum(longs), maxvalue=maximum(longs)),
		Theme(line_width = 0.1mm),
        Guide.xlabel("Latitude"),
        Guide.ylabel("Longitude"),
        Guide.title("λ = $(M.MP.lambda) ($(ns) sensors)"),
	)
    Gadfly.draw(SVGJS(svg_name), p)
    p
end

"""
Converts result of a successful model solve into JSON
"""
function save_as_json(M::RoadModel)
    logs = Dict()

    sensor_all = true
    sensor_none = false
    # JSON does not have tuples (hashable), so a Dict for edges won't work
    repr_edge((src, dst, prop_dict),) = let
        sensor_all &= prop_dict[:sensor]
        sensor_none |= prop_dict[:sensor]
        [(src, dst), Dict(
            "sensor" => prop_dict[:sensor],
            "flow" => prop_dict[:flow],
        )]
    end
    # Not a particularly good way to serialise model - ids depends on latitude/longitude bounding box
    logs["edges"] = collect(It.map(repr_edge, edge_prop_triples(M.G)))
    logs["objective"] = objective_value(M.m)
    
    logs["trivial_all"] = sensor_all
    logs["trivial_none"] = !sensor_none

    logs
end

function load(path, G)
    logs = JSON.parsefile(path)

    # other parameters ignored for now
    MP = ModelParams(
        lambda = logs["model_params"]["lambda"],
        bidir = logs["model_params"]["force_sensor_bidirectional"],
        junctions = logs["model_params"]["junctions"]
    )

    # Loading an existing solution so solve should be near-instantaneous
    SP = SolveParams(
        time_limit = 10,
    )

    M = RoadModel(G, MP, SP)
    setup!(M)

    # fix variables to loaded values
    for ((u, v), json_edge_props) in logs["edges"]
        if json_edge_props["sensor"]
            fix(get_prop(M.G, u, v, :sensor_var), 1, force=true)
        else
            fix(get_prop(M.G, u, v, :sensor_var), 0, force=true)
        end
    end
    solve!(M)
    draw!(M)
    M
end

"""
Takes a graph and parameters, runs the model and dumps output as JSON in /out
"""
function run_model(G, MP::ModelParams, SP::SolveParams, name=""; draw = true)
    file_prefix = replace(Dates.format(now(), Dates.ISODateTimeFormat), ":" => "-")
    if !isempty(name)
        name = "-$(name)"
    end

    # JSON at the end
    logs = Dict()
    logs["model_params"] = MP
    logs["solve_params"] = SP

    # Record Gurobi solver output
    old_stdout = stdout    
    stdout_rd, stdout_wr = redirect_stdout()
    M = let
        M = RoadModel(G, MP, SP)
        setup!(M)
        try
            solve!(M)
            if draw
                draw!(M, "out/$(file_prefix)$(name).svg")
            end
            merge!(logs, save_as_json(M))
        catch e
            println("ERROR: $(e)")
            # save failure report to JSON as merge! in try block did not happen
            name = "-FAILED$(name)"
            logs["exception"] = "$(e)"
        end
        Base.Libc.flush_cstdio()
        M
    end
    logs["stdout"] = String(readavailable(stdout_rd))
    redirect_stdout(old_stdout)

    # Dump Dict to file
    mkpath("out")
    open("out/$(file_prefix)$(name).json", "w") do io
        write(io, JSON.json(logs, 4))
    end

    # return solved model
    M
end

export RoadModel, setup!, solve!, draw!, save_as_json, run_model, load
