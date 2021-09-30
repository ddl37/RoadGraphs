### A Pluto.jl notebook ###
# v0.16.1

using Markdown
using InteractiveUtils

# ╔═╡ cf95caf8-1b5f-11ec-2e4d-bfa2ebd7c94d
begin
	using Revise
	using Pkg
	Pkg.activate()
	using LinearAlgebra
	
	using LightGraphs, MetaGraphs
	import JSON

	using JuMP
	using BilevelJuMP
	using Cbc
	using Gurobi
	
	using Gadfly
	
	using ResumableFunctions
end

# ╔═╡ 20d26a5d-e30f-48fe-8987-8671063f1084
const GRB_ENV = Gurobi.Env()

# ╔═╡ 5f7dd6fa-a03a-4f19-92b1-542c7fe90325
"""
Convert dictionary keys from strings to symbols
"""
function symbolify(d::Dict)
	Dict((Symbol(k) => v) for (k, v) ∈ pairs(d))
end

# ╔═╡ 0e44234d-3d87-4fcf-85d8-7f738d351bf2
function read_road_graph(path, bbox_ll = [-Inf, -Inf], bbox_ur = [Inf, Inf])
	data = open(path, "r") do io
		JSON.parse(io)
	end
	
	G = MetaDiGraph()
	
	# can't enumerate: some vertices skipped
	i = 1
	for v_data ∈ data["nodes"]
		# filter latitudes, longitudes
		if !(	bbox_ll[1] <= v_data["latitude"] <= bbox_ur[1] &&
				bbox_ll[2] <= v_data["longitude"] <= bbox_ur[2])
			continue
		end
		
		add_vertex!(G)
		@assert set_props!(G, i, symbolify(v_data))
		i += 1
	end
	set_indexing_prop!(G, :id)
	
	for e ∈ data["edges"]
		g_src_id = get(G[:id], e["from"], -1)
		g_dst_id = get(G[:id], e["to"], -1)
		if g_src_id < 0 || g_dst_id < 0
			continue
		end
		
		@assert add_edge!(G, g_src_id, g_dst_id)
		@assert set_props!(G, g_src_id, g_dst_id, symbolify(e))
	end
	
	G
end

# ╔═╡ f273d6fe-2a3d-4fe1-8efe-3923edcd3a92
function to_adjacency_list(H)
	n = length(vertices(H))
	open("region_adj.txt", "w") do io
		println(io, n)
		
		for i ∈ 1:n
			println(io, join(string.(outneighbors(H, i)), " "))
		end
	end
end

# ╔═╡ db202c8d-6664-41d3-b24b-3cd42a8ca3cc
REGION_BT = ([-36.9323, 174.8616], [-36.8878, 174.9565])

# ╔═╡ b22090a3-9ea0-4551-b0e8-bbbc46870ec3
REGION = REGION_BT

# ╔═╡ 910b9928-adb2-4bcd-b1e0-df67a3c3b5a0
# G = read_road_graph("graph.json", [-36.916483, 174.895322], [-36.857740, 174.946185])
# G = read_road_graph("graph.json", [-36.904911, 174.931674], [-36.891935, 174.944982])
# G = read_road_graph("graph.json")

# from 1435, 2045 to... 521/921
G = read_road_graph("graph.json", REGION...)

# ╔═╡ 25265fb3-7925-43a8-a38b-03d231b502ef
to_adjacency_list(G)

# ╔═╡ 58399008-f753-4444-904b-f4f5533531b0
md"""
# TODO: 
- Clean up code
- Normalise lambda objective by number of edges
- Anti-cycling
- would be really useful to just "pick up where last solution left off"
  - needs re-write of solver/constraints
- some way to store past solutions would be nice
- Clarify definition and units of "flow": Total cars on edge-segment per unit-time per length? 
"""

# ╔═╡ 26903824-7261-491f-9322-bfc152730dab
"""
Creates enumerated representation of `edges(G)` and returns a tuple of bound accessor functions
- `index_to_pair :: (Int) -> (Int, Int)`
- `pair_to_index :: (Int, Int) -> Int`
- `index_to_edge_props :: (Int) -> Dict{Symbol, Any}`
- `pair_to_edge_props :: (Int, Int) -> Dict{Symbol, Any}`
"""
function label_edges(G)
	_i2e = Dict()
	_e2i = Dict()
	
	for (i, e) ∈ enumerate(edges(G))
		u, v = e.src, e.dst
		_i2e[i] = (u, v)
		_e2i[(u, v)] = i
	end
	
	i2e(i) = _i2e[i]
	e2i(u, v) = _e2i[(u, v)]
	i2p(i) = props(G, _i2e[i]...)
	e2p(u, v) = props(G, u, v)
	
	return i2e, e2i, i2p, e2p
end

# ╔═╡ c9fc1926-381d-4673-ad18-5505ab0abf3f
begin
	struct SolveParameters
		# Problem
		Lambda
		LimitRatio
		ForceBidirectional
		ForceSensors
		# Solver
		TimeLimit
		SolveMode
		SolverAttributes::Dict{String, Any}
		# three types of constraints
		Ctype1
		Ctype2
		Ctype3
	end
	
	function SolveParameters(λ;
		LimitRatio = 1.0,
		ForceBidirectional = false,
		ForceSensors = [],
		TimeLimit = 60,
		SolveMode = BilevelJuMP.MixedMode(default = BilevelJuMP.IndicatorMode()),
		SolverAttributes = Dict(),
		Ctype1 = BilevelJuMP.IndicatorMode(),
		Ctype2 = BilevelJuMP.IndicatorMode(),
		Ctype3 = BilevelJuMP.IndicatorMode(),
		)
		
		SolveParameters(λ, LimitRatio, ForceBidirectional, ForceSensors, TimeLimit, SolveMode, SolverAttributes, Ctype1, Ctype2, Ctype3)
	end
end

# ╔═╡ 375d1f19-59a4-4666-9ba2-65f3044243c2
"""
Flow constraints that hold in the lower/inner model
"""
function flow_constraints(lm, G, flow_var::Function, P::SolveParameters)
	# NON-TRIVIAL CYCLE CONSTRAINTS
	for v ∈ vertices(G)
		for iv ∈ inneighbors(G, v)
			expr = -flow_var(iv, v)
			
			ov_count = 0
			for ov ∈ outneighbors(G, v)
				# THIS SHOULD BE `iv` NOT V!!!! was leading to unbounded source of cars u-turning
				if ov == iv
					continue
				end
				expr += flow_var(v, ov)
				ov_count += 1
			end
			
			if ov_count == 0
				# dead-end
				continue
			end
			# ambiguity of traffic flow direction: use \geq
			# (in-flow less than out-flow in non-loop direction)
			# TODO: switch to equality by adding per-direction/junction flow variables
			# BilevelJuMP.set_mode(@constraint(lm, expr >= 0), P.Ctype1)
			@constraint(lm, expr >= 0)
		end
	end
	
	# FLOW CONSERVATION CONSTRAINTS
	for v ∈ vertices(G)
		if length(Set(all_neighbors(G, v))) <= 1
			continue
		end
		# if indegree(G, v) < 2 || outdegree(G, v) < 2
		# 	continue
		# end
		
		expr = 0
		for i ∈ inneighbors(G, v)
			expr += flow_var(i, v)
		end
		
		for o ∈ outneighbors(G, v)
			expr -= flow_var(v, o)
		end
		
		# BilevelJuMP.set_mode(@constraint(lm, expr == 0), P.Ctype2)
		@constraint(lm, expr == 0)
	end
end

# ╔═╡ 12c208b7-9656-4266-8d74-d74eb04af8b2
"""
Smaller λ: Fewer sensors, higher uncertainty
Larger λ: More sensors, lower uncertainty. As λ → +∞, will approach using all (except redundant I suppose) sensors
"""
function model(G, P::SolveParameters)
	num_vertices = length(vertices(G))
	num_edges = length(edges(G))
	λ = P.Lambda
	
	edge_index = Dict((e.src, e.dst) => i for (i, e) ∈ enumerate(edges(G)))
	edge_index_rev = Dict(i => e for (e, i) ∈ pairs(edge_index))
	
	# model = BilevelModel(Cbc.Optimizer, mode = BilevelJuMP.SOS1Mode())
	model = BilevelModel(() -> Gurobi.Optimizer(GRB_ENV), mode = P.SolveMode)
	
	set_optimizer_attribute(model, "TimeLimit", P.TimeLimit)
	for (k, v) ∈ pairs(P.SolverAttributes)
		set_optimizer_attribute(model, k, v)
	end
	
	@variable(Upper(model), sensors[1:num_edges], Bin)
	
	# 100/200/300 capacity for T/S/P roads
	# 500 for motorway
	# 50 for others?
	
	CAPS = (Dict)(
		"motorway" => 500, 
		"trunk" => 400, 
		"primary" => 300,
		"secondary" => 200,
		"tertiary" => 100,
	)
	
	ROAD_MAXCAP = 500
	
	# how to make model composable?
	# really need to implement heuristic and initial solutions
	
	assign_cap(i) = let
		road_type = props(G, edge_index_rev[i]...)[:street_type]
		c = get(CAPS, road_type, 50)
	end
	
	capacities = (1:num_edges) .|> assign_cap
	@variable(Lower(model), capacities[i] >= flows[i=1:num_edges] >= 0)
	
	flow_var(u, v) = flows[edge_index[(u, v)]]
	
	# for i ∈ 1:num_edges
	# 	fix(sensors[i], rand([0, 1]))
	# end
	
	get_sensor = i -> let
		u, v = edge_index_rev[i]
		
		if P.ForceBidirectional && haskey(edge_index, (v, u))
			j = edge_index[(min(u, v), max(u, v))]
			return sensors[j]
		else
			return sensors[i]
		end
	end
	
	# INNER FLOW CONSTRAINTS
	flow_constraints(Lower(model), G, flow_var, P)
	
	
	# SENSOR-FLOW CONSTRAINTS (lower)
	for sensor_no ∈ 1:num_edges
		# BilevelJuMP.set_mode(@constraint(Lower(model), flows[sensor_no] <= ROAD_MAXCAP * (1 - P.LimitRatio * get_sensor(sensor_no))), P.Ctype3)
		@constraint(Lower(model), flows[sensor_no] <= ROAD_MAXCAP * (1 - P.LimitRatio * get_sensor(sensor_no)))
	end
	
	# OBJECTIVES
	# should this sum also be weighted?
	@objective(Lower(model), Max, sum(flows))
	
	sensor_sum = sum(get_sensor.(1:num_edges))
	# @objective(Upper(model), Min, sensor_sum + λ * sum(flows))
	
	# really should figure out proper consistent normalisation
	flow_weights = (i -> props(G, edge_index_rev[i]...)[:length] / 100).(1:num_edges)
	@objective(Upper(model), Min, sensor_sum + λ * dot(flows, flow_weights))
	
	optimize!(model)
	
	println(termination_status(model))
	
	# return set of edges
	sensor_indicators = [value(get_sensor(i)) for i ∈ 1:num_edges]
	flowz = Dict(edge_index_rev[i] => value(flows[i]) for i ∈ 1:num_edges)
	
	sensor_set = []
	for (i, val) ∈ enumerate(sensor_indicators)
		if val ≈ 1
			push!(sensor_set, edge_index_rev[i])
		end
	end
	
	# also map edges to flows
	
	# return a struct of some sort
    return sensor_set, flowz, objective_value(model)
end

# ╔═╡ 129334f9-f415-458a-8a03-f21945b01c99
const SOLVE_MODES = [
	BilevelJuMP.SOS1Mode(),
	BilevelJuMP.IndicatorMode(),
	BilevelJuMP.StrongDualityMode(1e-5),
	# BilevelJuMP.StrongDualityMode(1e5)
]

# ╔═╡ aefa0d62-5f27-47a3-85ed-160dcea0d0a8
sensors, flows, score = let
	
	default_params = SolveParameters(0.0007; ForceBidirectional = true, TimeLimit = 60, SolverAttributes = Dict("MIPFocus" => 1),
	Ctype1 = SOLVE_MODES[3],
	Ctype2 = SOLVE_MODES[3],
	Ctype3 = SOLVE_MODES[3],
	SolveMode = SOLVE_MODES[3]
	)
	
	# it seems model starts with using all sensors and gradually removes sensors
	model(G, default_params)
end

# ╔═╡ 60085da4-477c-4e42-b27c-80e59e7eaa08
score

# ╔═╡ d7db632a-e6c8-4d82-920a-3169f0b4a3e8
length(sensors)

# ╔═╡ f454876e-039f-429b-ad3d-9aa40d6345b5
function hyperparameter_search(params::Vector{SolveParameters})
	function wrapper(P)
		try
			return model(G, P)
		catch
			return nothing
		end
	end
	return Dict(P => wrapper(P) for P ∈ params)
end

# ╔═╡ 03f180b1-836c-4917-a152-f1e7cb9e05fb
# HS = hyperparameter_search(vec([
# 	SolveParameters(0.001, TimeLimit = 60, Ctype1 = c1, Ctype2 = c2, Ctype3 = c3)
# 	for (c1, c2, c3) ∈ Iterators.product(repeat([SOLVE_MODES[1:2]], 3)...)
# ]))

# ╔═╡ 00427d95-929d-4861-b0b3-51653055f7b6
# Ctype2 is the only constraint that admits SOS-type (it's the only equality constraint)

# ╔═╡ 415da465-f933-42e1-9b64-8be44e7a608b
md"""
### Draw sensor and road graph
"""

# ╔═╡ 272eaeb9-4a5e-4155-ae58-04290be298e6
let
	@resumable function zipv(its)
		# Given an iterable of iterables, collect the `n`th element of each into a vector and yield these sequentially
		# Necessary because collect(zip(its...)) will cause a StackOverflow because of how Julia does type inference
		its = Iterators.Stateful.(its)
		while true
			nth = []
			for it ∈ its
				isempty(it) && return
				push!(nth, popfirst!(it))
			end
			@yield nth
		end
	end
	
	@resumable function position_edges(G)
		# object destructuring in Julia 1.7 would be useful here
		for e ∈ edges(G)
			up, vp = props.(Ref(G), [e.src, e.dst])
			@yield (
				up[:latitude], 
				up[:longitude], 
				vp[:latitude],
				vp[:longitude],
				flows[(e.src, e.dst)]
			)
		end
	end
	
	pair_edge(e) = (e.src, e.dst)
	
	@resumable function position_sensors(sensors)
		edge_set = Set(sensors)
		
		for (u, v) ∈ filter(∈(edge_set), map(pair_edge, edges(G)))
			@yield (
				(props(G, u)[:latitude] + props(G, v)[:latitude]) / 2,
				(props(G, u)[:longitude] + props(G, v)[:longitude]) / 2
			)
		end
	end
	
	p = plot(
		# plot sensors on top layer as points instead of lines
		layer(
			Geom.point;
			zip([:x, :y], zipv(position_sensors(sensors)))...
		),
		layer(
			Geom.segment(arrow=true, filled=true);
			zip([:x, :y, :xend, :yend, :color], zipv(position_edges(G)))...
		),
		Scale.x_continuous(minvalue=REGION[1][1], maxvalue=REGION[2][1]), 
		Scale.y_continuous(minvalue=REGION[1][2], maxvalue=REGION[2][2]),
		Theme(line_width = 0.1mm)
	)
	
	draw(SVGJS("sensors.svg"), p)
end

# ╔═╡ Cell order:
# ╠═cf95caf8-1b5f-11ec-2e4d-bfa2ebd7c94d
# ╠═20d26a5d-e30f-48fe-8987-8671063f1084
# ╠═5f7dd6fa-a03a-4f19-92b1-542c7fe90325
# ╠═0e44234d-3d87-4fcf-85d8-7f738d351bf2
# ╠═f273d6fe-2a3d-4fe1-8efe-3923edcd3a92
# ╠═25265fb3-7925-43a8-a38b-03d231b502ef
# ╠═db202c8d-6664-41d3-b24b-3cd42a8ca3cc
# ╠═b22090a3-9ea0-4551-b0e8-bbbc46870ec3
# ╠═910b9928-adb2-4bcd-b1e0-df67a3c3b5a0
# ╠═58399008-f753-4444-904b-f4f5533531b0
# ╠═26903824-7261-491f-9322-bfc152730dab
# ╠═c9fc1926-381d-4673-ad18-5505ab0abf3f
# ╠═375d1f19-59a4-4666-9ba2-65f3044243c2
# ╠═12c208b7-9656-4266-8d74-d74eb04af8b2
# ╠═129334f9-f415-458a-8a03-f21945b01c99
# ╠═aefa0d62-5f27-47a3-85ed-160dcea0d0a8
# ╠═60085da4-477c-4e42-b27c-80e59e7eaa08
# ╠═d7db632a-e6c8-4d82-920a-3169f0b4a3e8
# ╠═f454876e-039f-429b-ad3d-9aa40d6345b5
# ╠═03f180b1-836c-4917-a152-f1e7cb9e05fb
# ╠═00427d95-929d-4861-b0b3-51653055f7b6
# ╟─415da465-f933-42e1-9b64-8be44e7a608b
# ╠═272eaeb9-4a5e-4155-ae58-04290be298e6
