### road_utils.jl

import JSON
import Base.Iterators as It
using ResumableFunctions

edge_pairs(G) = It.map(e -> (e.src, e.dst), edges(G))

edge_prop_triples(G) = It.map(e -> (e.src, e.dst, props(G, e.src, e.dst)), edges(G))

"Convert dictionary keys from strings to symbols"
symbolify(d::Dict) = Dict((Symbol(k) => v) for (k, v) ∈ pairs(d))

function read_graph(filename, bbox_ll = (-Inf, -Inf), bbox_ur = (Inf, Inf))::MetaDiGraph
    data = open(filename, "r") do io
		JSON.parse(io)
	end
	
	G = MetaDiGraph()
	
    # filter by latitude and longitude
	in_bbox(v_data) = (bbox_ll[1] <= v_data["latitude"] <= bbox_ur[1]) && (bbox_ll[2] <= v_data["longitude"] <= bbox_ur[2])
	
	for (i, v_data) ∈ enumerate(It.filter(in_bbox, data["nodes"]))
		@assert add_vertex!(G)
		@assert set_props!(G, i, symbolify(v_data))
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

"Distance between lat/long pair in metres ~ from StackOverflow"
function geo_distance(lat1, lon1, lat2, lon2)
    p = π/180
    a = 0.5 - cos((lat2-lat1)*p)/2 + cos(lat1*p) * cos(lat2*p) * (1-cos((lon2-lon1)*p))/2
    return 12742 * asin(sqrt(a)) * 1000
end

function triangle_area(lat_long_pairs)
    @assert length(lat_long_pairs) == 3
    
    d(u, v) = geo_distance(lat_long_pairs[u]..., lat_long_pairs[v]...)
    
    # Heron's formula
    a, b, c = d(1, 2), d(2, 3), d(3, 1)
    @assert a >= 0 lat_long_pairs
    @assert b >= 0 lat_long_pairs
    @assert c >= 0 lat_long_pairs
    s = (a + b + c) / 2
    x = s * (s - a) * (s - b) * (s - c)
    @assert x >= 0
    return sqrt(x)
end

mean(x) = sum(x) / length(x)

"""
Crude approximation to the area (m^2) enclosed by a cycle (does not take into account spherical geometry of Earth like `areaquad` in Matlab)

Note: Assumes vertices not repeated
"""
function cycle_area(G, cycle)
    # Replace vertices by their latitude and longitude
    geo_cycle = collect(It.map(i -> (get_prop(G, i, :latitude), get_prop(G, i, :longitude)), cycle))

    # Probably not an "actual" midpoint in the case of weird cycles, but good enough
    midpoint = (mean(first.(geo_cycle)), mean(last.(geo_cycle)))

    idxmod = i -> mod1(i, length(cycle))
    
    # TODO: Formula broken and does not work
    sum(triangle_area([geo_cycle[idxmod(i)], geo_cycle[idxmod(i + 1)], midpoint]) for i in 1:length(cycle))
end

"""
Returns a list of lists of directed basic cycles (grouped by their corresponding undirected cycle - so 0/1/2 per group), along with their area and total length

Use `It.flatten(basic_cycles(G))` if grouping is irrelevant
"""
function basic_cycles(G)
end

# @resumable 
function basic_cycles(G)
    # Convert `G` to an undirected graph (vertices *should* stay the same)
    # cycle_basis requires G <: AbstractSimpleGraph but MetaGraph, MetaDiGraph <: AbstractMetaGraph
    out = []

    UG = Graph(MetaGraph(G))
    for bs in cycle_basis(UG)
        # Note: Cannot use idxmod(i) = ... syntax
        idxmod = i -> mod1(i, length(bs))
        ℓ = (i, j) -> let 
            ix, jx = bs[idxmod(i)], bs[idxmod(j)]
            if has_edge(G, ix, jx)
                get_prop(G, ix, jx, :length)
            else
                get_prop(G, jx, ix, :length)
            end
        end

        area = cycle_area(G, bs)
        circ = sum(ℓ(i, i + 1) for i in 1:length(bs))
        # could probably abstract into cycle iterator
        
        cs = []
        if all(has_edge(G, idxmod(i), idxmod(i + 1)) for i in 1:length(bs))
            push!(cs, (bs, area, circ))
        end
        if all(has_edge(G, idxmod(i), idxmod(i - 1)) for i in 1:length(bs))
            push!(cs, (reverse(bs), area, circ))
        end
        # @yield cs
        push!(out, cs)
    end
    return out
end

"""
Given an iterable of iterables, collect the `n`th element of each into a vector and yield these sequentially.
Necessary because collect(zip(its...)) will cause a StackOverflow due to how Julia does type inference
"""
function zipv(its)
end

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

export edge_pairs, edge_prop_triples, symbolify, read_graph, geo_distance, triangle_area, cycle_area, basic_cycles, zipv
