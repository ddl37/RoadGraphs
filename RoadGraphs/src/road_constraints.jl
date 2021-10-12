### road_constraints.jl

flow_var(G, i, j) = get_prop(G, i, j, :flow_var)

"""
Standard flow conservation constraints, not applied at dead-ends
"""
function cs_conserve_flow!(m, M::RoadModel)
    G = M.G
    for v in vertices(G)
		if length(Set(all_neighbors(G, v))) <= 1
			continue
		end
        # additional conditions?
		
		expr = 0
		for i in inneighbors(G, v)
			expr += flow_var(G, i, v)
		end
		
		for o in outneighbors(G, v)
			expr -= flow_var(G, v, o)
		end

		push!(M.constraints, @constraint(m, expr == 0))
	end
end

function cs_inflow!(m, M::RoadModel)
    G = M.G
	for v in vertices(G)
		for iv in inneighbors(G, v)
			expr = -flow_var(G, iv, v)
			
			ov_count = 0
			for ov in outneighbors(G, v)
				if ov == iv
					continue
				end
				expr += flow_var(G, v, ov)
				ov_count += 1
			end
			
			if ov_count == 0
				continue # dead-end
			end

			push!(M.constraints, @constraint(m, expr >= 0))
		end
	end
end

function cs_junction!(m, M::RoadModel)

end

"""
Impose "max flow creation" constraints on larger basic cycles. Not feasible to do this for *all* cycles (unless using lazy constraints?)
"""
function cs_anticycle_local!(m, M::RoadModel)
    # TODO: Waiting on cycle area implementation in `road_utils`
end

"""
Form connected components of "bad/small" cycles and bound containing edges by the net flow entering the component
"""
function cs_anticycle_nonlocal!(m, M::RoadModel)
    # TODO: Construct dual/incidence graph on basic cycles
end

function cs_sensor!(m, M::RoadModel)
    for (u, v, prop) in edge_prop_triples(M.G)
        push!(M.constraints, @constraint(m, prop[:flow_var] <= prop[:capacity] * (1 - M.MP.sensor_cap_ratio * prop[:sensor_var])))
    end

    if M.MP.force_sensor_bidirectional
        # TODO
    end
end

export flow_var, cs_conserve_flow!, cs_inflow!, cs_junction!, cs_anticycle_local!, cs_anticycle_nonlocal!, cs_sensor!
