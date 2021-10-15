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

"""
Introduces additional variables for which direction flow takes at each junction. Can be used to model allowed turning directions
"""
function cs_junction!(m, M::RoadModel)
	G = M.G
	for v in vertices(G)
		if length(outneighbors(G, v)) == 0 || length(inneighbors(G, v)) == 0
			continue
		end
		
		out_exprs = Dict(ov => -flow_var(G, v, ov) for ov in outneighbors(G, v))
		for iv in inneighbors(G, v)
			in_expr = -flow_var(G, iv, v)
			for ov in outneighbors(G, v)
				if ov == iv
					continue
				end

				# declare anonymous junction variable
				iv_ov_var = @variable(m)
				set_lower_bound(iv_ov_var, 0)
				# set_upper_bound(iv_ov_var, 500) # maybe upper bound speeds up solver? (nope)

				out_exprs[ov] += iv_ov_var
				in_expr += iv_ov_var
			end
			push!(M.constraints, @constraint(m, in_expr == 0))
		end

		for oc in values(out_exprs)
			push!(M.constraints, @constraint(m, oc == 0))
		end
	end
end

"""
Impose "max flow creation" constraints on larger basic cycles. Not feasible to do this for *all* cycles (unless using lazy constraints?)
"""
function cs_anticycle_local!(m, M::RoadModel)
    # TODO: Waiting on cycle area implementation in `road_utils`
end

"""
Form connected components of "bad/small" cycles and bound all contained edges by the net flow entering the component
"""
function cs_anticycle_nonlocal!(m, M::RoadModel)
	# Not implemented
end

function cs_sensor!(m, M::RoadModel)
    for (u, v, prop) in edge_prop_triples(M.G)
        push!(M.constraints, @constraint(m, prop[:flow_var] <= prop[:capacity] * (1 - M.MP.sensor_cap_ratio * prop[:sensor_var])))
    end

    if M.MP.force_sensor_bidirectional
		for (u, v, prop) in edge_prop_triples(M.G)
			# sort edge components so that constraint is only added once
			if has_edge(M.G, v, u) && u < v
				push!(M.constraints, @constraint(m, prop[:sensor_var] == get_prop(M.G, v, u, :sensor_var)))
			end
		end
    end
end

export flow_var, cs_conserve_flow!, cs_inflow!, cs_junction!, cs_anticycle_local!, cs_anticycle_nonlocal!, cs_sensor!
