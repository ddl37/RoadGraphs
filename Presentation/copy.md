### The Game

Player 1

- Places some sensors on the edges of the road graph

Player 2

- Sends as much traffic through the network as possible, whilst avoiding the sensors that player 1 placed

Note: As player 1 you have to consider the **worst-case scenario**

(so this is kinda a difficult problem)

Start by describing how we turn the problem brief into an optimisation problem

---

### The model

For each edge, $(u, v) \in E$:
- Sensor placement: 0/1 binary variable $s_{uv}$
- Nonnegative flow: $f_{uv}$, zero if $s_{uv} = 1$

Model allowed traffic flows by linear equality and inequality constraints, creating region $F(S)$

$f_{uv} \geq 0$ cuts out a cone

$f_{ij} = f_{k\ell}$ cuts out a linear subspace

Note: Constraints cut out some convex *feasible region* of valid traffic flows, so $F(S)$ is a *set*

Geometriaclly, cone is positive quadrant

In terms of constraints, we have conservation of mass: flow in = flow out, as well as being able to set up which directions you're allowed to turn, at an intersection. 

---

### The objective

$$\newcommand{\abs}[1]{\left| #1 \right|}
\newcommand{\norm}[1]{ \left\Vert #1 \right\Vert }
\min_{S \in \\{0, 1\\}^{\abs E}} \left( \overbrace{\norm S_1}^\text{# sensors} + \lambda \cdot \underbrace{\max_{F(S)} \left[ \sum_{(u, v) \in E} \ell_{uv} f_{uv} \right]}_\text{uncertainty} \right)$$

where $F(S)$ is the set of "valid flows" as a function of the sensor placement, $\ell_{uv}$ is the length of the edge/road in metres

Note: This looks like an awful expression, the exact form is unimportant, but:

The minimisation corresponds to player 1 picking the position of the sensors, and the maximisation consists of player 2 sending as much traffic flow as possible - where flow is weighted by the edge length: longer edges are more important. 

What does this mean?
- There's two terms in the objective
- First is just the number of sensors - we want to make that small
- Second is uncertainty - maximised traffic flow - how well player 2 can do. 
- lambda is a "regularisation parameter" - how important is information vs cost. 
- if lambda is zero, the second term disappears - how do you minimise a sum of nonnegative things? set all of them to zero -- i.e. we don't place any sensors at all
- if lambda is extremely large, increasing the number of sensors is insignificant, compared to any potential traffic flow reduction - so we place almost all sensors (non-redundant ones, that is)

How do we solve this? (not important) Bi-level mixed integer linear program
- Belongs to branch of mathematics called convex analysis, consider a transformed problem with a related solution, duality theory
- Basically we introduce new variables and fiddle with the constraints a bit

^^^

<!-- .slide: data-background="#ffffff" data-fullscreen -->
<object data="lambda-1.7.svg" style="height: 95vh"></object>

Note: Here are some examples of what our model produces for some values of lambda. The blue dots are positions of sensors, while the colour of an edge is its flow, in the scenario where a maximal amount of traffic tries to avoid the sensors - so by assumption, the flow on edges with sensors is zero - which corresponds to blue. (zoom)

^^^

<!-- .slide: data-background="#ffffff" data-fullscreen -->
<object data="lambda-1.1.svg" style="height: 95vh"></object>

^^^

<!-- .slide: data-background="#ffffff" data-fullscreen -->
<object data="lambda-0.7.svg" style="height: 95vh"></object>

^^^

<!-- .slide: data-background="#ffffff" data-fullscreen -->
<object data="lambda-0.3.svg" style="height: 95vh"></object>

Note: You can see that as the lambda parameter gets smaller, there are fewer placed - and the edges get warmer - less information is known about the traffic flow on them. However, the model still tries to prioritise "important" edges - longer/more capacity. 

---

### The code

```julia
function build_model()
	model = BilevelModel(() -> Gurobi.Optimizer())
	@variable(Upper(model), sensors[1:num_edges], Bin)
	@variable(Lower(model), flows[1:num_edges] >= 0)

	@objective(Lower(model), Max, 
		dot(flows, flow_weights))
	@objective(Upper(model), Min, 
		sensor_sum + λ * dot(flows, flow_weights))

	apply_constraints!(model)

	optimize!(model)
end
```

Note: We implemented our model in a programming language called Julia, this is syntactically valid Julia code. (not really important)

^^^

```julia
function cs_conserve_flow!(m, G::MetaDiGraph)
    for v ∈ vertices(G)
		if (length ∘ Set)(all_neighbors(G, v)) <= 1
			continue
		end
		expr = 0
		for i ∈ inneighbors(G, v)
			expr += flow_var(G, i, v)
		end
		for o ∈ outneighbors(G, v)
			expr -= flow_var(G, v, o)
		end
		@constraint(m, expr == 0)
	end
end
```

Note: This is the "programmatic" version of the conservation of mass constraints.  Math expressions, symbolic, procedurally manipulate with code. Use these to programmatically declare constraints. 

---

# The results


