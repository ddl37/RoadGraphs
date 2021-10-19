# Intro

Presentation slides incomplete...

Note: Earlier we gave conditions for obtaining *complete* information. However, that may be too many sensors for NZTA to consider, so we want to scale down the number of sensors while preserving as much information as possible. 

---

# The Game

Player 1

- Places some sensors on edges

Player 2

- Sends as much traffic through the network as possible, whilst avoiding the sensors that player 1 placed

Note: So now we want to actually construct a solution (that is, a placement of sensors on edges of the graph)

As player 1 you have to consider the **worst-case scenario**

(so this is kinda a difficult problem)

Start by describing how we turn the problem brief into an optimisation problem

---

# The model

As usual, binary variables for sensors

Can impose linear equality and inequality constraints to model real-world roads

You can imagine the set of possible traffic flows as some convex region in space, like $\mathbb R^{3242}$, for a graph with 3242 directed edges

$f_{uv} \geq 0$ cuts out a cone

$f_{ij} = f_{k\ell}$ cuts out a linear subspace

Note: Give some examples verbally, don't spend too much time on it

As said earlier, ... flow in = flow out, at each junction which direction you're allowed to turn, maximum capacity of each road, etc.

---

# The objective

$$\newcommand{\abs}[1]{\left| #1 \right|}
\newcommand{\norm}[1]{ \left\Vert #1 \right\Vert }
\min_{S \in \\{0, 1\\}^{\abs E}} \left( \overbrace{\norm S_1}^\text{# sensors} + \frac{\lambda}{\abs E} \underbrace{\max_{F(S)} \left[ \sum_{(u, v) \in E} \ell_{uv} f_{uv} \right]}_\text{uncertainty} \right)$$

where $F(S)$ is the convex feasible region of "valid flows", as a function of the sensor placement

Note: The exact form of this expression is unimportant, but:

The minimisation corresponds to player 1 picking the position of the sensors, and the maximisation consists of player 2 sending as much traffic flow as possible. 

What does this mean?
- There's two terms in the objective
- lambda is a "regularisation parameter" - how important is information vs cost. 
- if lambda is zero, the second term disappears - how do you minimise a sum of nonnegative things? set all of them to zero -- i.e. we don't place any sensors at all
- if lambda is extremely large, increasing the number of sensors is insignificant, compared to any potential traffic flow reduction - so we place almost all sensors (non-redundant ones, that is)

Each of those constraints is encoded in $F(S)$

How do we solve this? Bi-level mixed integer linear program

- Belongs to branch of mathematics called convex analysis, consider a transformed problem with a related solution, duality theory
- Basically we introduce new variables and fiddle with the constraints a bit

^^^^
DOWN

plot with different lambda values

---

# The code

Placeholder

<!-- <pre class="stretch"> -->
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
<!-- </pre> -->

Note: No need to learn some domain-specific modelling language

---

# The results

See next slide