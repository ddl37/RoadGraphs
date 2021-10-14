using RoadGraphs

# Real-time demo, displaying solver output
M = let 
    R0 = ((-36.9323, 174.8616), (-36.8878, 174.9565))
    G = read_graph("graph.json", R0...)
    println(G)

    MP = ModelParams(
        lambda = 0.5,
        bidir = true,
        junctions = true,
        )

    # 2 minute time limit for solve    
    SP = SolveParams(
        time_limit = 60 * 2,
        )

    rm = RoadModel(G, MP, SP)
    setup!(rm)
    solve!(rm)
    draw!(rm)
end
