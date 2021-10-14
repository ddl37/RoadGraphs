using RoadGraphs

let 
    R0 = ((-36.9323, 174.8616), (-36.8878, 174.9565))
    G = read_graph("graph.json", R0...)

    # 13 configs x 5 minutes = 1 hour
    for λ in [0.1, 0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5, 1.7, 1.9, 2.1, 2.3, 2.5]
        MP = ModelParams(
            lambda = λ,
            bidir = true,
            junctions = true,
            )
        SP = SolveParams(
            time_limit = 60 * 5,
            )
        
        run_model(G, MP, SP, "lambda-$(λ)", draw = true)
    end
end
