using RoadGraphs

let
    # TODO: output graph region hash in .json log as well
    R0 = ((-36.9323, 174.8616), (-36.8878, 174.9565))
    G = read_graph("graph.json", R0...)

    MP = ModelParams(
        lambda = 0.5,
        bidir = true,
        junctions = true,
        )
    SP = SolveParams(
        time_limit = 60,
        )

    # Could accept parameters via web requests - would help with compile times
    run_model(G, MP, SP, "test", draw = true)
end
