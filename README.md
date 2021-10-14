# Install stuff

- Obtain an [Academic Gurobi License](https://www.gurobi.com/downloads/end-user-license-agreement-academic/) using your university email

- Install the Gurobi solver

- Install the latest version (1.6.3) of the [Julia](https://julialang.org/downloads/) programming language

# Setup

- Clone this repository: `git clone https://github.com/ddl37/RoadGraphs.git`

- Open a command prompt/terminal/shell, `cd` to the `RoadGraphs` folder of this repo

- Enter the Julia REPL: type `julia`

- Type `]dev .` to install this package and its dependencies

- Exit the REPL: `Ctrl + D` (probably)

You can now run `julia demo.jl` from the command line (at the root folder) which will show realtime progress of the integer program solver. It should also generate a `road_sensors.svg` file that can be opened in the browser. 

Alternatively, `julia gen_test.jl` should run a test model and dump output in the `out/` folder. If the `.json` file name contains `-FAILED`, something went wrong - check the `exception` field. 
