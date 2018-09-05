# EuclideanIRP
Solves Inventory Routing Problem (IRP) on Euclidean Instances

# Dependencies
[dapcstp](https://github.com/mluipersbeck/dapcstp)

[Gurobi](https://www.gurobi.com)

# Usage
The source files are currently set to run on one instance from EuclideanIRPinstances/inputsIterNmult100Htimes100_260. To obtain the output, create the corresponding outputs directory outputsIterNmult100Htimes100_260 in the folder of the heuristic called. Each of these may be modified to run over all instances in any instance directory by adding a loop over the parameter (H,N, or T) that is varying and a loop over the instance number from 1 to 100.
1. In the EuclideanIRPlp directory, solveMetricIPscf8coresCutOffLBToursSoln3displayrhCC.cpp models a given IRP instance as a MIP and calls Gurobi to find a lower bound on the optimal cost within 10% MIPgap.
2. In the EuclideanIRPdeleteAddLS directory, EuclideanBestPrioritizedOperationsLSwCutOffInitializeAddSolnToursSoln3cc.cpp finds a feasible solution using local search applying DELETEs, ADDs, and pairwise DELETE-ADDs.
3. In the EuclideanIRPaddLS directory, EuclideanBestAddLSwCutOffToursSolnForServer6cc.cpp finds a feasible solution using local search applying ADDs.
4. In the EuclideanIRPdeleteLS directory, EuclideanBestDeleteLS4wCutOffToursSolnCC.cpp finds a feasible solution using local search applying DELETEs.
5. In the EuclideanIRPgreedy directory, EuclideanGreedy7PrioritizedOperationsCutOffToursSolnCC.cpp finds a feasible solution by greedily chosing low density set covers to serve demands.
6. In the EuclideanIRPpd directory, EuclideanPrimalDualPrimalOnlyPrioritizedOperationsCutOffToursSolnCC.cpp finds a feasible solution via a primal dual framework to determine visits.
