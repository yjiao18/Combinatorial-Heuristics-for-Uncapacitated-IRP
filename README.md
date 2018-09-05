# EuclideanIRP
Solves Inventory Routing Problem (IRP) on Euclidean Instances

# Dependencies
[dapcstp](https://github.com/mluipersbeck/dapcstp)
[Gurobi](https://www.gurobi.com)

# Usage
The source files are currently set to run on one instance from inputsIterNmult100Htimes100_260. To obtain the output, create the corresponding outputs folder outputsIterNmult100Htimes100_260 in the folder of the heuristic called. Each of these may be modified to run over all instances in an instance folder by adding a loop over the parameter (H,N, or T) that is varying and a loop over the instance number from 1 to 100.
1. solveMetricIPscf8coresCutOffLBToursSoln3displayrhCC.cpp models a given IRP instance as a MIP and calls Gurobi to find a lower bound on the optimal cost within 10% MIPgap.
2. EuclideanBestPrioritizedOperationsLSwCutOffInitializeAddSolnToursSoln3cc.cpp finds a feasible solution using local search applying DELETEs, ADDs, and pairwise DELETE-ADDs.
3. EuclideanBestAddLSwCutOffToursSolnForServer6cc finds a feasible solution using local search applying ADDs.
4. EuclideanBestDeleteLS4wCutOffToursSolnCC finds a feasible solution using local search applying DELETEs.
5. EuclideanGreedy7PrioritizedOperationsCutOffToursSolnCC finds a feasible solution by greedily chosing low density set covers to serve demands.
6. EuclideanPrimalDualPrimalOnlyPrioritizedOperationsCutOffToursSolnCC finds a feasible solution via a primal dual framework to determine visits.
