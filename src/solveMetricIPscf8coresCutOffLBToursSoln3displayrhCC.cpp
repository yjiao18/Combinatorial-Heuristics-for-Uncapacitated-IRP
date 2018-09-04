/*models and solves the IP for IRP on any metric*/


#include<iostream>
#include<cmath>
#include<vector>
#include<fstream>
#include<assert.h>
#include<ratio>
#include<algorithm>
#include<chrono>
#include<unordered_set>
#include<gurobi_c++.h>
#include <stdexcept>

using namespace std;

/*
* Comment the line after this comment to disable all messages
* printed using DEBUG_MSG.
* All printing between the two timers should be done using DEBUG_MSG.
*/
#define DEBUG

#ifdef DEBUG
#define DEBUG_MSG(str) do {cout << str << std::endl; } while( false )
#else
#define DEBUG_MSG(str) do { } while ( false )
#endif


//for declaring size n vector with entries of type T
template<class T> vector<T> init1D(int n)
{
	vector<T> result;
	result.resize(n);
	return result;
}

//for declaring size n by m vector with entries of type T
template<class T> vector<vector<T> > init2D(int n, int m)
{
	vector<vector<T> > result;
	result.resize(n);
	for (int i = 0; i < n; i++) {
		result[i].resize(m);
	}
	return result;
}

//for declaring size n by m by p vector with entries of type T
template<class T> vector<vector<vector<T> > > init3D(int n, int m, int p)
{
	vector<vector<vector<T> > > result;
	result.resize(n);
	for (int i = 0; i < n; i++) {
		result[i].resize(m);
		for (int j = 0; j < m; j++) {
			result[i][j].resize(p);
		}
	}
	return result;
}


//for declaring size n by m by p by q vector with entries of type T
template<class T> vector<vector<vector<vector<T> > > > init4D(int n, int m, int p, int q)
{
	vector<vector<vector<vector<T> > > > result;
	result.resize(n);
	for (int i = 0; i < n; i++) {
		result[i].resize(m);
		for (int j = 0; j < m; j++) {
			result[i][j].resize(p);
			for (int k = 0; k < p; k++){
				result[i][j][k].resize(q);
			}
		}
	}
	return result;
}



int main()
{	//N clients
	int N = 100;

	//T days
	int T = 6;

	//unit holding cost min
	double min_h = .01;

	//holding cost scale
	double H = 2.6;

	//instance number
	int multiplicity = 1;


	char filename[1000];
	//directory to read input
	sprintf(filename, "../EuclideanIRPinstances/inputsIterNmult100Htimes100_260/input-N-%d-T-%d-Htimes100-%d-inst-%d.txt", N, T, (int)(H * 100), multiplicity);
	std::ifstream fin(filename);

	//directory to write output
	char outputname[1000];
	sprintf(outputname, "../EuclideanIRPlp/outputsIterNmult100Htimes100_260/output-N-%d-T-%d-Htimes100-%d-inst-%d.txt", N, T, (int)(H * 100), multiplicity);
	std::ofstream fout;
	fout.open(outputname, std::ofstream::out | std::ofstream::app);

	//read in the depot
	int depot;
	fin >> depot;

	//read in the size of problem
	int numClients, numDays;
	fin >> numClients;
	fin >> numDays;

	fout << depot << ' ';
	fout << numClients << ' ';
	fout << numDays << ' ';
	fout << min_h << ' ';
	fout << H << ' ';
	fout << "IPscf8coresCutOffLBToursSoln10percentMIPgap" << ' ';

	//declare demands
	//need to read in demands to compute holding costs
	//d^i_t goes to demand[i-1][t-1]
	vector<vector<double> > demand = init2D<double>(numClients, numDays);
	for (int i = 0; i < numClients; i++) {
		for (int j = 0; j < numDays; j++) {
			fin >> demand[i][j];			
		}
	}

	//read in the per unit holding costs
	vector<double> unitHolding = init1D<double>(numClients);
	for (int i = 0; i < numClients; i++){
		fin >> unitHolding[i];		
	}

	//read in the Euclidean positions of the clients
	vector<vector<int> > position = init2D<int>(numClients, 2);
	for (int i = 0; i < numClients; i++){
		for (int axis = 0; axis < 2; axis++){
			fin >> position[i][axis];			
		}
	}

	/*
	* At this point, we are done reading input so we can start the timer.
	*/
	auto start_timer = std::chrono::high_resolution_clock::now();

	//compute holding costs
	//H^i_{s,t}=(t-s)*d^i_t*h_i goes to holding[i-1][s-1][t-1]
	vector<vector<vector<double> > > holding = init3D<double>(numClients, numDays, numDays);
	for (int i = 0; i < numClients; i++) {
		for (int t = 0; t < numDays; t++) {
			for (int s = 0; s <= t; s++) {
				holding[i][s][t] = (t - s) * demand[i][t] * unitHolding[i];
			}
		}
	}

	//compute edge weights
	//w_{ij} = sqrt((X_i - X_j)^2+(Y_i - Y_j)^2)
	vector<vector<double> > edgeWeights = init2D<double>(numClients, numClients);
	for (int i = 0; i < numClients; i++){
		for (int j = 0; j < numClients; j++){
			edgeWeights[i][j] = sqrt(pow(position.at(i).at(0) - position.at(j).at(0), 2) + pow(position.at(i).at(1) - position.at(j).at(1), 2));
			
		}
	}

	//declare gurobi environment
	GRBEnv env = GRBEnv();

	//declare gurobi model
	GRBModel model = GRBModel(env);

	//number of cores allowed
	model.set("Threads", "8");

	//set termination condition based on when gap is reached
	model.getEnv().set(GRB_DoubleParam_MIPGap, .1);

	//Create variables x^i_{s,t} whether serve i,t on day s
	vector<vector<vector<GRBVar> > > x = init3D<GRBVar>(numClients, numDays, numDays);
	//Debug: check if need to add all variables declared in the vector, or only the ones with s <= t and positive demand
	for (int i = 0; i < numClients; i++){
		for (int t = 0; t < numDays; t++){
			//only add variable for those (i,t) of positive demand
			if (demand.at(i).at(t) > 0){
				for (int s = 0; s <= t; s++){					
					x.at(i).at(s).at(t) = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				}
			}
		}
	}


	//create variables X^v_s whether visit v on day s
	vector<vector<GRBVar> > X = init2D<GRBVar>(numClients, numDays);
	for (int v = 0; v < numClients; v++){
		for (int s = 0; s < numDays; s++){
			X.at(v).at(s) = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
		}
	}

	//create variables z^(u,v)_s whether use arc uv on day s
	vector<vector<vector<GRBVar> > > z = init3D<GRBVar>(numClients, numClients, numDays);
	for (int u = 0; u < numClients; u++){
		//count each arc
		for (int v = 0; v < numClients; v++){
			if (v != u){
				for (int s = 0; s < numDays; s++){					
					z.at(u).at(v).at(s) = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
				}
			}
		}
	}
	DEBUG_MSG("created z variables" << endl);

	//create variables h^(u,w)_s single commodity flow from depot using arc uw on day s
	vector<vector<vector<GRBVar> > > h = init3D<GRBVar>(numClients, numClients, numDays);
	for (int u = 0; u < numClients; u++){
		//count each arc in both directions
		for (int w = 0; w < numClients; w++){
			if (w != u){
				for (int s = 0; s < numDays; s++){
					h.at(u).at(w).at(s) = model.addVar(0.0, (double)(numClients - 1), 0.0, GRB_CONTINUOUS);
				}
			}
		}
	}

	DEBUG_MSG("created h variables" << endl);

	//compute objective function
	//compute routing part of objective
	GRBLinExpr lpRouting = 0.0;
	for (int u = 0; u < numClients; u++){
		//count each arc
		for (int v = 0; v < numClients; v++){
			if (v != u){
				for (int s = 0; s < numDays; s++){
					lpRouting += edgeWeights.at(u).at(v) * z.at(u).at(v).at(s);
				}
			}
		}
	}

	//compute holding part of objective
	GRBLinExpr lpHolding = 0.0;
	for (int i = 0; i < numClients; i++){
		for (int t = 0; t < numDays; t++){
			if (demand.at(i).at(t) > 0){
				for (int s = 0; s <= t; s++){
					lpHolding += holding.at(i).at(s).at(t) * x.at(i).at(s).at(t);
				}
			}
		}
	}

	GRBLinExpr lpObjective = lpRouting + lpHolding;

	//set the objective function
	model.setObjective(lpObjective, GRB_MINIMIZE);

	//type 0 constraints: edge use upper bound
	vector<vector<vector<GRBLinExpr> > > lhs0 = init3D<GRBLinExpr>(numClients, numClients, numDays);
	for (int u = 0; u < numClients; u++){
		for (int w = u + 1; w < numClients; w++){
			for (int s = 0; s < numDays; s++){
				lhs0.at(u).at(w).at(s) = z.at(u).at(w).at(s) + z.at(w).at(u).at(s);
				model.addConstr(lhs0.at(u).at(w).at(s) <= 1.0);
			}
		}
	}
	DEBUG_MSG("added type 0 constraints" << endl);
	
	//cycle out of constraints for depot:
	vector<GRBLinExpr> lhsDepotCycleOut = init1D<GRBLinExpr>(numDays);
	for (int s = 0; s < numDays; s++){		
		lhsDepotCycleOut.at(s) = 0;
		for (int w = 0; w < numClients; w++){
			if (w != depot){
				lhsDepotCycleOut.at(s) += z.at(depot).at(w).at(s);
			}

		}
		model.addConstr(lhsDepotCycleOut.at(s) <= 1);
	}

	//cycle into constraints for depot:
	vector<GRBLinExpr> lhsDepotCycleInto = init1D<GRBLinExpr>(numDays);
	for (int s = 0; s < numDays; s++){
		lhsDepotCycleInto.at(s) = 0;
		for (int w = 0; w < numClients; w++){
			if (w != depot){
				lhsDepotCycleInto.at(s) += z.at(w).at(depot).at(s);
			}

		}
		model.addConstr(lhsDepotCycleInto.at(s) <= 1);
	}

	//cycle out of constraints for clients:
	vector<vector<GRBLinExpr>> lhsCycleOut = init2D<GRBLinExpr>(numClients, numDays);
	for (int v = 0; v < numClients; v++){
		if (v != depot){
			for (int s = 0; s < numDays; s++){
				lhsCycleOut.at(v).at(s) = 0;
				for (int w = 0; w < numClients; w++){
					if (w != v){
						lhsCycleOut.at(v).at(s) += z.at(v).at(w).at(s);

					}

				}
				model.addConstr(lhsCycleOut.at(v).at(s) == X.at(v).at(s));
			}
		}
	}

	//cycle into constraints for clients:
	vector<vector<GRBLinExpr>> lhsCycleIn = init2D<GRBLinExpr>(numClients, numDays);
	for (int v = 0; v < numClients; v++){
		if (v != depot){
			for (int s = 0; s < numDays; s++){
				lhsCycleIn.at(v).at(s) = 0;
				for (int w = 0; w < numClients; w++){
					if (w != v){
						lhsCycleIn.at(v).at(s) += z.at(w).at(v).at(s);
					}

				}
				model.addConstr(lhsCycleIn.at(v).at(s) == X.at(v).at(s));
			}
		}
	}

	DEBUG_MSG("added cycle constraints" << endl);

	//compute LHS of type 1 constraints: coverage
	vector<vector<GRBLinExpr> > lhs1 = init2D<GRBLinExpr>(numClients, numDays);
	for (int i = 0; i < numClients; i++){
		for (int t = 0; t < numDays; t++){
			if (demand.at(i).at(t) > 0){
				//initialize lhs1[i][t] = 0
				lhs1.at(i).at(t) = 0;
				for (int s = 0; s <= t; s++){
					lhs1.at(i).at(t) += x.at(i).at(s).at(t);
				}
				model.addConstr(lhs1.at(i).at(t) == 1.0);
			}
		}
	}

	DEBUG_MSG("added type 1 constraints" << endl);

	//compute LHS of type 2 constraints: X lower bounded by x for all t
	vector<vector<vector<GRBLinExpr> > > lhs2 = init3D<GRBLinExpr>(numClients, numDays, numDays);
	for (int v = 0; v < numClients; v++){
		for (int t = 0; t < numDays; t++){
			for (int s = 0; s <= t; s++){
				lhs2.at(v).at(t).at(s) = X.at(v).at(s) - x.at(v).at(s).at(t);
				model.addConstr(lhs2.at(v).at(t).at(s) >= 0.0);
			}
		}
	}

	DEBUG_MSG("added type 2 constraints" << endl);

	//type 3 constraints: connectivity
	vector<vector<GRBLinExpr> > lhs3 = init2D<GRBLinExpr>(numClients, numDays);
	for (int u = 0; u < numClients; u++){
		for (int s = 0; s < numDays; s++){
			//initialize lhs3
			lhs3.at(u).at(s) = 0.0;
			//create lhs variable to check attempted solution
			double tempLhs3 = 0;
			//flow from r into u on day s
			for (int w = 0; w < numClients; w++){
				if (w != u){
					lhs3.at(u).at(s) += h.at(w).at(u).at(s);
				}
			}
			//flow from r out of u on day s
			for (int w = 0; w < numClients; w++){
				if (w != u){
					lhs3.at(u).at(s) -= h.at(u).at(w).at(s);
				}
			}
			//if u is not depot
			if (u != depot){
				model.addConstr(lhs3.at(u).at(s) == X.at(u).at(s));
			}
			//if u is depot
			else{
				GRBLinExpr totalCoverage = 0.0;
				int tempTC = 0;
				for (int a = 0; a < numClients; a++){
					if (a != depot){
						totalCoverage += -X.at(a).at(s);
					}
				}
				model.addConstr(lhs3.at(u).at(s) == totalCoverage);
			}
		}
	}

	DEBUG_MSG("added type 3 constraints" << endl);

	//type 4 constraints: edge use lower bound
	vector<vector<vector<GRBLinExpr> > > lhs4 = init3D<GRBLinExpr>(numClients, numClients, numDays);
	for (int u = 0; u < numClients; u++){
		for (int w = 0; w < numClients; w++){
			if (w != u){
				for (int s = 0; s < numDays; s++){
					lhs4.at(u).at(w).at(s) = h.at(u).at(w).at(s) - (numClients-1)*z.at(u).at(w).at(s);
					model.addConstr(lhs4.at(u).at(w).at(s) <= 0.0);
				}
			}
		}
	}

	DEBUG_MSG("added type 4 constraints" << endl);

	/*extra constraints to tighten the IP*/
	
	//can assume depot is served exactly on each deadline for free	
	for (int t = 0; t < numDays; t++){
		model.addConstr(x.at(depot).at(t).at(t) == 1.0);
	}

	//can assume depot is visited every day for free
	for (int s = 0; s < numDays; s++){
		model.addConstr(X.at(depot).at(s) == 1.0);
	}

	DEBUG_MSG("added tightening cosntraints" << endl);

	/*end of all constraints*/

	//optimize model
	model.optimize();

	//get the lower bound found so far
	double totalCost = model.get(GRB_DoubleAttr_ObjBoundC);

	cout << "routing cost is " << lpRouting.getValue() << endl;
	cout << "holding cost is " << lpHolding.getValue() << endl;
	cout << "the gap is " << model.get(GRB_DoubleAttr_MIPGap) << endl;
	fout << model.get(GRB_DoubleAttr_MIPGap) << ' ';

	/*
	* Stop the timer. We should do this before printing the final answer.
	*/
	auto elapsed = std::chrono::high_resolution_clock::now() - start_timer;
	long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();

	cout << "Program execution time = " << microseconds << " microseconds\n";
	fout << microseconds << ' ';
				
	//write routing cost and holding cost
	fout << lpRouting.getValue() << ' ';
	fout << lpHolding.getValue() << ' ';

	//output total cost
	cout << "The total cost is " << totalCost << "\n";
	fout << totalCost << endl;
	fout.close();


	return 0;
}