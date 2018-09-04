/*
Computes add-only local search starting with the serve everything at time 1 solution for IRP

Note all indices in the paper are shifted by -1 in the vectors below e.g. index of 1 becomes index of 0 in code

Note indices for the pcst solver need to start from 1, not 0
*/

#include<iostream>
#include<cmath>
#include<vector>
#include<fstream>
#include<assert.h>
#include<chrono>
#include<unordered_set>

using namespace std;

/*
* Comment the line after this comment to disable all messages
* printed using DEBUG_MSG.
* All printing between the two timers should be done using DEBUG_MSG.
*/
//#define DEBUG

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


//find t_hat where t_hat is the t >= t' with no visits to v occuring within [t',t]
int t_hat(int _client, int _day, int _numDays, vector<vector<bool> > _visit){
	int targetDay = _day;
	//go through t >= _day stopping just before the first time that _client is visited after _day
	for (int t = _day; t < _numDays; t++){
		//stop if _client is visited at time t
		if (_visit.at(_client).at(t)){
			break;
		}
		else{
			targetDay = t;
		}
	}
	return targetDay;
}

//find cost of tree given the adjacency matrix of the tree
double treeCost(int _numClients, vector<vector<double> > _edgeWeights, vector<vector<bool> > _tree){
	double cost = 0;
	for (int i = 0; i < _numClients; i++){
		//go through all arcs of PCST solution
		for (int j = 0; j < _numClients; j++){
			if (_tree.at(i).at(j)){
				cost += _edgeWeights.at(i).at(j);				
			}
		}
	}
	return cost;
}



//find the reward cost of a given pcst solution, i.e. sum of penalty(v) over v covered by the pcst tree
double rewardCost(int _numClients, vector<double> _penalties, unordered_set<int> _pcstSet){
	double cost = 0;
	for (int v : _pcstSet){		
		cost += _penalties.at(v);
	}	
	return cost;
}

//find the change in cost given the cost of the existing tree, the cost of the pcst tree, and the cost of the pcst penalties
double costChange(double _pcstTreeCost, double _pcstRewardCost){
	double costDiff = _pcstTreeCost - _pcstRewardCost;
	return costDiff;
}

//update routing cost and holding cost
void updateCosts(double _pcstTreeCost, double _pcstRewardCost, double &routingCost, double &holdingCost){
	routingCost += _pcstTreeCost;
	holdingCost += -_pcstRewardCost;
}

//create and solve a TSP instance given the total number of clients, all the Euclidean positions, and which clients to cover
double tourCostOfVisit(int _numClients, vector<vector<int> > _position, vector<bool> _visit, vector<vector<double> > _edgeWeights){
	// Remove any existing input and output files
	std::remove("tourInst.tsp");
	std::remove("tourLog.txt");

	//find how many are to be visited
	int numVisit = 0;
	for (int v = 0; v < _numClients; v++){
		if (_visit.at(v)){
			numVisit++;
		}
	}

	//declare cost of tour
	double tourCost;

	//concorde requires at least 3 nodes to run on instance
	if (numVisit >= 3){
		/*create tsp input file singleTourInstPD.tsp*/
		// Open a temprary input file
		char filename[1000];
		sprintf(filename, "tourInst.tsp");
		std::ofstream fout;
		fout.open(filename, std::ofstream::out | std::ofstream::app);

		fout << "NAME : tourInst" << endl;
		fout << "COMMENT : tour of a given subset of vertices" << endl;
		fout << "TYPE : TSP" << endl;
		fout << "DIMENSION : " << numVisit << endl;
		fout << "EDGE_WEIGHT_TYPE : EUC_2D" << endl;
		fout << "NODE_COORD_SECTION" << endl;
		for (int v = 0; v < _numClients; v++){
			if (_visit.at(v)){
				fout << v << " " << _position.at(v).at(0) << " " << _position.at(v).at(1) << endl;
			}
		}
		fout << "EOF" << endl;

		//finalize input
		fout.close();
		/*end of tsp input file creation*/


		/*solve singleTourInstPD.tsp using concorde and write log file to output singleTourLogPD.txt*/
		system("concorde tourInst.tsp > tourLog.txt");

		/*compute the cost of the tour found by Concorde*/
		/*WARNING: the cycle output by Concorde relabels the vertices by the minimal available label*/

		//store the labels of the visited vertices in a vector
		vector<int> vertexLabels;
		for (int v = 0; v < _numClients; v++){
			if (_visit.at(v)){
				vertexLabels.push_back(v);
			}
		}

		//open the solution file singleTourInstPD.sol
		char solFilename[1000];
		//directory to read input
		sprintf(solFilename, "tourInst.sol");
		std::ifstream fin(solFilename);

		//the first value in singleTourInstPD.sol is the number of visited vertices
		int tspLength;
		fin >> tspLength;


		//store the ordered positions of vertices from the tour into a vector
		vector<int> posOrder = init1D<int>(tspLength);
		for (int i = 0; i < tspLength; i++){
			fin >> posOrder.at(i);

		}

		//store the edges of the cycle wrt the original vertex labels
		vector<vector<bool> > cycle = init2D<bool>(_numClients, _numClients);
		for (int u = 0; u < _numClients; u++){
			for (int v = 0; v < _numClients; v++){
				cycle.at(u).at(v) = false;
			}
		}
		for (int pos = 0; pos < tspLength - 1; pos++){
			cycle.at(vertexLabels.at(posOrder.at(pos))).at(vertexLabels.at(posOrder.at(pos + 1))) = true;
			DEBUG_MSG("storing into cycle the edge " << vertexLabels.at(posOrder.at(pos)) << " to " << vertexLabels.at(posOrder.at(pos + 1)) << endl);
		}
		cycle.at(vertexLabels.at(posOrder.at(tspLength - 1))).at(vertexLabels.at(posOrder.at(0))) = true;
		DEBUG_MSG("storing into cycle the edge " << vertexLabels.at(posOrder.at(tspLength - 1)) << " to " << vertexLabels.at(posOrder.at(0)) << endl);

		//store cost of the cycle
		tourCost = treeCost(_numClients, _edgeWeights, cycle);



		/*check if the cost is consistent with Concorde's cost output*/
		double tourCostCheck;
		/*extract the tour cost from the log file*/
		FILE *fp;
		char buf[256];

		// Open the solution file
		if ((fp = fopen("tourLog.txt", "r")) == NULL) {
			fflush(stderr);
			fprintf(stderr, "error parse_output: file not found tourLog\n");
		}

		while (fgets(buf, 256, fp) != NULL) {

			if (sscanf(buf, "Optimal Solution: %lf", &tourCostCheck) == 1) {

			}
		}

		fclose(fp);

	}
	else if (numVisit == 2){
		//find which two are visited
		int v1, v2;
		//find first client visited
		for (int v = 0; v < _numClients; v++){
			if (_visit.at(v)){
				v1 = v;
				break;
			}
		}
		//find second client visited
		for (int v = v1 + 1; v < _numClients; v++){
			if (_visit.at(v)){
				v2 = v;
				break;
			}
		}
		//update the cost of the tour
		tourCost = 2 * _edgeWeights.at(v1).at(v2);
	}
	else{
		tourCost = 0;
	}

	//output cost of the tour
	return tourCost;
}

//create a pcst instance given the desired edge weights and penalties
void createPCST(int _depot, int _numNodes, int _numArcs, vector<vector<double> > _edgeWeights, vector<vector<bool> > _existingTree, vector<double> _penalties){
	// Remove any existing input and output files
	std::remove("testInst.stp");
	std::remove("testSoln.txt");

	// Open a temprary input file
	char filename[1000];
	sprintf(filename, "testInst.stp");
	std::ofstream fout;
	fout.open(filename, std::ofstream::out | std::ofstream::app);

	// Fill in input file
	fout << "33D32945 STP File, STP Format Version 1.0" << std::endl;
	fout << std::endl;
	fout << "SECTION Graph" << endl;
	fout << "Nodes " << _numNodes << endl;
	fout << "Arcs " << _numArcs << endl;
	for (int u = 0; u < _numNodes; u++){
		for (int v = 0; v < _numNodes; v++){
			if(v != u){
				if (!_existingTree.at(u).at(v)){
					fout << "A " << u + 1 << " " << v + 1 << " " << _edgeWeights.at(u).at(v) << endl;
				}
				else{
					//those edge already in the existing tree are free to use in the PCST
					fout << "A " << u + 1 << " " << v + 1 << " " << 0 << endl;
				}
			}
		}
	}

	fout << endl;
	fout << "SECTION Terminals" << endl;
	fout << "Terminals " << _numNodes << endl;
	for (int v = 0; v < _numNodes; v++){
		fout << "TP " << v+1 << " " << _penalties.at(v) << endl;
	}
	fout << std::endl;
	fout << "RootP " << _depot+1 << endl;
	fout << "END" << endl;
	fout << endl;
	fout << "EOF" << endl;

	// Finalize input
	fout.close();

	// Run executable and create output to "testSoln.txt"
	system("./dapcstp -f testInst.stp -o testSoln.txt 1>/dev/null 2>/dev/null");
	
}

//returns the tree edges of pcst solution to input file testInst.stp
vector<vector<bool> > pcstEdges(int _numNodes, vector<vector<bool> > _existingTree){
	//declare the edges of the pcst solution
	vector<vector<bool> > edges = init2D<bool>(_numNodes,_numNodes);
	for (int u = 0; u < _numNodes; u++){
		for (int v = 0; v < _numNodes; v++){
			if (!_existingTree.at(u).at(v)){
				edges.at(u).at(v) = false;
			}
			else{
				//add the edges in the existingTree to PCST solution for free in case the PCST did not need some of them
				edges.at(u).at(v) = true;
			}
		}
	}
	
	FILE *fp;
	char buf[256];

	int e1, e2; /* current edge endpoints */

	// Open the solution file
	if ((fp = fopen("testSoln.txt", "r")) == NULL) {
		fflush(stderr);
		fprintf(stderr, "error parse_output: file not found testSoln\n");
	}

	while (fgets(buf, 256, fp) != NULL) {
		if (sscanf(buf, "E %d %d", &e1, &e2) == 2) {
			// Add edge e1->e2 to adjacency matrix
			edges.at(e1-1).at(e2-1) = true;

		}
	}

	fclose(fp);

	return edges;
}



//modifies the tree edges and visited set based on the pcst solution in testSoln.txt
void solvePCST(int _numClients, vector<vector<bool> > _existingTree, vector<vector<bool> > &edgesOut, unordered_set<int> &verticesOut) {
	FILE *fp;
	char buf[256];
	int v;      /* The vertex */
	int e1, e2; /* The edge endpoints */

	//add edges of existingTree to PCST solution for free
	for (int u = 0; u < _numClients; u++){
		for (int v = 0; v < _numClients; v++){
			if (_existingTree.at(u).at(v)){
				edgesOut.at(u).at(v) = true;
			}
		}
	}

	// Open the solution file
	if ((fp = fopen("testSoln.txt", "r")) == NULL) {
		fflush(stderr);
		fprintf(stderr, "error parse_output: file not found testSoln\n");
	}

	while (fgets(buf, 256, fp) != NULL) {
		if (sscanf(buf, "E %d %d", &e1, &e2) == 2) {
			// Add edge e1->e2 to adjacencey matrix
			edgesOut.at(e1-1).at(e2-1) = true;

		}

		if (sscanf(buf, "V %d", &v) == 1) {
			// Add vertex v to the visit set
			verticesOut.insert(v-1);

		}
	}

	
	fclose(fp);
}


int main()
{	//N clients
	int N = 100;
	
	//T days
	int T = 6;

	//holing cost multiplier
	double H = 2.6;

	//unit holding cost min
	double min_h = 0.01;

	//instance number
	int multiplicity = 1;

	char filename[1000];
	//directory to read input
	sprintf(filename, "../EuclideanIRPinstances/inputsIterNmult100Htimes100_260/input-N-%d-T-%d-Htimes100-%d-inst-%d.txt", N, T, (int)(H * 100), multiplicity);
	std::ifstream fin(filename);

	//directory to write output
	char outputname[1000];
	sprintf(outputname, "../EuclideanIRPaddLS/outputsIterNmult100Htimes100_260/output-N-%d-T-%d-Htimes100-%d-inst-%d.txt", N, T, (int)(H * 100), multiplicity);
	std::ofstream fout;
	fout.open(outputname, std::ofstream::out | std::ofstream::app);

	//cin the depot index
	int depot;
	fin >> depot;

	//cin the size of problem. How many clients. How many days.
	int numClients, numDays;
	fin >> numClients;
	fin >> numDays;

	fout << depot << ' ';
	fout << numClients << ' ';
	fout << numDays << ' ';
	fout << min_h << ' ';
	fout << H << ' ';
	fout << "addLScutOffAt0.01progressTours6" << ' ';


	//read in demands to compute holding costs
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
			edgeWeights.at(i).at(j) = sqrt(pow(position.at(i).at(0) - position.at(j).at(0), 2) + pow(position.at(i).at(1) - position.at(j).at(1), 2));

		}
	}

	//declare initial solution to visit all demands at time 1
	//declare visits. visit^i_s==true means visit client i at time s
	vector<vector<bool> > visit = init2D<bool>(numClients, numDays);
	for (int i = 0; i < numClients; i++) {
		for (int j = 0; j < numDays; j++) {
			if (j == 0){
				visit[i][j] = true;
			}
			else
				visit[i][j] = false;
		}
	}

	//declare variables latestVisit for each demand point (i,t)
	vector<vector<int> > latestVisit = init2D<int>(numClients, numDays);	
	for (int i = 0; i < numClients; i++){
		for (int t = 0; t < numDays; t++){
			for (int s = 0; s <= t; s++){
				if (visit[i][s] == true){
					latestVisit[i][t] = s;
				}
			}
		}
	}


	//declare variable existingTree
	//existingTree[s][i][j] = true iff edge ij is used at time s
	vector<vector<vector<bool> > > existingTree = init3D<bool>(numDays, numClients, numClients);
	for (int s = 0; s < numDays; s++){
		for (int u = 0; u < numClients; u++){
			for (int v = 0; v < numClients; v++){
				existingTree[s][u][v] = false;
			}					
		}
	}

	/*
	call boost kruskal_minimum_spanning_tree to find the MST visiting all clients at time 0.
	
	alternatively, call PCST with each penalty set to (\sum_{e \in E} c_e + 1) to get MST spanning all clients.

	update existingTree[0][i][j] = true if use edge ij at time "1".
	*/

	//set large enough penalty for PCST to force it to visit all clients at time 0
	double largePenalty = 0;
	for (int u = 0; u < numClients; u++){
		for (int v = 0; v < numClients; v++){
			largePenalty += edgeWeights.at(u).at(v);
		}
	}
	
	vector<double> largePenalties = init1D<double>(numClients);
	for (int v = 0; v < numClients; v++){
		largePenalties.at(v) = largePenalty;
	}


	//number of arcs is N*(N-1)
	int numArcs = numClients*(numClients-1);

	//create pcst instance to find initial MST for time 0
	createPCST(depot, numClients, numArcs, edgeWeights, existingTree[0], largePenalties);

	//update "existingTree" using the pcst solver
	existingTree[0] = pcstEdges(numClients, existingTree[0]);

	//declare initial routing cost
	double routingCost = 0;
	for (int i = 0; i < numClients; i++){
		//count each arc
		for (int j = 0; j < numClients; j++){
			if (existingTree.at(0).at(i).at(j)){
				routingCost += edgeWeights.at(i).at(j);
			}
		}
	}

	//declare initial holding cost
	double holdingCost = 0;
	for (int v = 0; v < numClients; v++){
		for (int t = 0; t < numDays; t++){
			if (demand.at(v).at(t) > 0){
				int s = latestVisit.at(v).at(t);
				holdingCost += holding.at(v).at(s).at(t);
			}
		}
	}

	cout << "initial routing cost is " << routingCost << endl;
	cout << "initial holding cost is " << holdingCost << endl;
	cout << "initial total cost is " << routingCost + holdingCost << endl;

	for (int v = 0; v < numClients; v++){
		for (int t = 0; t < numDays; t++){
			DEBUG_MSG("latest visit of " << v << " , " << t << " is on day " << latestVisit.at(v).at(t) << endl);
		}
	}

	/*
	done initializing a feasible solution
	*/

	
	/*
	start of local search
	*/
	bool improve = true;
	double improveRatio = 1;
	int numRounds = 0;
	
	while (improve && improveRatio > 0.01){
		//update number of rounds
		numRounds++;

		//don't try augmenting visits unless find an improvement
		improve = false;

		//initialize the day achieving the best improvement
		int bestDay = 1;
		double bestImprovement = 0;

		//find best addDay
		for (int addDay = 1; addDay < numDays; addDay++){


			//declare variable penalties
			vector<double> penalties = init1D<double>(numClients);
			for (int v = 0; v < numClients; v++){
				penalties[v] = 0;
			}

			//penalties[v] = sum_{t \in [t',t_hat]} h_v * d^v_t * (t' - s(v,t))
			for (int v = 0; v < numClients; v++){
				//only get reward for those who were not already visited on addDay
				if ((demand.at(v).at(addDay) > 0) && (!visit.at(v).at(addDay))){
					int endDay = t_hat(v, addDay, numDays, visit);
					for (int s = addDay; s < endDay + 1; s++){
						penalties[v] += unitHolding.at(v) * demand.at(v).at(s) * (addDay - latestVisit.at(v).at(s));
					}
				}
			}


			//create input file for pcst instance
			createPCST(depot, numClients, numArcs, edgeWeights, existingTree.at(addDay), penalties);

			//run pcst solver on pcst instance and create output file containing pcstSet, pcstTree
			vector<vector<bool> > pcstTree = init2D<bool>(numClients, numClients);
			for (int i = 0; i < numClients; i++){
				for (int j = 0; j < numClients; j++){
					pcstTree.at(i).at(j) = false;
				}
			}

			unordered_set<int> pcstSet;
			solvePCST(numClients, existingTree.at(addDay), pcstTree, pcstSet);

			//compute pcstTreeCost and pcstPenaltyCost from pcstTree and pcstSet
			double existingTreeCost = treeCost(numClients, edgeWeights, existingTree.at(addDay));
			//the pcstTreeCost should exclude any edges in existingTree
			double pcstTreeCost = treeCost(numClients, edgeWeights, pcstTree) - existingTreeCost;
			double pcstRewardCost = rewardCost(numClients, penalties, pcstSet);
			double difference = costChange(pcstTreeCost, pcstRewardCost);

			//update best day and best improvement
			if (difference < bestImprovement){
				bestImprovement = difference;
				bestDay = addDay;
			}
		}
	

			//if cost improves, update new IRP solution
			if (bestImprovement < -1e-10){


				//update improve
				improve = true;

				//upate improveRatio
				improveRatio = -bestImprovement / (routingCost + holdingCost);

				/*redo add phase on bestDay*/

				//declare variable penalties
				vector<double> penalties = init1D<double>(numClients);
				for (int v = 0; v < numClients; v++){
					penalties[v] = 0;
				}

				//penalties[v] = sum_{t \in [t',t_hat]} h_v * d^v_t * (t' - s(v,t))
				for (int v = 0; v < numClients; v++){
					//only get reward for those who were not already visited on addDay
					if ((demand.at(v).at(bestDay) > 0) && (!visit.at(v).at(bestDay))){
						int endDay = t_hat(v, bestDay, numDays, visit);
						for (int s = bestDay; s < endDay + 1; s++){
							penalties[v] += unitHolding.at(v) * demand.at(v).at(s) * (bestDay - latestVisit.at(v).at(s));
						}
					}
				}


				//create input file for pcst instance
				createPCST(depot, numClients, numArcs, edgeWeights, existingTree.at(bestDay), penalties);

				//run pcst solver on pcst instance and create output file containing pcstSet, pcstTree
				vector<vector<bool> > pcstTree = init2D<bool>(numClients, numClients);
				for (int i = 0; i < numClients; i++){
					for (int j = 0; j < numClients; j++){
						pcstTree.at(i).at(j) = false;
					}
				}

				unordered_set<int> pcstSet;
				solvePCST(numClients, existingTree.at(bestDay), pcstTree, pcstSet);

				//compute pcstTreeCost and pcstPenaltyCost from pcstTree and pcstSet
				double existingTreeCost = treeCost(numClients, edgeWeights, existingTree.at(bestDay));
				double pcstTreeCost = treeCost(numClients, edgeWeights, pcstTree) - existingTreeCost;
				double pcstRewardCost = rewardCost(numClients, penalties, pcstSet);
				double difference = costChange(pcstTreeCost, pcstRewardCost);


				/*end of add phase*/

				//update visit
				for (int v : pcstSet){
					visit.at(v).at(bestDay) = true;
	
				}

				//update latestVisit
				for (int i = 0; i < numClients; i++){
					for (int t = 0; t < numDays; t++){
						for (int s = 0; s <= t; s++){
							if (visit[i][s] == true){
								latestVisit[i][t] = s;

							}
						}
					}
				}

				//update existingTree
				existingTree.at(bestDay) = pcstTree;


				//update routing cost and holding cost
				updateCosts(pcstTreeCost, pcstRewardCost, routingCost, holdingCost);
				DEBUG_MSG("new routing cost is " << routingCost << endl);
				DEBUG_MSG("new holding cost is " << holdingCost << endl);

				/*
				start of sanity check
				*/

				//compute and output routing cost
				double routingCostCheck = 0;
				for (int s = 0; s < numDays; s++){
					for (int u = 0; u < numClients; u++){
						//count each arc per direction
						for (int v = 0; v < numClients; v++){
							if (existingTree.at(s).at(u).at(v)){
								routingCostCheck += edgeWeights.at(u).at(v);
							}
						}
					}
				}
				DEBUG_MSG("the new routing cost should be " << routingCostCheck << endl);

				//compute and output holding cost
				double holdingCostCheck = 0;
				for (int i = 0; i < numClients; i++){
					for (int t = 0; t < numDays; t++){
						holdingCostCheck += holding[i][latestVisit[i][t]][t];
					}
				}
				DEBUG_MSG("the new holding cost shoud be " << holdingCostCheck << endl);

				/*
				end of sanity check
				*/


			} 
			else {

			}
		
	}
	//end of while loop

	/*
	end of local search
	*/

	//compute total cost
	double totalCost = routingCost + holdingCost;

	/*compute total tour cost*/
	double tourRoutingCost = 0;
	//create visitReversed reverse the indices of visit
	vector<vector<bool> > visitReversed = init2D<bool>(numDays,numClients);
	for (int v = 0; v < numClients; v++){
		for (int s = 0; s < numDays; s++){
			visitReversed.at(s).at(v) = visit.at(v).at(s);
		}
	}

	//keep track of costs
	double tourToday;
	double treeToday;
	for (int today = 0; today < numDays; today++){
		//find tour cost of visit set on day today
		tourToday = tourCostOfVisit(numClients, position, visitReversed.at(today),edgeWeights);
		DEBUG_MSG("the tour cost on day " << today << " is " << tourToday << endl);
		tourRoutingCost += tourToday;

		//find tree cost of visit set on day 0
		treeToday = treeCost(numClients, edgeWeights, existingTree.at(today));
		DEBUG_MSG("the tree cost on day " << today << " is " << treeToday << endl);
	}
	/*end of finding total tour cost*/

	//total cost of tours-solution
	double totalCostToursSoln = tourRoutingCost + holdingCost;


	for (int v = 0; v < numClients; v++){
		for (int t = 0; t < numDays; t++){
			DEBUG_MSG("latest visit of " << v << " , " << t << " is on day " << latestVisit.at(v).at(t) << endl);
		}
	}

	

	/*
	* Stop the timer. We should do this before printing the final answer.
	*/
	auto elapsed = std::chrono::high_resolution_clock::now() - start_timer;
	long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
	cout << "Number of rounds = " << numRounds << endl;
	fout << numRounds << ' ';
	cout << "Program execution time = " << microseconds << " microseconds\n";
	fout << microseconds << ' ';

	cout << "The tree-total cost is " << totalCost << "\n";
	fout << totalCost << ' ';
	cout << "The tour-total cost is " << totalCostToursSoln << endl;
	fout << totalCostToursSoln << endl;
	fout.close();

	return 0;
}
