/*
Primal dual heuristic for IRP on metrics

Self contained implementation without dual values and without loop over s from tau to T

Note all indices in the paper are shifted by -1 in the vectors below e.g. index of 1 becomes index of 0 in code

Note indices for the pcst solver need to start from 1, not 0

N, T stay constant

numClients changes after instance conversion, numDays stays constant
*/

#include<iostream>
#include <numeric> //iota
#include<cmath>
#include<vector>
#include<fstream>
#include<assert.h>
#include<ratio>
#include<algorithm> //sort, etc.
#include<chrono>
#include<unordered_set>
#include <stdexcept>
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

//find cost of tree given the adjacency matrix of the subgraph
double subgraphCost(int _numClients, vector<vector<double> > _edgeWeights, vector<vector<bool> > _subgraph){
	double cost = 0;
	for (int i = 0; i < _numClients; i++){
		//go through all arcs of PCST solution
		for (int j = 0; j < _numClients; j++){
			if (_subgraph.at(i).at(j)){
				cost += _edgeWeights.at(i).at(j);
			}
		}
	}
	return cost;
}

//find t_hatDelete where t_hatDelete is the t >= t' with no visits to v occuring within (t',t]
int t_hatDelete(int _client, int _day, int _numDays, vector<vector<bool> > _visit){

	int targetDay = _day;
	//go through t >= _day+1 stopping just before the first time that _client is visited after _day
	for (int t = _day + 1; t < _numDays; t++){
		//stop if _client is visited at time t
		if (_visit.at(_client).at(t)){
			//cout << "visit at " << _client << " time " << t << " is " << _visit.at(_client).at(t) << endl;
			break;
		}
		else{

			targetDay = t;
		}
	}
	return targetDay;
}

//find t_hatAdd where t_hatAdd is the t >= t' with no visits to v occuring within [t',t]
int t_hatAdd(int _client, int _day, int _numDays, vector<vector<bool> > _visit){

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

//find the penalty cost of deleting everyone on a given day
double penaltyCostDelete(int _numClients, vector<double> _penalties, int _deleteDay, vector<vector<bool> > _visit){
	double cost = 0;
	for (int v = 0; v < _numClients; v++){
		//add the penalty of clients who are being deleted
		if (_visit.at(v).at(_deleteDay)){
			cost += _penalties.at(v);
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

//for delete: find the change in cost given the cost of the existing tree and the penalty cost of deleting the existing tree
double costChangeDelete(double _existingTreeCost, double _penaltyCost){
	double costDiff = -_existingTreeCost + _penaltyCost;
	return costDiff;
}

//for add: find the change in cost given the cost of the pcst tree from the PCST instance, and the cost of the pcst penalties
double costChange(double _pcstTreeCost, double _pcstRewardCost){
	double costDiff = _pcstTreeCost - _pcstRewardCost;
	return costDiff;
}

//for delete: update routing cost and holding cost
void updateCostsDelete(double _existingTreeCost, double _penaltyCost, double &routingCost, double &holdingCost){
	routingCost += -_existingTreeCost;
	holdingCost += +_penaltyCost;
}

//for add: update routing cost and holding cost
void updateCosts(double _pcstTreeCost, double _pcstRewardCost, double &routingCost, double &holdingCost){
	routingCost += _pcstTreeCost;
	holdingCost += -_pcstRewardCost;
}

//for PD phase: create a pcst instance given the desired edge weights and penalties
void createPCSTpd(int _depot, int _numNodes, int _numArcs, vector<vector<bool> > _instEdges, vector<vector<double> > _edgeWeights, vector<vector<bool> > _existingTree, vector<double> _penalties){
	// Remove any existing input and output files
	std::remove("singleTestInstPD.stp");
	std::remove("singleTestSolnPD.txt");

	// Open a temprary input file
	char filename[1000];
	sprintf(filename, "singleTestInstPD.stp");
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
			//no self loops
			if (v != u){
				//only create arcs that are in the instance
				if (_instEdges.at(u).at(v)){
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
	}

	fout << endl;
	fout << "SECTION Terminals" << endl;
	fout << "Terminals " << _numNodes << endl;
	for (int v = 0; v < _numNodes; v++){
		fout << "TP " << v + 1 << " " << _penalties.at(v) << endl;
	}
	fout << std::endl;
	fout << "RootP " << _depot + 1 << endl;
	fout << "END" << endl;
	fout << endl;
	fout << "EOF" << endl;

	// Finalize input
	fout.close();

	// Run executable and create output to "singleTestSolnPD.txt"
	system("./dapcstp -f singleTestInstPD.stp -o singleTestSolnPD.txt 1> singleError1.txt 2> singleError2.txt");

}

//for add: create a pcst instance given the desired edge weights, existing tree, and penalties
void createPCST(int _depot, int _numNodes, int _numArcs, vector<vector<double> > _edgeWeights, vector<vector<bool> > _existingTree, vector<double> _penalties){
	// Remove any existing input and output files
	std::remove("singleAllOpsAddPhaseInst.stp");
	std::remove("singleAllOpsAddPhaseSoln.txt");

	// Open a temprary input file
	char filename[1000];
	sprintf(filename, "singleAllOpsAddPhaseInst.stp");
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
			if (v != u){
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
		fout << "TP " << v + 1 << " " << _penalties.at(v) << endl;
	}
	fout << std::endl;
	fout << "RootP " << _depot + 1 << endl;
	fout << "END" << endl;
	fout << endl;
	fout << "EOF" << endl;

	// Finalize input
	fout.close();

	// Run executable and create output to "singleAllOpsAddPhaseSoln.txt"
	system("./dapcstp -f singleAllOpsAddPhaseInst.stp -o singleAllOpsAddPhaseSoln.txt 1>/dev/null 2>/dev/null");

}

//returns the tree edges of pcst solution to input file singleTestInstPD.stp
vector<vector<bool> > pcstEdgesPd(int _numNodes, vector<vector<bool> > _existingTree){
	//declare the edges of the pcst solution
	vector<vector<bool> > edges = init2D<bool>(_numNodes, _numNodes);
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
	if ((fp = fopen("singleTestSolnPD.txt", "r")) == NULL) {
		fflush(stderr);
		fprintf(stderr, "error parse_output: file not found singleTestSolnPD\n");
	}

	while (fgets(buf, 256, fp) != NULL) {
		if (sscanf(buf, "E %d %d", &e1, &e2) == 2) {
			// Add edge e1->e2 to adjacency matrix
			edges.at(e1 - 1).at(e2 - 1) = true;

		}
	}

	fclose(fp);

	return edges;
}

//for add: returns the tree edges of pcst solution to input file addPhaseInst.txt
vector<vector<bool> > pcstEdges(int _numNodes, vector<vector<bool> > _existingTree){
	//declare the edges of the pcst solution
	vector<vector<bool> > edges = init2D<bool>(_numNodes, _numNodes);
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
	if ((fp = fopen("singleAllOpsAddPhaseSoln.txt", "r")) == NULL) {
		fflush(stderr);
		fprintf(stderr, "error parse_output: file not found singleAllOpsAddPhaseSoln\n");
	}

	while (fgets(buf, 256, fp) != NULL) {
		if (sscanf(buf, "E %d %d", &e1, &e2) == 2) {
			// Add edge e1->e2 to adjacency matrix
			edges.at(e1 - 1).at(e2 - 1) = true;

		}
	}

	fclose(fp);

	return edges;
}

//modifies the tree edges and visited set based on the pcst solution in singleTestSolnPD.txt
void solvePCSTpd(int _numClients, vector<vector<bool> > _existingTree, vector<vector<bool> > &edgesOut, unordered_set<int> &verticesOut) {
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
	if ((fp = fopen("singleTestSolnPD.txt", "r")) == NULL) {
		fflush(stderr);
		fprintf(stderr, "error parse_output: file not found singleTestSolnPD\n");
		exit(-1);  // Exit the program with an error
	}

	while (fgets(buf, 256, fp) != NULL) {
		if (sscanf(buf, "E %d %d", &e1, &e2) == 2) {
			// Add edge e1->e2 to adjacencey matrix
			edgesOut.at(e1 - 1).at(e2 - 1) = true;

		}

		if (sscanf(buf, "V %d", &v) == 1) {
			// Add vertex v to the visit set
			verticesOut.insert(v - 1);

		}
	}


	fclose(fp);
}

//for add: modifies the tree edges and visited set based on the pcst solution in addPhaseSoln.txt
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
	if ((fp = fopen("singleAllOpsAddPhaseSoln.txt", "r")) == NULL) {
		fflush(stderr);
		fprintf(stderr, "error parse_output: file not found singleAllOpsAddPhaseSoln\n");
	}

	while (fgets(buf, 256, fp) != NULL) {
		if (sscanf(buf, "E %d %d", &e1, &e2) == 2) {
			// Add edge e1->e2 to adjacencey matrix
			edgesOut.at(e1 - 1).at(e2 - 1) = true;

		}

		if (sscanf(buf, "V %d", &v) == 1) {
			// Add vertex v to the visit set
			verticesOut.insert(v - 1);

		}
	}


	fclose(fp);
}

//create and solve a TSP instance given the total number of clients, all the Euclidean positions, and which clients to cover
double tourCostOfVisit(int _numClients, vector<vector<int> > _position, vector<bool> _visit, vector<vector<double> > _edgeWeights){
	// Remove any existing input and output files
	std::remove("singleTourInstPD.tsp");
	std::remove("singleTourLogPD.txt");

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
		sprintf(filename, "singleTourInstPD.tsp");
		std::ofstream fout;
		fout.open(filename, std::ofstream::out | std::ofstream::app);

		fout << "NAME : singleTourInst" << endl;
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
		system("concorde singleTourInstPD.tsp > singleTourLogPD.txt");

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
		sprintf(solFilename, "singleTourInstPD.sol");
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

		}
		cycle.at(vertexLabels.at(posOrder.at(tspLength - 1))).at(vertexLabels.at(posOrder.at(0))) = true;


		//store cost of the cycle
		tourCost = subgraphCost(_numClients, _edgeWeights, cycle);



		/*check if the cost is consistent with Concorde's cost output*/
		double tourCostCheck;
		/*extract the tour cost from the log file*/
		FILE *fp;
		char buf[256];

		// Open the solution file
		if ((fp = fopen("singleTourLogPD.txt", "r")) == NULL) {
			fflush(stderr);
			fprintf(stderr, "error parse_output: file not found singleTourLog\n");
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

//convert original instance to one such that each client has at most one day with positive demand
void instanceConversion(int &numClientsOut, int _numDays, vector<double> _unitHolding, vector<vector<double> > _demands, vector<vector<double> > _edgeWeights, vector<double> &unitHoldingOut, vector<vector<double> > &demandsOut, vector<vector<double> > &edgeWeightsOut, vector<vector<bool> > &instEdgesOut){
	//keep track of initial N
	int oldNumClients = numClientsOut;

	//newN will change to NT, T will stay T
	numClientsOut = oldNumClients * _numDays;

	//assuming unitHoldingOut starts out empty, update unitHoldingOut for new instance
	for (int v = 0; v < oldNumClients; v++){
		for (int t = 0; t < _numDays; t++){
			unitHoldingOut.push_back(_unitHolding.at(v));
		}		
	}

	//check unitHoldingOut
	for (int v = 0; v < unitHoldingOut.size(); v++){

	}

	//assuming that demandsOut was declared to size NT by T, update demandsOut according to (v,t) gets transformed to (v*T+t,t)
	//initialize demandsOut entries to 0s
	for (int v = 0; v < numClientsOut; v++){
		for (int t = 0; t < _numDays; t++){
			demandsOut.at(v).at(t) = 0;
		}
	}
	//update the relevant demands
	for (int v = 0; v < oldNumClients; v++){
		for (int t = 0; t < _numDays; t++){
			demandsOut.at(v * _numDays + t).at(t) = _demands.at(v).at(t);
		}
	}

	//check demandsOut
	for (int v = 0; v < numClientsOut; v++){
		for (int t = 0; t < _numDays; t++){

		}
	}

	//assuming that instEdgesOut was delared to size NT by NT, update instEdgesOut according to v gets transformed to vT, and new vertices are created along a line metric of 0 cost per old vertex
	//initialize instEdgesOut to all 0s
	for (int u = 0; u < numClientsOut; u++){
		for (int v = 0; v < numClientsOut; v++){
			if (u != v){
				instEdgesOut.at(u).at(v) = false;
			}
		}
	}
	//old edges to new edges
	for (int u = 0; u < oldNumClients; u++){
		for (int v = 0; v < oldNumClients; v++){
			if (u != v){
				instEdgesOut.at(u * _numDays).at(v * _numDays) = true;
				//assuming that edgeWeightsOut was declared to size NT by NT, update edgeWeightsOut according to the same transformation as for instEdgesOut
				edgeWeightsOut.at(u * _numDays).at(v * _numDays) = _edgeWeights.at(u).at(v);
			}
		}
	}

	//the new forward edges of cost 0 for each copy of old vertex
	for (int v = 0; v < oldNumClients; v++){
		for (int t = 0; t < _numDays - 1; t++){
			instEdgesOut.at(v * _numDays + t).at(v * _numDays + (t + 1)) = true;
			//assuming that edgeWeightsOut was declared to size NT by NT, update edgeWeightsOut according to the same transformation as for instEdgesOut
			edgeWeightsOut.at(v * _numDays + t).at(v * _numDays + (t + 1)) = 0;
		}
	}

	//the new backward edges of cost 0 for each copy of old vertex
	for (int v = 0; v < oldNumClients; v++){
		for (int t = _numDays - 1; t > 0; t--){
			instEdgesOut.at(v * _numDays + t).at(v * _numDays + (t - 1)) = true;
			//assuming that edgeWeightsOut was declared to size NT by NT, update edgeWeightsOut according to the same transformation as for instEdgesOut
			edgeWeightsOut.at(v * _numDays + t).at(v * _numDays + (t - 1)) = 0;
		}
	}

	//check instEdgesOut and edgeWeightsOut
	for (int u = 0; u < numClientsOut; u++){
		for (int v = 0; v < numClientsOut; v++){

		}
	}
}

//given a subgraph on the converted instance, return the corresponding subgraph on the original instance
vector<vector<bool> > subgraphConversionBack(int _N, int _T, vector<vector<bool> > _subgraph){
	
	vector<vector<bool> > subgraphOut = init2D<bool>(_N, _N);
	for (int u = 0; u < _N; u++){
		for (int v = 0; v < _N; v++){
			if (u != v){
				//place corresponding arc for any edge occuring in the input subgraph
				if (_subgraph.at(u*_T).at(v*_T)){
					subgraphOut.at(u).at(v) = true;
				}
				//all other arcs are not in this subgraph
				else{
					subgraphOut.at(u).at(v) = false;
				}
			}
			//no self loops
			else{
				subgraphOut.at(u).at(v) = false;
			}
		}
	}

	return subgraphOut;
}

//given the visits on the converted instance, return the corresponding visits on the original instance
vector<vector<bool> > visitConversionBack(int _N, int _T, vector<vector<bool> > _visit){
	//initialize the visits to return
	vector<vector<bool> > visitOut = init2D<bool>(_N, _T);
	for (int v = 0; v < _N; v++){
		for (int s = 0; s < _T; s++){
			visitOut.at(v).at(s) = false;			
		}
	}
	
	//update according to the input visits
	for (int v = 0; v < _N; v++){
		for (int s = 0; s < _T; s++){
			//v should get visited if any copy of it was visited
			for (int r = 0; r < _T; r++){
				if (_visit.at(v*_T + r).at(s)){

					visitOut.at(v).at(s) = true;

				}
			}			
		}
	}

	return visitOut;
}

int main()
{	//N clients, REMEMBER TO UPDATE N WHEN TESTING NEW INSTANCES
	int N = 100;
	
	//T days, REMEMBER TO UPDATE T WHEN TESTING NEW INSTANCES
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

	sprintf(outputname, "../EuclideanIRPpd/outputsIterNmult100Htimes100_260/output-N-%d-T-%d-Htimes100-%d-inst-%d.txt", N, T, (int)(H * 100), multiplicity);
	std::ofstream fout;
	fout.open(outputname, std::ofstream::out | std::ofstream::app);

	//read in the depots
	int depot;
	fin >> depot;

	//read in the size of problem
	int numClients, numDays;
	fin >> numClients;
	fin >> numDays;
	//check if a boolean statement is true or false
	assert(numDays > 0);
	assert(numClients > 0);

	fout << depot << ' ';
	fout << numClients << ' ';
	fout << numDays << ' ';
	fout << min_h << ' ';
	fout << H << ' ';
	fout << "primalDualPrimalOnly" << ' ';

	//declare demands
	//need to read in demands to compute holding costs
	//d^i_t goes to demand[i-1][t-1]
	vector<vector<double> > originalDemand = init2D<double>(numClients, numDays);
	for (int i = 0; i < numClients; i++) {
		for (int j = 0; j < numDays; j++) {
			fin >> originalDemand[i][j];


		}
	}

	//read in the per unit holding costs
	vector<double> originalUnitHolding = init1D<double>(numClients);
	for (int i = 0; i < numClients; i++){
		fin >> originalUnitHolding[i];

	}

	//read in the Euclidean positions of the clients
	vector<vector<int> > position = init2D<int>(numClients, 2);
	for (int i = 0; i < numClients; i++){
		for (int axis = 0; axis < 2; axis++){
			fin >> position[i][axis];

		}
	}

	DEBUG_MSG("done reading input" << endl);

	/*
	* At this point, we are done reading input so we can start the timer.
	*/
	auto start_timer = std::chrono::high_resolution_clock::now();

	//compute initial originalHolding costs
	//H^i_{s,t}=(t-s)*d^i_t*h_i goes to originalHolding[i-1][s-1][t-1]
	vector<vector<vector<double> > > originalHolding = init3D<double>(numClients, numDays, numDays);
	for (int i = 0; i < numClients; i++) {
		for (int t = 0; t < numDays; t++) {
			for (int s = 0; s <= t; s++) {
				originalHolding[i][s][t] = (t - s) * originalDemand[i][t] * originalUnitHolding[i];
			}
		}
	}

	//compute initial edge weights
	//w_{ij} = sqrt((X_i - X_j)^2+(Y_i - Y_j)^2)
	vector<vector<double> > originalEdgeWeights = init2D<double>(numClients, numClients);
	for (int i = 0; i < numClients; i++){
		for (int j = 0; j < numClients; j++){
			originalEdgeWeights.at(i).at(j) = sqrt(pow(position.at(i).at(0) - position.at(j).at(0), 2) + pow(position.at(i).at(1) - position.at(j).at(1), 2));

		}
	}

	/*convert initial instance to extended instance such with 0 cost edges such that each client has at most one demand day*/
	
	//declare empty unitHolding
	vector<double> unitHolding;
	
	//declare size NT by T demand
	vector<vector<double> > demand = init2D<double>(numClients * numDays, numDays);
	
	//declare size NT by NT instEdges
	vector<vector<bool> > instEdges = init2D<bool>(numClients * numDays, numClients * numDays);
	
	//declare size NT by NT edgeWeights
	vector<vector<double> > edgeWeights = init2D<double>(numClients * numDays, numClients * numDays);
	
	//convert instance to one whose clients have at most one demand day each
	instanceConversion(numClients, numDays, originalUnitHolding, originalDemand, originalEdgeWeights, unitHolding, demand, edgeWeights, instEdges);

	//new instance will have new depot label of depot*T
	int convertedDepot = depot * T;

	//new instance will have N(N-1) + 2N(T-1) many arcs when creating PCST instance
	int numArcs = N * (N - 1) + 2 * N * (T - 1);

	//compute holding costs of converted instance
	//H^i_{s,t}=(t-s)*d^i_t*h_i goes to holding[i-1][s-1][t-1]
	vector<vector<vector<double> > > holding = init3D<double>(numClients, numDays, numDays);
	for (int i = 0; i < numClients; i++) {
		for (int t = 0; t < numDays; t++) {
			for (int s = 0; s <= t; s++) {
				holding[i][s][t] = (t - s) * demand[i][t] * unitHolding[i];
			}
		}
	}

	/*done instance conversion*/


	/*initializations*/

	//for each v, keep track of t(v)
	vector<int> demandDay = init1D<int>(numClients);
	for (int v = 0; v < numClients; v++){
		for (int t = 0; t < numDays; t++){
			if (demand.at(v).at(t) > 0){
				demandDay.at(v) = t;
				//there's only one demand day per client, no need to look further once one is found
				break;
			}
		}
	}

	//declare initial visit, only visit convertedDepot on all days
	vector<vector<bool> > visit = init2D<bool>(numClients, numDays);
	for (int v = 0; v < numClients; v++){
		for (int s = 0; s < numDays; s++){
			if ((v >= convertedDepot) && (v < convertedDepot + T)){
				visit.at(v).at(s) = true;
			}
			else{
				visit.at(v).at(s) = false;
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

	//declare variables latestVisit for each demand point (i,t)
	vector<vector<int> > latestVisit = init2D<int>(numClients, numDays);
	for (int i = 0; i < numClients; i++){
		for (int t = 0; t < numDays; t++){
			if (demand.at(i).at(t) > 0){
				for (int s = 0; s <= t; s++){
					if (visit[i][s] == true){
						latestVisit[i][t] = s;
					}
				}
			}
		}
	}

	//declare routing cost and holding cost
	double routingCost = 0;
	double holdingCost = 0;

	//initialize empty frozen set s.t. isFrozen[v][t] indicates whether (v,t) is frozen
	vector<vector<bool> > isFrozen = init2D<bool>(numClients, numDays);
	for (int v = 0; v < numClients; v++){
		for (int t = 0; t < numDays; t++){
			if (demand.at(v).at(t) > 0){
				isFrozen.at(v).at(t) = false;
			}
			//we don't ever raise the duals of nondemand points
			else{
				isFrozen.at(v).at(t) = true;
			}
		}
	}

	//define the active set A(s) of day s to {v: v in D, t(v) >= s} s.t. isActive[s][v][t] indicates whether (v,t) is in the active set of day s
	vector<vector<vector<bool> > > isActive = init3D<bool>(numDays, numClients, numDays);
	for (int s = 0; s < numDays; s++){
		for (int v = 0; v < numClients; v++){
			for (int t = 0; t < numDays; t++){
				if (demand.at(v).at(t) > 0){
					if (t >= s){
						isActive.at(s).at(v).at(t) = true;
					}
					else{
						isActive.at(s).at(v).at(t) = false;
					}
				}
				//don't consider nondemand points
				else{
					isActive.at(s).at(v).at(t) = false;
				}
			}
		}
	}

	//declare the left and right endpoint of clients, representing the interval of days through which the budget of each client will be assigned
	vector<int> leftDay = init1D<int>(numClients);
	
	/*done initializations*/


	/*start primal dual phase*/
	//for tau from second last day to the second day
	for (int tau = numDays - 2; tau > 0; tau--){
		//for s from ceil(tau) to last day

		//s should start at ceiling of tau if tau is fractional, otherwise tau + 1 if tau is integral

			//set penalties pi(v) = H^v_{tau,s} for unfrozen v, and pi(v) = 0 for frozen v for the PCST problem
			//set the penalties of those not in A(s) to 0. Note they might still be spanned by the PCST as Steiner nodes.
			vector<double> penalties = init1D<double>(numClients);
			for (int v = 0; v < numClients; v++){
				if (isActive.at(tau).at(v).at(demandDay.at(v))){
					if (!isFrozen.at(v).at(demandDay.at(v))){
						penalties.at(v) = holding.at(v).at(tau).at(demandDay.at(v));

					}
					else{//frozen clients should not be incentivized to be covered
						penalties.at(v) = 0;
					}
				}
				else{//inactive clients should not be incentivized to be covered
					penalties.at(v) = 0;
				}
			}
			//create PCST instance on the active set on day s
			createPCSTpd(convertedDepot, numClients, numArcs, instEdges, edgeWeights, existingTree.at(tau), penalties);
			/*solve PCST instance. If PCSTset has any unfrozen client v, set l(v) = tau, r(v) = s, and b^v_t = H^v_{l(v),r(v)}; visit v on day ceil(l(v)); freeze everything that was visited*/
			vector<vector<bool> > PCSTedges = init2D<bool>(numClients, numClients);
			for (int u = 0; u < numClients; u++){
				for (int v = 0; v < numClients; v++){
					PCSTedges.at(u).at(v) = false;
				}
			}
			unordered_set<int> PCSTset;
			solvePCSTpd(numClients, existingTree.at(tau), PCSTedges, PCSTset);
			//update relevant variables for unfrozen clients in PCSTset
			for (int v : PCSTset){
				//only serve those demand points in the active set of day s
				if (isActive.at(tau).at(v).at(demandDay.at(v))){
					if (!isFrozen.at(v).at(demandDay.at(v))){
						//update intervals
						leftDay.at(v) = tau;
						//update visit
						visit.at(v).at(leftDay.at(v)) = true;

						//freeze the demand point
						isFrozen.at(v).at(demandDay.at(v)) = true;
					}
				}
			}			

	}
	//Upon reaching tau = 1, set the budget of any unfrozen vertex to b^v_t = H^v_{1,t(v)} and visit it on the first day
	for (int v = 0; v < numClients; v++){
		if (!isFrozen.at(v).at(demandDay.at(v))){
			//update visit
			visit.at(v).at(0) = true;

		}
	}

	/*done primal dual phase*/

	/*compute the Steiner tree on the visit set per day*/
	//set large enough penalty for PCST to force it to visit the clients in the visit set
	double largePenalty = 0;
	for (int u = 0; u < N; u++){
		for (int v = 0; v < N; v++){
			largePenalty += originalEdgeWeights.at(u).at(v);
		}
	}
	//find the Steiner tree per day
	for (int s = 0; s < numDays; s++){
		//set penalties based on whether v is visted on day s
		vector<double> inducedPenalties = init1D<double>(numClients);
		for (int v = 0; v < numClients; v++){
			if (visit.at(v).at(s)){
				inducedPenalties.at(v) = largePenalty;
			}
			else{
				inducedPenalties.at(v) = 0;
			}
		}
		//find PCST on day s
		createPCSTpd(convertedDepot, numClients, numArcs, instEdges, edgeWeights, existingTree.at(s), inducedPenalties);
		vector<vector<bool> > tempEdges = init2D<bool>(numClients, numClients);
		for (int u = 0; u < numClients; u++){
			for (int v = 0; v < numClients; v++){
				tempEdges.at(u).at(v) = false;
			}
		}
		unordered_set<int> tempSet;
		solvePCSTpd(numClients, existingTree.at(s), tempEdges, tempSet);
		//store the tree found for day s into existingTree[s]
		existingTree.at(s) = tempEdges;
	}

	//update routing cost
	for (int s = 0; s < numDays; s++){
		routingCost += subgraphCost(numClients, edgeWeights, existingTree.at(s));
	}


	//update latestVisit
	//CORRECTION: For a demand point (vT+t,t) and s <= t, any of the points {(vT+r,s)}_{r=0}^{T-1} can be used to serve it
	//original label v
	for (int v = 0; v < N; v++){
		//original label t
		for (int t = 0; t < T; t++){
			//new label at (v*T+t,t)
			if (demand.at(v * T + t).at(t) > 0){
				for (int s = 0; s <= t; s++){
					for (int r = 0; r < T; r++){
						if (visit.at(v*T + r).at(s)){
							latestVisit.at(v*T + t).at(t) = s;
						}
					}
				}
			}
		}
	}


	//update holding cost
	for (int v = 0; v < numClients; v++){
		int t = demandDay.at(v);
		holdingCost += holding.at(v).at(latestVisit.at(v).at(t)).at(t);
	}

	double totalCost = routingCost + holdingCost;
	

	//write the time the primal dual phase took
	auto elapsedPD = std::chrono::high_resolution_clock::now() - start_timer;
	long long microsecondsPD = std::chrono::duration_cast<std::chrono::microseconds>(elapsedPD).count();
	fout << microsecondsPD << ' ';
	cout << "primal dual phase runtime is " << microsecondsPD << endl;

	//write the cost of the primal dual phase without pruning
	fout << totalCost << ' ';
	DEBUG_MSG("primal dual phase routing cost before conversion is " << routingCost << endl);
	DEBUG_MSG("primal dual phase holding cost before conversion is " << holdingCost << endl);
	cout << "primal dual phase trees solution cost is " << totalCost << endl;

	/*transform solution on the converted instance back to solution on the original instance */	
	//transform existingTree
	vector<vector<vector<bool> > > existingTreeOriginal = init3D<bool>(numDays, N, N);
	for (int s = 0; s < numDays; s++){
		existingTreeOriginal.at(s) = subgraphConversionBack(N, numDays, existingTree.at(s));
	}

	//transform visit
	vector<vector<bool> > visitOriginal = init2D<bool>(N, numDays);
	visitOriginal = visitConversionBack(N, numDays, visit);

	//compute latestVisitOriginal from visitOriginal
	vector<vector<int> > latestVisitOriginal = init2D<int>(N, numDays);
	for (int i = 0; i < N; i++){
		for (int t = 0; t < numDays; t++){
			if (originalDemand.at(i).at(t) > 0){

				for (int s = 0; s <= t; s++){
					if (visitOriginal.at(i).at(s) == true){						
						latestVisitOriginal.at(i).at(t) = s;
					}
				}
			}
		}
	}

	/*sanity check on costs*/
	double rCheck = 0;
	for (int s = 0; s < numDays; s++){
		rCheck += subgraphCost(N, originalEdgeWeights, existingTreeOriginal.at(s));
	}
	DEBUG_MSG("routing cost on original instance is " << rCheck << endl);


	double hCheck = 0;
	for (int v = 0; v < N; v++){
		for (int t = 0; t < numDays; t++){
			hCheck += originalHolding.at(v).at(latestVisitOriginal.at(v).at(t)).at(t);
		}
	}
	DEBUG_MSG("holding cost on the original instance is " << hCheck << endl);


	/*end of sanity check*/

	int numArcsOriginal = N*(N - 1);

	/*done transforming solution back to a solution on original graph*/

	/*convert trees-solution to tours-solution*/
	DEBUG_MSG("start of transform trees to tours in PD phase" << endl);

	double tourRoutingCostPD = 0;
	//create visitReversed reverse the indices of visit
	vector<vector<bool> > visitReversedPD = init2D<bool>(numDays, N);
	for (int v = 0; v < N; v++){
		for (int s = 0; s < numDays; s++){
			visitReversedPD.at(s).at(v) = visitOriginal.at(v).at(s);
		}
	}

	//keep track of costs
	double tourTodayPD;
	double treeTodayPD;
	for (int today = 0; today < numDays; today++){
		//find tour cost of visit set on day today
		tourTodayPD = tourCostOfVisit(N, position, visitReversedPD.at(today), originalEdgeWeights);

		tourRoutingCostPD += tourTodayPD;

		//find tree cost of visit set on day 0
		treeTodayPD = subgraphCost(N, originalEdgeWeights, existingTreeOriginal.at(today));

	}



	//total cost of tours-solution
	double totalCostToursSolnPD = tourRoutingCostPD + holdingCost;
	/*done converting trees to tours*/
	
	cout << "tours soln cost after primal dual phase is " << totalCostToursSolnPD << endl;
	fout << totalCostToursSolnPD << ' ';

	/*start pruning phase using prioritized delete-add*/

	/*
	start of delete-add local search
	*/


	//whether any operation improves the cost
	bool improveDA = true;

	//ratio of improvement for any operation
	double improveRatioDA = 1;

	//total number of rounds
	int numRoundsDA = 0;

	while (improveDA && improveRatioDA > 0.01){
		//update numRounds
		numRoundsDA++;

		//don't try changing visits unless find an improvement
		improveDA = false;

		/*first, try delete*/
		//initialize the day achieving the best improvement
		int bestDeleteOnlyDay = 1;
		double bestDeleteOnlyImprovement = 0;

		//find the best deleteDay
		//We keep the solution always feasible by keeping the visit to everyone on day 0. This is also necessary for feasibility since the demands are positive at every (v,t), including t = 0.
		for (int deleteDay = numDays - 1; deleteDay > 0; deleteDay--){

			//compute latest visit before deleteDay for each demand point
			vector<vector<int> > latestVisitBeforeDeleteDay = init2D<int>(N, numDays);
			for (int i = 0; i < N; i++){
				for (int t = 0; t < numDays; t++){
					//only consider visits before deleteDay
					for (int s = 0; s < deleteDay; s++){
						//make sure s is also at most t
						if (s <= t){
							if (visitOriginal[i][s]){
								latestVisitBeforeDeleteDay[i][t] = s;

							}
						}
					}
				}
			}

			//declare variable penalties			
			vector<double> penalties = init1D<double>(N);
			for (int v = 0; v < N; v++){
				penalties[v] = 0;
			}

			//penalties[v] = sum_{t in [t',t_hat]} h_v * d^v_{t} * (t' - s(v,t,t')), where s(v,t,t') is the latest visit to (v,t) BEFORE day t'
			for (int v = 0; v < N; v++){
				//only get penalized for removing those who were already visited on deleteDay
				if ((originalDemand.at(v).at(deleteDay) > 0) && (visitOriginal.at(v).at(deleteDay))){
					int endDay = t_hatDelete(v, deleteDay, numDays, visitOriginal);

					for (int s = deleteDay; s < endDay + 1; s++){
						penalties[v] += originalUnitHolding.at(v) * originalDemand.at(v).at(s) * (deleteDay - latestVisitBeforeDeleteDay.at(v).at(s));

					}
				}

			}

			//find cost of the tree on day deleteDay
			double existingTreeCost = subgraphCost(N, originalEdgeWeights, existingTreeOriginal.at(deleteDay));

			//find the total penalty of removing the entire tree on deleteDay
			double extraHoldingCost = penaltyCostDelete(N, penalties, deleteDay, visitOriginal);

			//find the change in cost
			double difference = costChangeDelete(existingTreeCost, extraHoldingCost);

			//update best day and best improvement
			if (difference < bestDeleteOnlyImprovement){
				bestDeleteOnlyImprovement = difference;
				bestDeleteOnlyDay = deleteDay;
			}
		}

		//if cost improves, update new IRP solution
		if (bestDeleteOnlyImprovement < -1e-10){
			DEBUG_MSG("Improves on day " << bestDeleteOnlyDay << endl);
			DEBUG_MSG("Cost difference is " << bestDeleteOnlyImprovement << endl);

			//update improveDA
			improveDA = true;

			//update improvementRatioDA
			improveRatioDA = -bestDeleteOnlyImprovement / (routingCost + holdingCost);

			/*redo delete phase on bestDay*/

			//compute latest visit before deleteDay for each demand point
			vector<vector<int> > latestVisitBeforeDeleteDay = init2D<int>(N, numDays);
			for (int i = 0; i < N; i++){
				for (int t = 0; t < numDays; t++){
					//only consider visits before deleteDay
					for (int s = 0; s < bestDeleteOnlyDay; s++){
						//make sure s is also at most t
						if (s <= t){
							if (visitOriginal[i][s]){
								latestVisitBeforeDeleteDay[i][t] = s;

							}
						}
					}
				}
			}

			//declare variable penalties			
			vector<double> penalties = init1D<double>(N);
			for (int v = 0; v < N; v++){
				penalties[v] = 0;
			}

			//penalties[v] = sum_{t in [t',t_hat]} h_v * d^v_{t} * (t' - s(v,t,t')), where s(v,t,t') is the latest visit to (v,t) BEFORE day t'
			for (int v = 0; v < N; v++){
				//only get penalized for removing those who were already visited on deleteDay
				if ((originalDemand.at(v).at(bestDeleteOnlyDay) > 0) && (visitOriginal.at(v).at(bestDeleteOnlyDay))){
					int endDay = t_hatDelete(v, bestDeleteOnlyDay, numDays, visitOriginal);

					for (int s = bestDeleteOnlyDay; s < endDay + 1; s++){
						penalties[v] += originalUnitHolding.at(v) * originalDemand.at(v).at(s) * (bestDeleteOnlyDay - latestVisitBeforeDeleteDay.at(v).at(s));

					}
				}

			}

			//find cost of the tree on day deleteDay
			double existingTreeCost = subgraphCost(N, originalEdgeWeights, existingTreeOriginal.at(bestDeleteOnlyDay));

			//find the total penalty of removing the entire tree on deleteDay
			double extraHoldingCost = penaltyCostDelete(N, penalties, bestDeleteOnlyDay, visitOriginal);

			//find the change in cost
			double difference = costChangeDelete(existingTreeCost, extraHoldingCost);


			/*end of delete phase*/

			//update visit
			for (int v = 0; v < N; v++){
				visitOriginal.at(v).at(bestDeleteOnlyDay) = false;

			}

			//update latestVisit
			for (int i = 0; i < N; i++){
				for (int t = 0; t < numDays; t++){
					for (int s = 0; s <= t; s++){
						if (visitOriginal[i][s]){
							latestVisitOriginal[i][t] = s;

						}
					}
				}
			}

			//update existingTree
			for (int u = 0; u < N; u++){
				for (int v = 0; v < N; v++){
					existingTreeOriginal.at(bestDeleteOnlyDay).at(u).at(v) = false;

				}
			}

			//update routing cost and holding cost
			updateCostsDelete(existingTreeCost, extraHoldingCost, routingCost, holdingCost);
			DEBUG_MSG("new routing cost from DELETE is " << routingCost << endl);
			DEBUG_MSG("new holding cost from DELETE is " << holdingCost << endl);

			/*
			start of sanity check
			*/

			//compute and output routing cost
			double routingCostCheck = 0;
			for (int s = 0; s < numDays; s++){
				for (int u = 0; u < N; u++){
					//count each arc
					for (int v = 0; v < N; v++){
						if (existingTreeOriginal.at(s).at(u).at(v)){
							routingCostCheck += originalEdgeWeights.at(u).at(v);
						}
					}
				}
			}
			DEBUG_MSG("the new routing cost from DELETE should be " << routingCostCheck << endl);

			//compute and output holding cost
			double holdingCostCheck = 0;
			for (int i = 0; i < N; i++){
				for (int t = 0; t < numDays; t++){

					holdingCostCheck += originalHolding[i][latestVisitOriginal[i][t]][t];
				}
			}
			DEBUG_MSG("the new holding cost from DELETE shoud be " << holdingCostCheck << endl);

			/*
			end of sanity check
			*/
		}
		else{


			/*second, try add*/
			//initialize the day achieving the best improvement
			int bestAddOnlyDay = 1;
			double bestAddOnlyImprovement = 0;

			//find best addDay
			for (int addDay = 1; addDay < numDays; addDay++){


				//declare variable penalties
				vector<double> penalties = init1D<double>(N);
				for (int v = 0; v < N; v++){
					penalties[v] = 0;
				}

				//penalties[v] = sum_{t \in [t',t_hat]} h_v * d^v_t * (t' - s(v,t))
				for (int v = 0; v < N; v++){
					//only get reward for those who were not already visited on addDay
					if ((originalDemand.at(v).at(addDay) > 0) && (!visitOriginal.at(v).at(addDay))){
						int endDay = t_hatAdd(v, addDay, numDays, visitOriginal);
						for (int s = addDay; s < endDay + 1; s++){
							penalties[v] += originalUnitHolding.at(v) * originalDemand.at(v).at(s) * (addDay - latestVisitOriginal.at(v).at(s));
						}
					}
				}


				//create input file for pcst instance
				createPCST(depot, N, numArcsOriginal, originalEdgeWeights, existingTreeOriginal.at(addDay), penalties);

				//run pcst solver on pcst instance and create output file containing pcstSet, pcstTree
				vector<vector<bool> > pcstTree = init2D<bool>(N, N);
				for (int i = 0; i < N; i++){
					for (int j = 0; j < N; j++){
						pcstTree.at(i).at(j) = false;
					}
				}

				unordered_set<int> pcstSet;
				solvePCST(N, existingTreeOriginal.at(addDay), pcstTree, pcstSet);

				//compute pcstTreeCost and pcstPenaltyCost from pcstTree and pcstSet
				double existingTreeCost = subgraphCost(N, originalEdgeWeights, existingTreeOriginal.at(addDay));
				//WARNING: pcstTreeCost is the cost of the solution from the PCST instance, with edge weights 0 for any edges in existingTree
				double pcstTreeCost = subgraphCost(N, originalEdgeWeights, pcstTree) - existingTreeCost;
				double pcstRewardCost = rewardCost(N, penalties, pcstSet);
				double difference = costChange(pcstTreeCost, pcstRewardCost);

				//update best day and best improvement
				if (difference < bestAddOnlyImprovement){
					bestAddOnlyImprovement = difference;
					bestAddOnlyDay = addDay;
				}
			}


			//if cost improves, update new IRP solution
			if (bestAddOnlyImprovement < -1e-10){
				DEBUG_MSG("Improves on day " << bestAddOnlyDay << endl);
				DEBUG_MSG("Cost difference is " << bestAddOnlyImprovement << endl);

				//update improveDA
				improveDA = true;

				//upate improveRatioDA
				improveRatioDA = -bestAddOnlyImprovement / (routingCost + holdingCost);

				/*redo add phase on bestDay*/

				//declare variable penalties
				vector<double> penalties = init1D<double>(N);
				for (int v = 0; v < N; v++){
					penalties[v] = 0;
				}

				//penalties[v] = sum_{t \in [t',t_hat]} h_v * d^v_t * (t' - s(v,t))
				for (int v = 0; v < N; v++){
					//only get reward for those who were not already visited on addDay
					if ((originalDemand.at(v).at(bestAddOnlyDay) > 0) && (!visitOriginal.at(v).at(bestAddOnlyDay))){
						int endDay = t_hatAdd(v, bestAddOnlyDay, numDays, visitOriginal);
						for (int s = bestAddOnlyDay; s < endDay + 1; s++){
							penalties[v] += originalUnitHolding.at(v) * originalDemand.at(v).at(s) * (bestAddOnlyDay - latestVisitOriginal.at(v).at(s));
						}
					}
				}


				//create input file for pcst instance
				createPCST(depot, N, numArcsOriginal, originalEdgeWeights, existingTreeOriginal.at(bestAddOnlyDay), penalties);

				//run pcst solver on pcst instance and create output file containing pcstSet, pcstTree
				vector<vector<bool> > pcstTree = init2D<bool>(N, N);
				for (int i = 0; i < N; i++){
					for (int j = 0; j < N; j++){
						pcstTree.at(i).at(j) = false;
					}
				}

				unordered_set<int> pcstSet;
				solvePCST(N, existingTreeOriginal.at(bestAddOnlyDay), pcstTree, pcstSet);

				//compute pcstTreeCost and pcstPenaltyCost from pcstTree and pcstSet
				double existingTreeCost = subgraphCost(N, originalEdgeWeights, existingTreeOriginal.at(bestAddOnlyDay));
				double pcstTreeCost = subgraphCost(N, originalEdgeWeights, pcstTree) - existingTreeCost;
				double pcstRewardCost = rewardCost(N, penalties, pcstSet);
				double difference = costChange(pcstTreeCost, pcstRewardCost);


				/*end of add phase*/

				//update visit
				for (int v : pcstSet){
					visitOriginal.at(v).at(bestAddOnlyDay) = true;

				}

				//update latestVisit
				for (int i = 0; i < N; i++){
					for (int t = 0; t < numDays; t++){
						for (int s = 0; s <= t; s++){
							if (visitOriginal[i][s] == true){
								latestVisitOriginal[i][t] = s;

							}
						}
					}
				}

				//update existingTree
				existingTreeOriginal.at(bestAddOnlyDay) = pcstTree;


				//update routing cost and holding cost
				updateCosts(pcstTreeCost, pcstRewardCost, routingCost, holdingCost);
				DEBUG_MSG("new routing cost from ADD is " << routingCost << endl);
				DEBUG_MSG("new holding cost from ADD is " << holdingCost << endl);

				
				/*
				start of sanity check
				*/

				//compute and output routing cost
				double routingCostCheck = 0;
				for (int s = 0; s < numDays; s++){
					for (int u = 0; u < N; u++){
						//count each arc per direction
						for (int v = 0; v < N; v++){
							if (existingTreeOriginal.at(s).at(u).at(v)){
								routingCostCheck += originalEdgeWeights.at(u).at(v);
							}
						}
					}
				}
				DEBUG_MSG("the new routing cost from ADD should be " << routingCostCheck << endl);

				//compute and output holding cost
				double holdingCostCheck = 0;
				for (int i = 0; i < N; i++){
					for (int t = 0; t < numDays; t++){
						holdingCostCheck += originalHolding[i][latestVisitOriginal[i][t]][t];
					}
				}
				DEBUG_MSG("the new holding cost from ADD shoud be " << holdingCostCheck << endl);

				/*
				end of sanity check
				*/


			}
			else{


				/*finally, try pairwise delete-add*/
				//initialize the day achieving the best improvement
				int bestDeleteDay = 1;
				int bestAddDay = 1;
				double bestImprovement = 0;

				//find day with best improvement
				//We keep the solution always feasible by keeping the visit to everyone on day 0. This is also necessary for feasibility since the demands are positive at every (v,t), including t = 0.
				for (int deleteDay = 1; deleteDay < numDays; deleteDay++){
					for (int addDay = 1; addDay < numDays; addDay++){

						/*
						start of delete phase
						*/

						//compute latest visit before deleteDay for each demand point
						vector<vector<int> > latestVisitBeforeFirstDeleteDay = init2D<int>(N, numDays);
						for (int i = 0; i < N; i++){
							for (int t = 0; t < numDays; t++){
								//only consider visits before deleteDay
								for (int s = 0; s < deleteDay; s++){
									//make sure s is also at most t
									if (s <= t){
										if (visitOriginal[i][s]){
											latestVisitBeforeFirstDeleteDay[i][t] = s;
											
										}
									}
								}
							}
						}


						//declare variable penalties			
						vector<double> penaltiesFirstDelete = init1D<double>(N);
						for (int v = 0; v < N; v++){
							penaltiesFirstDelete[v] = 0;
						}

						//penalties[v] = sum_{t in [t',t_hat]} h_v * d^v_{t} * (t' - s(v,t,t')), where s(v,t,t') is the latest visit to (v,t) BEFORE day t'
						for (int v = 0; v < N; v++){
							//only get penalized for removing those who were already visited on deleteDay
							if ((originalDemand.at(v).at(deleteDay) > 0) && (visitOriginal.at(v).at(deleteDay))){
								int endDay = t_hatDelete(v, deleteDay, numDays, visitOriginal);

								for (int s = deleteDay; s < endDay + 1; s++){
									penaltiesFirstDelete[v] += originalUnitHolding.at(v) * originalDemand.at(v).at(s) * (deleteDay - latestVisitBeforeFirstDeleteDay.at(v).at(s));

								}
							}

						}

						//find cost of the tree on day deleteAddDay
						double existingTreeCostFirstDelete = subgraphCost(N, originalEdgeWeights, existingTreeOriginal.at(deleteDay));

						//find the total penalty of removing the entire tree on deleteAddDay
						double extraHoldingFirstCost = penaltyCostDelete(N, penaltiesFirstDelete, deleteDay, visitOriginal);

						//find the change in cost
						double differenceFirstDelete = costChangeDelete(existingTreeCostFirstDelete, extraHoldingFirstCost);


						/*
						end of delete phase, start of add phase
						*/

						//declare variable penalties
						vector<double> penaltiesAdd = init1D<double>(N);
						for (int v = 0; v < N; v++){
							penaltiesAdd[v] = 0;
						}

						//to calculate the correct t_hat, need to call it on the updated visit vector that has no visits on day deleteDay
						vector<vector<bool> > tempVisit = visitOriginal;
						for (int v = 0; v < N; v++){
							tempVisit.at(v).at(deleteDay) = false;
						}

						//to calculate the correct s(v,t), need to call it on the updated latest visits derived from tempVisit
						vector<vector<int> > tempLatestVisit = init2D<int>(N, numDays);
						//update tempLatestVisit
						for (int i = 0; i < N; i++){
							for (int t = 0; t < numDays; t++){
								for (int s = 0; s <= t; s++){
									if (tempVisit[i][s]){
										tempLatestVisit[i][t] = s;

									}
								}
							}
						}

						//penalties[v] = sum_{t \in [t',t_hat]} h_v * d^v_t * (t' - s(v,t))
						for (int v = 0; v < N; v++){
							//only get reward for those who were not already visited on addDay				
							if ((originalDemand.at(v).at(addDay) > 0) && (!tempVisit.at(v).at(addDay))){
								int endDay = t_hatAdd(v, addDay, numDays, tempVisit);
								for (int s = addDay; s < endDay + 1; s++){
									//use tempLatestVisit which accounts for deletion on day deleteAddDay
									penaltiesAdd[v] += originalUnitHolding.at(v) * originalDemand.at(v).at(s) * (addDay - tempLatestVisit.at(v).at(s));
								}
							}
						}

						//use updated existing tree depending on what was deleted
						vector<vector<bool> > tempExistingTree = init2D<bool>(N, N);
						//initialize it to empty tree
						for (int u = 0; u < N; u++){
							for (int v = 0; v < N; v++){
								tempExistingTree.at(u).at(v) = false;
							}
						}
						//if add day is different from delete day, then tempExistingTree is not necessarily empty
						if (deleteDay != addDay){
							tempExistingTree = existingTreeOriginal.at(addDay);
						}

						//create input file for pcst solver				
						createPCST(depot, N, numArcsOriginal, originalEdgeWeights, tempExistingTree, penaltiesAdd);

						//run pcst solver on pcst instance and create output file containing pcstSet, pcstTree

						//declare pcstTree to eventually store the pcst tree found by the solver
						//declare an empty tree to put into the existingTree argument of solvePCST
						vector<vector<bool> > pcstTree = init2D<bool>(N, N);
						vector<vector<bool> > emptyTree = init2D<bool>(N, N);
						for (int i = 0; i < N; i++){
							for (int j = 0; j < N; j++){
								pcstTree[i][j] = false;
								emptyTree[i][j] = false;
							}
						}

						//declare the pcst set
						unordered_set<int> pcstSet;

						//update pcstTree
						solvePCST(N, tempExistingTree, pcstTree, pcstSet);

						//compute pcstTreeCost and pcstRewardCost from pcstTree and pcstSet
						double existingTreeCost = subgraphCost(N, originalEdgeWeights, tempExistingTree);
						double pcstTreeCost = subgraphCost(N, originalEdgeWeights, pcstTree) - existingTreeCost;
						double pcstRewardCost = rewardCost(N, penaltiesAdd, pcstSet);
						double differenceAdd = costChange(pcstTreeCost, pcstRewardCost);

						double totalDifference = differenceFirstDelete + differenceAdd;

						//update best day and best improvement
						if (totalDifference < bestImprovement){
							bestImprovement = totalDifference;
							bestDeleteDay = deleteDay;
							bestAddDay = addDay;
						}
					}
				}

				//if cost improves, update new IRP solution
				if (bestImprovement < -1e-10){
					DEBUG_MSG("best pairwise delete day is " << bestDeleteDay << endl);
					DEBUG_MSG("best pairwise add day is " << bestAddDay << endl);

					//update improve
					improveDA = true;

					//upate improveRatio
					improveRatioDA = -bestImprovement / (routingCost + holdingCost);

					/*redo the delete-add phase on bestDay, which deletes on bestDay and bestDay+1 and adds on bestDay*/

					/*
					start of delete phase
					*/

					//compute latest visit before bestDeleteDay for each demand point
					vector<vector<int> > latestVisitBeforeFirstDeleteDay = init2D<int>(N, numDays);
					for (int i = 0; i < N; i++){
						for (int t = 0; t < numDays; t++){
							//only consider visits before deleteDay
							for (int s = 0; s < bestDeleteDay; s++){
								//make sure s is also at most t
								if (s <= t){
									if (visitOriginal[i][s]){
										latestVisitBeforeFirstDeleteDay[i][t] = s;

									}
								}
							}
						}
					}


					//declare variable penalties			
					vector<double> penaltiesFirstDelete = init1D<double>(N);
					for (int v = 0; v < N; v++){
						penaltiesFirstDelete[v] = 0;
					}

					//penalties[v] = sum_{t in [t',t_hat]} h_v * d^v_{t} * (t' - s(v,t,t')), where s(v,t,t') is the latest visit to (v,t) BEFORE day t'
					for (int v = 0; v < N; v++){
						//only get penalized for removing those who were already visited on deleteDay
						if ((originalDemand.at(v).at(bestDeleteDay) > 0) && (visitOriginal.at(v).at(bestDeleteDay))){
							int endDay = t_hatDelete(v, bestDeleteDay, numDays, visitOriginal);

							for (int s = bestDeleteDay; s < endDay + 1; s++){
								penaltiesFirstDelete[v] += originalUnitHolding.at(v) * originalDemand.at(v).at(s) * (bestDeleteDay - latestVisitBeforeFirstDeleteDay.at(v).at(s));

							}
						}

					}

					//find cost of the tree on day deleteAddDay
					double existingTreeCostFirstDelete = subgraphCost(N, originalEdgeWeights, existingTreeOriginal.at(bestDeleteDay));

					//find the total penalty of removing the entire tree on deleteAddDay
					double extraHoldingFirstCost = penaltyCostDelete(N, penaltiesFirstDelete, bestDeleteDay, visitOriginal);

					//find the change in cost
					double differenceFirstDelete = costChangeDelete(existingTreeCostFirstDelete, extraHoldingFirstCost);

					/*
					end of delete phase, start of add phase
					*/

					//declare variable penalties
					vector<double> penaltiesAdd = init1D<double>(N);
					for (int v = 0; v < N; v++){
						penaltiesAdd[v] = 0;
					}

					//to calculate the correct t_hat, need to call it on the updated visit vector that has no visits on day deleteDay
					vector<vector<bool> > tempVisit = visitOriginal;
					for (int v = 0; v < N; v++){
						tempVisit.at(v).at(bestDeleteDay) = false;
					}

					//to calculate the correct s(v,t), need to call it on the updated latest visits derived from tempVisit
					vector<vector<int> > tempLatestVisit = init2D<int>(N, numDays);
					//update tempLatestVisit
					for (int i = 0; i < N; i++){
						for (int t = 0; t < numDays; t++){
							for (int s = 0; s <= t; s++){
								if (tempVisit[i][s]){
									tempLatestVisit[i][t] = s;

								}
							}
						}
					}

					//penalties[v] = sum_{t \in [t',t_hat]} h_v * d^v_t * (t' - s(v,t))
					for (int v = 0; v < N; v++){
						//only get reward for those who were not already visited on addDay
						if ((originalDemand.at(v).at(bestAddDay) > 0) && (!tempVisit.at(v).at(bestAddDay))){
							int endDay = t_hatAdd(v, bestAddDay, numDays, tempVisit);
							for (int s = bestAddDay; s < endDay + 1; s++){
								//use the updated latest visits that pretends bestDay visits were deleted
								penaltiesAdd[v] += originalUnitHolding.at(v) * originalDemand.at(v).at(s) * (bestAddDay - tempLatestVisit.at(v).at(s));
							}
						}
					}

					//use updated existing tree depending on what was deleted
					vector<vector<bool> > tempExistingTree = init2D<bool>(N, N);
					//initialize it to empty tree
					for (int u = 0; u < N; u++){
						for (int v = 0; v < N; v++){
							tempExistingTree.at(u).at(v) = false;
						}
					}
					//if add day is different from delete day, then tempExistingTree is not necessarily empty
					if (bestDeleteDay != bestAddDay){
						tempExistingTree = existingTreeOriginal.at(bestAddDay);
					}

					//create input file for pcst solver			
					createPCST(depot, N, numArcsOriginal, originalEdgeWeights, tempExistingTree, penaltiesAdd);

					//run pcst solver on pcst instance and create output file containing pcstSet, pcstTree

					//declare pcstTree to eventually store the pcst tree found by the solver
					//declare an empty tree to put into the existingTree argument of solvePCST
					vector<vector<bool> > pcstTree = init2D<bool>(N, N);
					vector<vector<bool> > emptyTree = init2D<bool>(N, N);
					for (int i = 0; i < N; i++){
						for (int j = 0; j < N; j++){
							pcstTree[i][j] = false;
							emptyTree[i][j] = false;
						}
					}

					//declare the pcst set
					unordered_set<int> pcstSet;

					//update pcstTree
					solvePCST(N, tempExistingTree, pcstTree, pcstSet);


					//compute pcstTreeCost and pcstRewardCost from pcstTree and pcstSet
					double existingTreeCost = subgraphCost(N, originalEdgeWeights, tempExistingTree);
					double pcstTreeCost = subgraphCost(N, originalEdgeWeights, pcstTree) - existingTreeCost;
					double pcstRewardCost = rewardCost(N, penaltiesAdd, pcstSet);
					double differenceAdd = costChange(pcstTreeCost, pcstRewardCost);

					double totalDifference = differenceFirstDelete + differenceAdd;

					/*end of delete-add phase*/

					/*delete phase updates*/
					//update visit on day bestDay
					for (int v = 0; v < N; v++){
						visitOriginal.at(v).at(bestDeleteDay) = false;
					}

					//update latestVisit
					for (int i = 0; i < N; i++){
						for (int t = 0; t < numDays; t++){
							for (int s = 0; s <= t; s++){
								if (visitOriginal[i][s]){
									latestVisitOriginal[i][t] = s;

								}
							}
						}
					}

					//update existingTree on bestDay
					for (int u = 0; u < N; u++){
						for (int v = 0; v < N; v++){
							existingTreeOriginal.at(bestDeleteDay).at(u).at(v) = false;
						}
					}

					updateCostsDelete(existingTreeCostFirstDelete, extraHoldingFirstCost, routingCost, holdingCost);
					DEBUG_MSG("new routing cost after delete phase of DA is " << routingCost << endl);
					DEBUG_MSG("new holding cost after delete phase of DA is " << holdingCost << endl);
					DEBUG_MSG("new total cost after delete phase of DA is " << routingCost + holdingCost << endl);

					/*
					start of sanity check after delete phase
					*/

					//compute and output routing cost
					double routingCostCheck = 0;
					for (int s = 0; s < numDays; s++){
						for (int u = 0; u < N; u++){
							//count arcs in both directions
							for (int v = 0; v < N; v++){
								if (existingTreeOriginal.at(s).at(u).at(v)){
									routingCostCheck += originalEdgeWeights.at(u).at(v);
								}
							}
						}
					}
					DEBUG_MSG("the new routing cost after delete phase of DA should be " << routingCostCheck << endl);

					//compute and output holding cost
					double holdingCostCheck = 0;
					for (int i = 0; i < N; i++){
						for (int t = 0; t < numDays; t++){

							holdingCostCheck += originalHolding[i][latestVisitOriginal[i][t]][t];
						}
					}
					DEBUG_MSG("the new holding cost after delete phase of DA shoud be " << holdingCostCheck << endl);

					/*
					end of sanity check after delete phase
					*/

					/*add phase updates*/
					//update visit on day bestDay+1
					for (int v = 0; v < N; v++){
						unordered_set<int>::iterator clientIter = pcstSet.find(v);
						//add visit to v if v is in pcstSet
						if (clientIter != pcstSet.end()){
							visitOriginal.at(v).at(bestAddDay) = true;

						}

					}


					//update latestVisit
					for (int i = 0; i < N; i++){
						for (int t = 0; t < numDays; t++){
							for (int s = 0; s <= t; s++){
								if (visitOriginal[i][s]){
									latestVisitOriginal[i][t] = s;

								}
							}
						}
					}

					//update existingTree on bestDay-1
					for (int u = 0; u < N; u++){
						for (int v = 0; v < N; v++){
							if (pcstTree.at(u).at(v)){
								existingTreeOriginal.at(bestAddDay).at(u).at(v) = true;

							}

						}
					}

					//update routing cost and holding cost
					updateCosts(pcstTreeCost, pcstRewardCost, routingCost, holdingCost);
					DEBUG_MSG("new routing cost after add phase of DA is " << routingCost << endl);
					DEBUG_MSG("new holding cost after add phase of DA is " << holdingCost << endl);
					DEBUG_MSG("new total cost after add phase of DA is " << routingCost + holdingCost << endl);

					/*
					start of sanity check
					*/

					//reset, compute and output routing cost
					routingCostCheck = 0;
					for (int s = 0; s < numDays; s++){
						for (int u = 0; u < N; u++){
							//count arcs in both directions
							for (int v = 0; v < N; v++){
								if (existingTreeOriginal.at(s).at(u).at(v)){
									routingCostCheck += originalEdgeWeights.at(u).at(v);
								}
							}
						}
					}
					DEBUG_MSG("the new routing cost after add phase of DA should be " << routingCostCheck << endl);

					//reset, compute and output holding cost
					holdingCostCheck = 0;
					for (int i = 0; i < N; i++){
						for (int t = 0; t < numDays; t++){

							holdingCostCheck += originalHolding[i][latestVisitOriginal[i][t]][t];
						}
					}
					DEBUG_MSG("the new holding cost after add phase of DA shoud be " << holdingCostCheck << endl);

					/*
					end of sanity check
					*/
				}
			}
		}



	}
	//end of while loop

	/*
	end of delete-add local search
	*/

	//update the cost
	totalCost = routingCost + holdingCost;

	/*done pruning phase*/


	/*convert trees-solution to tours-solution*/

	double tourRoutingCost = 0;
	//create visitReversed reverse the indices of visit
	vector<vector<bool> > visitReversed = init2D<bool>(numDays, N);
	for (int v = 0; v < N; v++){
		for (int s = 0; s < numDays; s++){
			visitReversed.at(s).at(v) = visitOriginal.at(v).at(s);
		}
	}

	//keep track of costs
	double tourToday;
	double treeToday;
	for (int today = 0; today < numDays; today++){
		//find tour cost of visit set on day today
		tourToday = tourCostOfVisit(N, position, visitReversed.at(today), originalEdgeWeights);

		tourRoutingCost += tourToday;

		//find tree cost of visit set on day 0
		treeToday = subgraphCost(N, originalEdgeWeights, existingTreeOriginal.at(today));

	}


	//total cost of tours-solution
	double totalCostToursSoln = tourRoutingCost + holdingCost;
	/*done converting trees to tours*/

	/*
	* Stop the timer. We should do this before printing the final answer.
	*/
	auto elapsed = std::chrono::high_resolution_clock::now() - start_timer;
	long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
	cout << "Number of rounds of prioritized delete-add = " << numRoundsDA << endl;
	fout << numRoundsDA << ' ';
	cout << "Program execution time = " << microseconds << " microseconds\n";
	fout << microseconds << ' ';
	
	cout << "The tree-routing cost is " << routingCost << "\n";
	cout << "The holding cost is " << holdingCost << "\n";
	cout << "The tree-total cost is " << totalCost << "\n";
	fout << totalCost << ' ';

	cout << "The tour-total cost is " << totalCostToursSoln << endl;
	fout << totalCostToursSoln << endl;
	fout.close();

	return 0;
}