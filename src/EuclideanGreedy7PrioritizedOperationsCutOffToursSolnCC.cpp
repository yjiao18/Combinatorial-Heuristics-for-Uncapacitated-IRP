/*
Greedy set cover algorithm that covers demand points by Steiner trees

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

//find cost of tree given the adjacency matrix of the tree
double treeCost(int _numClients, vector<vector<double> > _edgeWeights, vector<vector<bool> > _tree){
	double cost = 0;
	for (int i = 0; i < _numClients; i++){
		//PCST solution might not give arcs in (small,large) order
		for (int j = 0; j < _numClients; j++){
			if (_tree.at(i).at(j)){
				cost += _edgeWeights.at(i).at(j);
			}
		}
	}
	return cost;
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

		}
		cycle.at(vertexLabels.at(posOrder.at(tspLength - 1))).at(vertexLabels.at(posOrder.at(0))) = true;


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

//find the penalty cost of a given pcst solution
double penaltyCost(int _numClients, vector<double> _penalties, unordered_set<int> _pcstSet){
	double cost = 0;
	for (int v = 0; v < _numClients; v++){
		unordered_set<int>::iterator clientIter = _pcstSet.find(v);
		//add the penalty of clients who are not in the visit set "_pcstSet"
		if (clientIter == _pcstSet.end()){
			cost += _penalties.at(v);
		}
	}
	return cost;
}


//update latestVisit given visit
void finalLatestVisit(int _numClients, int _numDays, vector<vector<bool> > _visit, vector<vector<int> > &latestVisitOut){
	for (int i = 0; i < _numClients; i++){
		for (int t = 0; t < _numDays; t++){
			for (int s = 0; s <= t; s++){
				if (_visit[i][s] == true){
					latestVisitOut[i][t] = s;
				}
			}
		}
	}
}

//compute routing cost given existingTree
double totalRoutingCost(int _numClients, int _numDays, vector<vector<double> > _edgeWeights, vector<vector<vector<bool> > > _existingTree){
	double totalRC = 0;
	for (int s = 0; s < _numDays; s++){
		totalRC += treeCost(_numClients, _edgeWeights, _existingTree.at(s));
	}
	return totalRC;
}

//compute holdingCost given latestVisit
double totalHoldingCost(int _numClients, int _numDays, vector<vector<int> > _latestVisit, vector<vector<vector<double> > > _holding){
	double totalHC = 0;
	for (int v = 0; v < _numClients; v++){
		for (int t = 0; t < _numDays; t++){
			totalHC += _holding.at(v).at(_latestVisit[v][t]).at(t);
		}
	}
	return totalHC;
}

//for greedy phase and add operation: create a pcst instance given the desired edge weights and penalties
void createPCST(int _depot, int _numNodes, int _numArcs, vector<vector<double> > _edgeWeights, vector<vector<bool> > _existingTree, vector<double> _penalties){
	// Remove any existing input and output files
	std::remove("greedyInst7c.stp");
	std::remove("greedySoln7c.txt");

	// Open a temprary input file
	char filename[1000];
	sprintf(filename, "greedyInst7c.stp");
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

	// Run executable and create output to "greedySoln7c.txt"
	system("./dapcstp -f greedyInst7c.stp -o greedySoln7c.txt 1>/dev/null 2>/dev/null");

}

//for greedy phase and add operation: modifies the tree edges and visited set based on the pcst solution in greedySoln7c.txt
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
	if ((fp = fopen("greedySoln7c.txt", "r")) == NULL) {
		fflush(stderr);
		fprintf(stderr, "error parse_output: file not found greedySoln7c\n");
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

int coverageNumber(int _numDays, int _client, int _day, double _density, vector<vector<vector<double> > > _holding, vector<vector<bool> > _isCovered, vector<vector<double> > _demand){
	//(_client,_day) might have already been covered, so can't assume to cover the demand point on _day
	int coverageNum = 0;
	//the amount of demand covered
	double coverageAmount = 0;
	//whether still within density tolerance
	bool withinDensity = true;
	//day to cover up to
	int currentDay = _day;
	//the holding cost
	double currentHoldingCost = 0;

	//can cover the uncovered demand point at location v at the next day as long as the average holding cost h(D')/demand(D') stays <= _density
	//we are using the fact that our linear holding costs obey h^v_{s,t_1} < h^v_{s,t_2} for t_1 < t_2
	while (withinDensity){

		//no more demands to serve if currentDay > T-1
		if (currentDay > _numDays - 1){

			break;
		}

		//only try the demand points that are not already covered
		if (!_isCovered.at(_client).at(currentDay)){

			//find holding cost if serve the uncovered next demand point at time _day
			double tempHoldingCost = currentHoldingCost + _holding.at(_client).at(_day).at(currentDay);

			//find number of demands served if serve the next demand point
			int tempCoverageNum = coverageNum + 1;
			//find amount of demand value covered if serve the next demand point
			double tempCoverageAmount = coverageAmount + _demand.at(_client).at(currentDay);

			double averageHoldingCost = tempHoldingCost / tempCoverageAmount;

			if (averageHoldingCost <= _density){

				//update coverageNum, tempCoverageAmount, currentDay, currentHoldingCost
				coverageNum = tempCoverageNum;
				coverageAmount = tempCoverageAmount;
				currentDay += 1;
				currentHoldingCost = tempHoldingCost;
			}
			else{

				withinDensity = false;
				//holding costs only would get larger the farther along the days
				break;
			}
		}
		else{
			currentDay += 1;
		}
	}

	return coverageNum;
}

//finds coverage amount of (v,t,rho)
int coverageValue(int _numDays, int _client, int _day, double _density, vector<vector<vector<double> > > _holding, vector<vector<bool> > _isCovered, vector<vector<double> > _demand){
	//(_client,_day) might have already been covered, so can't assume to cover the demand point on _day
	int coverageNum = 0;
	//the amount of demand covered
	double coverageAmount = 0;
	//whether still within density tolerance
	bool withinDensity = true;
	//day to cover up to
	int currentDay = _day;
	//the holding cost
	double currentHoldingCost = 0;

	//can cover the uncovered demand point at location v at the next day as long as the average holding cost h(D')/demand(D') stays <= _density
	//we are using the fact that our linear holding costs obey h^v_{s,t_1} < h^v_{s,t_2} for t_1 < t_2
	while (withinDensity){

		//no more demands to serve if currentDay > T-1
		if (currentDay > _numDays - 1){

			break;
		}

		//only try the demand points that are not already covered
		if (!_isCovered.at(_client).at(currentDay)){

			//find holding cost if serve the uncovered next demand point at time _day
			double tempHoldingCost = currentHoldingCost + _holding.at(_client).at(_day).at(currentDay);

			//find number of demands served if serve the next demand point
			int tempCoverageNum = coverageNum + 1;
			//find amount of demand value covered if serve the next demand point
			double tempCoverageAmount = coverageAmount + _demand.at(_client).at(currentDay);

			double averageHoldingCost = tempHoldingCost / tempCoverageAmount;

			if (averageHoldingCost <= _density){

				//update coverageNum, tempCoverageAmount, currentDay, currentHoldingCost
				coverageNum = tempCoverageNum;
				coverageAmount = tempCoverageAmount;
				currentDay += 1;
				currentHoldingCost = tempHoldingCost;
			}
			else{

				withinDensity = false;
				//holding costs only would get larger the farther along the days
				break;
			}
		}
		else{
			currentDay += 1;
		}
	}

	return coverageAmount;
}

//finds if a given vertex is spanned by a given tree
bool isSpanned(int _client, int _numClients, vector<vector<bool> > _tree){
	bool spanned = false;
	//scan for edge incident to _client
	for (int u = 0; u < _numClients; u++){
		if (_tree.at(_client).at(u) || _tree.at(u).at(_client)){
			spanned = true;
			//don't need to find more edges beyond the first edge incident to _client
			break;
		}
	}
	return spanned;
}


//finds the day with lowest density value given the density values per day
int bestDay(int _numDays, vector<double> _bestDensity, vector<bool> _isThereSet, vector<int> _totalCovered){
	//find a day which covers some new demand
	int candidateDay;
	for (int t = 0; t < _numDays; t++){
		if (_isThereSet.at(t) && _totalCovered.at(t) > 0){
			candidateDay = t;
			//don't need to search further after finding one
			break;
		}
	}

	for (int s = 0; s < _numDays; s++){
		//update candidateDay to day s only if day s gets to cover any new demands
		if (_isThereSet.at(s) && _totalCovered.at(s) > 0){
			if (_bestDensity.at(s) < _bestDensity.at(candidateDay)){
				candidateDay = s;
			}
		}
	}
	return candidateDay;
}

//find if there is a set of density <= _targetDensity and add such set if it exists to the set cover
void addSet(double _targetDensity, int _depot, int _numClients, int _numDays, vector<vector<bool> > &visitOut, vector<vector<double> > _edgeWeights, vector<vector<vector<bool> > > &existingTreeOut, vector<vector<vector<double> > > _holding, vector<vector<bool> > &isCoveredOut, bool &isThereNewCoverageOut, vector<vector<double> > _demand){
	//start assuming isThereSet is false, will be updated to true if pcstSet has more than just depot	
	bool isThereSet = false;

	for (int minimizingDay = 0; minimizingDay < _numDays; minimizingDay++){



		/*
		solve PCST with penalties induced by targetDensity
		*/

		//don't need to define new edge costs bc createPCST already takes care of setting existingTree edges to be free to use

		//define the penalties: pi(v) = coverageValue(v,targetDay,tempDensity) for v not in existingTree, 0 for v in existingTree
		vector<double> pcstPenalties = init1D<double>(_numClients);
		for (int v = 0; v < _numClients; v++){
			if (isSpanned(v, _numClients, existingTreeOut.at(minimizingDay))){
				pcstPenalties.at(v) = 0;
			}
			else{
				pcstPenalties.at(v) = coverageValue(_numDays, v, minimizingDay, _targetDensity, _holding, isCoveredOut, _demand)*_targetDensity;

			}
		}



		//declare pcstSet, the set to be visited by the PCST solution
		unordered_set<int> pcstSet;

		//declare pcstTree, the indicator of which edges are used by the PCST solution
		vector<vector<bool> > pcstTree = init2D<bool>(_numClients, _numClients);
		for (int i = 0; i < _numClients; i++){
			for (int j = 0; j < _numClients; j++){
				pcstTree.at(i).at(j) = false;
			}
		}

		//create input file for the PCST instance
		createPCST(_depot, _numClients, _numClients*(_numClients - 1), _edgeWeights, existingTreeOut.at(minimizingDay), pcstPenalties);


		//update pcstSet and pcstTree using the solution file
		solvePCST(_numClients, existingTreeOut.at(minimizingDay), pcstTree, pcstSet);


		/*
		end of solving PCST wrt targetDensity
		*/


		//check if pcstSet has any client other than the depot
		if (pcstSet.size() > 1){
			isThereSet = true;


			/*at this point, there is a nontrivial pcstSet, but it might not cover any new demands. Next, need to find out if any uncovered demands get covered*/
			int totalCovered = 0;

			//declare pcstEdgeWeights, 0 on edges in existingTree, w_e on remaining edges
			vector<vector<double> > pcstEdgeWeights = init2D<double>(_numClients, _numClients);
			for (int u = 0; u < _numClients; u++){
				for (int v = 0; v < _numClients; v++){
					if (existingTreeOut.at(minimizingDay).at(u).at(v)){
						pcstEdgeWeights.at(u).at(v) = 0;
					}
					else{
						pcstEdgeWeights.at(u).at(v) = _edgeWeights.at(u).at(v);
					}
				}
			}

			//update routing cost of covering pcstSet
			double extraRoutingCost = treeCost(_numClients, pcstEdgeWeights, pcstTree);

			//update totalCovered, and holding cost of covering the coverage set associated with pcstSet
			double extraHoldingCost = 0;


			for (int v : pcstSet){


				//keep track of how many demands at location v has been covered so far
				int numberCovered = 0;
				for (int t = minimizingDay; t < _numDays; t++){
					//only cover up to coverageNumber[v] many uncovered demands
					if (numberCovered < coverageNumber(_numDays, v, minimizingDay, _targetDensity, _holding, isCoveredOut, _demand)){
						if (!isCoveredOut.at(v).at(t)){

							//add the cost of covering (v,t) at by (v,minimizingDay)
							extraHoldingCost += _holding.at(v).at(minimizingDay).at(t);
							//update numberCovered
							numberCovered += 1;
							//update totalCovered
							totalCovered += 1;
						}
					}
					//don't cover any more demands once reach coverage coverageNumber[v]
					else{
						break;
					}
				}

			}



			if (totalCovered > 0){
				//update new coverage
				isThereNewCoverageOut = true;

				//update visit
				for (int v : pcstSet){
					visitOut.at(v).at(minimizingDay) = true;
				}

				/*update isCoveredOut*/
				for (int v : pcstSet){

					//keep track of how many demands at location v has been covered so far
					int numberCovered = 0;
					for (int t = minimizingDay; t < _numDays; t++){

						//only cover up to coverageNumber[v] many uncovered demands
						if (numberCovered < coverageNumber(_numDays, v, minimizingDay, _targetDensity, _holding, isCoveredOut, _demand)){
							if (!isCoveredOut.at(v).at(t)){
								//update isCoveredOut
								isCoveredOut.at(v).at(t) = true;

								//update numberCovered
								numberCovered += 1;
							}
							else{

							}
						}
						//don't cover any more demands once reach coverage pcstPenalties[v]
						else{
							break;
						}
					}
			
				}

				//update existing tree
				existingTreeOut.at(minimizingDay) = pcstTree;

				//once find a day that works, get out of the search over days loop
				break;
			}
		}

	}

}

/*functions exclusive to prioritized delete-add phase*/

//find t_hatDelete where t_hatDelete is the t >= t' with no visits to v occuring within (t',t]
int t_hatDelete(int _client, int _day, int _numDays, vector<vector<bool> > _visit){

	int targetDay = _day;
	//go through t >= _day+1 stopping just before the first time that _client is visited after _day
	for (int t = _day + 1; t < _numDays; t++){
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

//for add: find the change in cost given the cost of the existing tree, the cost of the pcst tree, and the cost of the pcst penalties
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

//for add: returns the tree edges of pcst solution to input file greedySoln7c.txt
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
	if ((fp = fopen("greedySoln7c.txt", "r")) == NULL) {
		fflush(stderr);
		fprintf(stderr, "error parse_output: file not found greedySoln7c\n");
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

int main()
{	//N clients
	int N = 100;
	
	//T days
	int T = 6;

	//unit holding cost min
	double min_h = 1;

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
	sprintf(outputname, "../EuclideanIRPgreedy/outputsIterNmult100Htimes100_260/output-N-%d-T-%d-Htimes100-%d-inst-%d.txt", N, T, (int)(H * 100), multiplicity);
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
	fout << "flexibleGreedy7WeightedDensityAlpha1.1DelAddCutOff.01Tours" << ' ';


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

	//declare initial "empty" solution which can visit the root for free
	//declare visits. visit^i_s==true means visit client i at time s
	vector<vector<bool> > visit = init2D<bool>(numClients, numDays);
	

	//declare variable existingTree
	//existingTree[s][i][j] = true iff edge ij is used at time s
	vector<vector<vector<bool> > > existingTree = init3D<bool>(numDays, numClients, numClients);
	

	//declare isCovered, isCovered[v][t] indicates whether demand point (v,t) is covered
	vector<vector<bool> > isCovered = init2D<bool>(numClients, numDays);


	/*
	done initializing solution
	*/


	//declare upper bound for optimal density value: max H^v_{s,t} + max d_{r,v}

	//find highest holding cost over demand points
	double maxHolding = 0;
	for (int v = 0; v < numClients; v++){
		for (int t = 0; t < numDays; t++){
			if (holding.at(v).at(0).at(t) > maxHolding){
				maxHolding = holding.at(v).at(0).at(t);
			}
		}
	}

	//find highest routing cost over clients
	double maxRouting = 0;
	for (int v = 0; v < numClients; v++){
		if (edgeWeights.at(depot).at(v) > maxRouting){
			maxRouting = edgeWeights.at(depot).at(v);
		}
	}

	//define upper bound for optimal density
	double densityUB = maxHolding + maxRouting;
	DEBUG_MSG("density upper bound is " << densityUB << endl);

	//define lower bound for optimal density
	double densityLB = 0;

	//define allowed ratio between upper bound and lower bound for the initial density
	double ratio = .9;

	//initialize routing cost, holding cost, feasibility, and lowest cost of feasible solutions found
	double routingCost = 0;
	double holdingCost = 0;

	//keep track of whether there is new demand point being covered, will be updated to true in addSet function if there is a low density set that covers some uncovered demand
	bool isThereNewCoverage = false;

	/*find lowest initial density to start adding sets at*/
	//as long as upper bound is far from lower bound, narrow the gap	
	while (densityLB/densityUB < ratio){
		//reset isThereNewCoverage to false
		isThereNewCoverage = false;

		//reset isCovered
		for (int v = 0; v < numClients; v++){
			for (int t = 0; t < numDays; t++){
				if (v == depot){
					isCovered.at(v).at(t) = true;
				}
				else{
					isCovered.at(v).at(t) = false;
				}
			}
		}

		//reset existingTree	
		for (int s = 0; s < numDays; s++){
			for (int u = 0; u < numClients; u++){
				for (int v = 0; v < numClients; v++){
					existingTree[s][u][v] = false;
				}
			}
		}

		//reset visit
		for (int i = 0; i < numClients; i++) {
			for (int s = 0; s < numDays; s++) {
				if (i == depot){
					visit.at(i).at(s) = true;
				}
				else{
					visit.at(i).at(s) = false;
				}
			}
		}

		//declare guess value for density as midpoint between upper bound and lower bound
		double tempDensity = 0.5*(densityUB + densityLB);
		DEBUG_MSG("upper bound is " << densityUB << endl);
		DEBUG_MSG("lower bound is " << densityLB << endl);
		DEBUG_MSG("midpoint is " << tempDensity << endl);

		//check if can get new coverage with tempDensity
		addSet(tempDensity, depot, numClients, numDays, visit, edgeWeights, existingTree, holding, isCovered, isThereNewCoverage, demand);
		
		DEBUG_MSG("isThereNewCoverage is " << isThereNewCoverage << endl);

		//if there is new coverage
		if (isThereNewCoverage){
			densityUB = tempDensity;
			DEBUG_MSG("updated upper bound to " << tempDensity << endl);
		}
		//otherwise, increase the lower bound to current density
		else{
			densityLB = tempDensity;
			DEBUG_MSG("updated lower bound to " << tempDensity << endl);
		}
	}

	auto elapsedInitDensity = std::chrono::high_resolution_clock::now() - start_timer;
	long long microsecondsInitDensity = std::chrono::duration_cast<std::chrono::microseconds>(elapsedInitDensity).count();

	cout << "initial density took " << microsecondsInitDensity << " microseconds" << endl;
	fout << microsecondsInitDensity << ' ';

	/*done finding initial density*/

	/*clear all the passed variables again*/
	//reset isThereNewCoverage to false
	isThereNewCoverage = false;

	//reset isCovered
	for (int v = 0; v < numClients; v++){
		for (int t = 0; t < numDays; t++){
			if (v == depot){
				isCovered.at(v).at(t) = true;
			}
			else{
				isCovered.at(v).at(t) = false;
			}
		}
	}

	//reset existingTree	
	for (int s = 0; s < numDays; s++){
		for (int u = 0; u < numClients; u++){
			for (int v = 0; v < numClients; v++){
				existingTree[s][u][v] = false;
			}
		}
	}

	//reset visit
	for (int i = 0; i < numClients; i++) {
		for (int s = 0; s < numDays; s++) {
			if (i == depot){
				visit.at(i).at(s) = true;
			}
			else{
				visit.at(i).at(s) = false;
			}
		}
	}

	/*start progressively add sets and relax the density whenever no sets can be added until all demands are covered*/

	//relaxation factor for density
	double alpha = 1.1;
	fout << alpha << ' ';

	//initialize target density
	double targetDensity = densityUB;

	bool isThereUncovered = true;

	//number of rounds in greedy
	int greedyRounds = 0;

	//while there is uncovered demand
	while (isThereUncovered){
		greedyRounds++;

		//reset isThereNewCoverage to true to start the covering process
		isThereNewCoverage = true;
		//while isThereNewCoverage is true, add next set until the target density needs to increase
		while (isThereNewCoverage){
			//reset isThereNewCoverage to false bc it will only get updated to true when appropriate, but is assumed to be false when passed to addSet
			isThereNewCoverage = false;
			//apply addSet
			addSet(targetDensity, depot, numClients, numDays, visit, edgeWeights, existingTree, holding, isCovered, isThereNewCoverage, demand);			
		}
		
		//once there aren't anymore sets of appropriate density to add, udpate the target density to alpha * (target density)
		targetDensity = alpha * targetDensity;
		
		//reset isThereUncovered to false, and update it to true when appropriate
		isThereUncovered = false;

		//update isThereUncovered
		for (int u = 0; u < numClients; u++){
			for (int t = 0; t < numDays; t++){
				if (!isCovered.at(u).at(t)){
					isThereUncovered = true;
					DEBUG_MSG("there is an uncovered client" << endl);
					//only need to find one uncovered demand to know it's not feasible
					u = numClients;
					t = numDays;
				}
			}
		}
	}

	/*end of set cover*/



	//declare variables latestVisit for each demand point (i,t)
	vector<vector<int> > latestVisit = init2D<int>(numClients, numDays);

	//update latestVisit
	finalLatestVisit(numClients, numDays, visit, latestVisit);

	//compute costs
	routingCost = totalRoutingCost(numClients, numDays, edgeWeights, existingTree);
	holdingCost = totalHoldingCost(numClients, numDays, latestVisit, holding);
	DEBUG_MSG("routing cost is " << routingCost << endl);
	DEBUG_MSG("holding cost is " << holdingCost << endl);
	
	double totalCostGreedy = routingCost + holdingCost;

	/*compute total tour cost*/
	double greedyTourRoutingCost = 0;
	//create visitReversed reverse the indices of visit
	vector<vector<bool> > greedyVisitReversed = init2D<bool>(numDays, numClients);
	for (int v = 0; v < numClients; v++){
		for (int s = 0; s < numDays; s++){
			greedyVisitReversed.at(s).at(v) = visit.at(v).at(s);
		}
	}

	//keep track of costs
	double greedyTourToday;
	double greedyTreeToday;
	for (int today = 0; today < numDays; today++){
		//find tour cost of visit set on day today
		greedyTourToday = tourCostOfVisit(numClients, position, greedyVisitReversed.at(today), edgeWeights);
		DEBUG_MSG("the tour cost on day " << today << " is " << greedyTourToday << endl);
		greedyTourRoutingCost += greedyTourToday;

		//find tree cost of visit set on day 0
		greedyTreeToday = treeCost(numClients, edgeWeights, existingTree.at(today));
		DEBUG_MSG("the tree cost on day " << today << " is " << greedyTreeToday << endl);
	}
	/*end of finding total tour cost*/

	//total cost of tours-solution
	double totalCostGreedyToursSoln = greedyTourRoutingCost + holdingCost;

	auto elapsedGreedy = std::chrono::high_resolution_clock::now() - start_timer;
	long long microsecondsGreedy = std::chrono::duration_cast<std::chrono::microseconds>(elapsedGreedy).count();

	fout << greedyRounds << ' ';
	fout << microsecondsGreedy << ' ';
	fout << totalCostGreedy << ' ';
	fout << totalCostGreedyToursSoln << ' ';



	cout << "Time greedy took is " << microsecondsGreedy << " microsec" << endl;

	cout << "The total tour cost after greedy is " << totalCostGreedyToursSoln << "\n";

	/*pruning phase using best delete add cut off at 30 seconds*/
	DEBUG_MSG("start pruning phase " << endl);


	DEBUG_MSG("updated latestVisit according to greedy solution" << endl);

	//number of arcs is N*(N-1)
	int numArcs = numClients*(numClients - 1);

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
			vector<vector<int> > latestVisitBeforeDeleteDay = init2D<int>(numClients, numDays);
			for (int i = 0; i < numClients; i++){
				for (int t = 0; t < numDays; t++){
					//only consider visits before deleteDay
					for (int s = 0; s < deleteDay; s++){
						//make sure s is also at most t
						if (s <= t){
							if (visit[i][s]){
								latestVisitBeforeDeleteDay[i][t] = s;
			
							}
						}
					}
				}
			}

			//declare variable penalties			
			vector<double> penalties = init1D<double>(numClients);
			for (int v = 0; v < numClients; v++){
				penalties[v] = 0;
			}

			//penalties[v] = sum_{t in [t',t_hat]} h_v * d^v_{t} * (t' - s(v,t,t')), where s(v,t,t') is the latest visit to (v,t) BEFORE day t'
			for (int v = 0; v < numClients; v++){
				//only get penalized for removing those who were already visited on deleteDay
				if ((demand.at(v).at(deleteDay) > 0) && (visit.at(v).at(deleteDay))){
					int endDay = t_hatDelete(v, deleteDay, numDays, visit);

					for (int s = deleteDay; s < endDay + 1; s++){
						penalties[v] += unitHolding.at(v) * demand.at(v).at(s) * (deleteDay - latestVisitBeforeDeleteDay.at(v).at(s));

					}
				}

			}

			//find cost of the tree on day deleteDay
			double existingTreeCost = treeCost(numClients, edgeWeights, existingTree.at(deleteDay));

			//find the total penalty of removing the entire tree on deleteDay
			double extraHoldingCost = penaltyCostDelete(numClients, penalties, deleteDay, visit);

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


			//update improveDA
			improveDA = true;

			//update improvementRatioDA
			improveRatioDA = -bestDeleteOnlyImprovement / (routingCost + holdingCost);

			/*redo delete phase on bestDay*/

			//compute latest visit before deleteDay for each demand point
			vector<vector<int> > latestVisitBeforeDeleteDay = init2D<int>(numClients, numDays);
			for (int i = 0; i < numClients; i++){
				for (int t = 0; t < numDays; t++){
					//only consider visits before deleteDay
					for (int s = 0; s < bestDeleteOnlyDay; s++){
						//make sure s is also at most t
						if (s <= t){
							if (visit[i][s]){
								latestVisitBeforeDeleteDay[i][t] = s;

							}
						}
					}
				}
			}

			//declare variable penalties			
			vector<double> penalties = init1D<double>(numClients);
			for (int v = 0; v < numClients; v++){
				penalties[v] = 0;
			}

			//penalties[v] = sum_{t in [t',t_hat]} h_v * d^v_{t} * (t' - s(v,t,t')), where s(v,t,t') is the latest visit to (v,t) BEFORE day t'
			for (int v = 0; v < numClients; v++){
				//only get penalized for removing those who were already visited on deleteDay
				if ((demand.at(v).at(bestDeleteOnlyDay) > 0) && (visit.at(v).at(bestDeleteOnlyDay))){
					int endDay = t_hatDelete(v, bestDeleteOnlyDay, numDays, visit);

					for (int s = bestDeleteOnlyDay; s < endDay + 1; s++){
						penalties[v] += unitHolding.at(v) * demand.at(v).at(s) * (bestDeleteOnlyDay - latestVisitBeforeDeleteDay.at(v).at(s));

					}
				}

			}

			//find cost of the tree on day deleteDay
			double existingTreeCost = treeCost(numClients, edgeWeights, existingTree.at(bestDeleteOnlyDay));

			//find the total penalty of removing the entire tree on deleteDay
			double extraHoldingCost = penaltyCostDelete(numClients, penalties, bestDeleteOnlyDay, visit);

			//find the change in cost
			double difference = costChangeDelete(existingTreeCost, extraHoldingCost);


			/*end of delete phase*/

			//update visit
			for (int v = 0; v < numClients; v++){
				visit.at(v).at(bestDeleteOnlyDay) = false;

			}

			//update latestVisit
			for (int i = 0; i < numClients; i++){
				for (int t = 0; t < numDays; t++){
					for (int s = 0; s <= t; s++){
						if (visit[i][s]){
							latestVisit[i][t] = s;

						}
					}
				}
			}

			//update existingTree
			for (int u = 0; u < numClients; u++){
				for (int v = 0; v < numClients; v++){
					existingTree.at(bestDeleteOnlyDay).at(u).at(v) = false;

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
				for (int u = 0; u < numClients; u++){
					//count each arc
					for (int v = 0; v < numClients; v++){
						if (existingTree.at(s).at(u).at(v)){
							routingCostCheck += edgeWeights.at(u).at(v);
						}
					}
				}
			}
			DEBUG_MSG("the new routing cost from DELETE should be " << routingCostCheck << endl);

			//compute and output holding cost
			double holdingCostCheck = 0;
			for (int i = 0; i < numClients; i++){
				for (int t = 0; t < numDays; t++){

					holdingCostCheck += holding[i][latestVisit[i][t]][t];
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
				vector<double> penalties = init1D<double>(numClients);
				for (int v = 0; v < numClients; v++){
					penalties[v] = 0;
				}

				//penalties[v] = sum_{t \in [t',t_hat]} h_v * d^v_t * (t' - s(v,t))
				for (int v = 0; v < numClients; v++){
					//only get reward for those who were not already visited on addDay
					if ((demand.at(v).at(addDay) > 0) && (!visit.at(v).at(addDay))){
						int endDay = t_hatAdd(v, addDay, numDays, visit);
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
				double pcstTreeCost = treeCost(numClients, edgeWeights, pcstTree) - existingTreeCost;
				double pcstRewardCost = rewardCost(numClients, penalties, pcstSet);
				double difference = costChange(pcstTreeCost, pcstRewardCost);

				//update best day and best improvement
				if (difference < bestAddOnlyImprovement){
					bestAddOnlyImprovement = difference;
					bestAddOnlyDay = addDay;
				}
			}


			//if cost improves, update new IRP solution
			if (bestAddOnlyImprovement < -1e-10){


				//update improveDA
				improveDA = true;

				//upate improveRatioDA
				improveRatioDA = -bestAddOnlyImprovement / (routingCost + holdingCost);

				/*redo add phase on bestDay*/

				//declare variable penalties
				vector<double> penalties = init1D<double>(numClients);
				for (int v = 0; v < numClients; v++){
					penalties[v] = 0;
				}

				//penalties[v] = sum_{t \in [t',t_hat]} h_v * d^v_t * (t' - s(v,t))
				for (int v = 0; v < numClients; v++){
					//only get reward for those who were not already visited on addDay
					if ((demand.at(v).at(bestAddOnlyDay) > 0) && (!visit.at(v).at(bestAddOnlyDay))){
						int endDay = t_hatAdd(v, bestAddOnlyDay, numDays, visit);
						for (int s = bestAddOnlyDay; s < endDay + 1; s++){
							penalties[v] += unitHolding.at(v) * demand.at(v).at(s) * (bestAddOnlyDay - latestVisit.at(v).at(s));
						}
					}
				}


				//create input file for pcst instance
				createPCST(depot, numClients, numArcs, edgeWeights, existingTree.at(bestAddOnlyDay), penalties);

				//run pcst solver on pcst instance and create output file containing pcstSet, pcstTree
				vector<vector<bool> > pcstTree = init2D<bool>(numClients, numClients);
				for (int i = 0; i < numClients; i++){
					for (int j = 0; j < numClients; j++){
						pcstTree.at(i).at(j) = false;
					}
				}

				unordered_set<int> pcstSet;
				solvePCST(numClients, existingTree.at(bestAddOnlyDay), pcstTree, pcstSet);

				//compute pcstTreeCost and pcstPenaltyCost from pcstTree and pcstSet
				double existingTreeCost = treeCost(numClients, edgeWeights, existingTree.at(bestAddOnlyDay));
				double pcstTreeCost = treeCost(numClients, edgeWeights, pcstTree) - existingTreeCost;
				double pcstRewardCost = rewardCost(numClients, penalties, pcstSet);
				double difference = costChange(pcstTreeCost, pcstRewardCost);


				/*end of add phase*/

				//update visit
				for (int v : pcstSet){
					visit.at(v).at(bestAddOnlyDay) = true;

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
				existingTree.at(bestAddOnlyDay) = pcstTree;


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
					for (int u = 0; u < numClients; u++){
						//count each arc per direction
						for (int v = 0; v < numClients; v++){
							if (existingTree.at(s).at(u).at(v)){
								routingCostCheck += edgeWeights.at(u).at(v);
							}
						}
					}
				}
				DEBUG_MSG("the new routing cost from ADD should be " << routingCostCheck << endl);

				//compute and output holding cost
				double holdingCostCheck = 0;
				for (int i = 0; i < numClients; i++){
					for (int t = 0; t < numDays; t++){
						holdingCostCheck += holding[i][latestVisit[i][t]][t];
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
						vector<vector<int> > latestVisitBeforeFirstDeleteDay = init2D<int>(numClients, numDays);
						for (int i = 0; i < numClients; i++){
							for (int t = 0; t < numDays; t++){
								//only consider visits before deleteDay
								for (int s = 0; s < deleteDay; s++){
									//make sure s is also at most t
									if (s <= t){
										if (visit[i][s]){
											latestVisitBeforeFirstDeleteDay[i][t] = s;
											
										}
									}
								}
							}
						}


						//declare variable penalties			
						vector<double> penaltiesFirstDelete = init1D<double>(numClients);
						for (int v = 0; v < numClients; v++){
							penaltiesFirstDelete[v] = 0;
						}

						//penalties[v] = sum_{t in [t',t_hat]} h_v * d^v_{t} * (t' - s(v,t,t')), where s(v,t,t') is the latest visit to (v,t) BEFORE day t'
						for (int v = 0; v < numClients; v++){
							//only get penalized for removing those who were already visited on deleteDay
							if ((demand.at(v).at(deleteDay) > 0) && (visit.at(v).at(deleteDay))){
								int endDay = t_hatDelete(v, deleteDay, numDays, visit);

								for (int s = deleteDay; s < endDay + 1; s++){
									penaltiesFirstDelete[v] += unitHolding.at(v) * demand.at(v).at(s) * (deleteDay - latestVisitBeforeFirstDeleteDay.at(v).at(s));

								}
							}

						}

						//find cost of the tree on day deleteAddDay
						double existingTreeCostFirstDelete = treeCost(numClients, edgeWeights, existingTree.at(deleteDay));

						//find the total penalty of removing the entire tree on deleteAddDay
						double extraHoldingFirstCost = penaltyCostDelete(numClients, penaltiesFirstDelete, deleteDay, visit);

						//find the change in cost
						double differenceFirstDelete = costChangeDelete(existingTreeCostFirstDelete, extraHoldingFirstCost);


						/*
						end of delete phase, start of add phase
						*/

						//declare variable penalties
						vector<double> penaltiesAdd = init1D<double>(numClients);
						for (int v = 0; v < numClients; v++){
							penaltiesAdd[v] = 0;
						}

						//to calculate the correct t_hat, need to call it on the updated visit vector that has no visits on day deleteDay
						vector<vector<bool> > tempVisit = visit;
						for (int v = 0; v < numClients; v++){
							tempVisit.at(v).at(deleteDay) = false;
						}

						//to calculate the correct s(v,t), need to call it on the updated latest visits derived from tempVisit
						vector<vector<int> > tempLatestVisit = init2D<int>(numClients, numDays);
						//update tempLatestVisit
						for (int i = 0; i < numClients; i++){
							for (int t = 0; t < numDays; t++){
								for (int s = 0; s <= t; s++){
									if (tempVisit[i][s]){
										tempLatestVisit[i][t] = s;

									}
								}
							}
						}

						//penalties[v] = sum_{t \in [t',t_hat]} h_v * d^v_t * (t' - s(v,t))
						for (int v = 0; v < numClients; v++){
							//only get reward for those who were not already visited on addDay				
							if ((demand.at(v).at(addDay) > 0) && (!tempVisit.at(v).at(addDay))){
								int endDay = t_hatAdd(v, addDay, numDays, tempVisit);
								for (int s = addDay; s < endDay + 1; s++){
									//use tempLatestVisit which accounts for deletion on day deleteAddDay
									penaltiesAdd[v] += unitHolding.at(v) * demand.at(v).at(s) * (addDay - tempLatestVisit.at(v).at(s));
								}
							}
						}

						//use updated existing tree depending on what was deleted
						vector<vector<bool> > tempExistingTree = init2D<bool>(numClients, numClients);
						//initialize it to empty tree
						for (int u = 0; u < numClients; u++){
							for (int v = 0; v < numClients; v++){
								tempExistingTree.at(u).at(v) = false;
							}
						}
						//if add day is different from delete day, then tempExistingTree is not necessarily empty
						if (deleteDay != addDay){
							tempExistingTree = existingTree.at(addDay);
						}

						//create input file for pcst solver				
						createPCST(depot, numClients, numArcs, edgeWeights, tempExistingTree, penaltiesAdd);

						//run pcst solver on pcst instance and create output file containing pcstSet, pcstTree

						//declare pcstTree to eventually store the pcst tree found by the solver
						//declare an empty tree to put into the existingTree argument of solvePCST
						vector<vector<bool> > pcstTree = init2D<bool>(numClients, numClients);
						vector<vector<bool> > emptyTree = init2D<bool>(numClients, numClients);
						for (int i = 0; i < numClients; i++){
							for (int j = 0; j < numClients; j++){
								pcstTree[i][j] = false;
								emptyTree[i][j] = false;
							}
						}

						//declare the pcst set
						unordered_set<int> pcstSet;

						//update pcstTree
						solvePCST(numClients, tempExistingTree, pcstTree, pcstSet);

						//compute pcstTreeCost and pcstRewardCost from pcstTree and pcstSet
						double pcstTreeCost = treeCost(numClients, edgeWeights, pcstTree) - treeCost(numClients, edgeWeights, tempExistingTree);
						double pcstRewardCost = rewardCost(numClients, penaltiesAdd, pcstSet);
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
					vector<vector<int> > latestVisitBeforeFirstDeleteDay = init2D<int>(numClients, numDays);
					for (int i = 0; i < numClients; i++){
						for (int t = 0; t < numDays; t++){
							//only consider visits before deleteDay
							for (int s = 0; s < bestDeleteDay; s++){
								//make sure s is also at most t
								if (s <= t){
									if (visit[i][s]){
										latestVisitBeforeFirstDeleteDay[i][t] = s;

									}
								}
							}
						}
					}


					//declare variable penalties			
					vector<double> penaltiesFirstDelete = init1D<double>(numClients);
					for (int v = 0; v < numClients; v++){
						penaltiesFirstDelete[v] = 0;
					}

					//penalties[v] = sum_{t in [t',t_hat]} h_v * d^v_{t} * (t' - s(v,t,t')), where s(v,t,t') is the latest visit to (v,t) BEFORE day t'
					for (int v = 0; v < numClients; v++){
						//only get penalized for removing those who were already visited on deleteDay
						if ((demand.at(v).at(bestDeleteDay) > 0) && (visit.at(v).at(bestDeleteDay))){
							int endDay = t_hatDelete(v, bestDeleteDay, numDays, visit);

							for (int s = bestDeleteDay; s < endDay + 1; s++){
								penaltiesFirstDelete[v] += unitHolding.at(v) * demand.at(v).at(s) * (bestDeleteDay - latestVisitBeforeFirstDeleteDay.at(v).at(s));

							}
						}

					}

					//find cost of the tree on day deleteAddDay
					double existingTreeCostFirstDelete = treeCost(numClients, edgeWeights, existingTree.at(bestDeleteDay));

					//find the total penalty of removing the entire tree on deleteAddDay
					double extraHoldingFirstCost = penaltyCostDelete(numClients, penaltiesFirstDelete, bestDeleteDay, visit);

					//find the change in cost
					double differenceFirstDelete = costChangeDelete(existingTreeCostFirstDelete, extraHoldingFirstCost);

					/*
					end of delete phase, start of add phase
					*/

					//declare variable penalties
					vector<double> penaltiesAdd = init1D<double>(numClients);
					for (int v = 0; v < numClients; v++){
						penaltiesAdd[v] = 0;
					}

					//to calculate the correct t_hat, need to call it on the updated visit vector that has no visits on day deleteDay
					vector<vector<bool> > tempVisit = visit;
					for (int v = 0; v < numClients; v++){
						tempVisit.at(v).at(bestDeleteDay) = false;
					}

					//to calculate the correct s(v,t), need to call it on the updated latest visits derived from tempVisit
					vector<vector<int> > tempLatestVisit = init2D<int>(numClients, numDays);
					//update tempLatestVisit
					for (int i = 0; i < numClients; i++){
						for (int t = 0; t < numDays; t++){
							for (int s = 0; s <= t; s++){
								if (tempVisit[i][s]){
									tempLatestVisit[i][t] = s;
			
								}
							}
						}
					}

					//penalties[v] = sum_{t \in [t',t_hat]} h_v * d^v_t * (t' - s(v,t))
					for (int v = 0; v < numClients; v++){
						//only get reward for those who were not already visited on addDay
						if ((demand.at(v).at(bestAddDay) > 0) && (!tempVisit.at(v).at(bestAddDay))){
							int endDay = t_hatAdd(v, bestAddDay, numDays, tempVisit);
							for (int s = bestAddDay; s < endDay + 1; s++){
								//use the updated latest visits that pretends bestDay visits were deleted
								penaltiesAdd[v] += unitHolding.at(v) * demand.at(v).at(s) * (bestAddDay - tempLatestVisit.at(v).at(s));
							}
						}
					}

					//use updated existing tree depending on what was deleted
					vector<vector<bool> > tempExistingTree = init2D<bool>(numClients, numClients);
					//initialize it to empty tree
					for (int u = 0; u < numClients; u++){
						for (int v = 0; v < numClients; v++){
							tempExistingTree.at(u).at(v) = false;
						}
					}
					//if add day is different from delete day, then tempExistingTree is not necessarily empty
					if (bestDeleteDay != bestAddDay){
						tempExistingTree = existingTree.at(bestAddDay);
					}

					//create input file for pcst solver			
					createPCST(depot, numClients, numArcs, edgeWeights, tempExistingTree, penaltiesAdd);

					//run pcst solver on pcst instance and create output file containing pcstSet, pcstTree

					//declare pcstTree to eventually store the pcst tree found by the solver
					//declare an empty tree to put into the existingTree argument of solvePCST
					vector<vector<bool> > pcstTree = init2D<bool>(numClients, numClients);
					vector<vector<bool> > emptyTree = init2D<bool>(numClients, numClients);
					for (int i = 0; i < numClients; i++){
						for (int j = 0; j < numClients; j++){
							pcstTree[i][j] = false;
							emptyTree[i][j] = false;
						}
					}

					//declare the pcst set
					unordered_set<int> pcstSet;

					//update pcstTree
					solvePCST(numClients, tempExistingTree, pcstTree, pcstSet);


					//compute pcstTreeCost and pcstRewardCost from pcstTree and pcstSet
					double pcstTreeCost = treeCost(numClients, edgeWeights, pcstTree) - treeCost(numClients, edgeWeights, tempExistingTree);
					double pcstRewardCost = rewardCost(numClients, penaltiesAdd, pcstSet);
					double differenceAdd = costChange(pcstTreeCost, pcstRewardCost);

					double totalDifference = differenceFirstDelete + differenceAdd;

					/*end of delete-add phase*/

					/*delete phase updates*/
					//update visit on day bestDay
					for (int v = 0; v < numClients; v++){
						visit.at(v).at(bestDeleteDay) = false;
					}

					//update latestVisit
					for (int i = 0; i < numClients; i++){
						for (int t = 0; t < numDays; t++){
							for (int s = 0; s <= t; s++){
								if (visit[i][s]){
									latestVisit[i][t] = s;

								}
							}
						}
					}

					//update existingTree on bestDay
					for (int u = 0; u < numClients; u++){
						for (int v = 0; v < numClients; v++){
							existingTree.at(bestDeleteDay).at(u).at(v) = false;
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
						for (int u = 0; u < numClients; u++){
							//count arcs in both directions
							for (int v = 0; v < numClients; v++){
								if (existingTree.at(s).at(u).at(v)){
									routingCostCheck += edgeWeights.at(u).at(v);
								}
							}
						}
					}
					DEBUG_MSG("the new routing cost after delete phase of DA should be " << routingCostCheck << endl);

					//compute and output holding cost
					double holdingCostCheck = 0;
					for (int i = 0; i < numClients; i++){
						for (int t = 0; t < numDays; t++){

							holdingCostCheck += holding[i][latestVisit[i][t]][t];
						}
					}
					DEBUG_MSG("the new holding cost after delete phase of DA shoud be " << holdingCostCheck << endl);

					/*
					end of sanity check after delete phase
					*/

					/*add phase updates*/
					//update visit on day bestDay+1
					for (int v = 0; v < numClients; v++){
						unordered_set<int>::iterator clientIter = pcstSet.find(v);
						//add visit to v if v is in pcstSet
						if (clientIter != pcstSet.end()){
							visit.at(v).at(bestAddDay) = true;

						}

					}


					//update latestVisit
					for (int i = 0; i < numClients; i++){
						for (int t = 0; t < numDays; t++){
							for (int s = 0; s <= t; s++){
								if (visit[i][s]){
									latestVisit[i][t] = s;

								}
							}
						}
					}

					//update existingTree on bestDay-1
					for (int u = 0; u < numClients; u++){
						for (int v = 0; v < numClients; v++){
							if (pcstTree.at(u).at(v)){
								existingTree.at(bestAddDay).at(u).at(v) = true;

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
						for (int u = 0; u < numClients; u++){
							//count arcs in both directions
							for (int v = 0; v < numClients; v++){
								if (existingTree.at(s).at(u).at(v)){
									routingCostCheck += edgeWeights.at(u).at(v);
								}
							}
						}
					}
					DEBUG_MSG("the new routing cost after add phase of DA should be " << routingCostCheck << endl);

					//reset, compute and output holding cost
					holdingCostCheck = 0;
					for (int i = 0; i < numClients; i++){
						for (int t = 0; t < numDays; t++){

							holdingCostCheck += holding[i][latestVisit[i][t]][t];
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

	//compute total cost
	double totalCost = routingCost + holdingCost;

	/*compute total tour cost*/
	double tourRoutingCost = 0;
	//create visitReversed reverse the indices of visit
	vector<vector<bool> > visitReversed = init2D<bool>(numDays, numClients);
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
		tourToday = tourCostOfVisit(numClients, position, visitReversed.at(today), edgeWeights);

		tourRoutingCost += tourToday;

		//find tree cost of visit set on day 0
		treeToday = treeCost(numClients, edgeWeights, existingTree.at(today));

	}
	/*end of finding total tour cost*/

	//total cost of tours-solution
	double totalCostToursSoln = tourRoutingCost + holdingCost;

	/*
	* Stop the timer. We should do this before printing the final answer.
	*/
	auto elapsed = std::chrono::high_resolution_clock::now() - start_timer;
	long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();	
	cout << "number of rounds in pruning phase is " << numRoundsDA << endl;
	fout << numRoundsDA << ' ';

	cout << "Program execution time = " << microseconds << " microseconds\n";
	fout << microseconds << ' ';

	cout << "Final tree cost after pruning is " << totalCost << endl;
	fout << totalCost << ' ';

	cout << "Final tour cost after pruning is " << totalCostToursSoln << endl;
	fout << totalCostToursSoln << endl;

	fout.close();

	return 0;
}
