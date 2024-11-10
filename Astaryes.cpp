#include <iostream>
#include <vector>
#include <cmath>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <ctime>
#include <queue>

using namespace std;

const int SIZE = 4;
const vector<vector<int>> goalState = {{0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11}, {12, 13, 14, 15}};

// Enum to represent directions
enum Direction { UP, DOWN, LEFT, RIGHT };
const vector<string> directionNames = {"UP", "DOWN", "LEFT", "RIGHT"};

// Function to find the position of a tile in the goal state
void blankPosition(int tile, const vector<vector<int>>& goal, int& row, int& col) {
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            if (goal[i][j] == tile) {
                row = i;
                col = j;
                return;
            }
        }
    }
}

// Function to calculate the Manhattan distance heuristic
int manhattanDistance(const vector<vector<int>>& startState) {
    int totalDistance = 0;
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            int tile = startState[i][j];
            if (tile != 0) { // Assuming 0 is the empty space
                int goalRow, goalCol;
                blankPosition(tile, goalState, goalRow, goalCol);
                totalDistance += abs(i - goalRow) + abs(j - goalCol);
            }
        }
    }
    return totalDistance;
}

// Function to generate successor states
vector<vector<vector<int>>> generateSuccessors(const vector<vector<int>>& state) {
    vector<vector<vector<int>>> moves;
    int zeroRow, zeroCol;

    // Find the position of the empty space (0)
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            if (state[i][j] == 0) {
                zeroRow = i;
                zeroCol = j;
                break;
            }
        }
    }

    // Possible directions to move the empty space
    vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    for (size_t i = 0; i < directions.size(); ++i) {
        auto dir = directions[i];
        int newRow = zeroRow + dir.first;
        int newCol = zeroCol + dir.second;

        // Check if the new position is within bounds
        if (newRow >= 0 && newRow < SIZE && newCol >= 0 && newCol < SIZE) {
            vector<vector<int>> newState = state;
            // Swap the empty space with the adjacent tile
            swap(newState[zeroRow][zeroCol], newState[newRow][newCol]);
            moves.push_back(newState);
        }
    }
    return moves;
}

// Function to print the solution path as directions
void printSolution(const vector<string>& directions) {
    if (directions.empty()) {
        cout << "No moves made." << endl;
        return;
    }
    
    for (size_t i = 0; i < directions.size(); ++i) {
        cout << directions[i];
        if (i < directions.size() - 1) {
            cout << " -> ";
        }
    }
    cout << endl; // End the line after printing all directions
}

// Function to convert state to string for hashing
string stateToString(const vector<vector<int>>& state) {
    string result;
    for (const auto& row : state) {
        for (int tile : row) {
            result += to_string(tile) + ",";
        }
    }
    return result;
}

// A* Search algorithm implementation
bool AstarSearch(const vector<vector<int>>& startState, vector<string>& solutionPath, int& expandedNodes) {
    unordered_set<string> visited;
    unordered_map<string, int> gCost;
    unordered_map<string, pair<vector<vector<int>>, string>> cameFrom; // To track the path
    
    using StateCostPair = pair<int, vector<vector<int>>>;
    priority_queue<StateCostPair, vector<StateCostPair>, greater<StateCostPair>> pq;

    int startCost = manhattanDistance(startState);
    pq.push({startCost, startState});
    gCost[stateToString(startState)] = 0;

    while (!pq.empty()) {
        auto current = pq.top();
        pq.pop();
        vector<vector<int>> currentState = current.second;

        expandedNodes++; // Increment the expanded nodes counter

        if (currentState == goalState) {
            // Reconstruct the path
            string currentKey = stateToString(currentState);
            while (cameFrom.find(currentKey) != cameFrom.end()) {
                solutionPath.push_back(cameFrom[currentKey].second);
                currentKey = stateToString(cameFrom[currentKey].first);
            }
            reverse(solutionPath.begin(), solutionPath.end());
            return true;
        }

        visited.insert(stateToString(currentState));

        // Find the position of the empty space (0) in the current state using blankPosition
        int zeroRow, zeroCol;
        blankPosition(0, currentState, zeroRow, zeroCol);

        for (const auto& successor : generateSuccessors(currentState)) {
            string successorKey = stateToString(successor);
            int newCost = gCost[stateToString(currentState)] + 1; // Assuming each move has a cost of 1

            if (visited.find(successorKey) == visited.end() || newCost < gCost[successorKey]) {
                gCost[successorKey] = newCost;
                int fCost = newCost + manhattanDistance(successor);
                pq.push({fCost, successor});

                // Use blankPosition to find the new position of the empty space in the successor state
                int newZeroRow, newZeroCol;
                blankPosition(0, successor, newZeroRow, newZeroCol);

                // Check which direction the empty space moved
                if (newZeroRow == zeroRow - 1 && newZeroCol == zeroCol) { // Moved UP
                    cameFrom[successorKey] = {currentState, "UP"};
                } else if (newZeroRow == zeroRow + 1 && newZeroCol == zeroCol) { // Moved DOWN
                    cameFrom[successorKey] = {currentState, "DOWN"};
                } else if (newZeroRow == zeroRow && newZeroCol == zeroCol - 1) { // Moved LEFT
                    cameFrom[successorKey] = {currentState, "LEFT"};
                } else if (newZeroRow == zeroRow && newZeroCol == zeroCol + 1) { // Moved RIGHT
                    cameFrom[successorKey] = {currentState, "RIGHT"};
                }
            }
        }
    }
    return false; // No solution found
}

int main() {
    vector<vector<int>> startState(SIZE, vector<int>(SIZE));
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            cout << "Enter the starting state of the 15-puzzle (0 for the blank space) at position ["<< i <<"] ["<< j << "]: ";
            cin >> startState[i][j];
        }
    }
    clock_t startTime = clock();
    vector<string> solutionPath; // Change to store state and direction
    int expandedNodes = 0; // Counter for expanded nodes
    if (AstarSearch(startState, solutionPath, expandedNodes)) {
        cout << "Solution found!" << endl;
        printSolution(solutionPath);
    } else {
        cout << "No solution exists." << endl;
    }
    clock_t endTime = clock();
    double elapsedTime = double(endTime - startTime) / CLOCKS_PER_SEC; // Calculate elapsed time in seconds
    cout << "Number of expanded nodes: " << expandedNodes << endl; // Print the number of expanded nodes
    cout << "Elapsed time: " << elapsedTime << " seconds" << endl;
    char endProgram;
    cout << "Press any key to exit...";
    cin >> endProgram;
    if(endProgram == 'y')
    {
        return 0;
    }
    return 0;

}