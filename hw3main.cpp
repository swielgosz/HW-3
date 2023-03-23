#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include "hw3tools.h"
#include <string>
using namespace std;

#define PI 3.14159265359

int main()
{

    // Problem definitions
    double pi = PI;
    double v_max = 5;
    double w_max = pi / 2;
    double a_max = 2;
    double gamma_max = pi / 2;
    double delta = .1;

    double start_x = 30;
    double start_y = -35;
    double start_theta = pi / 2;
    double goal_x = 0;
    double goal_y = 0;
    double goal_r = 5;
    double epsilon = 10;
    
    // Seed random number generator
    srand(time(NULL)); 

    // Reading in obstacles
    ifstream filestream("obstacles.txt");
    if (!filestream.is_open())
    {
        cout << "File failed to open" << endl;
        return 0;
    }

    int i = 0;
    double x_ob[51], y_ob[51], r_ob[51];
    string line;
    string X, Y, R;
    while (getline(filestream, line))
    {
        stringstream ss(line);
        getline(ss, X, ',');
        x_ob[i] = stod(X);
        getline(ss, Y, ',');
        y_ob[i] = stod(Y);
        getline(ss, R, ',');
        r_ob[i] = stod(R);
        i++;
    }
    int num_ob = i;

    // Reading in robot points
    ifstream filestream2("H3_robot.txt");
    if (!filestream.is_open())
    {
        cout << "File failed to open" << endl;
        return 0;
    }

    i = 0;
    string line2, X2, Y2;
    double x_robot[37], y_robot[37];
    while (getline(filestream2, line2))
    {
        stringstream ss(line2);
        getline(ss, X2, ',');
        x_robot[i] = stod(X2);
        getline(ss, Y2, ',');
        y_robot[i] = stod(Y2);
        cout<< "["<<x_robot[i]<<","<<y_robot[i]<<"]"<<endl;
        i++;
    }
    int num_robot_pt = i;

    State* startState = new State();
    startState->x = start_x;
    startState->y = start_y;
    startState->theta = start_theta;
    startState->v = 0;
    startState->w = 0;
    startState->a = 1;
    startState->gamma = .2;
    
    Path initialTrajectory = Path(startState);

    Tree T = Tree();
    T.add_path(&initialTrajectory);

    // Loop through forward and steering accelerations and create trajectories
    double da = 0.1; // forward acceleration step
    double dgamma = pi/10; // steering acceration step
    int idx1 = 0;
    int idx2 = 0;
    int count = 0;
    std::list<State*> robotStates;
    std::list<Path> recordPath;

    for (double a = -a_max; a <= a_max; a += da)
    {
        for (double gamma = -gamma_max; gamma <= gamma_max; gamma += dgamma )
        {
            State* s = new State();
            s->t = 0;
            s->x = 0;
            s->y = 0;
            s->theta = 0;
            s->v = 0;
            s->w = 0;
            s->a = a;
            s->gamma = gamma;
            robotStates.push_back(s);

            Path path = Path(s);
            path.euler(epsilon,0.1);
            recordPath.push_back(path);
           // P.euler(epsilon,0.1);
            path.saveTrajectoryToFile("./output/hw3/tests/trajectory_" + std::to_string(count) + ".csv");
            count++;

        }
    }


/*
    // Start building tree
    Node *startNode = new Node();
    startNode->x = start_x;
    startNode->y = start_y;

    Tree T = Tree();
    T.add_node(startNode);

    Node *goalNode = NULL;
*/


    

    return 0;
}

///////////////////////////////// FUNCTIONS //////////////////////////////////////

// random forward acceleration
double rand_acceleration()
{
    double Max = 2.00, Min = -2.00;
    double rand_a = (double(rand()) / double(RAND_MAX)) * (Max - Min) + Min;
    return rand_a;
} 

///////////////////////////////////////////////////////////////////////////////////

// random steering acceleration
double rand_gamma() 
{
    double Max = PI/2, Min = -PI/2;
    double rand_gamma = (double(rand()) / double(RAND_MAX)) * (Max - Min) + Min;
    return rand_gamma;
}

///////////////////////////////////////////////////////////////////////////////////

// Randomly sample from configuration space that is [-50 50] x [-50 50]
Node *sample()
{
    double Max = 50.00, Min = -50.00;

    double randx = (double(rand()) / double(RAND_MAX)) * (Max - Min) + Min;
    double randy = (double(rand()) / double(RAND_MAX)) * (Max - Min) + Min;
    Node *newNode = new Node();
    newNode->x = randx;
    newNode->y = randy;
    return newNode;
}

///////////////////////////////////////////////////////////////////////////////////

// Find distance between two nodes
double euclidean(Node *Node1, Node *Node2)
{
    return (sqrt(pow(Node1->x - Node2->x, 2) + pow(Node1->y - Node2->y, 2)));
}

///////////////////////////////////////////////////////////////////////////////////

// Function to find new node that is in direction of random node from the closest node to it
Node *eulerGoal(Node *closestNode, Node *randNode, double epsilon)
{
    double x1 = closestNode->x;
    double y1 = closestNode->y;
    double x2 = randNode->x;
    double y2 = randNode->y;
    double multiplier;

    double distance = (sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)));

    // travel distance of node 1 to node 2, or distance epsilon, whichever is shorter
    if (distance < epsilon)
    {
        multiplier = distance;
    }
    else
    {
        multiplier = epsilon;
    }

    // components of unit vector giving direction from (x1,y1) to (x2,y2)
    double unitvec_x = (x2 - x1) / (sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)));
    double unitvec_y = (y2 - y1) / (sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2)));

    // define new node in direction of random node from closest node
    Node *newNode = new Node();
    newNode->x = x1 + multiplier * unitvec_x;
    newNode->y = y1 + multiplier * unitvec_y;
    newNode->parentNode = closestNode;
    return newNode;
}

///////////////////////////////////////////////////////////////////////////////////
/*
State *euler(State *currentState, double epsilon, double dt)
{
    int iter = 0;
    double trajectory_distance = 0;

    while (trajectory_distance <= epsilon && iter < 10000)
    {
        State* newState = new State();

        // currentState->a = rand_a();
        // currentState->gamma = rand_gamma();

        newState->x = currentState->x + dt * (currentState->v * cos(currentState->theta));
        newState->y = currentState->y + dt * (currentState->v * sin(currentState->theta));
        newState->theta = currentState->theta + dt * currentState->w;
        newState->v = currentState->v + dt * currentState->a;
        newState->w = currentState->w + dt * currentState->gamma;
        newState->t = currentState->t + dt;

        this->stateList.push_back(newState);

        trajectory_distance = trajectory_distance + sqrt(pow((newState->x - currentState->x), 2) + pow((newState->y - currentState->y), 2));
        iter++;
    }
}
/* Euler integration to get trajectory using car dynamics
Node *euler(Node *closestNode, Node *eulerGoalNode, double delta, double epsilon)
{

}
*/