#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <list>
#include <vector>
#include <string>

class Node
{
public:
    double x; // physical x location
    double y; // physical y location

    Node *parentNode; // used for graph search, this is a pointer to this node's parent

    Node() // constructor
    {
        this->x = 0;
        this->y = 0;
        this->parentNode = NULL;
    }
};

/*
class State
{
public:
    double x;
    double y;
    double theta;
    double v;
    double w;
    double a;
    double gamma;
    double t;

    State() // constructor
    {
        this->x = 0;
        this->y = 0;
        this->theta = 0;
        this->v = 0;
        this->w = 0;
        this->a = 0;
        this->gamma = 0;
        this->t = 0;
    }
};
*/


struct State
{
    double x;
    double y;
    double theta;
    double v;
    double w;
    double a;
    double gamma;
    double t;
};

class Trajectory
{
public:

    State intialState;
    Trajectory* parentTrajectory;
    std::list<State> stateList; // list of states to report results

    Trajectory(State initialConditions) // constructor
    {
        this->intialState = initialConditions;
        this->stateList = std::list<State>();
        this->stateList.push_back(initialConditions);
        this->parentTrajectory = NULL;
    }    

    void euler(double epsilon, double dt, Node *eulerGoal)
    {

        int iter = 0;
        double trajectory_distance = 0;

        while (trajectory_distance <= epsilon && iter < 10000)
        {
            State currentState = this->stateList.back();
            State newState;

           // currentState.a = rand_a();
            //currentState.gamma = rand_gamma();

            newState.x = currentState.x + dt * (currentState.v * cos(currentState.theta));
            newState.y = currentState.y + dt * (currentState.v * sin(currentState.theta));
            newState.theta = currentState.theta + dt * currentState.w;
            newState.v = currentState.v + dt * currentState.a;
            newState.w = currentState.w + dt * currentState.gamma;
            newState.t = currentState.t + dt;

            this->stateList.push_back(newState);

            trajectory_distance = trajectory_distance + sqrt(pow((newState.x - currentState.x), 2) + pow((newState.y - currentState.y), 2));
            iter++;
        }

    }

    bool saveTrajectoryToFile(std::string TrajectoryFile)
    {
        FILE *pFile = fopen(TrajectoryFile.c_str(), "w");
        if (pFile == NULL)
        {
            return false;
        }

        int vecSize = stateList.size();
        for (State const& i : stateList)
        {
            fprintf(pFile, "%f, %f, %f, %f, %f, %f\n",
                    i.t, i.x, i.y, i.theta, i.v, i.w, i.a, i.gamma);
        }
        fclose(pFile);
        printf("saved search tree in %s\n", TrajectoryFile.c_str());

        return true;
    }
};

class Tree
{
public:
    // list of nodes to build tree
    std::vector<Node *> node_list;

    // add node to list
    void add_node(Node *node)
    {
        node_list.push_back(node);
    }

    // list of trajectories
    std::list<Trajectory*> trajectoryList;

    void add_trajectory(Trajectory *trajectory) 
    {
        trajectoryList.push_back(trajectory);
    }

    Tree()
    {
        this->trajectoryList = std::list<Trajectory*>();
    }

    // find nearest node to specified input node
    Node *nearest_Node(Node *newNode)
    {
        double min_distance = 1000000000000000; // intialize with a really big number ("infinity")
        unsigned int vecSize = node_list.size();
        Node *nearestNode = NULL;

        // loop through all nodes to find closest - this is inefficient but still works quickly, sorry :(
        for (unsigned int i = 0; i < vecSize; i++)
        {
            Node *currentNode = node_list[i];
            double distance = sqrt(pow(currentNode->x - newNode->x, 2) + pow(currentNode->y - newNode->y, 2));
            if (distance < min_distance)
            {
                min_distance = distance;
                nearestNode = node_list[i];
            }
        }
        return nearestNode;
    }

    // save path (code modified from Dr. Otte)
    bool savePathToFile(std::string pathFile, Node *goalNode)
    {
        FILE *pFile = fopen(pathFile.c_str(), "w");
        if (pFile == NULL)
        {
            return false;
        }

        Node *thisNode = goalNode;
        while (thisNode != NULL)
        {
            // format is x, y
            fprintf(pFile, "%f, %f\n", thisNode->x, thisNode->y);
            thisNode = thisNode->parentNode;
        }

        fclose(pFile);
        printf("saved path in %s\n", pathFile.c_str());

        return true;
    }

    // Save tree (code modified from Dr. Otte)
    bool saveSearchTreeToFile(std::string searchTreeFile)
    {
        FILE *pFile = fopen(searchTreeFile.c_str(), "w");
        if (pFile == NULL)
        {
            return false;
        }

        int trajectorySize = trajectoryList.size();
        for (std::list<Trajectory*>::iterator i = this->trajectoryList.begin(); i != this->trajectoryList.end(); ++ i)
        {
            for (std::list<State>::iterator i2 = (*i)->stateList.begin(); i2 != (*i)->stateList.end(); ++ i2) 
            {

                // format is x1, y1, id2, x2, y2
                fprintf(pFile, "%f, %f, %f, %f, %f, %f\n",
                        i2->t, i2->x, i2->y,i2->theta, i2->v, i2->w, i2->a, i2->gamma);
            }
        }
        fclose(pFile);
        printf("saved search tree in %s\n", searchTreeFile.c_str());

        return true;
    }
};
// Function prototypes

double rand_a(); // random forward acceleration

double rand_gamma(); // random steering acceleration

Node *sample(); // function to randomly sample C space

int collision_check(Node *, Node *, double x_ob[], double y_ob[], double r_ob[], int num_ob);

int goal_check(Node *, double goal_x, double goal_y, double goal_r);

Node *eulerGoal(Node *, Node*, double);

// Node* euler(Node*, Node*, double);

double euclidean(Node *Node1, Node *Node2);