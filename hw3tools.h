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
    double distFromParent; // define the distance of a node from its parent node

    Node() // constructor
    {
        this->x = 0;
        this->y = 0;
        this->parentNode = NULL;
        this->distFromParent = 0;
    }
};

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
        this->t = 0;
        this->x = 0;
        this->y = 0;
        this->theta = 0;
        this->v = 0;
        this->w = 0;
        this->a = 0;
        this->gamma = 0;
    }
};

class Path
{
public:

    std::list<State> stateList;
    Path* parent;
    State intialPathState;
    double dist2randNode;
    bool inBounds;
    bool collision;

    Path(State initialState)
    {
        this->stateList = std::list<State>();
        this->parent = NULL;  
        this->intialPathState = initialState;
        this->stateList.push_back(initialState);
        this->dist2randNode = 0;
        this->inBounds = NULL;
        this->collision = NULL;
    }

    void euler(double epsilon, double distance, double dt)
    {
        double pi = 3.14159265359;
        double v_max = 5;
        double w_max = pi / 2;
        int iter = 0;
        double trajectory_distance = 0;
        double max_prop_dist;

        if (epsilon <= distance)
         {
            max_prop_dist = epsilon;
         }
        else
        {
            max_prop_dist = distance;
        }
        //std::cout<<"epsilon = "<<epsilon<<std::endl;
        //std::cout<<"distance = "<<distance<<std::endl;
        //  std::cout<<"max prop dist = "<<max_prop_dist<<std::endl;
        // Propagate trajectory distance of epsilon or distance from start to goal node, whichever is shorter
        while (trajectory_distance <= max_prop_dist && iter < 10000)
        {
            State currentState = this->stateList.back();
            State newState = State();

            // currentState->a = rand_a();
            // currentState->gamma = rand_gamma();

            newState.x = currentState.x + dt * (currentState.v * cos(currentState.theta));
            newState.y = currentState.y + dt * (currentState.v * sin(currentState.theta));
            newState.theta = currentState.theta + dt * currentState.w;
            newState.v = currentState.v + dt * currentState.a;
            newState.w = currentState.w + dt * currentState.gamma;
            newState.t = currentState.t + dt;

            this->stateList.push_back(newState);

            if(abs(newState.v) > v_max || abs(newState.w) > w_max)
            {
                this->inBounds = false;
            }
            else
            {
                this->inBounds = true;
            }

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

    /*
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

        }*/
};

class Tree
{
public:

    // list of paths to build tree
    std::list<Path *> pathList;

    // Tree constructor
    Tree()
    {
        this->pathList = std::list<Path*>();
    }

    // add path to list
    void add_path(Path *path)
    {
        pathList.push_back(path);
    }

    // list of nodes to build tree
    std::vector<Node *> node_list;

    // add node to list
    void add_node(Node *node)
    {
        node_list.push_back(node);
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

    // find nearest path to specified input node
    Path *nearest_Path(Node *newNode)
    {
        double min_distance = 1000000000000000; // intialize with a really big number ("infinity")
        Path *nearestPath = NULL;

        // loop through all nodes to find closest - this is inefficient but still works quickly, sorry :(
        //for (Path* const& i :pathList)
        for (std::list<Path*>::iterator it = this->pathList.begin(); it != this->pathList.end(); it++)
        {
           // double endpt_x = near
            double distance = sqrt(pow((*it)->stateList.back().x - newNode->x, 2) + pow((*it)->stateList.back().y - newNode->y, 2));
            if (distance < min_distance)
            {
                min_distance = distance;   
                nearestPath = *it;
            }
        }
        nearestPath->dist2randNode = min_distance;
        return nearestPath;
    }


    // save path (code modified from Dr. Otte)
    bool savePathToFile(std::string pathFile, Path *goalPath)
    {
        FILE *pFile = fopen(pathFile.c_str(), "w");
        if (pFile == NULL)
        {
            return false;
        }

        while (goalPath != NULL)
        {
            // write reversed trajectory to file
        for (std::list<State>::iterator it = goalPath->stateList.begin(); it != goalPath->stateList.end(); it++) {
            fprintf(pFile, "%f, %f, %f, %f, %f, %f, %f, %f\n", it->t, it->x, it->y, it->theta, it->v, it->w, it->a, it->gamma);
        }
            goalPath = goalPath->parent;
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

        for(std::list<Path*>::iterator it1 = this->pathList.begin(); it1 != this->pathList.end(); it1++)
        {
        for (std::list<State>::iterator it = (*it1)->stateList.begin(); it != (*it1)->stateList.end(); it++) {
            fprintf(pFile, "%f, %f, %f, %f, %f, %f, %f, %f\n", it->t, it->x, it->y, it->theta, it->v, it->w, it->a, it->gamma);
        }
        }
        fclose(pFile);
        printf("saved search tree in %s\n", searchTreeFile.c_str());

        return true;
    }
};

// Function prototypes

Node *sample(); // function to randomly sample C space

int collision_check(Path *,  double x_ob[], double y_ob[], double r_ob[], int num_ob);

int goal_check(Path *, double goal_x, double goal_y, double goal_r);
