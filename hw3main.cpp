#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include "hw3tools.h"
#include <string>
using namespace std;

int main()
{

    // Problem definitions
    double pi = 3.14159265359;
    double v_max = 5;
    double w_max = pi / 2;
    double a_max = 2;
    double gamma_max = pi / 2;
    double delta = .1;

    /*double start_x = 30;
    double start_y = -35;
    double start_theta = pi / 2;
    double goal_x = 0;
    double goal_y = 0;
    double goal_r = 5;
    double epsilon = 1;*/
    double start_x = -30;
    double start_y = -35;
    double start_theta = pi/4;
    double goal_x = -35;
    double goal_y = 30;
    double goal_r = 5;
    double epsilon = 2;

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
        i++;
    }
    int num_robot_pt = i;

    State startState = State();
    startState.x = start_x;
    startState.y = start_y;
    startState.theta = start_theta;
    startState.v = 0;
    startState.w = 0;
    startState.a = 0;
    startState.gamma = 0;

    Path initialPath = Path(startState);

    Tree T = Tree();
    T.add_path(&initialPath);

    int goal_reached = 0, iter = 0, itermax = 10000;
    Path *goalPath = NULL;

    while (goal_reached != 1 && iter < itermax)
    {
        // select random node from C space
        Node *randNode = sample();

        // find which path in the tree has its end point closest to the random node
        Path *closestPath2rand = T.nearest_Path(randNode);

        double s2gdist = closestPath2rand->dist2randNode;
        //cout<<"dist2randnode"<<s2gdist<<endl;
        // Loop through forward and steering accelerations and create trajectories
        double da = 0.5;        // forward acceleration step
        double dgamma = pi / 4; // steering acceration step
        int idx1 = 0;
        int idx2 = 0;
        int count = 0;
        std::list<State> robotStates;
        std::list<Path *> robotPaths;

        // Find paths over a range of forward and steering accelerations from a specified start node
        double min_dist_2_rand = 1000000000; // initialize distance from end of path to desired random node
        Path *bestPath = NULL;
        for (double a_test = -a_max; a_test <= a_max; a_test += da)
        {
            for (double gamma_test = -gamma_max; gamma_test <= gamma_max; gamma_test += dgamma)
            {
                State newState = State();
                newState.t = closestPath2rand->stateList.back().t;
                newState.x = closestPath2rand->stateList.back().x;
                newState.y = closestPath2rand->stateList.back().y;
                newState.theta = closestPath2rand->stateList.back().theta;
                newState.v = closestPath2rand->stateList.back().v;
                newState.w = closestPath2rand->stateList.back().w;
                newState.a = a_test;
                newState.gamma = gamma_test;
                robotStates.push_back(newState);

                Path *path = new Path(newState);

                path->euler(epsilon, s2gdist, 0.1);
                robotPaths.push_back(path);
                
                // Record which path ends closest to desired random node and falls in acceptable range of accelerations
                if (path->inBounds)
                {
                    double dist_2_rand = sqrt(pow(path->stateList.back().x - randNode->x, 2) + pow(path->stateList.back().y - randNode->y, 2));
                    if (dist_2_rand < min_dist_2_rand)
                    {
                        min_dist_2_rand = dist_2_rand;
                        bestPath = path;
                    }
                }

                
                count++;
            }
        }
        //bestPath->saveTrajectoryToFile("./test/outputbestPath.csv");
        // Check if best path is in collision
        int collision = collision_check(bestPath, x_ob, y_ob, r_ob, num_ob);

        if (collision == 0)
        {
            T.add_path(bestPath);
            bestPath->parent = closestPath2rand;

            goal_reached = goal_check(bestPath, goal_x, goal_y, goal_r);
            if (goal_reached == 1)
            {
                cout << "Goal region reached." << endl;
                goalPath = bestPath;
                break;
            }
        }
        cout<<iter<<endl;
        iter++;
    }

    if (iter >= itermax)
    {
        cout << "Goal region is not reached after " << itermax << " iterations" << endl;
    }
    // Determine which of the above paths ends closest to the desired random node

    /*
        // Start building tree
        Node *startNode = new Node();
        startNode->x = start_x;
        startNode->y = start_y;

        Tree T = Tree();
        T.add_node(startNode);

        Node *goalNode = NULL;
    */

   int run = 1;
   T.savePathToFile("output_path_" + to_string(run+1) + ".txt",goalPath);
   T.saveSearchTreeToFile("search_tree_" + to_string(run+1) + ".txt");


    return 0;
}

///////////////////////////////// FUNCTIONS //////////////////////////////////////

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
    newNode->distFromParent = multiplier;
    return newNode;
}

///////////////////////////////////////////////////////////////////////////////////

/*int collision_check(Path *bestPath, double x_ob[], double y_ob[], double r_ob[], int num_ob)
{
    double pi = 3.14159265359;
    double r_robot = 1; // max radius of robot, model robot as a circle
    int collision;
    int point_check;
    int robot_check;
    double distance;

    // check if first or last point of curve are in collision with object
    for (int j = 0; j < num_ob; j++)
        {
            // distance from center of object to new node
            distance = sqrt(pow(x_ob[j] - bestPath->stateList.front().x, 2) + pow(y_ob[j] - bestPath->stateList.front().y, 2));

            // if distance is less than radius, the point is in an object
            if (distance <= r_ob[j])
            {
                point_check = 1;
                break;
            }
            else
            {
                point_check = 0;
            }
            
        }

        for (int j = 0; j < num_ob; j++)
        {
            // distance from center of object to new node
            distance = sqrt(pow(x_ob[j] - bestPath->stateList.back().x, 2) + pow(y_ob[j] - bestPath->stateList.back().y, 2));

            // if distance is less than radius, the point is in an object
            if (distance <= r_ob[j])
            {
                point_check = 1;
                break;
            }
            else
            {
                point_check = 0;
            }
        }


    // loop through the rest of the curve at increments of .5 arc length
    double new_delta = 0;

    for (auto it = bestPath->stateList.begin(); it != bestPath->stateList.end(); it++)
    {
        
        new_delta = new_delta + sqrt(pow(it->x - std::next(it)->x, 2) + pow(it->y - std::next(it)->y, 2));
        //cout<<"nd="<<new_delta<<endl;
        if(new_delta >= 0.5)
        {
            new_delta = 0;
          //  cout<<"new_delta reset"<<new_delta<<endl;
        for (int j = 0; j < num_ob; j++)
        {
            // distance from center of object to new node
            double distance = sqrt(pow(x_ob[j] - it->x, 2) + pow(y_ob[j] - it->y, 2));

            // if distance is less than radius, the point is in an object
            if (distance <= r_ob[j])
            {
                point_check = 1;
                break;
            }
            else
            {
                point_check = 0;
            }
           // cout<<"j = "<<j<<endl;
            
        }
        if (point_check == 1)
        {
            break;
        }

        // check if any point at a radius of 1 (i.e. max radius of the robot) away from a point is in collision with an object
        for (int j = 0; j < num_ob; j++)
        {
            for (float k = 0; k < 2 * pi; k += pi/20)
            {
                // distance from center of object to new node
                double distance = sqrt(pow(x_ob[j] - (it->x + r_robot * cos(k)), 2) + pow(y_ob[j] - (it->y + r_robot * sin(k)), 2));

                // if distance is less than radius of object + radius of robot, the point is in an object
                if (distance <= (r_ob[j] + r_robot))
                {
                    robot_check = 1;
                    break;
                }
                else
                {
                    robot_check = 0;
                }
            }
            if (robot_check == 1)
            {
                break;
            }
        }
        

        if (point_check == 1 || robot_check == 1)
        {
            break;
        }
        }
        if (point_check == 1 || robot_check == 1)
        {
            break;
        }
    }

    if (point_check == 1 || robot_check == 1)
    {
        collision = 1;
    }
    else
    {
        collision = 0;
    }

    return collision;
}
*/
int collision_check(Path *bestPath, double x_ob[], double y_ob[], double r_ob[], int num_ob)
{
    double pi = 3.14159265359;
    double r_robot = 1; // max radius of robot, model robot as a circle
    int collision;
    int point_check;
    int robot_check;
    double distance;

    // loop through all points of the curve
    for (auto i = bestPath->stateList.begin(); i != bestPath->stateList.end(); i++)
    {
        for (int j = 0; j < num_ob; j++)
        {
            // distance from center of object to new node
            distance = sqrt(pow(x_ob[j] - i->x, 2) + pow(y_ob[j] - i->y, 2));

            // if distance is less than radius, the point is in an object
            if (distance <= r_ob[j])
            {
                point_check = 1;
            }
            else
            {
                point_check = 0;
            }
        }

        // check if any point at a radius of 1 (i.e. max radius of the robot) away from a point is in collision with an object
        for (int j = 0; j < num_ob; j++)
        {
            for (double k = 0; k < 2 * pi; k += pi / 50)
            {
                // distance from center of object to new node
                distance = sqrt(pow(x_ob[j] - (i->x + r_robot * cos(k)), 2) + pow(y_ob[j] - (i->y + r_robot * sin(k)), 2));

                // if distance is less than radius of object + radius of robot, the point is in an object
                if (distance <= (r_ob[j] + r_robot))
                {
                    robot_check = 1;
                }
                else
                {
                    robot_check = 0;
                }
            }
        }

        if (point_check == 1 || robot_check == 1)
        {
            break;
        }
    }

    if (point_check == 1 || robot_check == 1)
    {
        collision = 1;
    }
    else
    {
        collision = 0;
    }

    return collision;
}
///////////////////////////////////////////////////////////////////////////////////

// Check if point is in goal region
int goal_check(Path *bestPath, double goal_x, double goal_y, double goal_r)
{
    int goal_check;
    double distance = sqrt(pow(goal_x - bestPath->stateList.back().x, 2) + pow(goal_y - bestPath->stateList.back().y, 2));
    if (distance <= goal_r)
    {
        goal_check = 1;
    }
    else
    {
        goal_check = 0;
    }
    return goal_check;
}