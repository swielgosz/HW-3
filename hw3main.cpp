#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#include "hw2tools.h"
#include <string>
using namespace std;

int main()
{
    // Problem definitions
    double start_x_arr[5] = {0, 27, 45, -16, 39};
    double start_y_arr[5] = {0, 30, -45, 10, 5};
    double goal_x_arr[5] = {-38, -48, -45, 18, 38};
    double goal_y_arr[5] = {20, 20, 45, -45, -8};
    double goal_r_arr[5] = {10, 10, 15, 5, 3};
    double epsilon_arr[5] = {10, 5, 5, 2, 1};

    // Loop through each problem
    for (int run = 0; run < 5; run++)
    {
        double start_x = start_x_arr[run];
        double start_y = start_y_arr[run];
        double goal_x = goal_x_arr[run];
        double goal_y = goal_y_arr[run];
        double goal_r = goal_r_arr[run];
        double epsilon = epsilon_arr[run];

        srand(time(NULL)); // seed random number generator

        // Reading in obstacles
        ifstream filestream("obstacles.txt");
        if (!filestream.is_open())
        {
            cout << "File failed to open" << endl;
            return 0;
        }

        int i = 0;
        double x_ob[20], y_ob[20], r_ob[20];
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

        // Start building tree
        Node *startNode = new Node();
        startNode->x = start_x;
        startNode->y = start_y;

        Tree T = Tree();
        T.add_node(startNode);

        Node *goalNode = NULL;
        
        // initialize variables
        int goal_reached = 0, iter = 0;

        // check to see if we start in goal region
        goal_reached = goal_check(startNode, goal_x, goal_y, goal_r);
        if (goal_reached == 1)
        {
            cout<<"Start point is already in goal region"<<endl;
        }

        // Implement RTT
        int itermax = 10000;
        while (goal_reached != 1 && iter < itermax)
        {
            // select random node from C space
            Node *randNode = sample();

            // find closest node in tree to the random node
            Node *closestNode = T.nearest_Node(randNode);

            // find node in direction of random node 1 epsilon away (or shorter if appropriate)
            Node *solve2BPNode = solve2BP(closestNode, randNode, epsilon);

            // check if this new node is in collision with an obstacle
            int collision = collision_check(solve2BPNode, closestNode, x_ob, y_ob, r_ob, num_ob);

            // if new node is not in collision, check if it is in the goal region. If not, add it to the search tree
            if (collision == 0)
            {
                T.add_node(solve2BPNode);
                goal_reached = goal_check(solve2BPNode, goal_x, goal_y, goal_r);
                if (goal_reached == 1)
                {
                    cout << "Goal region reached." << endl;
                    goalNode = solve2BPNode;
                    break;
                }
            }

            iter++;
        }

        if (iter >= itermax)
        {
            cout<<"Goal region is not reached after "<<itermax<<" iterations"<<endl;
        }

        // save files that contain the search tree and best path 
        T.savePathToFile("output_path_" + to_string(run+1) + ".txt", goalNode);
        T.saveSearchTreeToFile("search_tree_" + to_string(run+1) + ".txt");
    }
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
Node *solve2BP(Node *closestNode, Node *randNode, double epsilon)
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

int collision_check(Node *newNode, Node *closestNode, double x_ob[], double y_ob[], double r_ob[], int num_ob)
{

    int point_check = 0;
    int seg_check = 0;
    int collision = 0;

    // check if newNode is in any of the objects
    for (int i = 0; i < num_ob; i++)
    {
        // distance from center of object to new node
        double distance = sqrt(pow(x_ob[i] - newNode->x, 2) + pow(y_ob[i] - newNode->y, 2));

        // if distance is less than radius, the point is in an object
        if (distance <= r_ob[i])
        {
            point_check = 1;
            break;
        }
        else
        {
            point_check = 0;
        }
    }

    if (point_check == 0)
    {
        // check if segment between node 1 and node 2 is in collision with an of the obstacles
        // algorithm from https://stackoverflow.com/questions/9052507/circle-and-line-segment-intersection user enobayram
        for (int i = 0; i < num_ob; i++)
        {
            double A_x = closestNode->x;
            double A_y = closestNode->y;
            double B_x = newNode->x;
            double B_y = newNode->y;
            double C_x = x_ob[i];
            double C_y = y_ob[i];
            double circle_r = r_ob[i];

            double dx = B_x - A_x;
            double dy = B_y - A_y;
            double unit_x = dx / (sqrt(pow(dx, 2) + pow(dy, 2)));
            double unit_y = dy / (sqrt(pow(dx, 2) + pow(dy, 2)));
            double d = (C_x - A_x) * unit_y - (C_y - A_y) * unit_x;
            if (abs(d) < circle_r)
            {
                if ((((unit_x * A_x + unit_y * A_y) < (unit_x * C_x + unit_y * C_y)) && ((unit_x * C_x + unit_y * C_y) < (unit_x * B_x + unit_y * B_y))) || (((unit_x * B_x + unit_y * B_y) < (unit_x * C_x + unit_y * C_y)) && ((unit_x * C_x + unit_y * C_y) < (unit_x * A_x + unit_y * A_y))))
                {
                    seg_check = 1;
                    break;
                }
            }
            else
            {
                seg_check = 0;
            }
        }
    }

    // if either point or line segment intersects object, collision has ocurred 
    if (seg_check == 1 || point_check == 1)
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
int goal_check(Node *newNode, double goal_x, double goal_y, double goal_r)
{
    int goal_check;
    double distance = sqrt(pow(goal_x - newNode->x, 2) + pow(goal_y - newNode->y, 2));
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