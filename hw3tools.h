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
        double x;   // physical x location
        double y;   // physical y location

        Node* parentNode;      // used for graph search, this is a pointer to this node's parent

        Node() // constructor
        {
          this->x = 0;
          this->y = 0;
          this->parentNode = NULL;
        }
};

class Tree
{
    public:
        
        // list of nodes to build tree
        std::vector<Node*> node_list;
        
        // add node to list
        void add_node(Node *node){
            node_list.push_back(node);  
        }

        // find nearest node to specified input node
        Node* nearest_Node(Node* newNode)
        {
            double min_distance = 1000000000000000; // intialize with a really big number ("infinity")
            unsigned int vecSize = node_list.size();
            Node *nearestNode = NULL;

            // loop through all nodes to find closest - this is inefficient but still works quickly, sorry :( 
            for (unsigned int i = 0; i<vecSize; i++)
            {
                Node *currentNode = node_list[i];
                double distance = sqrt(pow(currentNode->x - newNode->x, 2) + pow(currentNode->y - newNode->y, 2));
                if(distance < min_distance)
                {
                    min_distance = distance;
                    nearestNode = node_list[i];                    
                }
            }
            return nearestNode;
        }

        //save path (code modified from Dr. Otte)
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

            int vecSize = node_list.size();
            for (int n = 0; n < vecSize; n++)
            {
                Node *thisNode = node_list[n];
                if (thisNode->parentNode != NULL)
                {

                    // format is x1, y1, id2, x2, y2
                    fprintf(pFile, "%f, %f, %f, %f\n",
                            thisNode->x, thisNode->y,
                            thisNode->parentNode->x, thisNode->parentNode->y);
                }
            }
            fclose(pFile);
            printf("saved search tree in %s\n", searchTreeFile.c_str());

            return true;
        }
};


// Function prototypes

Node* sample(); // function to randomly sample C space

int collision_check(Node *, Node*, double x_ob[], double y_ob[], double r_ob[], int num_ob);

int goal_check(Node*, double goal_x, double goal_y, double goal_r);

Node* solve2BP(Node*, Node*, double);

double euclidean(Node *Node1, Node *Node2);