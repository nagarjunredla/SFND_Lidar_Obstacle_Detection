/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id, int dimension)
	{
		if(*node == NULL)
		{
			*node  = new Node(point, id);
		}

		else
		{
			uint cd = depth % dimension;

			if(point[cd] < ((*node)->point[cd]))
			{
				insertHelper(&((*node)->left) , depth+1, point, id, dimension);
			}
			else
			{
				insertHelper(&((*node)->right), depth+1, point, id, dimension);
			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root

		if(point.size() >=2 && point.size() <=3)
		{
			int dimension = point.size();
			insertHelper(&root, 0, point, id, dimension);
		}
		else
		{
			std::cerr << "Invalid Dimensions provided" << std::endl;
		}
	}

	bool insideBox(std::vector<float> target, std::vector<float> point, float distanceTol)
	{
		if (target.size() != point.size()) {
            std::cerr << "Point and Target dimensions do not match" << std::endl;
        }

	    bool isInside = true;
	    for (int i=0; i<target.size(); i++){
	        isInside *= (target[i] - distanceTol <= point[i]) && (target[i] + distanceTol >= point[i]);
	    }
	    return isInside;
	}

	float getDistance(std::vector<float> target, std::vector<float> point){

        if (target.size() != point.size()) {
            std::cerr << "Point and Target dimensions do not match" << std::endl;
        }
	    float sqDistance = 0.0f;
        for (int i=0; i<target.size(); i++){
            float absDistance = target[i] - point[i];
            sqDistance += absDistance * absDistance;
        }
        return sqrt(sqDistance);
	}

	void searchHelper(std::vector<float> target, Node *node, int depth, float distanceTol, std::vector<int>& ids, int dimension)
	{
		if(node != NULL)
		{
			// if((node->point[0]>=(target[0]-distanceTol)&&node->point[0]<=(target[0]+distanceTol)) && (node->point[1]>=(target[1]-distanceTol) &&node->point[1]<=(target[1]+distanceTol)))
			if(insideBox(target, node->point, distanceTol))
			{
				// float distance = sqrt((node->point[0]-target[0])*(node->point[0]-target[0])+(node->point[1]-target[1])*(node->point[1]-target[1]));
				float distance = getDistance(target, node->point);
				if(distance <= distanceTol)
					ids.push_back(node->id);
			}

			int cd = depth % dimension;
			if((target[cd]-distanceTol)<node->point[cd])
				searchHelper(target, node->left, depth+1, distanceTol, ids, dimension);
			if((target[cd]+distanceTol)>node->point[cd])
				searchHelper(target, node->right, depth+1, distanceTol, ids, dimension);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;

		if(target.size() >=2 && target.size() <=3)
		{
			int dimension = target.size();
			// insertHelper(&root, 0, point, id, dimension);
			searchHelper(target, root, 0, distanceTol, ids, dimension);
		}
		else
		{
			std::cerr << "Invalid Dimensions provided" << std::endl;
		}
		return ids;
	}
	

};




