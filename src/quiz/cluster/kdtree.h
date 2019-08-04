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
	void insertPoint(Node *&ptr, int id, std::vector<float> point, bool compX)
	{

		if (ptr == NULL)
		{
			ptr = new Node(point, id);
		}
		if (compX)
		{
			compX = !compX;
			if (point[0] < ptr->point[0])
			{
				insertPoint(ptr->left, id, point, compX);
			}
			else if (point[0] > ptr->point[0])
			{
				insertPoint(ptr->right, id, point, compX);
			}

		}
		else
		{
			compX = !compX;
			if (point[1] < ptr->point[1])
			{
				insertPoint(ptr->left, id, point, compX);

			}
			else if (point[1] > ptr->point[1])
			{
				insertPoint(ptr->right, id, point, compX);

			}
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertPoint(root, id, point, true);
	}

	void searchPoint(std::vector<float> target, Node* root, float distanceTol, int depth, std::vector<int> &id)
	{
		if (root != NULL)
		{
			if ((root->point[0] <= (target[0] + distanceTol) && root->point[0] >= (target[0] - distanceTol)) && (root->point[1] <= (target[1] + distanceTol) && root->point[1] >= (target[1] - distanceTol)))
			{
				float dis = sqrt(((target[0] - root->point[0]) * (target[0] - root->point[0])) + ((target[1] - root->point[1]) * (target[1] - root->point[1])));
				if (dis <= distanceTol)
				{
					id.push_back(root->id);
				}
			}

			if ((target[depth % 2] - distanceTol) < root->point[depth % 2])
			{
				searchPoint(target, root->left, distanceTol, depth + 1, id);
			}
			if ((target[depth % 2] + distanceTol) > root->point[depth % 2])
			{
				searchPoint(target, root->right, distanceTol, depth + 1, id);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchPoint(target, root, distanceTol, 0, ids);
		return ids;
	}


};




