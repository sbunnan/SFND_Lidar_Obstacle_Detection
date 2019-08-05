/* \author Aaron Brown */
// Quiz on implementing kd tree
#include <unordered_set>

#ifndef KDTREE_H_
#define KDTREE_H_


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
	void insertPoint(Node *&ptr, int id, std::vector<float> point, int& depth)
	{

		if (ptr == NULL)
		{
			ptr = new Node(point, id);
		}
		if (depth == 0)
		{
			depth = 1;
			if (point[0] < ptr->point[0])
			{
				insertPoint(ptr->left, id, point, depth);
			}
			else if (point[0] > ptr->point[0])
			{
				insertPoint(ptr->right, id, point, depth);
			}

		}
		else if (depth == 1 )
		{
			depth = 2;
			if (point[1] < ptr->point[1])
			{
				insertPoint(ptr->left, id, point, depth);

			}
			else if (point[1] > ptr->point[1])
			{
				insertPoint(ptr->right, id, point, depth);

			}
		}
		else if (depth == 3)
		{
			depth = 0;
			if (point[3] < ptr->point[3])
			{
				insertPoint(ptr->left, id, point, depth);

			}
			else if (point[3] > ptr->point[3])
			{
				insertPoint(ptr->right, id, point, depth);

			}

		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		int depth = 0;
		insertPoint(root, id, point, depth);
	}

	void searchPoint(std::vector<float> target, Node* root, float distanceTol, int depth, std::vector<int> &id)
	{
		if (root != NULL)
		{
			if (((root->point[0] <= (target[0] + distanceTol)) && (root->point[0] >= (target[0] - distanceTol))) \
			 && ((root->point[1] <= (target[1] + distanceTol)) && (root->point[1] >= (target[1] - distanceTol))) \
			 && ((root->point[2] <= (target[2] + distanceTol)) && (root->point[2] >= (target[2] - distanceTol))))
			{
				float dis = sqrt(((target[0] - root->point[0]) * (target[0] - root->point[0])) + ((target[1] - root->point[1]) * (target[1] - root->point[1])) +  ((target[3] - root->point[3]) * (target[3] - root->point[3])));
				if (dis <= distanceTol)
				{
					id.push_back(root->id);
				}
			}

			if ((target[depth % 3] - distanceTol) < root->point[depth % 3])
			{
				searchPoint(target, root->left, distanceTol, depth + 1, id);
			}
			if ((target[depth % 3] + distanceTol) > root->point[depth % 3])
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

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);


#endif