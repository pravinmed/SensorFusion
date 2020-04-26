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

    void InsertNode(Node** root, const std::vector<float> point, int id, int depth)
	{
		if(*root == NULL)
		{
			*root = new Node(point, id);
			return;
		}
			std::cout << "In y depth " << depth << std::endl;
		
		if ( point[depth%2] > (*root)->point[depth%2])
		{
			std::cout << "In y depth to right " << depth << std::endl;	
			InsertNode(&(*root)->right, point, id, depth+1);
		} else {
			std::cout << "In y depth to left " << depth << std::endl;	
			InsertNode(&(*root)->left, point, id, depth+1);		
		}
	
		
		return;
	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		if (root == NULL)
		{
			root = new Node(point, id);
			return;
		}
		int depth = 0;
		Node* root_copy = root;
		InsertNode(&root_copy, point, id, depth);
	}

	void findPoints(std::vector<float> target,
	 float dist,
	  Node* root,
	  std::vector<int>& idList, int depth)
	{

		if(root != NULL)
		{
			float dx = root->point[0] - target[0];
			float dy = root->point[1] - target[1];
			std::cout << root->point[0] << " x " << std::endl;
			std::cout << root->point[1] << " y " << std::endl;
			std::cout <<" distance " <<  std::sqrt(dx* dx + dy*dy)  << std::endl;
			if (std::sqrt(dx* dx + dy*dy) <= dist)
			{
				idList.push_back(root->id);
			}
			if ((target[depth%2] + dist) > root->point[depth%2])
			{
				std::cout << " to right " << std::endl;
				findPoints(target, dist, root->right, idList, depth+1);
			} 
			if ((target[depth%2] - dist) < root->point[depth%2])
			{
				std::cout << " to left " << std::endl;
				findPoints(target, dist, root->left, idList, depth+1);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		findPoints(target, distanceTol, root, ids, 0);
		return ids;
	}
	

};




