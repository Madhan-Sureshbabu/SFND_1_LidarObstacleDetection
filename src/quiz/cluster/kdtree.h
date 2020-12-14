/* \author Aaron Brown */
// Quiz on implementing kd tree
#ifndef KDTREE_H_
#define KDTREE_H_

#include "../../render/render.h"
#include <math.h>

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

	void insertHelper(Node** node,uint depth,std::vector<float> point, int id)
	{
		if (*node==NULL)
			*node = new Node(point,id);

		else
		{
			uint cd = depth%2;
			if (point[cd] > (*node)->point[cd])
				insertHelper(&((*node)->right),depth+1,point,id);
			else
				insertHelper(&((*node)->left),depth+1,point,id);
		}
	}

	void insert(std::vector<float> point, int id)
	{

		if (root==NULL)
		{
			root = new Node(point,id);
		}

		else
			insertHelper(&root,0,point,id);

	}

	bool withinBox(std::vector<float> point, std::vector<float> target, float distanceTol)
	{
		bool inside = true;
		for (int dim=0; dim<target.size(); dim++)
		{
			inside = inside && (abs(point[dim] - target[dim]) <= distanceTol);
			// std::cout<<"In"<<std::endl;
			// float dis = sqrt(pow(target[0]-point[0],2)+pow(target[1]-point[1],2));
			// if (dis < distanceTol)
		}
		return inside;
	}

	bool withinRadius(std::vector<float> point, std::vector<float> target, float distanceTol)
	{
		float dis = 0;
		for (int dim=0; dim < target.size(); dim++)
		{
			dis += pow(target[dim]-point[dim],2);
		}
		// float dis = sqrt(pow(target[0]-(*node)->point[0],2) + pow(target[1]-(*node)->point[1],2));
		return sqrt(dis) <= distanceTol;
	}

	void searchHelper(Node** node, uint depth, std::vector<float> target, float distanceTol, std::vector<int>& ids)
	{
		if ((*node)!=NULL)
		{
			// std::cout<<(*node)->point[0]<<" "<<(*node)->point[1]<<std::endl;
			// std::cout<<"id "<<(*node)->id<<" "<<depth<<std::endl;
			// std::cout<<(*node)->point[0]<<" "<<(*node)->point[1]<<std::endl;
			if (withinBox((*node)->point,target,distanceTol))
			{
				// std::cout<<"In here"<<std::endl;
				if (withinRadius((*node)->point,target,distanceTol))
					ids.push_back((*node)->id);
				
			}
			if ((*node)->point[depth%3]>target[depth%3]-distanceTol)
				searchHelper(&((*node)->left),depth+1,target,distanceTol,ids);
			if ((*node)->point[depth%3]<target[depth%3]+distanceTol)
				searchHelper(&((*node)->right),depth+1,target,distanceTol,ids);
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		// std::cout<<ids.size()<<std::endl;

		searchHelper(&root,0,target,distanceTol,ids);
		// if (withinBox())
		// std::cout<<ids.size()<<std::endl;
		return ids;
	}
	

};




#endif