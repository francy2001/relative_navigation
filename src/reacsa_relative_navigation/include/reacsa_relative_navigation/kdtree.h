/*
* A K-Dimensional Tree is a binary search tree where data in each node is a K-Dimensional point in space. 
* In short, it is a space partitioning data structure for organizing points in a K-Dimensional space. 
* A non-leaf node in K-D tree divides the space into two parts, called as half-spaces. Points to the left of this space 
* are represented by the left subtree of that node and points to the right of the space are represented by the right subtree. 
* Let's understand a 2-D Tree: the root would have an x-aligned plane, the root's children would both have y-aligned planes, 
* the root's grandchildren would all have x-aligned planes, and the root's great-grandchildren would all have y-aligned planes and so on.
*
* Generalization: 
* Let us number the planes as 0, 1, 2, …(K – 1). From the above example, it is quite clear that a point (node) at depth D will have 
* A aligned plane where A is calculated as: A = D mod K
* How to determine if a point will lie in the left subtree or in right subtree? 
* If the root node is aligned in planeA, then the left subtree will contain all points whose coordinates in that plane are smaller than that of root node. 
* Similarly, the right subtree will contain all points whose coordinates in that plane are greater-equal to that of root node. 
*
* The KD-Tree enables efficient nearest neighbor search with a time complexity of O(log(n)) as opposed to O(n). 
* This is primarily because, by grouping the points into regions in a KD-Tree, the search space is narrowed down drastically 
* and expensive distance computations for potentially thousands of points can be avoided. 
*
* Once points are able to be inserted into the tree, the next step is being able to search for nearby points inside the tree compared to a given target point. 
* Points within a distance tolerance are considered to be nearby.
* With the KD-Tree, a boxed square of size 2 X distance tolerance, centered around the target point is used. 
* If the current node point is within this box, only then the Euclidean distance is calculated and depending on this, 
* it can be determined if the point should be added to the list of nearby points. Further, if this box does not cross over the node division region, 
* the branch on the other side of the region is completely skipped. If it crosses over, then that side is recursively explored. 
*
*/

#ifndef KDTREE_H
#define KDTREE_H

#include <vector>
#include <cstddef>



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

	~Node()
	{
		delete left;
		delete right;
	}
};


struct KdTree
{
	Node* root;
	int idx;
	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	// either use return or use a double pointer/pointer by reference. 
	// using the latter methods, the changes are directly reflected in the calling function
	void insertRec(Node** root, const std::vector<float>& point, int depth, int id)
	{
		if (*root == NULL)
		{
			*root = new Node(point, id);
		}
		else
		{
			int idx = depth % 3;  // Cycles through dimensions 0, 1, 2 (x, y, z)
			if (point[idx] < ((*root)->point[idx]))
				insertRec(&((*root)->left), point, depth + 1, id);
			else
				insertRec(&((*root)->right), point, depth + 1, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertRec(&root, point, 0, id);
	}

	// return a list of point ids in the tree that are within distance of target
	bool distCompare(const std::vector<float>& point1, const std::vector<float>& point2, float distTol)
	{
		// Ensure both points have at least 3 dimensions
		if (point1.size() < 3 || point2.size() < 3) {
			return false;
		}
		
		float x = point1[0] - point2[0];
		float y = point1[1] - point2[1];
		float z = point1[2] - point2[2];
		
		float distSquared = x * x + y * y + z * z;
		return distSquared <= (distTol * distTol);
	}

	void searchRec(const std::vector<float>& target, Node* root, int depth, float distTol, std::vector<int>& ids)
	{
		if (root != NULL)
		{	
			// 3D bounding box check
			if((root->point[0] >= (target[0] - distTol) && root->point[0] <= (target[0] + distTol)) && 
			(root->point[1] >= (target[1] - distTol) && root->point[1] <= (target[1] + distTol)) &&
			(root->point[2] >= (target[2] - distTol) && root->point[2] <= (target[2] + distTol))) {
				
				if (distCompare(target, root->point, distTol))
				{
					ids.push_back(root->id);
				}
			}

			int idx = depth % 3;  // Cycles through x, y, z dimensions
			
			if ((target[idx] - distTol) < root->point[idx])
				searchRec(target, root->left, depth + 1, distTol, ids);

			if ((target[idx] + distTol) > root->point[idx])
				searchRec(target, root->right, depth + 1, distTol, ids);
		}
	}

	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchRec(target, root, 0, distanceTol, ids);
		return ids;
	}
};


#endif // KDTREE_H




