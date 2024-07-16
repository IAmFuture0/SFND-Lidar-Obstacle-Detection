#include "processPointClouds.h"
#include <vector>
struct Node{
  std::vector<float> point;
  int id;
  Node* left;
  Node* right;
  
  Node(std::vector<float> arr, int setId): point(arr), id(setId), left(NULL), right(NULL){}
};
       
struct KdTree{
    Node* root;
    KdTree():root(NULL){}
    
    void insert(pcl::PointXYZI pointI, int id){
      std::vector<float> point;
      point.push_back(pointI.x);
      point.push_back(pointI.y);
      point.push_back(pointI.z);
      inserthelpfunc(&root, point, 0, id);
    }
    
    void inserthelpfunc(Node** node, std::vector<float> point, int depth, int id){
      uint cd = depth%2;
      if(*node == NULL){
        *node = new Node(point, id);
      } else{
      	if(point[cd] < ((*node)->point[cd])){
        	inserthelpfunc(&(*node)->left, point, depth+1, id);
        }else{
          	inserthelpfunc(&(*node)->right, point, depth+1, id);
        }
      }
    }
  
  std::vector<int> search(pcl::PointXYZI targetPoint, float distanceTol){
  	std::vector<int> ids;
    std::vector<float> target;
    target.push_back(targetPoint.x);
    target.push_back(targetPoint.y);
    target.push_back(targetPoint.z);
    searchHelperfunc(target, root, 0, distanceTol, ids);
    return ids;
  }
  
  void searchHelperfunc(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int> &ids){
  	if(fabs(node->point[0]-target[0]) <= distanceTol && fabs(node->point[1]-target[1] <= distanceTol)){
    	float distance = sqrt(pow(node->point[0]-target[0], 2)+pow(node->point[1]-target[1], 2));
      if(distance < distanceTol){
      	ids.push_back(node->id);
      }
      /*** ISSUE ***/
      if(node->point[depth%2] > target[depth%2] - distanceTol){
      	searchHelperfunc(target, node->left, depth+1, distanceTol, ids);
      }
      if(node->point[depth%2] < target[depth%2] + distanceTol){
      	searchHelperfunc(target, node->right, depth+1, distanceTol, ids);  
      }
      /*** ISSUE ***/
    }
  }
}; 
