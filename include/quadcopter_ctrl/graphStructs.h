/*
 * graphNodeStruct.h
 *
 *  Created on: May 2, 2014
 *      Author: francescow
 */

#ifndef GRAPHSTRUCTS_H_
#define GRAPHSTRUCTS_H_


struct graphNode{
  float posx, posy, posz;
  int access;   // 0 = not accessible, 1 = accessible (further expansion could be to keep
                // track of how many times node is visited by increasing 'access')
  int occupied; // How many robots are on the node
  int nodeCount;

  /// TO BE EXTEND WITH POINTER TO OTHER NODES (Neighbours)

  graphNode() : posx(0.f), posy(0.f), posz(4.0f), access(1), occupied(0), nodeCount(0) {}   //Default access is 1

  graphNode(float x, float y, float z, int acc, int occ) :      //Struct filler constructor
    posx(x), posy(y), posz(z), access(acc), occupied(occ), nodeCount(0) {}

  void setPos(float x, float y){
    posx = x;
    posy = y;
  }
};

struct vip{
  int v;                                // Vertex (node) index (of the unvisited list)
  std::vector< std::vector<int> >::iterator i;    // Index of Path (#robots)
  std::vector<int>::iterator p;              // Position index inside the path
  bool neighb;
  std::vector<int> interPath;

  void set_vip( int nodeIndex,
                 std::vector< std::vector<int> >::iterator pathIndex,
                 std::vector<int>::iterator position){
    v = nodeIndex;
    i = pathIndex;
    p = position;
  }

  void set_vipn( int nodeIndex,
                 std::vector< std::vector<int> >::iterator pathIndex,
                 std::vector<int>::iterator position,
                 bool neighbour){
    v = nodeIndex;
    i = pathIndex;
    p = position;
    neighb = neighbour;
  }
};

#endif /* GRAPHSTRUCTS_H_ */
