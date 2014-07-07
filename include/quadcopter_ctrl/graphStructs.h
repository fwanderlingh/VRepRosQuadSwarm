/*
 * graphNodeStruct.h
 *
 *  Created on: May 2, 2014
 *      Author: francescow
 */

#ifndef GRAPHSTRUCTS_H_
#define GRAPHSTRUCTS_H_


struct graphNode{
  double posx, posy, posz;
  //FIXME
  int occupied;  //1 = not accessible, 0 = accessible (further expansion could be to keep
  int access; //1 = not accessible, 0 = accessible
  int nodeCount;
  int numEdges;

  /// TO BE EXTEND WITH POINTER TO OTHER NODES (Neighbours)

  graphNode() : posx(0.f), posy(0.f), posz(4.0f), occupied(1), access(1), nodeCount(0), numEdges(0) {}   //Default access is 1

  graphNode(double x, double y, double z, int acc, int occ) :      //Struct filler constructor
    posx(x), posy(y), posz(z), access(acc), occupied(occ), nodeCount(0) {}

  void setPos(double x, double y){
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
