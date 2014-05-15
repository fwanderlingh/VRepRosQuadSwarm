/*
 * graphNodeStruct.h
 *
 *  Created on: May 2, 2014
 *      Author: francescow
 */

#ifndef GRAPHNODESTRUCT_H_
#define GRAPHNODESTRUCT_H_


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


#endif /* GRAPHNODESTRUCT_H_ */
