/*******************************************************************************
 * DANIEL'S ALGORITHM IMPLEMENTAIONS - slightly edited and fixed by MEROSSS
 *
 *  /\  |  _   _  ._ o _|_ |_  ._ _   _ 
 * /--\ | (_| (_) |  |  |_ | | | | | _> 
 *         _|                      
 *
 * A* ALGORITHM
 * 
 * Features:
 *    In computer science, A* (pronounced "A star" ,is a computer algorithm
 * that is widely used in path-finding and graph traversal, the process of
 * plotting an efficiently traversable path between points, called nodes. Noted 
 * for its performance and accuracy, it enjoys widespread use. (However, in 
 * practical travel-routing systems, it is generally outperformed by algorithms 
 * which can pre-process the graph to attain better performance.[1])
 *
 * http://en.wikipedia.org/wiki/A*_search_algorithm
 *
 ******************************************************************************/

#ifndef __ASTAR_H__
#define __ASTAR_H__

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "heap.h"
#include "hash_table.h"
#include "2darray.h"

namespace alg {
class AStar {
public:
  /**
   * A-Star algorithm result;
   */
  struct AStarResult {
    int * path; // the path format:
    // [V1,V2,V3,V4,...,Vn]
    // where V are the nodes that form the path
    int num_nodes;
    ~AStarResult()
    {
      delete [] path;
      path = NULL;
    }
  };

  static const unsigned char WALL = 0xFF;

private:
  const Array2D<unsigned char> & m_grid;
  // The set of nodes already evaluated.
  Array2D<bool> m_closedset;
  // Cost from start along best known path.
  Array2D<float> g_score;
  // Estimated total cost from start to goal through y.
  Array2D<float> f_score;
public:
  AStar(const Array2D<unsigned char> & grid) :
    m_grid(grid),
    m_closedset(grid.row(),grid.col()),
    g_score(grid.row(),grid.col()),
    f_score(grid.row(),grid.col()) { }

  virtual ~AStar(){

  }

  /**
   * the A* algorithm, search a path from (x1,y1) to (x2,y2).
   * a integer representing path is returned, you should delete it after.
   */
  AStarResult * run(uint32_t startNode, uint32_t targetNode) {


    static float SQRT2 = 1.414213562373095;
    uint32_t nrow = m_grid.row();
    uint32_t ncol = m_grid.col();
    m_closedset.clear(false);

    uint32_t x1, y1, x2 ,y2;
    x1 = startNode/ncol;
    y1 = startNode%ncol;
    x2 = targetNode/ncol;
    y2 = targetNode%ncol;

    // the set of tentative nodes to be evaluated,
    // initially containing the start node
    // encoding [x,y] to [x*ncol + y]
    // using binary heap ...
    Heap<uint32_t> openset(nrow*ncol);
    openset.insert(0, x1*ncol+y1);

    // The map of navigated nodes.
    HashTable<uint32_t, uint32_t> came_from(nrow*ncol);

    g_score(y1,x1) = 0.0f;
    f_score(y1,x1) = g_score(y1,x1) + estimate(x1,y1,x2,y2);

    AStarResult * as = new AStarResult;
    as->path = NULL;
    as->num_nodes = 0;

    while(!openset.is_empty()) {
      uint32_t value = openset.min_value();
      int	cx = value/ncol;
      int	cy = value%ncol;

      if(cx == (int)x2 && cy==(int)y2) {	// we reached (x2,y2)
        // reconstruct path & return
        uint32_t tmp = x2*ncol+y2;
        while((tmp=came_from[tmp]) != x1*ncol+y1) {
          as->num_nodes++;
        }

        as->path = new int[as->num_nodes];

        // *merosss edit*: Function modified so that it
        // returns the path in the correct order using node index
        tmp = x2*ncol+y2;
        int idx=(as->num_nodes)-1;
        while((tmp=came_from[tmp]) != x1*ncol+y1) {
          as->path[idx--] = tmp;
        }
        return as;
      }

      openset.delete_min();
      m_closedset(cx, cy) = true;

      // for each neighbor
      int nx, ny;
      for(nx=cx-1;nx<=cx+1;nx++) {
        if (nx<0 || nx>=(int)ncol) continue;
        for(ny=cy-1;ny<=cy+1;ny++) {
          if (ny<0 || ny>=(int)nrow) continue;

          // except the wall;
          if(m_grid(nx,ny) == WALL) continue;
          // except the cur itself
          if(nx == cx && ny==cy) continue;
          // if neighbour in the closed set
          if(m_closedset(nx,ny)) continue;

          float tentative = g_score(cx,cy);
          if (nx == cx || ny == cy) {
            tentative += 1 + m_grid(nx,ny);
          } else {
            tentative += (1 + m_grid(nx,ny)) * SQRT2;
          }

          // if neighbour not in the openset or dist < g_score[neighbour]
          if (!openset.contains(nx*ncol+ny) || tentative < g_score(nx,ny)) {
            came_from[nx*ncol+ny] = cx*ncol+cy; // record path
            g_score(nx,ny) = tentative;
            f_score(nx,ny) = tentative + estimate(nx,ny,x2,y2);
            if (!openset.contains(nx*ncol+ny)) {
              openset.insert(f_score(nx,ny), nx*ncol+ny);
            }
          }
        }
      }
    }
    return as;
  }
private:
  inline float estimate(int x1, int y1, int x2, int y2) const {
    return sqrtf((x2-x1) * (x2-x1) + (y2-y1)*(y2-y1));
  }
};

}

#endif //
