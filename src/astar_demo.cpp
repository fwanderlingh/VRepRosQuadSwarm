#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "astar.h"
#include "termColors.h"
#include <vector>

#define INTSTRSIZE ((CHAR_BIT * sizeof(int) - 1) / 3 + 2)

#define N 15

#define MARK 0xEE

using namespace alg;

void printMap(Array2D<unsigned char> & grid);

std::vector<std::string> rainbow;

int main(void)
{

	//Array2D<unsigned char> grid(N,N);
	Array2D<unsigned char> grid;
	grid.resize(N,N);
	grid.clear(0);
	srand(time(NULL));
	int i,j;

	///CONVEX OBSTACLE
/*
        for     (i=N/4;i<=3*N/4;i++) {
                grid(3*N/4,i) = AStar::WALL;
                grid(i,3*N/4) = AStar::WALL;
        }*/
	/// SNAKE PATH
	for (i=0;i*2<N;i++) {
	  for (j=0;j<N;j++)
		if(i%2 == 0 && j>2) { grid(i*2,j) = AStar::WALL; }
		else if(i%2 == 1 && j<N-2) { grid(i*2,j) = AStar::WALL; }
	}

	///CHESS-BOARD PATTERN
/*
        for (i=0;i<N;i++) {
          for (j=1;j<N-1;j++)
                if(i%2 == 1 && j%2 == 1) { grid(i,j) = AStar::WALL; }
        }
*/
	uint32_t start = 0;
	uint32_t target = N*N-1;

	grid(start/N,start%N) = 'S';
	grid(target/N,target%N) = 'T';

	printf("search a path from %d to %d\n", start, target);

	printMap(grid);
        printf("\n");

	AStar astar(grid);
	AStar::AStarResult * as = astar.run(start, target);

	//rainbow.resize(as->num_nodes);
	//char index[INTSTRSIZE];

	int numNodes = as->num_nodes;

	if(as->num_nodes > 0){
          printf("Shortest path is: ");
          printf("%d ",start);
          for(i=0;i<as->num_nodes;i++){
                  printf("%d ",as->path[i]);
                  grid((as->path[i])/N,(as->path[i])%N) = MARK;

                  //float colorIndex = (i*6)/numNodes+31;
                  //sprintf(index, "%d", (int)colorIndex);
                  //std::string indexString(index);
                  //rainbow.at(i) = "\e[1;" + indexString + "m";

          }
          printf("%d ",target);
          printf("\n");

          printMap(grid);

          printf("\n");
	}else{
	 printf("There exist no free path!\n");
	}

	
	return 0;
}

void printMap(Array2D<unsigned char> & grid){

  int ci = 0;
    for     (int i=0;i<N;i++) {
            for(int j=0;j<N;j++){
                    if (grid(i,j) == AStar::WALL) { printf("■ "); }
                    else if (grid(i,j) == MARK) {printf("%sX%s ", TC_GREEN, TC_NONE);}
                    else if (grid(i,j) == 'S') {printf("S ");}
                    else if (grid(i,j) == 'T') {printf("T ");}
                    else printf(". ");
            }
            printf("\n");
    }

}
