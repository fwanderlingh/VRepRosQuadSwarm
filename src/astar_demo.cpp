#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "astar.h"

#define N 10

#define MARK 0xEE

using namespace alg;

void printMap(Array2D<unsigned char> & grid);

int main(void)
{


	//Array2D<unsigned char> grid(N,N);
	Array2D<unsigned char> grid;
	grid.resize(N,N);
	grid.clear(0);
	srand(time(NULL));
	int i,j;
	
	for (i=N/4;i<=3*N/4;i++) {
	  for (j=N/4;j<=N-1;j++)
		grid(i,j) = AStar::WALL;
	}

	uint32_t start = 0;
	uint32_t target = N*N-1;

	grid(start/N,start%N) = 'S';
	grid(target/N,target%N) = 'T';

	printf("search a path from %d to %d\n", start, target);

	printMap(grid);

	AStar astar(grid);
	AStar::AStarResult * as = astar.run(start, target);

        printf("Shortest path is:\n");
        printf("%d ",start);
	for(i=0;i<as->num_nodes;i++){
		printf("%d ",as->path[i]);
		grid((as->path[i])/N,(as->path[i])%N) = MARK;
	}
	printf("%d ",target);
	printf("\n");

	printMap(grid);

	printf("\n");
	
	return 0;
}

void printMap(Array2D<unsigned char> & grid){

    for     (int i=0;i<N;i++) {
            for(int j=0;j<N;j++){
                    if (grid(i,j) == AStar::WALL) { printf("■ "); }
                    else if (grid(i,j) == MARK) {printf("X ");}
                    else if (grid(i,j) == 'S') {printf("S ");}
                    else if (grid(i,j) == 'T') {printf("T ");}
                    else printf(". ");
            }
            printf("\n");
    }

}
