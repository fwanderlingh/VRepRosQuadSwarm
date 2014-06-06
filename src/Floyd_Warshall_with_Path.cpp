#include <iostream>
#include <curses.h>
#include <cstdlib>
#include <climits>
#include <vector>
#include <../include/quadcopter_ctrl/termColors.h>

using std::cout;
using std::endl;
using std::vector;

const int Inf = INT_MAX/2-1;// graph[i][j] = Inf if no edge
int _access_vec[] = {0, 0, 0, 0,
                     0, 0, 0, 1,
                     1, 1, 0, 0,
                     0, 0, 0, 1};


void printMatrix(vector< vector<int> >& matrix);
void getPath(int start, int target, vector< vector <int> >& P, vector <int>& path);
void floydWarshall(vector< vector <int> >& G, vector< vector <int> >& D, vector< vector <int> >& P);
void spaced_cout(int value);


int main(int argc, char **argv){

  int gridSizeX = 4;
  int gridSizeY = 4;
  const int n = gridSizeX*gridSizeY;
  vector<int> access_vec(_access_vec, _access_vec + sizeof(_access_vec)/sizeof(int));
  
  vector<int> _graph(n, Inf);
  vector< vector<int> > graph(n, _graph);
  int row, col;    // Main indexes
  int row_shift, col_shift; // To move around spacial adjacents
  int nb_row, nb_col;       // Adjacents indexes
  
  printf("%sOccupancy Map:%s",TC_RED, TC_NONE);
  for(int j=0; j<n; j++){
    if(j%gridSizeY == 0) cout << endl;
    if( access_vec[j] == 1 ){
      printf("%s",TC_RED);
      spaced_cout(j);
      printf("%s", TC_NONE);
    }
    else spaced_cout(j);
  }
  cout << endl << endl;
  
  for(int i=0; i<n; i++){
    if(access_vec[i] == 0){
      row = i/gridSizeX;
      col = i%gridSizeY;
      for(row_shift=-1; row_shift<=1; row_shift++){
        for(col_shift=-1; col_shift<=1; col_shift++){
          nb_row = row + row_shift;
          nb_col = col + col_shift;
          if( (nb_row>=0) && (nb_row<gridSizeX) && (nb_col>=0) && (nb_col<gridSizeY) ///RANGE CHECK
            && (row_shift!=0 || col_shift!=0)  ///<--- don't check same node of current
            && (row_shift*col_shift == 0)  ///<--- don't allow diagonal movements
            )
          {
            if(access_vec[nb_row*gridSizeY+nb_col] == 0){
              graph[i][nb_row*gridSizeY+nb_col] = 1;  // Create the edge between i and its free neighbour
            }
          }
        }
      }
    }
  }
  
  //printf("%sGraph:%s", TC_GREEN, TC_NONE);
  //printMatrix(graph);

  vector< vector<int> > dist = graph;
  vector< vector<int> > parent(n, vector<int>(n));
  
  floydWarshall(graph, dist, parent);
  
  
  printf("%sDistances:%s", TC_GREEN, TC_NONE);
  printMatrix(dist);
  
  //printf("%sParent Matrix:%s", TC_GREEN, TC_NONE);
  //printMatrix(parent);
  
  int start = 0;
  int target = 12;
  vector <int> path;
  printf("%sThe path from %d to %d is:%s\n", TC_CYAN, start, target, TC_NONE);
  getPath(start, target, parent, path);
  for(int k=0; k<path.size(); k++){
    cout << path.at(k) << " ";
  }
  cout << endl;
  
  return 0;
  
}




///*** Functions ***///

void floydWarshall(vector< vector <int> >& graph, vector< vector <int> >& dist, vector< vector <int> >& parent){

  int n = graph.size();
  for(int i = 0; i < n; i++ ){
    for(int j = 0; j < n; j++ ){
      if ( i == j || graph[i][j] == Inf ) parent[i][j] = -1;
      else parent[i][j] = i;
    }
  }

  for( int k = 0; k < n; k++ ){
    for(int i = 0; i < n; i++ ){
      for(int j = 0; j < n; j++ ) {
        if( i != j ){
          int newD = dist[i][k] + dist[k][j];
          if( newD < dist[i][j] ) {
            dist[i][j] = newD;
            parent[i][j] = parent[k][j];
          }
        }else{
         dist[i][j] = 0;
        }
      }
    }
  }
  
}

void getPath(int i, int j, vector< vector <int> >& P, vector <int>& path){
  if (i == j){
    //cout << i << " ";
    path.push_back(i);
  }else if (P[i][j] == -1){
    cout << "Path does not exist" << endl;
  }else{
    getPath(i, P[i][j], P, path);
    //cout << j << " ";
    path.push_back(j);
  }
}


void printMatrix(vector< vector<int> >& matrix){
  int i, j ,k;
  int n = matrix.size();
  printf("%s\n   ", TC_GREEN);
  for(j=0; j<n; j++){
   spaced_cout(j);
  }
  printf("%s", TC_NONE);
  cout << endl;
  for(j=0; j<n; j++){
    printf("%s", TC_GREEN);
    spaced_cout(j);
    printf("%s", TC_NONE);
    for(k=0; k<n; k++){
      if (matrix[j][k] == Inf || matrix[j][k] == -1) cout << "  -";
      else{
        spaced_cout(matrix[j][k]);
      }
    }
    cout << endl;
  }
  cout << endl;
}

void spaced_cout(int value){
    if(value < 10) cout << "  " << value;
    else cout << " " << value;
}

