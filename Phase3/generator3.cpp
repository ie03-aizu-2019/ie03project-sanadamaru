#include <iostream>
#include <random>
#include <map>
#include <utility>
using namespace std;

typedef pair<int, int> Pii;

int main() {
  int N = 1000, M = 1000, P = 1000, Q = 1000;
  int MAX_xy = 100000;

  cout << N << " " << M << " " << P << " " << Q << endl;
  
  random_device rnd;
  {
    map<Pii, bool> used;  
    for ( int i = 0; i < N; i++ ) {
      int x = rnd()%(MAX_xy+1);
      int y = rnd()%(MAX_xy+1);
      if ( used.count(Pii(x, y)) ) {
	i--;
	continue;
      }
      cout << x << " " << y << endl;
      used[Pii(x, y)] = true;
    }
    
    used.clear();  
    for ( int i = 0; i < M; i++ ) {
      int x = rnd()%(N)+1;
      int y = rnd()%(N)+1;
      if ( x > y ) swap(x, y);    
      if ( used.count(Pii(x, y)) || x == y ) {
	i--;
	continue;
      }
      cout << x << " " << y << endl;
      used[Pii(x, y)] = true;
    }

    used.clear();
    for ( int i = 0; i < P; i++ ) {
      int x = rnd()%(MAX_xy+1);
      int y = rnd()%(MAX_xy+1);
      if ( used.count(Pii(x, y)) ) {
	i--;
	continue;
      }
      cout << x << " " << y << endl;
      used[Pii(x, y)] = true;
    }

    used.clear();
    for ( int i = 0; i < Q; i++ ) {
      int x = rnd()%(N)+1;
      int y = rnd()%(N)+1;
      if ( rnd()%2 == 0 ) x += N;
      if ( rnd()%2 == 0 ) y += N;      
      if ( used.count(Pii(x, y)) ) {
	i--;
	continue;
      }
      if ( x > N ) cout << 'C' << x-N << " ";
      else cout << x << " ";
      if ( y > N ) cout << 'C' << y-N << " ";
      else cout << y << " ";
      cout << rnd()%10+1 << endl;      
      used[Pii(x, y)] = true;
    }
  }
  
  return 0;
}
