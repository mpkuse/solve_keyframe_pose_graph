//Modified version of http://www.geeksforgeeks.org/breadth-first-traversal-for-a-graph/

# include <iostream>
# include <algorithm>
# include <vector>
# include <list>
#include <iomanip>

using namespace std;

#if 0
class MyDirectionalGraph
{
    vector< vector<int> > Edge;
    vector<unsigned char> visited; //We can use vector<bool>, but it's slow and broken :( , CPP's byte array http://stackoverflow.com/questions/10077771/what-is-the-correct-way-to-deal-with-medium-sized-byte-arrays-in-modern-c
    vector<int> parent; //to keep track of parent in BFS for path computation
public:

    MyDirectionalGraph(int V)
    {
        Edge.resize(V);
        visited.resize(V);
        parent.resize(V);
    }

    void reset()
    {
        for(auto i : visited) i = false;
        for(auto i : parent) i = -1;
    }

    void add_edge(int v,int w)
    {
        Edge[v].push_back(w);
    }

    void BFS(int s)
    {
        list<int> q;
        visited[s] = true;
        parent[s] = -2;
        q.push_back(s);
        while (!q.empty())
        {
            s = q.front();
            // cout<<s<<" ";
            q.pop_front();
            for(auto i : Edge[s])
            {
                if(!visited[i])
                {
                    visited[i] = true;
                    parent[i] = s;
                    q.push_back(i);
                }
            }
        }


        #if 0
        cout << endl << "---\n";
        for( int i=0 ; i<parent.size() ; i++ ) {
            cout << " i=" << std::setw(4) << i ;
            cout << " parent=" << std::setw(4) << parent[i] ;
            cout << " visited=" << std::setw(4) << (bool)visited[i] ;
            cout << endl;
        }
        cout << "---\n";
        #endif

    }

    // returns path from v to root.
    void get_path_from( int v, vector<int>& pathl )
    {
        if( (bool)visited[v] == false )
            return; //no path

        for( int i=0 ; i<100 ; i++ ) //maximum 100 steps
        {
            pathl.push_back( v );
            if( parent[v] == -2 )
                break;
            v = parent[v];
        }
    }

};
#endif

#include "utils/MyDirectionalGraph.h"

int main()
{
    MyDirectionalGraph g(7);
    g.add_edge(0,1);
    g.add_edge(0,2);
    g.add_edge(1,2);
    g.add_edge(2,0);
    g.add_edge(2,3);
    // g.add_edge(3,4);
    g.add_edge(4,5);
    g.add_edge(4,6);
    g.add_edge(5,6);
    int root=2;
    // cout<<"Enter vertex from where to perform BFS"<<endl;
    // cin>>v;
    cout << "start bfs with root=" << root << endl;
    g.BFS(root);

    g.print_intermediate_debug(); 

    int u = 0;
    cout << "path from u=" << u << " to root= " << root << ": ";
    vector<int> path;
    g.get_path_from( u, path );
    if( path.size() == 0 ) cout << " No path ";
    for( int i=0 ; i<path.size() ; i++ )
        cout << path[i] << "--->";
    cout << endl;

}
