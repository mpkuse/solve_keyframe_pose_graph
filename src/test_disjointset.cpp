// Testing the implementations of disjoint sets


#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;
#include "utils/FalseColors.h"

int main()
{
    for( int i=0 ; i<10; i++)
        cout << FalseColors::randomColor( i ) << endl;

}
#if 0
#include <iostream>

#include "utils/DisjointSet.h"
using namespace mp;
using namespace std;
int main()
{
    std::cout << "hello\n";
    mp::DisjointSetForest<float> ds;
    ds.add_element( 0, 213.1 );
    // ds.add_element( 0, 21.1 );
    ds.add_element( 1, 4.44 );
    ds.add_element( 2, -0.4 );
    ds.add_element( 3, -120.4 );
    ds.union_sets( 0, 1);
    cout << "exist(2) = " << ds.exists( 2 ) << endl;
    cout << "exist(56) = " << ds.exists( 56 ) << endl;
    // ds.union_sets( 0, 56);
    std::cout << "element count: " << ds.element_count() << std::endl;
    cout << "set_count: " << ds.set_count() << endl;


    for( int i=0 ; i<ds.element_count() ; i++ ) {
        cout << "element#" << i << "  find_set=" << ds.find_set( i ) ;
        cout << "  value=" << ds.value_of( i );
        cout << endl;
    }
}
#endif

#if 0
#include <iostream>
#include <vector>

#include <boost/pending/disjoint_sets.hpp>

int main()
{
    std::cout << "Hello\n";

    std::vector<int> rank(200);
    std::vector<int> parent(200);
    boost::disjoint_sets<int*, int*> ds( &rank[0], &parent[0] );

    int n=80;

    for( int i=0 ; i<n ; i++ ) {
        ds.make_set( i );
    }

    ds.union_set( 2, 0 );
    ds.union_set( 3, 2 );
    ds.union_set( 20, 3 );
    ds.union_set( 40, 20 );


    for( int i=0 ; i<n ; i++ ) {
        std::cout << i << " " << ds.find_set( i ) << std::endl;
    }
}
#endif
