/***
 * millipede: DisjointSetForest.h
 * Copyright Stuart Golodetz, 2009. All rights reserved.


 // Sample usage:
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


 ***/

#ifndef H_MILLIPEDE_DISJOINTSETFOREST
#define H_MILLIPEDE_DISJOINTSETFOREST

#include <map>

// #include <common/exceptions/Exception.h>
// #include <common/io/util/OSSWrapper.h>
// #include <common/util/NullType.h>

namespace mp {

/**
@brief  A disjoint set forest is a fairly standard data structure used to represent the partition of
        a set of elements into disjoint sets in such a way that common operations such as merging two
        sets together are computationally efficient.

This implementation uses the well-known union-by-rank and path compression optimizations, which together
yield an amortised complexity for key operations of O(a(n)), where a is the (extremely slow-growing)
inverse of the Ackermann function.

The implementation also allows clients to attach arbitrary data to each element, which can be useful for
some algorithms.

@tparam T   The type of data to attach to each element (arbitrary)
*/
template <typename T>
class DisjointSetForest
{
    //#################### NESTED CLASSES ####################
private:
    struct Element
    {
        T m_value;
        int m_parent;
        int m_rank;

        Element(const T& value, int parent)
        :   m_value(value), m_parent(parent), m_rank(0)
        {}
    };

    //#################### PRIVATE VARIABLES ####################
private:
    mutable std::map<int,Element> m_elements;
    int m_setCount;

    //#################### CONSTRUCTORS ####################
public:
    /**
    @brief  Constructs an empty disjoint set forest.
    */
    DisjointSetForest()
    :   m_setCount(0)
    {}

    /**
    @brief  Constructs a disjoint set forest from an initial set of elements and their associated values.

    @param[in]  initialElements     A map from the initial elements to their associated values
    */
    explicit DisjointSetForest(const std::map<int,T>& initialElements)
    :   m_setCount(0)
    {
        add_elements(initialElements);
    }

    //#################### PUBLIC METHODS ####################
public:
    /**
    @brief  Adds a single element x (and its associated value) to the disjoint set forest.

    @param[in]  x       The index of the element
    @param[in]  value   The value to initially associate with the element
    @pre
        -   x must not already be in the disjoint set forest
    */
    void add_element(int x, const T& value = T())
    {
        m_elements.insert(std::make_pair(x, Element(value, x)));
        ++m_setCount;
    }

    /**
    @brief  Adds multiple elements (and their associated values) to the disjoint set forest.

    @param[in]  elements    A map from the elements to add to their associated values
    @pre
        -   None of the elements to be added must already be in the disjoint set forest
    */
    void add_elements(const std::map<int,T>& elements)
    {
        for(typename std::map<int,T>::const_iterator it=elements.begin(), iend=elements.end(); it!=iend; ++it)
        {
            m_elements.insert(std::make_pair(it->first, Element(it->second, it->first)));
        }
        m_setCount += elements.size();
    }

    /**
    @brief  Returns the number of elements in the disjoint set forest.

    @return As described
    */
    int element_count() const
    {
        return static_cast<int>(m_elements.size());
    }

    /**
    @brief  Finds the index of the root element of the tree containing x in the disjoint set forest.

    @param[in]  x   The element whose set to determine
    @pre
        -   x must be an element in the disjoint set forest
    @throw Exception
        -   If the precondition is violated
    @return As described
    */
    int find_set(int x) const
    {
        Element& element = get_element(x);
        int& parent = element.m_parent;
        if(parent != x)
        {
            parent = find_set(parent);
        }
        return parent;
    }

    /**
    @brief  Returns the current number of disjoint sets in the forest (i.e. the current number of trees).

    @return As described
    */
    int set_count() const
    {
        return m_setCount;
    }

    /**
    @brief  Merges the disjoint sets containing elements x and y.

    If both elements are already in the same disjoint set, this is a no-op.

    @param[in]  x   The first element
    @param[in]  y   The second element
    @pre
        -   Both x and y must be elements in the disjoint set forest
    @throw Exception
        -   If the precondition is violated
    */
    void union_sets(int x, int y)
    {
        int setX = find_set(x);
        int setY = find_set(y);
        if(setX != setY) link(setX, setY);
    }

    /**
    @brief  Returns the value associated with element x.

    @param[in]  x   The element whose value to return
    @pre
        -   x must be an element in the disjoint set forest
    @throw Exception
        -   If the precondition is violated
    @return As described
    */
    T& value_of(int x)
    {
        return get_element(x).m_value;
    }

    /**
    @brief  Returns the value associated with element x.

    @param[in]  x   The element whose value to return
    @pre
        -   x must be an element in the disjoint set forest
    @throw Exception
        -   If the precondition is violated
    @return As described
    */
    const T& value_of(int x) const
    {
        return get_element(x).m_value;
    }


    /**
    @brief Returns true if element x exists. False other wise
    */
    bool exists( int x ) {
        if( m_elements.count( x ) == 0 ) return false;
        else return true;
    }

    //#################### PRIVATE METHODS ####################
private:
    Element& get_element(int x) const
    {
        typename std::map<int,Element>::iterator it = m_elements.find(x);
        if(it != m_elements.end()) return it->second;
        else throw ;//Exception( "No such element: " + std::to_string(x));
        // else throw Exception(OSSWrapper() << "No such element: " << x);
    }

    void link(int x, int y)
    {
        Element& elementX = get_element(x);
        Element& elementY = get_element(y);
        int& rankX = elementX.m_rank;
        int& rankY = elementY.m_rank;
        if(rankX > rankY)
        {
            elementY.m_parent = x;
        }
        else
        {
            elementX.m_parent = y;
            if(rankX == rankY) ++rankY;
        }
        --m_setCount;
    }
};

}

#endif
