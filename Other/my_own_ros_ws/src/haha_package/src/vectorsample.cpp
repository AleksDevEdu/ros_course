#include "vectorsample.h"

int VectorSample::stat_num = 0;

VectorSample::VectorSample(int lim)
{
    for ( int i = 0; i < lim; i++ )
    {
        squares.push_back(i*i);
    }
}

void VectorSample::test_front()
{
    cout << "vector.front() --- " <<
            squares.front() << endl;
}

void VectorSample::test_back()
{
    cout << "vector.back() --- " <<
            squares.back() << endl;
}

void VectorSample::test_at(int i)
{
    cout << "vector.at() --- " <<
            squares.at(i) << endl;
}

void VectorSample::test_pop_back()
{
    while (squares.size() > 0)
    {
        squares.pop_back();
        cout << "vector.pop_back() {size} --- " <<
                squares.size() << endl;
    }
}

void VectorSample::test_ref(vector<int> & ref_vect)
{
    for ( int i = 0; i < ref_vect.size(); i++ )
    {
        cout << "vector.test_ref() --- " <<
                ref_vect.at(i) << endl;
    }


}

void VectorSample::show_message()
{
    cout << "Hello! =)" << endl;
    stat_num += 2;
}

void VectorSample::test_op_equal()
{
    equal = squares;

    vector<int> &ref = equal;

    ref[2] = 11111;
    equal.at(3) = 9*6;
    test_ref( ref );
    squares.at(1) = 0;
    test_ref( squares );

}
























