#ifndef VECTORSAMPLE_H
#define VECTORSAMPLE_H

#include <vector>
#include <iostream>

using namespace std;

class VectorSample
{
public:
    VectorSample(int lim);

    void test_front();
    void test_back();
    void test_at(int i);
    void test_pop_back();
    void test_op_equal();
    void test_ref(vector<int> &ref_vect);

    static void show_message();

    static int stat_num;
private:
    vector<int> squares;
    vector<int> equal;


};

#endif // VECTORSAMPLE_H
