#include <gtest/gtest.h>
#include <ToLoG/unstable/SimplexSolver.hpp>

namespace ToLoG
{

TEST(SparseTest, SparseVectorTest1)
{
    size_t n(100000000);
    SparseVector<double> v(n);
    v.set(0, 1);
    v.set(4, 5);
    EXPECT_EQ(n, v.size());
    EXPECT_EQ(1, v.at(0));
    EXPECT_EQ(5, v.at(4));
    EXPECT_EQ(26, v.dot(v));
    EXPECT_EQ(0, v.at(9999));
    auto w = -v;
    EXPECT_EQ(-w, v);
    EXPECT_EQ(-1, w.at(0));
    EXPECT_EQ(-5, w.at(4));
    EXPECT_EQ(-26, w.dot(v));

    v += w;
    v.prune();
    EXPECT_TRUE(v.is_zero());
}

TEST(SparseTest, SegmentTest)
{
    SparseVector<double> v(100);
    v.set(50, 111);
    v.set(51, 111);
    v.set(52, 111);
    v.set(53, 111);
    v.set(54, 111);

    SparseVector<double> s1(5);
    s1.set(0, 222);
    s1.set(4, 222);

    v.set_segment(50, s1);
    EXPECT_EQ(222, v.at(50));
    EXPECT_EQ(0, v.at(51));
    EXPECT_EQ(0, v.at(52));
    EXPECT_EQ(0, v.at(53));
    EXPECT_EQ(222, v.at(54));

    SparseMatrix<double,MatrixMajor::ROW> t(1,4);
    SparseMatrix<double,MatrixMajor::ROW> a(2,2);
    a.set(0,0, 111); a.set(0,1, 222);
    EXPECT_EQ(111, a.at(0,0));
    EXPECT_EQ(222, a.at(0,1));
    t.row(0).set(0,1.5);
    EXPECT_EQ(1.5, t.at(0,0));
    t.row(0).set_segment(0, a.row(0));
    EXPECT_EQ(111, t.at(0,0));
    EXPECT_EQ(222, t.at(0,1));
    EXPECT_EQ(t.row(0).get_segment(0,2), a.row(0));
}

TEST(SparseTest, SparseMatrixTest1)
{
    size_t R(10000);
    size_t C(10000);
    SparseMatrix<double, MatrixMajor::ROW> m(R, C);
    for (int row = 0; row < R; ++row) {
        EXPECT_TRUE(m.row(row).is_zero());
    }
}

TEST(SparseTest, SparseVectorArithmetic)
{
    SparseVector<double> a(5);
    SparseVector<double> b(5);

    a.set(0, 1.0);
    a.set(2, 3.0);

    b.set(1, 2.0);
    b.set(2, 4.0);

    auto c = a + b;
    EXPECT_EQ(c.at(0), 1.0);
    EXPECT_EQ(c.at(1), 2.0);
    EXPECT_EQ(c.at(2), 7.0);
    EXPECT_EQ(c.at(3), 0.0);

    a += b;
    EXPECT_EQ(a.at(0), 1.0);
    EXPECT_EQ(a.at(1), 2.0);
    EXPECT_EQ(a.at(2), 7.0);

    auto d = -a;
    EXPECT_EQ(d.at(0), -1.0);
    EXPECT_EQ(d.at(1), -2.0);
    EXPECT_EQ(d.at(2), -7.0);
}

TEST(SparseTest, SparseVectorDot)
{
    SparseVector<double> a(3);
    SparseVector<double> b(3);

    a.set(0, 1.0);
    a.set(2, 2.0);

    b.set(0, 3.0);
    b.set(1, 4.0);
    b.set(2, 5.0);

    double res = a.dot(b); // 1*3 + 2*5 = 13
    EXPECT_EQ(res, 13.0);
}

TEST(SparseTest, SparseVectorPrune)
{
    SparseVector<double> v(5);
    v.set(0, 1e-20);
    v.set(1, 2.0);
    v.set(2, 0.0);

    v.prune();
    EXPECT_FALSE(v.is_zero());
    EXPECT_EQ(v.at(0), 0.0);
    EXPECT_EQ(v.at(1), 2.0);
    EXPECT_EQ(v.at(2), 0.0);
}

TEST(SparseTest, SparseMatrixBasic)
{
    SparseMatrix<double, MatrixMajor::ROW> m(3, 3);
    EXPECT_EQ(m.n_rows(), 3);
    EXPECT_EQ(m.n_cols(), 3);

    m.set(0, 0, 1.0);
    m.set(1, 2, 2.0);

    EXPECT_EQ(m.at(0, 0), 1.0);
    EXPECT_EQ(m.at(1, 2), 2.0);
    EXPECT_EQ(m.at(2, 1), 0.0); // default zero
}

TEST(SparseTest, SparseMatrixRowColAccess)
{
    SparseMatrix<double, MatrixMajor::ROW> m(2, 3);
    EXPECT_EQ(2, m.n_rows());
    EXPECT_EQ(3, m.n_cols());
    EXPECT_EQ(3, m.row(0).size());
    EXPECT_EQ(3, m.row(1).size());
    m.row(0).set(1, 5.0);
    EXPECT_EQ(m.at(0, 1), 5.0);

    SparseMatrix<double, MatrixMajor::COL> mc(3, 2);
    EXPECT_EQ(3, mc.n_rows());
    EXPECT_EQ(2, mc.n_cols());
    EXPECT_EQ(3, mc.col(0).size());
    EXPECT_EQ(3, mc.col(1).size());
    mc.col(1).set(0, 7.0);
    EXPECT_EQ(mc.at(0, 1), 7.0);
}

TEST(SparseTest, SparseMatrixMultiply)
{
    SparseMatrix<double, MatrixMajor::ROW> m(2, 3);
    m.set(0, 0, 1.0);
    m.set(0, 2, 2.0);
    m.set(1, 1, 3.0);

    SparseVector<double> x(3);
    x.set(0, 2.0);
    x.set(1, 1.0);
    x.set(2, 1.0);

    SparseVector<double> y = m * x;
    EXPECT_EQ(y.size(), 2);
    EXPECT_EQ(y.at(0), 1*2 + 2*1); // 4
    EXPECT_EQ(y.at(1), 3*1);       // 3
}

TEST(SparseVectorTest, ArithmeticOps) {
    SparseVector<double> a(3), b(3);
    a.set(0, 1.0); a.set(2, 3.0);
    b.set(1, 2.0); b.set(2, 1.0);

    auto c = a + b;
    EXPECT_EQ(c.at(0), 1.0);
    EXPECT_EQ(c.at(1), 2.0);
    EXPECT_EQ(c.at(2), 4.0);

    c -= a;
    EXPECT_EQ(c.at(0), 0.0);
    EXPECT_EQ(c.at(1), 2.0);
    EXPECT_EQ(c.at(2), 1.0);

    auto d = -c;
    EXPECT_EQ(d.at(0), 0.0);
    EXPECT_EQ(d.at(1), -2.0);
    EXPECT_EQ(d.at(2), -1.0);

    EXPECT_EQ(a.dot(b), 3.0); // 3*1.0 = 3.0
}

TEST(SparseVectorTest, Prune) {
    SparseVector<double> v(3);
    v.set(0, 1e-20);
    v.set(1, 0.1);
    v.prune();
    EXPECT_EQ(v.at(0), 0.0);
    EXPECT_EQ(v.at(1), 0.1);
}

TEST(SparseMatrixTest, RowMajorBasic) {
    SparseMatrix<double, MatrixMajor::ROW> m(2, 3);

    // Initially zero
    EXPECT_TRUE(m.row(0).is_zero());
    EXPECT_TRUE(m.row(1).is_zero());

    // Set elements
    m.set(0, 1, 5.0);
    m.set(1, 2, 3.5);

    EXPECT_EQ(m.at(0, 1), 5.0);
    EXPECT_EQ(m.at(1, 2), 3.5);

    // Row access
    EXPECT_EQ(m.row(0).at(1), 5.0);
    EXPECT_EQ(m.row(1).at(2), 3.5);

    // Multiplication with SparseVector
    SparseVector<double> x(3);
    x.set(1, 2.0);
    x.set(2, 1.0);

    auto y = m * x; // y = m * x
    EXPECT_EQ(y.size(), 2);
    EXPECT_EQ(y.at(0), 10.0); // 5*2 + 0*1
    EXPECT_EQ(y.at(1), 3.5);  // 3.5*1
}

TEST(SparseMatrixTest, ColMajorBasic) {
    SparseMatrix<double, MatrixMajor::COL> mc(2, 3); // 2 rows, 3 cols

    // Initially zero
    for (int col = 0; col < 3; ++col) {
        EXPECT_TRUE(mc.col(col).is_zero());
    }

    // Set elements
    mc.set(0, 1, 7.0);
    mc.set(1, 2, 4.5);

    EXPECT_EQ(mc.at(0, 1), 7.0);
    EXPECT_EQ(mc.at(1, 2), 4.5);

    // Column access
    EXPECT_EQ(mc.col(1).at(0), 7.0);
    EXPECT_EQ(mc.col(2).at(1), 4.5);

    // Multiplication with SparseVector
    SparseVector<double> x(3);
    x.set(1, 2.0);
    x.set(2, 1.0);

    auto y = mc * x;
    EXPECT_EQ(y.size(), 2);
    EXPECT_EQ(y.at(0), 14.0); // 7*2 + 0*0
    EXPECT_EQ(y.at(1), 4.5);  // 4.5*1
}

TEST(SparseMatrixTest, Resize) {
    SparseMatrix<double, MatrixMajor::ROW> m(2, 2);
    m.set(0, 0, 1.0);
    m.set(1, 1, 2.0);

    m.resize(3, 4); // grow
    EXPECT_EQ(m.n_rows(), 3);
    EXPECT_EQ(m.n_cols(), 4);
    EXPECT_EQ(m.at(0,0), 1.0);
    EXPECT_EQ(m.at(1,1), 2.0);
    EXPECT_EQ(m.at(2,2), 0.0); // new entries are zero

    m.resize(2, 2); // shrink
    EXPECT_EQ(m.n_rows(), 2);
    EXPECT_EQ(m.n_cols(), 2);
    EXPECT_EQ(m.at(0,0), 1.0);
}

}
