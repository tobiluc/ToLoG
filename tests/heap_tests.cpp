// #include <gtest/gtest.h>
// #include <ToLoG/Heap.hpp>
// #include <queue>
// #include <random>

// namespace ToLoG
// {

// TEST(HeapTest, SimpleMinHeap)
// {
//     Heap<double,std::greater<double>> q;
//     EXPECT_TRUE(q.empty());
//     q.insert(10);
//     q.insert(4);
//     q.insert(7);
//     EXPECT_EQ(3, q.size());
//     EXPECT_EQ(4, q.front());
//     q.pop_front();
//     EXPECT_EQ(2, q.size());
//     EXPECT_EQ(7, q.front());
//     q.pop_front();
//     EXPECT_EQ(1, q.size());
//     EXPECT_EQ(10, q.front());
// }

// TEST(HeapTest, SimpleMaxHeap)
// {
//     Heap<double,std::less<double>> q;
//     EXPECT_TRUE(q.empty());
//     q.insert(10);
//     q.insert(4);
//     q.insert(7);
//     EXPECT_EQ(3, q.size());
//     EXPECT_EQ(10, q.front());
//     q.pop_front();
//     EXPECT_EQ(2, q.size());
//     EXPECT_EQ(7, q.front());
//     q.pop_front();
//     EXPECT_EQ(1, q.size());
//     EXPECT_EQ(4, q.front());
// }

// TEST(HeapTest, PerformanceTest)
// {
//     std::chrono::steady_clock::time_point t1, t2;
//     std::mt19937 gen(42);
//     std::uniform_real_distribution<double> rand(-9999.0, 9999.0);
//     size_t n = 10000000;

//     Heap<double> heap(n);
//     std::priority_queue<double> q;

//     // Insertion
//     long long dth = 0, dtq = 0;
//     for (int i = 0; i < n; ++i) {
//         double v = rand(gen);

//         t1 = std::chrono::steady_clock::now();
//         heap.insert(v);
//         t2 = std::chrono::steady_clock::now();
//         dth += std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();

//         t1 = std::chrono::steady_clock::now();
//         q.emplace(v);
//         t2 = std::chrono::steady_clock::now();
//         dtq += std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();

//     }
//     EXPECT_EQ(n, heap.size());
//     EXPECT_EQ(n, q.size());

//     std::cerr << "Heap Insertion:" << std::endl;
//     std::cerr << dth << "[µs] (Heap)" << std::endl;
//     std::cerr << dtq << "[µs] (std::priority_queue)" << std::endl;

//     // Popping
//     dth = dtq = 0;
//     for (int i = 0; i < n; ++i) {
//         double v = heap.front();
//         EXPECT_EQ(v, q.top());

//         t1 = std::chrono::steady_clock::now();
//         heap.pop_front();
//         t2 = std::chrono::steady_clock::now();
//         dth += std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();

//         t1 = std::chrono::steady_clock::now();
//         q.pop();
//         t2 = std::chrono::steady_clock::now();
//         dtq += std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();

//     }

//     std::cerr << "Heap Popping:" << std::endl;
//     std::cerr << dth << "[µs] (Heap)" << std::endl;
//     std::cerr << dtq << "[µs] (std::priority_queue)" << std::endl;

//     // Check empty
//     EXPECT_TRUE(heap.empty());
//     EXPECT_TRUE(q.empty());
// }

// }
