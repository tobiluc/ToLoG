#include <gtest/gtest.h>
#include <ToLoG/predicates/ExactPredicates.hpp>

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    exactinit();
    return RUN_ALL_TESTS();
}
