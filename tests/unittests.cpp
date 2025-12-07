#include <gtest/gtest.h>
#include <ToLoG/predicates/ExactPredicates.h>

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    exactinit();
    return RUN_ALL_TESTS();
}
