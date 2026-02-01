#include <gtest/gtest.h>
#include <ToLoG/predicates/predicates_wrapper.hpp>

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    exactinit();
    return RUN_ALL_TESTS();
}
