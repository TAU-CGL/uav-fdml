#pragma once

#include <fmt/core.h>
using fmt::format, fmt::print;

#include <gtest/gtest.h>

#include <se3loc/se3loc.h>


#define TEST_MAIN() \
    int main(int argc, char** argv) {\
        testing::InitGoogleTest(&argc, argv);\
        return RUN_ALL_TESTS();\
    }