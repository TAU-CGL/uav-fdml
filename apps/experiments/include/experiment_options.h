#pragma once

#include <chrono>
#include <iostream>

#include <fmt/core.h>
using fmt::print, fmt::format;

#include <boost/program_options.hpp>
namespace po = boost::program_options;


#define BEGIN_EXPERIMENT(description) \
    int main(int argc, char** argv) {\
        po::options_description desc((description));\
        int32_t __num_experiments;\
        std::chrono::steady_clock::time_point begin, end;\
        std::chrono::duration<double, std::milli> __duration;\
        desc.add_options()\
            ("help", "produce help message")\
            ("num_experiments", po::value<int>(&__num_experiments)->default_value(10), "number of experiments");

#define END_EXPERIMENT() return 0; }

#define ADD_OPTION(type, varname, defaultval, description) \
    type varname; \
    desc.add_options()(#varname, po::value<type>(&varname)->default_value(defaultval), description); 

#define PARSE_ARGS() \
    po::variables_map vm;\
    po::store(po::parse_command_line(argc, argv, desc), vm);\
    po::notify(vm);\
    // if (vm.count("help")) { std::cout << desc << std::endl; return 1; }


#define START_RUN() \
    begin = std::chrono::steady_clock::now(); \
    for (int32_t __exp_idx = 0; __exp_idx < __num_experiments; ++__exp_idx) {

#define END_RUN() }\
    end = std::chrono::steady_clock::now();\
    __duration = end - begin;\
    // std::cout << __duration.count() << std::endl;