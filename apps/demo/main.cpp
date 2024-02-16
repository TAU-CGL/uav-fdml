#include <queue>

#include <fmt/core.h>
using fmt::print, fmt::format;

#include <se3loc/se3loc.h>

int main() {
    se3loc::Random::seed(0);

    boost::container::vector<se3loc::Point<2, double>> points = {
        se3loc::Point<2, double>(2, 0.548813502304256, 0.5928446163889021),
        se3loc::Point<2, double>(2, 0.715189364971593, 0.8442657440900803),
        se3loc::Point<2, double>(2, 0.6027633703779429, 0.8579456198494881),
        se3loc::Point<2, double>(2, 0.5448831773828715, 0.8472517372574657),
        se3loc::Point<2, double>(2, 0.42365479678846896, 0.6235636963974684),
        se3loc::Point<2, double>(2, 0.6458941150922328, 0.3843817082233727),
        se3loc::Point<2, double>(2, 0.43758720997720957, 0.29753460525535047),
        se3loc::Point<2, double>(2, 0.8917730017565191, 0.05671297572553158),
        se3loc::Point<2, double>(2, 0.963662764057517, 0.27265629451721907),
        se3loc::Point<2, double>(2, 0.38344152132049203, 0.4776651116553694),
        se3loc::Point<2, double>(2, 0.79172503342852, 0.8121687264647335),
        se3loc::Point<2, double>(2, 0.5288949215319008, 0.4799771714024246),
        se3loc::Point<2, double>(2, 0.568044563755393, 0.3927847931627184),
        se3loc::Point<2, double>(2, 0.925596633227542, 0.8360787688288838),
        se3loc::Point<2, double>(2, 0.07103605871088803, 0.3373961616307497),
        se3loc::Point<2, double>(2, 0.08712929696775973, 0.6481718765571713),
    };

    for (auto point : points) {
        print("{} {}\n", point[0], point[1]);
    }

    se3loc::KDTree<2, double> kdtree;
    kdtree.fit(points);
    
    print("{}\n", kdtree.str());
    
    return 0;
}