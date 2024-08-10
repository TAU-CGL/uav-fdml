#include "experiment_options.h"
#include <le3.h>
using namespace le3;

#include <fdml/fdml.h>
#include <fdml/random.h>

void LoadEnvironment(le3::LE3AssetManager& assets, std::list<Triangle>& triangles, AABBTree& tree, fdml::R3xS1_Voxel& boundingBox);
fdml::R3xS1 SampleValidOdometry(AABBTree& tree, fdml::R3xS1 q, fdml::R3xS1_Voxel boundingBox);

BEGIN_EXPERIMENT("Test accuracy (and success rate) of localization in an environment with random valid k")
    ADD_OPTION(int, k, 10, "number of odometries");
    ADD_OPTION(int, depth, 10, "localization recursion depth");
    ADD_OPTION(std::string, environment, "/fdml/scans/240521-141038/240521-141038-scaled.obj", "Path to OBJ file of the scene");
    PARSE_ARGS();

    le3::LE3AssetManager assets;
    std::list<Triangle> triangles;
    AABBTree tree;
    fdml::R3xS1_Voxel boundingBox;
    
    LE3Application app(std::make_unique<LE3GameLogic>());
    app.init();
    
    le3::LE3GetDatFileSystem().addArchive("fdml", "fdml.dat");
    assets.addStaticMesh("environment", environment, true);
    LoadEnvironment(assets, triangles, tree, boundingBox);
    
    double accuracy = 0.f;
    int success = 0;

    START_RUN();
        fdml::R3xS1 q0(Point(0, 0, 0), 0), q;
        q0 = SampleValidOdometry(tree, q0, boundingBox);
        q = q0;

        fdml::OdometrySequence odometrySequence;
        odometrySequence.push_back(fdml::R3xS1(Point(0, 0, 0), 0));
        for (int i = 0; i < k - 1; i++) {
            fdml::R3xS1 g_i = SampleValidOdometry(tree, q, boundingBox);
            q = g_i * q;
            odometrySequence.push_back(g_i);
        }

        fdml::OdometrySequence groundTruths = fdml::getGroundTruths(odometrySequence, q0);
        fdml::MeasurementSequence measurements = fdml::getMeasurementSequence(tree, groundTruths);
        fdml::VoxelCloud localization = fdml::localize(tree, odometrySequence, measurements, boundingBox, depth);

        for (auto v : localization) {
            if (v.contains(q0)) {
                success++; break;
            }
        }
    
    END_RUN();

    std::cout << "100" << std::endl;
    std::cout << 100.0 * (double)success / (double)__num_experiments << std::endl;
END_EXPERIMENT()


void LoadEnvironment(le3::LE3AssetManager& assets, std::list<Triangle>& triangles, AABBTree& tree, fdml::R3xS1_Voxel& boundingBox) {
    auto mesh = assets.getStaticMesh("environment");
    auto vertices = mesh->getKeptData();
    auto indices = mesh->getKeptIndices();

    for (int i = 0; i < indices.size(); i += 3) {
        Point p1(vertices[indices[i]].position[0], vertices[indices[i]].position[2], vertices[indices[i]].position[1]);
        Point p2(vertices[indices[i + 1]].position[0], vertices[indices[i + 1]].position[2], vertices[indices[i + 1]].position[1]);
        Point p3(vertices[indices[i + 2]].position[0], vertices[indices[i + 2]].position[2], vertices[indices[i + 2]].position[1]);
        Triangle t(p1, p2, p3);
        triangles.push_back(t);
    }
    tree = AABBTree(triangles.begin(), triangles.end());
    boundingBox.bottomLeftPosition = Point(tree.bbox().xmin(), tree.bbox().ymin(), tree.bbox().zmin());
    boundingBox.topRightPosition = Point(tree.bbox().xmax(), tree.bbox().ymax(), tree.bbox().zmax());
    boundingBox.bottomLeftRotation = 0.0f;
    boundingBox.topRightRotation = 2.f * M_PI;
}

fdml::R3xS1 SampleValidOdometry(AABBTree& tree, fdml::R3xS1 q, fdml::R3xS1_Voxel boundingBox) {
    while (true) {
        FT x = boundingBox.bottomLeftPosition.x() + fdml::Random::randomDouble() * (boundingBox.topRightPosition.x() - boundingBox.bottomLeftPosition.x());
        FT y = boundingBox.bottomLeftPosition.y() + fdml::Random::randomDouble() * (boundingBox.topRightPosition.y() - boundingBox.bottomLeftPosition.y());
        FT z = boundingBox.bottomLeftPosition.z() + fdml::Random::randomDouble() * (boundingBox.topRightPosition.z() - boundingBox.bottomLeftPosition.z());
        FT r = fdml::Random::randomDouble() * 2 * M_PI;
        fdml::R3xS1 g(Point(x, y, z), r);
        fdml::R3xS1 q_ = g * q;
        if (q_.measureDistance(tree) < 0) continue;
        return g;
    }
}