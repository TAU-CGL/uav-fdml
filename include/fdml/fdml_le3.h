/*
* Utilities for combining FDML with LightEngine3, for the demo & experiments.
* This is not necessary for running the FDML method in general.
*/

#pragma once

#include <string>

#include <le3.h>
#include <fdml/fdml_utils.h>


// We assume that the environment is a static mesh loaded in the asset manager
static void FDML_LE3_LoadEnvironment(le3::LE3AssetManager& assets, std::string envName, fdml::ExperimentEnv& env) {
    auto mesh = assets.getStaticMesh(envName);
    auto vertices = mesh->getKeptData();
    auto indices = mesh->getKeptIndices();

    // Since in LightEngine3 the up axis is Y, we need to swap the Y and Z coordinates
    std::list<Triangle> triangles;
    for (int i = 0; i < indices.size(); i += 3) {
        Point p1(vertices[indices[i]].position[0], vertices[indices[i]].position[2], vertices[indices[i]].position[1]);
        Point p2(vertices[indices[i + 1]].position[0], vertices[indices[i + 1]].position[2], vertices[indices[i + 1]].position[1]);
        Point p3(vertices[indices[i + 2]].position[0], vertices[indices[i + 2]].position[2], vertices[indices[i + 2]].position[1]);
        Triangle t(p1, p2, p3);
        triangles.push_back(t);
    }
    env.loadTriangles(triangles);
}


