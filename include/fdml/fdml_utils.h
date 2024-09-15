#pragma once

#include <fdml/fdml.h>

#include <set>
#include <ctime>
#include <list>
#include <vector>
#include <memory>

#include <fmt/core.h>

#include <CGAL/Kd_tree.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/K_neighbor_search.h>

using Kd_tree_Traits = CGAL::Search_traits_3<K>;
using Kd_tree = CGAL::Kd_tree<Kd_tree_Traits>;
using K_neighbor_search = CGAL::K_neighbor_search<Kd_tree_Traits>;

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>

namespace fdml {

    class Random {
    public:
        static double randomDouble() {
            boost::random::uniform_real_distribution<> dist(0.0, 1.0);
            return dist(instance()->rng);
        }

        static double randomGaussian(double sigma2) {
            boost::random::normal_distribution<> dist(0.0, sigma2);
            return dist(instance()->rng);
        }

        static int randomInt() {
            // Like randomDouble but for int
            double d = randomDouble();
            return (int)(d * (double)(0x01 << 30));
        }

        static void seed(int32_t seed = -1) {
            if (seed < 0) seed = std::time(0);
            instance()->rng.seed((uint32_t)seed);
        }

    private:
        static Random* instance() {
            static Random* _i = new Random();
            return _i;
        }

        Random() {}
        boost::mt19937 rng;
    };

    // Class representing a roadmap of valid drone position (orientation is ignored)
    // Can be used to generate random trajectories
    struct RoadmapNode {
        Point p;
        std::vector<std::shared_ptr<RoadmapNode>> neighbors;
    };
    class Roadmap {
    public:
        void addNode(Point p) {
            m_nodes.push_back(std::make_shared<RoadmapNode>(RoadmapNode{p}));
            m_nodeMap[p] = m_nodes.back();
        }

        void downsample(int numSamples = 2500) {
            if (m_nodes.size() <= numSamples) return;
            std::vector<std::shared_ptr<RoadmapNode>> newNodes;
            for (int i = 0; i < numSamples; i++) {
                newNodes.push_back(m_nodes[Random::randomInt() % m_nodes.size()]);
            }
            m_nodes = newNodes;
        }

        void buildRoadmap(int k = 15) {
            // Build KDTree (CGAL)
            std::list<Point> points;
            for (auto n : m_nodes) {
                points.push_back(n->p);
            }
            Kd_tree kdTree(points.begin(), points.end());

            // Find k-nearest neighbors for each node
            for (auto n : m_nodes) {
                K_neighbor_search kns(kdTree, n->p, k);
                // TODO: Collision checking
                for (auto it = kns.begin(); it != kns.end(); it++) {
                        n->neighbors.push_back(m_nodeMap[it->first]);
                }    
            }
        }

        std::vector<std::shared_ptr<RoadmapNode>>& getNodes() {
            return m_nodes;
        }

        OdometrySequence randomWalk(int numSteps, FT minDistance = 3.0) {
            OdometrySequence groundTruths;
            auto currentNode = m_nodes[Random::randomInt() % m_nodes.size()];
            groundTruths.push_back(R3xS1(currentNode->p, 0));
            int numTries = 0;
            while (groundTruths.size() < numSteps) {
                // TODO: Sample by most gradual angle with last node
                auto nextNode = currentNode->neighbors[Random::randomInt() % currentNode->neighbors.size()];
                if (CGAL::squared_distance(nextNode->p, groundTruths.back().position) >= minDistance * minDistance)
                    groundTruths.push_back(R3xS1(nextNode->p, 0));
                currentNode = nextNode;

                if (numTries > 100) {
                    minDistance -= 0.25;
                    if (minDistance < 0) minDistance = 0;
                    numTries = 0;
                }
                numTries++;
            }

            return groundTruths;
        }


    private:
        std::vector<std::shared_ptr<RoadmapNode>> m_nodes;
        std::map<Point, std::shared_ptr<RoadmapNode>> m_nodeMap;
    };

    struct ExperimentParams {
        int k = 10;
        FT delta = 0.05;
        FT epsilon = 0.05;

        bool enforceGoodTrajectory = false;
    };

    struct ExperimentMetrics {
        int conservativeSuccess = 0;
        FT errorXYZ = 0, errorTheta = 0;
        int numVoxels = 0, numClusters = 0;
        FT localizationVolume = 0, localizationVolumePercentage = 0;
        FT localizationVolumeXYT = 0, localizationVolumePercentageXYT = 0;
        double timeMiliseconds = 0;
        
        bool resultsReady = false;
        int numTimeouts = 0;

        void operator+=(const ExperimentMetrics& other) {
            if (other.numVoxels == 0) {
                numTimeouts++;
                return;
            }
            conservativeSuccess += other.conservativeSuccess;
            errorXYZ += other.errorXYZ;
            errorTheta += other.errorTheta;
            numVoxels += other.numVoxels;
            numClusters += other.numClusters;
            localizationVolume += other.localizationVolume;
            localizationVolumePercentage += other.localizationVolumePercentage;
            localizationVolumeXYT += other.localizationVolumeXYT;
            localizationVolumePercentageXYT += other.localizationVolumePercentageXYT;
            timeMiliseconds += other.timeMiliseconds;
        }
    };

    // Class that encapsulates the environment of an experiment, and utility functions that help run the experiment
    class ExperimentEnv {
    public:
        ExperimentEnv() {}

        void loadTriangles(std::list<Triangle> triangles) {
            m_triangles = triangles;
            m_tree = AABBTree(m_triangles.begin(), m_triangles.end());
            m_tree.accelerate_distance_queries();

            // Compute bounding box
            m_boundingBox.bottomLeftPosition = Point(m_tree.bbox().xmin(), m_tree.bbox().ymin(), m_tree.bbox().zmin());
            m_boundingBox.topRightPosition = Point(m_tree.bbox().xmax(), m_tree.bbox().ymax(), m_tree.bbox().zmax());
            m_boundingBox.bottomLeftRotation = 0.0f;
            m_boundingBox.topRightRotation = 2.f * M_PI;

            // Construct roadmap
            buildRoadmap();
        }

        R3xS1_Voxel& getBoundingBox() {
            return m_boundingBox;
        }

        AABBTree& getTree() {
            return m_tree;
        }

        Roadmap& getRoadmap() {
            return m_roadmap;
        }

        void clear() {
            odometrySequence.clear();
            groundTruths.clear();
            measurements.clear();
            localization.clear();
        }

        void runExperiment(int k, FT delta, FT epsilon, bool noise, bool enforceGoodTrajectory = false) {
            clear();
            if (predeterminedPath == "") {
                generateRandomWalk(k, enforceGoodTrajectory);
                measurements = getMeasurementSequence(m_tree, groundTruths);
            } else {
                loadTrajectory();
            }
            odometrySequence.push_back(R3xS1(Point(0, 0, 0), 0));
            for (int i = 1; i < groundTruths.size(); i++) {
                odometrySequence.push_back(groundTruths[i] / groundTruths[i - 1]);
            }

            // Do localization
            ErrorBounds errorBounds;
            errorBounds.errorDistance = epsilon;
            errorBounds.errorOdometryX = epsilon;
            errorBounds.errorOdometryY = epsilon;
            errorBounds.errorOdometryZ = epsilon;
            errorBounds.errorOdometryR = epsilon;

            if (noise) noiseOdometries(epsilon);
            
            std::chrono::steady_clock::time_point begin, end;    
            std::chrono::duration<double, std::milli> __duration;
            begin = std::chrono::steady_clock::now();

                localization = localize(m_tree, odometrySequence, measurements, m_boundingBox, getDepthFromDelta(delta), errorBounds);
                predictions = fdml::clusterLocations(localization);
            
            end = std::chrono::steady_clock::now();
            __duration = end - begin;

            // Populate metrics
            metrics.timeMiliseconds = __duration.count();
            updateExperimentMetrics();
        }

        void runExperiment(ExperimentParams params) {
            runExperiment(params.k, params.delta, params.epsilon, params.epsilon < 1e-6, params.enforceGoodTrajectory); 
        }

        void generateRandomWalk(int k, bool enforceGoodTrajectory) {
            groundTruths = m_roadmap.randomWalk(k);
            int numTries = 0;
            if (enforceGoodTrajectory) {
                while (!isGoodTrajectory()) {
                    groundTruths = m_roadmap.randomWalk(k);
                    numTries++;
                    if (numTries > 100) break;
                }
            }
            // fmt::print("Good trajectory == {}\n", isGoodTrajectory());
            fflush(stdout);
            q0 = groundTruths[0];
        }

        void loadTrajectory() {
            // Trajectories are of the form "dx,dy,dz,tof\n"
            // The string predeterminedPath is the trajectory itself
            std::istringstream ss(predeterminedPath);
            std::string line;
            while (std::getline(ss, line)) {
                std::istringstream lss(line);
                FT dx, dy, dz, tof;
                lss >> dx >> dy >> dz >> tof;
                groundTruths.push_back(R3xS1(Point(dx, dy, dz), 0));
                measurements.push_back(tof);
            }

            if (actualQ0 != "") {
                std::istringstream q0ss(actualQ0);
                FT x, y, z, theta;
                q0ss >> x >> y >> z >> theta;
                q0 = R3xS1(Point(x, y, z), theta);
                for (int i = 1; i < groundTruths.size(); i++) {
                    groundTruths[i] = (groundTruths[0] / groundTruths[i]) * q0;
                }
                groundTruths[0] = q0; 
            } else q0 = groundTruths[0];
        }

        bool isGoodTrajectory() {
            FT d0 = groundTruths[0].measureDistance(m_tree);
            FT d1 = groundTruths[1].measureDistance(m_tree) - (groundTruths[1].position.z() - groundTruths[0].position.z());
            FT d2 = groundTruths[2].measureDistance(m_tree) - (groundTruths[2].position.z() - groundTruths[0].position.z());
            FT d3 = groundTruths[3].measureDistance(m_tree) - (groundTruths[3].position.z() - groundTruths[0].position.z());
            FT d4 = groundTruths[4].measureDistance(m_tree) - (groundTruths[4].position.z() - groundTruths[0].position.z());
            // fmt::print("d0 = {}, d1 = {}, d2 = {}, d3 = {}, d4 = {}\n", d0, d1, d2, d3, d4);
            // fflush(stdout);
            // Check that they are all pairwise different (with difference at least 1e-1)
            FT eps = 2e-1;
            return (abs(d0 - d1) > eps) && (abs(d0 - d2) > eps) && (abs(d0 - d3) > eps) && 
                (abs(d1 - d2) > eps) && (abs(d1 - d3) > eps) && (abs(d2 - d3) > eps) && 
                (abs(d0 - d4) > eps) && (abs(d1 - d4) > eps) && (abs(d2 - d4) > eps) && (abs(d3 - d4) > eps);
        }

        void updateExperimentMetrics() {
            std::set<std::tuple<std::pair<FT, FT>, std::pair<FT, FT>, std::pair<FT, FT>>> xytProj;
            metrics.localizationVolume = 0;
            metrics.numVoxels = localization.size();
            metrics.numClusters = predictions.size();
            for (auto v : localization) {
                if (v.contains(q0)) metrics.conservativeSuccess = true;
                metrics.localizationVolume += v.volume();
                xytProj.insert(std::tuple<std::pair<FT, FT>, std::pair<FT, FT>, std::pair<FT, FT>>(
                    std::pair<FT, FT>(v.bottomLeftPosition.x(), v.bottomLeftPosition.y()),
                    std::pair<FT, FT>(v.topRightPosition.x(), v.topRightPosition.y()),
                    std::pair<FT, FT>(v.bottomLeftRotation, v.topRightRotation)
                ));
            }
            if (xytProj.size() > 0) metrics.localizationVolumeXYT = xytProj.size() * localization[0].volumeXYT();
            metrics.localizationVolumePercentage = metrics.localizationVolume / m_boundingBox.volume();
            metrics.localizationVolumePercentageXYT = metrics.localizationVolumeXYT / m_boundingBox.volumeXYT();

            // Compute best error
            metrics.errorXYZ = INFTY;
            metrics.errorTheta = INFTY;
            for (auto p : predictions) {
                FT error = sqrt((p.position.x() - q0.position.x()) * (p.position.x() - q0.position.x()) +
                    (p.position.y() - q0.position.y()) * (p.position.y() - q0.position.y()) +
                    (p.position.z() - q0.position.z()) * (p.position.z() - q0.position.z()));
                metrics.errorXYZ = MIN(metrics.errorXYZ, error);
                metrics.errorTheta = MIN(metrics.errorTheta, abs(p.orientation - q0.orientation));
            }
            
            metrics.resultsReady = true;
        }

    public:
        // Experiment results; after running `runExperiment`, these are guaranteed to have the correct values of the last run
        fdml::R3xS1 q0;
        fdml::OdometrySequence odometrySequence, groundTruths, predictions;
        fdml::MeasurementSequence measurements;
        std::vector<fdml::R3xS1_Voxel> localization;
        ExperimentMetrics metrics;
        std::string predeterminedPath, actualQ0;


    private:
        std::list<Triangle> m_triangles;
        AABBTree m_tree;
        R3xS1_Voxel m_boundingBox;
        Roadmap m_roadmap;

        int getDepthFromDelta(FT delta) {
            // delta0 is diamater of bounding box
            FT delta0 = sqrt(CGAL::squared_distance(m_boundingBox.bottomLeftPosition, m_boundingBox.topRightPosition));
            return (int)ceil(log2(delta0 / delta));
        }

        void buildRoadmap() {
            for (auto t : m_triangles) {
                auto n = t.supporting_plane().orthogonal_vector();
                if (abs(abs(n.z() / sqrt(n.squared_length())) - 1.0) > 0.5) continue;
                Point mid = Point((t[0].x() + t[1].x() + t[2].x()) / 3, (t[0].y() + t[1].y() + t[2].y()) / 3, (t[0].z() + t[1].z() + t[2].z()) / 3);
                for (int i = 0; i < 4; i++) {
                    Point pt(mid.x(), mid.y(), mid.z() + 0.5 * (i+1));
                    if (pt.z() > m_boundingBox.topRightPosition.z()) continue;
                    m_roadmap.addNode(pt);
                }
            }
            m_roadmap.downsample();
            m_roadmap.buildRoadmap();
        }

        Point samplePointInBB() {
            FT x = Random::randomDouble() * (m_boundingBox.topRightPosition.x() - m_boundingBox.bottomLeftPosition.x()) + m_boundingBox.bottomLeftPosition.x();
            FT y = Random::randomDouble() * (m_boundingBox.topRightPosition.y() - m_boundingBox.bottomLeftPosition.y()) + m_boundingBox.bottomLeftPosition.y();
            FT z = Random::randomDouble() * (m_boundingBox.topRightPosition.z() - m_boundingBox.bottomLeftPosition.z()) + m_boundingBox.bottomLeftPosition.z();
            return Point(x, y, z);
        }

        void noiseOdometries(FT epsilon) {
            for (int i = 0; i < odometrySequence.size(); i++) {
                odometrySequence[i].position = Point(
                    odometrySequence[i].position.x() + Random::randomGaussian(epsilon),
                    odometrySequence[i].position.y() + Random::randomGaussian(epsilon),
                    odometrySequence[i].position.z() + Random::randomGaussian(epsilon)
                );
                odometrySequence[i].orientation += Random::randomGaussian(epsilon);
            }
        }
    };

}