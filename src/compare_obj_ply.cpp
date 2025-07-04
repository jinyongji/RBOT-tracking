/*
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <limits>
#include <algorithm>
#include <cfloat>


struct Point3f {
    float x, y, z;
};

std::vector<Point3f> loadOBJ(const std::string& filename) {
    std::vector<Point3f> vertices;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        if (line.substr(0, 2) == "v ") {
            std::istringstream iss(line.substr(2));
            Point3f p;
            iss >> p.x >> p.y >> p.z;
            vertices.push_back(p);
        }
    }
    return vertices;
}

std::vector<Point3f> loadPLY(const std::string& filename) {
    std::vector<Point3f> vertices;
    std::ifstream file(filename);
    std::string line;
    int vertexCount = 0;

    while (std::getline(file, line)) {
        if (line.find("element vertex") != std::string::npos) {
            std::istringstream iss(line);
            std::string word;
            iss >> word >> word >> vertexCount;
        } else if (line == "end_header") {
            break;
        }
    }

    for (int i = 0; i < vertexCount; ++i) {
        std::getline(file, line);
        std::istringstream iss(line);
        Point3f p;
        iss >> p.x >> p.y >> p.z;
        vertices.push_back(p);
    }

    return vertices;
}

void compareModels(const std::vector<Point3f>& a, const std::vector<Point3f>& b, const std::string& nameA, const std::string& nameB) {
    auto computeStats = [](const std::vector<Point3f>& pts) {
        Point3f minPt = {FLT_MAX, FLT_MAX, FLT_MAX};
        Point3f maxPt = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
        Point3f center = {0, 0, 0};

        for (const auto& p : pts) {
            minPt.x = std::min(minPt.x, p.x);
            minPt.y = std::min(minPt.y, p.y);
            minPt.z = std::min(minPt.z, p.z);
            maxPt.x = std::max(maxPt.x, p.x);
            maxPt.y = std::max(maxPt.y, p.y);
            maxPt.z = std::max(maxPt.z, p.z);
        }

        center.x = (minPt.x + maxPt.x) / 2;
        center.y = (minPt.y + maxPt.y) / 2;
        center.z = (minPt.z + maxPt.z) / 2;

        return std::make_tuple(minPt, maxPt, center);
    };

    auto [minA, maxA, centerA] = computeStats(a);
    auto [minB, maxB, centerB] = computeStats(b);

    std::cout << "=== Model Comparison ===\n";
    std::cout << nameA << " vertex count: " << a.size() << "\n";
    std::cout << nameB << " vertex count: " << b.size() << "\n\n";

    std::cout << nameA << " center: (" << centerA.x << ", " << centerA.y << ", " << centerA.z << ")\n";
    std::cout << nameB << " center: (" << centerB.x << ", " << centerB.y << ", " << centerB.z << ")\n\n";

    std::cout << nameA << " range x: [" << minA.x << ", " << maxA.x << "]\n";
    std::cout << nameB << " range x: [" << minB.x << ", " << maxB.x << "]\n";

    std::cout << nameA << " range y: [" << minA.y << ", " << maxA.y << "]\n";
    std::cout << nameB << " range y: [" << minB.y << ", " << maxB.y << "]\n";

    std::cout << nameA << " range z: [" << minA.z << ", " << maxA.z << "]\n";
    std::cout << nameB << " range z: [" << minB.z << ", " << maxB.z << "]\n";
}

int main() {
    std::string objPath = "/media/jyj/JYJ/RBOT-dataset/RBOT_dataset_p2/cat/cat_simple.obj";
    std::string plyPath = "/media/jyj/JYJ/RBOT-dataset/RBOT_dataset_p2/cat/cat.ply";

    std::vector<Point3f> objPoints = loadOBJ(objPath);
    std::vector<Point3f> plyPoints = loadPLY(plyPath);

    compareModels(objPoints, plyPoints, "OBJ", "PLY");

    return 0;
}
*/
