#include <QApplication>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include "object3d.h"
#include "pose_estimator6d.h"

using namespace std;
using namespace cv;

// --------- 工具函数 ---------
Matx44f parsePoseLine(const string& line) {
    istringstream iss(line);
    float vals[12];
    for (int i = 0; i < 12; ++i) iss >> vals[i];
    return Matx44f(
            vals[0], vals[1], vals[2],  vals[9],
            vals[3], vals[4], vals[5],  vals[10],
            vals[6], vals[7], vals[8],  vals[11],
            0, 0, 0, 1
    );
}

vector<Matx44f> loadPoses(const string& filename) {
    vector<Matx44f> poses;
    ifstream file(filename);
    string line;
    while (getline(file, line)) {
        if (!line.empty()) poses.push_back(parsePoseLine(line));
    }
    return poses;
}

void savePoseToFile(const Matx44f& pose, const string& filename) {
    ofstream file(filename, ios::app);
    file << fixed << setprecision(6);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            file << pose(i, j) << "\t";
    for (int i = 0; i < 3; ++i)
        file << pose(i, 3) << "\t";
    file << "\n";
}

// ---------- 主程序 ----------
int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    int width = 640, height = 512;
    float zNear = 10.0f, zFar = 10000.0f;
    Matx33f K(650.048, 0, 324.328, 0, 647.183, 257.323, 0, 0, 1);
    Matx14f distCoeffs(0, 0, 0, 0);

    vector<Matx44f> gtDuck = loadPoses("poses_first.txt");
    vector<Matx44f> gtSquirrel = loadPoses("poses_second.txt");
    if (gtDuck.empty() || gtSquirrel.empty()) {
        cerr << "Failed to load ground truth poses." << endl;
        return -1;
    }

    vector<float> distances = {200.0f, 400.0f, 600.0f};

    // 跟踪目标 duck
    Object3D* duck = new Object3D("/media/jyj/JYJ/RBOT-dataset/ape/ape_simple.obj",
                                  0, 0, 500, 0, 0, 0, 1.0, 0.55f, distances);
    duck->setPose(gtDuck[0]);
    duck->setInitialPose(gtDuck[0]);

    // 遮挡物 squirrel（仅渲染）
    Object3D* squirrel = new Object3D("/media/jyj/JYJ/RBOT-dataset/RBOT_dataset_other/squirrel_small.obj",
                                      0, 0, 500, 0, 0, 0, 1.0, 0.55f, distances);
    squirrel->setPose(gtSquirrel[0]);

    vector<Object3D*> trackObjects = { duck };             // 仅 duck 用于估计
    vector<Object3D*> renderObjects = { duck, squirrel };  // duck + squirrel 用于渲染

    PoseEstimator6D* estimator = new PoseEstimator6D(width, height, zNear, zFar, K, distCoeffs, trackObjects);
    RenderingEngine::Instance()->makeCurrent();

    TickMeter timer;
    double totalTimeMs = 0.0;
    int validFrames = 0;
    int successFrames = 0;

    for (int i = 0; i <= 1000; ++i) {
        stringstream ss;
        ss << "/media/jyj/JYJ/RBOT-dataset/ape/frames/d_occlusion"
           << setfill('0') << setw(4) << i << ".png";
        Mat frame = imread(ss.str());
        if (frame.empty()) break;

        squirrel->setPose(gtSquirrel[i]);  // 仅渲染用，无需参与优化

        timer.start();

        if (i == 0) {
            estimator->toggleTracking(frame, 0, true);
            estimator->estimatePoses(frame, true, false);
        } else {
            estimator->estimatePoses(frame, true, false);

            // 误差检查（duck）
            Matx44f pred = duck->getPose();
            Matx44f gt = gtDuck[i];
            Vec3f t_pred(pred(0,3), pred(1,3), pred(2,3));
            Vec3f t_gt(gt(0,3), gt(1,3), gt(2,3));
            float t_err = norm((t_pred - t_gt) * 0.001f);

            Matx33f R_pred(pred.val);
            Matx33f R_gt(gt.val);
            Matx33f R_err = R_pred.t() * R_gt;
            float trace = R_err(0,0) + R_err(1,1) + R_err(2,2);
            float angle_rad = acosf(std::max(-1.0f, std::min(1.0f, (trace - 1.0f) / 2.0f)));

            if (t_err < 0.05f && angle_rad < 0.0873f)
                successFrames++;
            else {
                // fallback 初始化
                duck->setPose(gt);
                duck->setInitialPose(gt);
                estimator->toggleTracking(frame, 0, true);
                estimator->estimatePoses(frame, true, false);
            }
        }

        timer.stop();
        totalTimeMs += timer.getTimeMilli();
        validFrames++;
        timer.reset();

        savePoseToFile(duck->getPose(), "predicted_poses_ape_occ1.txt");

        // ✅ 可视化（建议注释以防崩溃）
//Mat rendering = drawResultOverlay(renderObjects, frame);
//        imshow("Tracking Result", rendering);
//        waitKey(1);





    }

    if (validFrames > 0) {
        cout << "---------------------------------------------" << endl;
        cout << "Avg. Runtime per frame: " << fixed << setprecision(2)
             << totalTimeMs / validFrames << " ms" << endl;
        cout << "Success Rate: " << fixed << setprecision(2)
             << (float(successFrames) / validFrames * 100.0f) << " %" << endl;
    }

    RenderingEngine::Instance()->doneCurrent();
    RenderingEngine::Instance()->destroy();
    delete duck;
    delete squirrel;
    delete estimator;
    return 0;
}
