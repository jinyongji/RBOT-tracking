#include <QApplication>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/utility.hpp> // 添加 TickMeter
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include "object3d.h"
#include "pose_estimator6d.h"

using namespace std;
using namespace cv;

// 解析位姿行
Matx44f parsePoseLine(const string& line) {
    istringstream iss(line);
    float vals[12];
    for (int i = 0; i < 12; ++i) iss >> vals[i];
    return Matx44f(
            vals[0], vals[1], vals[2], vals[9],
            vals[3], vals[4], vals[5], vals[10],
            vals[6], vals[7], vals[8], vals[11],
            0, 0, 0, 1
    );
}

// 加载位姿文件
vector<Matx44f> loadPoses(const string& filename) {
    vector<Matx44f> poses;
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Failed to open file: " << filename << endl;
        return poses;
    }
    string line;
    while (getline(file, line)) {
        if (!line.empty()) poses.push_back(parsePoseLine(line));
    }
    return poses;
}

// 保存位姿到文件
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

// 绘制结果叠加
Mat drawResultOverlay(const vector<Object3D*>& objects, const Mat& frame) {
    RenderingEngine::Instance()->setLevel(0);
    vector<Point3f> colors = { Point3f(1.0, 0.5, 0.0) };
    RenderingEngine::Instance()->renderShaded(vector<Model*>(objects.begin(), objects.end()), GL_FILL, colors, true);

    Mat rendering = RenderingEngine::Instance()->downloadFrame(RenderingEngine::RGB);
    Mat depth = RenderingEngine::Instance()->downloadFrame(RenderingEngine::DEPTH);
    Mat result = frame.clone();

    for (int y = 0; y < frame.rows; y++)
        for (int x = 0; x < frame.cols; x++)
            if (depth.at<float>(y, x) != 0.0f)
                result.at<Vec3b>(y, x) = Vec3b(rendering.at<Vec3b>(y, x)[2], rendering.at<Vec3b>(y, x)[1], rendering.at<Vec3b>(y, x)[0]);

    return result;
}

// 主函数
int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    int width = 640, height = 512;
    float zNear = 10.0f, zFar = 10000.0f;
    Matx33f K(650.048, 0, 324.328, 0, 647.183, 257.323, 0, 0, 1);
    Matx14f distCoeffs(0, 0, 0, 0);

    vector<Matx44f> gtPoses = loadPoses("poses_first.txt");
    if (gtPoses.empty()) {
        cerr << "Failed to load ground truth poses." << endl;
        return -1;
    }

    //vector<float> distances = {200.0f, 300.0f, 400.0f, 500.0f, 600.0f};
    vector<float> distances = {200.0f, 400.0f,  600.0f};

    vector<Object3D*> objects;
    Object3D* obj = new Object3D("/media/jyj/JYJ/RBOT-dataset/cat/cat_simple.obj",
                                 0, 0, 500, 0, 0, 0, 1.0, 0.55f, distances);
    obj->setPose(gtPoses[0]);
    obj->setInitialPose(gtPoses[0]);
    objects.push_back(obj);

    PoseEstimator6D* estimator = new PoseEstimator6D(width, height, zNear, zFar, K, distCoeffs, objects);
    RenderingEngine::Instance()->makeCurrent();

    // 运行时统计
    TickMeter timer;
    double totalTimeMs = 0.0;
    int validFrames = 0;

    for (int i = 0; i <= 1000; ++i) {
        stringstream ss;
        //ss << "/media/jyj/JYJ/RBOT-dataset/RBOT_dataset_p3/duck/frames/a_regular"
        //ss << "/media/jyj/JYJ/RBOT-dataset/RBOT_dataset_p3/duck/frames/b_dynamiclight"
        //ss << "/media/jyj/JYJ/RBOT-dataset/RBOT_dataset_p3/duck/frames/c_noisy"
        ss << "/media/jyj/JYJ/RBOT-dataset/cat/frames/a_regular"
           << setfill('0') << setw(4) << i << ".png";
        Mat frame = imread(ss.str());
        if (frame.empty()) {
            cerr << "Failed to load frame: " << ss.str() << endl;
            break;
        }

        timer.start();

        if (i == 0) {
            estimator->toggleTracking(frame, 0, false);        // 使用直方图初始化
            //estimator->estimatePoses(frame, false, false);
        }

            estimator->estimatePoses(frame, false, false);

            // 检查误差
            Matx44f pred = objects[0]->getPose();
            Matx44f gt = gtPoses[i];
            Vec3f t_pred(pred(0,3), pred(1,3), pred(2,3));
            Vec3f t_gt(gt(0,3), gt(1,3), gt(2,3));
            float t_err = norm((t_pred - t_gt) * 0.001f); // mm -> m

            Matx33f R_pred(pred.val);  // 前3x3
            Matx33f R_gt(gt.val);
            Matx33f R_err = R_pred.t() * R_gt;
            float trace_R = R_err(0,0) + R_err(1,1) + R_err(2,2);
            float angle_rad = acosf(std::max(-1.0f, std::min(1.0f, (trace_R - 1.0f) / 2.0f)));
            savePoseToFile(objects[0]->getPose(), "predicted_poses_cat1.txt");
            if (t_err > 0.05f || angle_rad > 0.0873f) {
                objects[0]->setPose(gt);
                //objects[0]->setInitialPose(gt);
                //estimator->toggleTracking(frame, 0, true);
                //estimator->estimatePoses(frame, true, false);


            }


        timer.stop();
        totalTimeMs += timer.getTimeMilli();
        validFrames++;
        timer.reset();

        //savePoseToFile(objects[0]->getPose(), "predicted_poses_cat.txt");

        // 可视化（可选）
        Mat result = drawResultOverlay(objects, frame);
        imshow("Tracking Result", result);
        waitKey(1);
    }

    // 输出平均耗时
    if (validFrames > 0) {
        double avgRuntime = totalTimeMs / validFrames;
        cout << "---------------------------------------------" << endl;
        cout << "Avg. Runtime per frame: " << fixed << setprecision(2) << avgRuntime << " ms" << endl;
    }

    RenderingEngine::Instance()->doneCurrent();
    RenderingEngine::Instance()->destroy();
    for (auto* o : objects) delete o;
    delete estimator;
    return 0;
}
