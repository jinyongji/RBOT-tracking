#include <opencv2/core.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace std;
using namespace cv;

// 解析一行 pose，返回 4x4 位姿矩阵
Matx44f parsePoseLine(const string& line) {
    istringstream iss(line);
    float vals[12];
    for (int i = 0; i < 12; ++i) iss >> vals[i];

    return Matx44f(
            vals[0], vals[1], vals[2],  vals[9],
            vals[3], vals[4], vals[5], vals[10],
            vals[6], vals[7], vals[8], vals[11],
            0,       0,       0,        1
    );
}

// 从 .txt 文件读取所有位姿
vector<Matx44f> loadPoses(const string& filename) {
    vector<Matx44f> poses;
    ifstream file(filename);
    string line;
    while (getline(file, line)) {
        if (!line.empty()) poses.push_back(parsePoseLine(line));
    }
    return poses;
}

// 从 .ply 文件读取顶点（ASCII 格式）
vector<Point3f> loadModelPointsFromPLY(const string& ply_file) {
    vector<Point3f> points;
    ifstream file(ply_file);
    if (!file.is_open()) {
        cerr << "Failed to open PLY file: " << ply_file << endl;
        return points;
    }

    string line;
    int vertex_count = 0;
    bool header_ended = false;

    while (getline(file, line)) {
        if (line.find("element vertex") != string::npos) {
            istringstream iss(line);
            string tmp;
            iss >> tmp >> tmp >> vertex_count;
        } else if (line == "end_header") {
            header_ended = true;
            break;
        }
    }

    if (!header_ended) {
        cerr << "PLY header not properly terminated!" << endl;
        return points;
    }

    for (int i = 0; i < vertex_count; ++i) {
        getline(file, line);
        istringstream iss(line);
        float x, y, z;
        iss >> x >> y >> z;
        points.emplace_back(x, y, z);
    }

    return points;
}

// 用 ADD 指标评估成功率（10% 直径）
void evaluate_ADD(const string& gt_file, const string& pred_file, const string& model_file) {
    vector<Matx44f> gtPoses = loadPoses(gt_file);
    vector<Matx44f> predPoses = loadPoses(pred_file);
    vector<Point3f> model_points = loadModelPointsFromPLY(model_file);

    if (gtPoses.size() != predPoses.size()) {
        cerr << "Mismatch in pose list size!" << endl;
        return;
    }
    if (model_points.empty()) {
        cerr << "Model points not loaded!" << endl;
        return;
    }

    // Compute model diameter in millimeters
    float max_dist = 0.0f;
    for (size_t i = 0; i < model_points.size(); ++i) {
        for (size_t j = i + 1; j < model_points.size(); ++j) {
            float d = norm(model_points[i] - model_points[j]);
            if (d > max_dist) max_dist = d;
        }
    }

    // 10% of model diameter (converted to meters)
    float add_threshold = 0.1f * max_dist / 1000.0f;

    int success_count = 0;
    int total = gtPoses.size();

    for (int i = 0; i < total; ++i) {
        const Matx44f& T_gt = gtPoses[i];
        const Matx44f& T_pred = predPoses[i];

        double add_error = 0.0;

        for (const Point3f& p : model_points) {
            Vec4f p_h(p.x, p.y, p.z, 1.0f);
            Vec4f p_gt = T_gt * p_h;
            Vec4f p_pred = T_pred * p_h;
            //是否矩阵相乘还是

            // ✅ Convert mm → m
            for (int j = 0; j < 3; ++j) {
                p_gt[j] /= 1000.0f;
                p_pred[j] /= 1000.0f;
            }

            Vec3f diff(p_gt[0] - p_pred[0], p_gt[1] - p_pred[1], p_gt[2] - p_pred[2]);
            add_error += norm(diff);
        }

        add_error /= model_points.size(); // average error in meters

        if (add_error < add_threshold) {
            success_count++;
        }

        if (i < 20 || i == total - 1) {
            cout << "Frame " << i << ": ADD error = " << fixed << setprecision(6) << add_error << " m" << endl;
        }
    }

    float success_rate = 100.0f * success_count / total;
    cout << "---------------------------------------------" << endl;
    cout << "Success frames: " << success_count << " / " << total << endl;
    cout << "ADD Success Rate (10% diameter): " << fixed << setprecision(2) << success_rate << " %" << endl;
}
float get_errort(cv::Vec3f t_gt, cv::Vec3f t_pred) {
    float l22 = pow(t_gt[0] - t_pred[0], 2) + pow(t_gt[1] - t_pred[1], 2) + pow(t_gt[2] - t_pred[2], 2);
    return sqrt(l22);
}
// 使用 5cm 平移误差 + 5° 旋转误差判断成功
void evaluate_5cm_5deg(const string& gt_file, const string& pred_file) {
    vector<Matx44f> gtPoses = loadPoses(gt_file);
    vector<Matx44f> predPoses = loadPoses(pred_file);

    if (gtPoses.size() != predPoses.size()) {
        cerr << "Mismatch in pose list size!" << endl;
        return;
    }

    int success_count = 0;
    int total = gtPoses.size();

    for (int i = 1; i < total; ++i) {
        const Matx44f& T_gt = gtPoses[i];
        const Matx44f& T_pred = predPoses[i];

        // 提取平移向量并换单位（mm → m）
        Vec3f t_gt(T_gt(0, 3), T_gt(1, 3), T_gt(2, 3));
        Vec3f t_pred(T_pred(0, 3), T_pred(1, 3), T_pred(2, 3));
        //float t_err = get_errort(t_gt,t_pred)/1000;  // m
        float t_err = norm(t_gt - t_pred) / 1000.0f;


        // 提取旋转矩阵
        Matx33f R_gt(
                T_gt(0, 0), T_gt(0, 1), T_gt(0, 2),
                T_gt(1, 0), T_gt(1, 1), T_gt(1, 2),
                T_gt(2, 0), T_gt(2, 1), T_gt(2, 2));

        Matx33f R_pred(
                T_pred(0, 0), T_pred(0, 1), T_pred(0, 2),
                T_pred(1, 0), T_pred(1, 1), T_pred(1, 2),
                T_pred(2, 0), T_pred(2, 1), T_pred(2, 2));

        // 旋转误差（弧度）
        Matx33f R_diff = R_pred.t() * R_gt;
        float trace = R_diff(0,0) + R_diff(1,1) + R_diff(2,2);
        trace = std::min(3.0f, std::max(-1.0f, trace));  // 避免 acos 越界
        float angle_rad = acos((trace - 1.0f) / 2.0f);
        float r_err_deg = angle_rad * 180.0f / CV_PI;

        if (t_err < 0.05 && r_err_deg < 5.0) {
            success_count++;
        }

        if (i < 20 || i == total - 1) {
            cout << "Frame " << i << ": t_err = " << fixed << setprecision(4) << t_err
                 << " m, r_err = " << r_err_deg << " deg" << endl;
        }
    }

    float success_rate = 100.0f * success_count / 1000;
    cout << "---------------------------------------------" << endl;
    cout << "Success frames: " << success_count << " / " << 1000 << endl;
    cout << "5cm/5deg Success Rate: " << fixed << setprecision(1) << success_rate << " %" << endl;
}

int main() {
    //evaluate_ADD("poses_first.txt", "predicted_poses_ape_regular.txt", "/media/jyj/JYJ/RBOT-dataset/ape/ape.ply");
    evaluate_5cm_5deg("poses_first.txt", "predicted_poses_bs.txt");
    return 0;
}
