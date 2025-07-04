#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

int main() {
    // 棋盘格的角点数量
    Size patternSize(7, 7); // 内角点数量，例如8x8棋盘格有7x7个内角点
    float squareSize = 25.0f; // 棋盘格方块的实际大小（单位：毫米）

    // 存储角点的世界坐标和图像坐标
    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> imagePoints;

    // 世界坐标中的角点位置
    vector<Point3f> objP;
    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            objP.push_back(Point3f(j * squareSize, i * squareSize, 0.0f));
        }
    }

    // 读取图像文件
    vector<String> images;
    glob("/media/jyj/JYJ/frames/*.jpg", images); // 替换为你的图像路径
    if (images.empty()) {
        cerr << "No images found in the specified directory!" << endl;
        return -1;
    }

    Mat img, gray;
    for (size_t i = 0; i < images.size(); i++) {
        img = imread(images[i], IMREAD_COLOR);
        if (img.empty()) {
            cerr << "Failed to load image: " << images[i] << endl;
            continue;
        }

        cvtColor(img, gray, COLOR_BGR2GRAY);

        // 查找棋盘格的角点
        vector<Point2f> corners;
        bool found = findChessboardCorners(gray, patternSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

        if (found) {
            // 提高角点定位精度
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));

            // 添加世界坐标和图像坐标
            objectPoints.push_back(objP);
            imagePoints.push_back(corners);

            // 可视化角点
            drawChessboardCorners(img, patternSize, corners, found);
        } else {
            cerr << "Chessboard corners not found in image: " << images[i] << endl;
        }

        imshow("Chessboard", img);
        waitKey(500);
    }

    if (objectPoints.empty() || imagePoints.empty()) {
        cerr << "No valid images found for calibration!" << endl;
        return -1;
    }

    // 相机标定
    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    double rms = calibrateCamera(objectPoints, imagePoints, gray.size(), cameraMatrix, distCoeffs, rvecs, tvecs);

    cout << "RMS error: " << rms << endl;
    cout << "Camera matrix: " << cameraMatrix << endl;
    cout << "Distortion coefficients: " << distCoeffs << endl;

    // 保存标定结果
    FileStorage fs("/home/jyj/Programs/RBOT-master/data/camera_calibration.yml", FileStorage::WRITE);
    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;
    fs.release();

    return 0;
}
