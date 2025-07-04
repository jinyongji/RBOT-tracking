#include <QApplication>
#include <QThread>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp> // 用于读取视频或图像序列

#include <fstream>
#include <iomanip>
#include <sstream>

#include "object3d.h"
#include "pose_estimator6d.h"

using namespace std;
using namespace cv;

cv::Mat drawResultOverlay(const vector<Object3D*>& objects, const cv::Mat& frame)
{
    // render the models with phong shading
    RenderingEngine::Instance()->setLevel(0);

    vector<Point3f> colors;
    colors.push_back(Point3f(1.0, 0.5, 0.0));
    RenderingEngine::Instance()->renderShaded(vector<Model*>(objects.begin(), objects.end()), GL_FILL, colors, true);

    // download the rendering to the CPU
    Mat rendering = RenderingEngine::Instance()->downloadFrame(RenderingEngine::RGB);

    // download the depth buffer to the CPU
    Mat depth = RenderingEngine::Instance()->downloadFrame(RenderingEngine::DEPTH);

    // compose the rendering with the current camera image for demo purposes (can be done more efficiently directly in OpenGL)
    Mat result = frame.clone();
    for(int y = 0; y < frame.rows; y++)
    {
        for(int x = 0; x < frame.cols; x++)
        {
            Vec3b color = rendering.at<Vec3b>(y,x);
            if(depth.at<float>(y,x) != 0.0f)
            {
                result.at<Vec3b>(y,x)[0] = color[2];
                result.at<Vec3b>(y,x)[1] = color[1];
                result.at<Vec3b>(y,x)[2] = color[0];
            }
        }
    }
    return result;
}

void savePoseToFile(const cv::Matx44f& pose, int frameIndex, const std::string& filename)
{
    std::ofstream file(filename, std::ios::app);
    if (!file.is_open()) {
        std::cerr << "Failed to open pose output file: " << filename << std::endl;
        return;
    }

    file << std::fixed << std::setprecision(6);
    file << frameIndex << "\t";
    for (int i = 0; i < 12; ++i)
    {
        file << pose.val[i] << "\t";
    }
    file << "\n";
    file.close();
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // camera image size
    int width = 1280;
    int height = 720;

    // near and far plane of the OpenGL view frustum
    float zNear = 10.0;
    float zFar = 10000.0;

    // camera instrinsics
    cv::Matx33f K(1440.6351, 0.0, 642.3443,
                  0.0, 1442.8019, 282.1061,
                  0.0, 0.0, 1.0);
    cv::Matx14f distCoeffs(0.15655, -0.99829, -0.01949, -0.00167);

    // distances for the pose detection template generation
    vector<float> distances = {900.0f, 1000.0f, 1100.0f};

    // load 3D objects
    vector<Object3D*> objects;
    objects.push_back(new Object3D(
            "/media/jyj/JYJ/RBOT-dataset/cube/cube_new.obj",
            96.0f,  // tx: 
            -34.0f, // ty: 
            1014.0f, // tz: 物距
            43.4f,   // alpha: x轴旋转
            39.5f,   // beta: y轴旋转
            68.0f,   // gamma: z轴旋转
            1.0f,   // scale
            0.8f,  // qualityThreshold
            distances
    ));
    // create the pose estimator
    PoseEstimator6D* poseEstimator = new PoseEstimator6D(width, height, zNear, zFar, K, distCoeffs, objects);

    RenderingEngine::Instance()->makeCurrent();
    string videoPath = "tracking_result.mp4";
    int fps = 10;
    VideoWriter writer;
    writer.open(videoPath, VideoWriter::fourcc('m','p','4','v'), fps, Size(width, height));
    if (!writer.isOpened()) {
        cerr << "Failed to open video writer!" << endl;
        return -1;
    }
    for (int i = 0; i <=644; ++i)
    {
        std::stringstream ss;
        ss << "/media/jyj/JYJ/frames/frame_"
           << std::setfill('0') << std::setw(5) << i << ".jpg";
        cv::Mat frame = cv::imread(ss.str());

        if (frame.empty()) {
            std::cerr << "Failed to load image: " << ss.str() << std::endl;
            break;
        }

        if (i == 0)
        {
            // 初始化位姿（粗定位）
            poseEstimator->toggleTracking(frame, 0, false);
            // 运行一次优化
            poseEstimator->estimatePoses(frame, false, false);
        }
        else
        {
            // 后续帧直接跟踪
            poseEstimator->estimatePoses(frame, false, false);

        }

        // 保存预测的位姿矩阵
        savePoseToFile(objects[0]->getPose(), i, "predicted_cube.txt");

        // 绘制结果
        cv::Mat result = drawResultOverlay(objects, frame);
        cv::imshow("Tracking Result", result);
        writer.write(result);
        cv::waitKey(1);  // 等待1ms，以便显示结果
    }
    writer.release();
    // deactivate the offscreen rendering OpenGL context
    RenderingEngine::Instance()->doneCurrent();

    // clean up
    RenderingEngine::Instance()->destroy();

    for (int i = 0; i < objects.size(); i++)
    {
        delete objects[i];
    }
    objects.clear();

    delete poseEstimator;

    return 0;
}
