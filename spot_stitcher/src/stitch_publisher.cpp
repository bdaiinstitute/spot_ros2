#include <opencv2/core/hal/interface.h>
#include <opencv2/imgcodecs.hpp>
// #include "opencv2/highgui.hpp"
#include <opencv2/stitching.hpp>
#include <iostream>
#include <opencv2/stitching/detail/camera.hpp>
using namespace std;
using namespace cv;
bool divide_images = false;
Stitcher::Mode mode = Stitcher::PANORAMA;
vector<Mat> imgs;
string result_name = "result.jpg";
void printUsage(char** argv);
int parseCmdArgs(int argc, char** argv);
/*
- Translation: [-0.003, 0.061, 0.040]
- Rotation: in Quaternion [0.519, -0.033, 0.013, 0.854]
- Rotation: in RPY (radian) [1.093, -0.069, -0.012]
- Rotation: in RPY (degree) [62.613, -3.978, -0.690]
- Matrix:
  0.998 -0.056 -0.043 -0.003
 -0.012  0.461 -0.887  0.061
  0.069  0.886  0.459  0.040
  0.000  0.000  0.000  1.000
*/

int main(int argc, char* argv[])
{
    cv::detail::CameraParams frontleft;
    cv::detail::CameraParams frontright;
    std::vector<double> r = {0.998, -0.056, -0.043, -0.012,  0.461, -0.887,  0.069,  0.886,  0.459 };
    frontright.R = cv::Mat::eye(3, 3, CV_32F);
    frontright.t = cv::Mat{0., 0., 0.}; 
    frontleft.R = cv::Mat(3, 3, CV_32F, r.data());
    frontleft.t = cv::Mat{-0.003, 0.061, 0.040}; 
    std::vector<cv::detail::CameraParams> cameras = {frontleft, frontright};
    int retval = parseCmdArgs(argc, argv);
    if (retval) return EXIT_FAILURE;
    Mat pano;
    Ptr<Stitcher> stitcher = Stitcher::create(mode);
    // Stitcher::Status status = stitcher->stitch(imgs, pano);
    auto const status_tf = stitcher->setTransform(imgs, cameras);
    if (status_tf != Stitcher::OK)
    {
        cout << "Can't set transform, error code = " << int(status_tf) << endl;
        return EXIT_FAILURE;
    }

    auto const status_pa = stitcher->composePanorama(pano);
    if (status_pa != Stitcher::OK)
    {
        cout << "Can't compose pano, error code = " << int(status_pa) << endl;
        return EXIT_FAILURE;
    }
    imwrite(result_name, pano);
    cout << "stitching completed successfully\n" << result_name << " saved!";
    return EXIT_SUCCESS;
}
void printUsage(char** argv)
{
    cout <<
         "Images stitcher.\n\n" << "Usage :\n" << argv[0] <<" [Flags] img1 img2 [...imgN]\n\n"
         "Flags:\n"
         "  --d3\n"
         "      internally creates three chunks of each image to increase stitching success\n"
         "  --mode (panorama|scans)\n"
         "      Determines configuration of stitcher. The default is 'panorama',\n"
         "      mode suitable for creating photo panoramas. Option 'scans' is suitable\n"
         "      for stitching materials under affine transformation, such as scans.\n"
         "  --output <result_img>\n"
         "      The default is 'result.jpg'.\n\n"
         "Example usage :\n" << argv[0] << " --d3 --mode scans img1.jpg img2.jpg\n";
}
int parseCmdArgs(int argc, char** argv)
{
    if (argc == 1)
    {
        printUsage(argv);
        return EXIT_FAILURE;
    }
    for (int i = 1; i < argc; ++i)
    {
        if (string(argv[i]) == "--help" || string(argv[i]) == "/?")
        {
            printUsage(argv);
            return EXIT_FAILURE;
        }
        else if (string(argv[i]) == "--d3")
        {
            divide_images = true;
        }
        else if (string(argv[i]) == "--output")
        {
            result_name = argv[i + 1];
            i++;
        }
        else if (string(argv[i]) == "--mode")
        {
            if (string(argv[i + 1]) == "panorama")
                mode = Stitcher::PANORAMA;
            else if (string(argv[i + 1]) == "scans")
                mode = Stitcher::SCANS;
            else
            {
                cout << "Bad --mode flag value\n";
                return EXIT_FAILURE;
            }
            i++;
        }
        else
        {
            Mat img = imread(samples::findFile(argv[i]));
            if (img.empty())
            {
                cout << "Can't read image '" << argv[i] << "'\n";
                return EXIT_FAILURE;
            }
            else {
                cout << "Image read: " << argv[i] << "'\n";
            }
            if (divide_images)
            {
                Rect rect(0, 0, img.cols / 2, img.rows);
                imgs.push_back(img(rect).clone());
                rect.x = img.cols / 3;
                imgs.push_back(img(rect).clone());
                rect.x = img.cols / 2;
                imgs.push_back(img(rect).clone());
            }
            else
                imgs.push_back(img);
        }
    }
    return EXIT_SUCCESS;
}
