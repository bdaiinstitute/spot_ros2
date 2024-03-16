#include <opencv2/core/hal/interface.h>
#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/imgcodecs.hpp>
// #include "opencv2/highgui.hpp"
#include <opencv2/imgproc.hpp>
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
# Intrinsics
Harriet/frontleft_fisheye
K:
331.015,     0.0, 318.082
    0.0, 330.075, 235.748
    0.0,     0.0,     1.0
*/
cv::Matx33d const Kl(          //
    331.015,     0.0, 318.082, // 
        0.0, 330.075, 235.748, //
        0.0,     0.0,     1.0);
/*
Harriet/frontright_fisheye
K:
332.744,     0.0, 315.983
    0.0, 331.731, 237.265
    0.0,     0.0,     1.0
*/ 
cv::Matx33d const Kr(          //
    332.744,     0.0, 315.983, //
        0.0, 331.731, 237.265, //
        0.0,     0.0,     1.0); 
/*
# Extrinsics
Harriet/body Harriet/frontleft_fisheye
- Translation: [0.384, 0.031, -0.046]
- Rotation: in Quaternion [0.144, 0.808, -0.218, 0.528]
- Rotation: in RPY (radian) [-2.616, 1.159, 3.138]
- Rotation: in RPY (degree) [-149.899, 66.391, 179.773]
- Matrix:
 -0.400  0.463  0.791  0.384
  0.002  0.863 -0.505  0.031
 -0.916 -0.201 -0.346 -0.046
  0.000  0.000  0.000  1.000
*/ 
cv::Matx33d const Rl(
 -0.400,  0.463,  0.791, //  
  0.002,  0.863, -0.505, //
 -0.916, -0.201, -0.346);
cv::Vec3d const tl(0.384, 0.031, -0.046);
/*
Harriet/body Harriet/frontright_fisheye
- Translation: [0.382, -0.043, -0.046]
- Rotation: in Quaternion [-0.155, 0.822, 0.231, 0.496]
- Rotation: in RPY (radian) [2.628, 1.093, -3.087]
- Rotation: in RPY (degree) [150.566, 62.624, -176.844]
- Matrix:
 -0.459 -0.484  0.745  0.382
 -0.025  0.846  0.533 -0.043
 -0.888  0.226 -0.400 -0.046
  0.000  0.000  0.000  1.000
*/
cv::Matx33d const Rr(
 -0.459, -0.484,  0.745, 
 -0.025,  0.846,  0.533,
 -0.888,  0.226, -0.400);
cv::Vec3d const tr(0.382, -0.043, -0.046);
/**
 * Load images 
 * Define extrinsics and intrinsics 
 * Define virtual camera using the average of the extrinsics 
 * Define T1 an T2 to be the transforms from the first and second camera to the virtual camera 
 * Use H = K * [r0, r1, t] * K ^ -1   for each camera     // Inverses might need to be swapped
 * cv::warpPerspective
 * profit?
 *
 * Alternatively, to test out, the transfrom from camera 1 to camera 2 could be defined, and just
 * experiment on the second camera
 * t = t2 - t1
 * R = R2 * R1 ^ -1
 */
cv::Matx33d between(cv::Matx33d const& R1, cv::Matx33d const& R2) {
    // Convert rotation matrices to quaternions
    // cv::Quatd q1 = cv::Quatd::createFromRotMat(R1);
    // Eigen::Quaterniond q1(R1);
    // Eigen::Quaterniond q2(R2);

    // Normalize quaternions (just to be safe)
    // q1.normalize();
    // q2.normalize();

    // Perform SLERP with t=0.5
    // Eigen::Quaterniond qMid = q1.slerp(0.5, q2);

    // Convert the resulting quaternion back to a rotation matrix
    // Eigen::Matrix3d RMid = qMid.toRotationMatrix();

    // return RMid;
    return R1 * R2;
}

void computeCameraTransform(cv::Matx33d const& R1, cv::Vec3d const& t1, cv::Matx33d const& R2, cv::Vec3d const& t2, cv::Matx33d& R_1to2, cv::Vec3d& t_1to2) {
  R_1to2 = R2 * R1.t();
  t_1to2 = R2 * (-R1.t() * t1) + t2;
}

cv::Matx33d computeHomography(cv::Matx33d const& R_1to2, cv::Vec3d const& tvec_1to2, double distance, cv::Vec3d const& normal) {
  double const d_inv = 1. / distance;
  return R_1to2 + d_inv * tvec_1to2 * normal.t();
};

void mosaic(cv::Mat const& left, cv::Mat const& right, cv::Mat& warped_left, cv::Mat& warped_right) {
  cv::Matx33d Rb = 0.5 * (Rl + Rr);
  cv::Vec3d tb = 0.5 * (tl + tr);
  cv::Matx33d R_1tob, R_2tob;
  cv::Vec3d t_1tob, t_2tob;
  computeCameraTransform(Rl, tl, Rb, tb, R_1tob, t_1tob);
  computeCameraTransform(Rr, tr, Rb, tb, R_2tob, t_2tob);
  cv::Vec3d const normal = Rb * cv::Vec3d(0, 0, 1);
  double const distance = 2.;
  cv::Matx33d homography_left = Kl * computeHomography(R_1tob, t_1tob, distance, normal) * Kl.inv();
  homography_left /= homography_left(2, 2);
  cv::Matx33d homography_right = Kr * computeHomography(R_2tob, t_2tob, distance, normal) * Kr.inv();
  homography_right /= homography_right(2, 2);

  cv::warpPerspective(left, warped_left, homography_left, cv::Size(left.cols, left.rows));
  cv::warpPerspective(right, warped_right, homography_right, cv::Size(right.cols, right.rows));
}

int main(int argc, char* argv[])
{
    int retval = parseCmdArgs(argc, argv);
    if (retval) return EXIT_FAILURE;
     // Warp the images using the homography matrices
    cv::Mat image1 = imgs.at(0);
    cv::Mat image2 = imgs.at(1);
    cv::Mat warpedImage1, warpedImage2;
    cv::rotate(image1, image1, ROTATE_90_CLOCKWISE);
    cv::rotate(image2, image2, ROTATE_90_CLOCKWISE);
    mosaic(image2, image1, warpedImage1, warpedImage2);
    cv::imwrite("image1.jpg", warpedImage1);
    // cv::imwrite("image1.jpg", image1);
    cv::imwrite("image2.jpg", warpedImage2);
    // cv::imwrite("image2.jpg", image2);
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
