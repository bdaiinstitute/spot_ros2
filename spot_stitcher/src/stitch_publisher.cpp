#include <opencv2/core/hal/interface.h>
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
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
// # Intrinsics
// Harriet/frontleft_fisheye
// cv::Matx33d const Kl(          //
//     331.015,     0.0, 318.082, // 
//         0.0, 330.075, 235.748, //
//         0.0,     0.0,     1.0);
// Lionel/frontleft_fisheye
cv::Matx33d const Kl(          //
330.619,     0.0, 319.644, //
    0.0, 329.641, 241.036, //
    0.0,     0.0,     1.0);

// Harriet/frontright_fisheye
// cv::Matx33d const Kr(          //
//     332.744,     0.0, 315.983, //
//         0.0, 331.731, 237.265, //
//         0.0,     0.0,     1.0); 
cv::Matx33d const Kr(          //
330.956,     0.0, 312.064, //
    0.0, 329.804, 240.527, //
    0.0,     0.0,     1.0);

// # Extrinsics
// Harriet/body Harriet/frontleft_fisheye
// cv::Matx33d const Rl(
//  -0.400,  0.463,  0.791, //  
//   0.002,  0.863, -0.505, //
//  -0.916, -0.201, -0.346);
// cv::Vec3d const tl(0.384, 0.031, -0.046);
// Lionel/body Lionel/frontleft_fisheye
cv::Vec3d const tl(0.383, 0.035, -0.047);
cv::Matx33d const Rl(
 -0.406,  0.468,  0.785,
 -0.001,  0.859, -0.512,
 -0.914, -0.209, -0.348);

// Harriet/body Harriet/frontright_fisheye
// cv::Matx33d const Rr(
//  -0.459, -0.484,  0.745, 
//  -0.025,  0.846,  0.533,
//  -0.888,  0.226, -0.400);
// cv::Vec3d const tr(0.382, -0.043, -0.046);
// Lionel/body Lionel/frontright_fisheye
cv::Vec3d const tr(0.386, -0.035, -0.048);
cv::Matx33d const Rr(
 -0.417, -0.491,  0.765,
  0.008,  0.840,  0.543,
 -0.909,  0.232, -0.346);

// cv::Matx33d const Rb(
//  -0.43332543, -0.01624629,  0.90109108,
//  -0.0027512,   0.99985669,  0.01670396,
//  -0.90123333,  0.00475917, -0.43330802);
// cv::Vec3d const tb(0.383, -0.006, -0.046);
// cv::Vec3d const tb = 0.5 * (tl + tr);
cv::Matx33d const Rb(
-0.41030468, -0.01426928,  0.91183686,
  0.00511951,  0.99982578,  0.01794987,
 -0.91193413,  0.01203307, -0.41016015);
cv::Vec3d const tb = 0.5 * (tl + tr);
cv::Vec3d normy(1, 0, 0);
double gdistance = 10.;
int d_slider = 10;
int const d_max = 100;
double x = 0.;
int x_slider = 0;
int const x_max = 100;
double y = 0.;
int y_slider = 0;
int const y_max = 100;
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

/*
 * Gives R2 in frame of R1, assuming they both started in the same frame.
 */
void computeCameraTransform(cv::Matx33d const& R1, cv::Vec3d const& t1, cv::Matx33d const& R2, cv::Vec3d const& t2, cv::Matx33d& R_1to2, cv::Vec3d& t_1to2) {
  R_1to2 = R2 * R1.t();
  t_1to2 = R2 * (-R1.t() * t1) + t2;
}

cv::Matx33d computeHomography(cv::Matx33d const& R_1to2, cv::Vec3d const& tvec_1to2, double thedistance, cv::Vec3d const& normal) {
  double const d_inv = 1. / thedistance;
  cv::Matx33d const Tt = d_inv * tvec_1to2 * normal.t();
  return R_1to2 + Tt;
};

void mosaic(cv::Mat const& left, cv::Mat const& right, cv::Mat& warped_left, cv::Mat& warped_right) {
  std::cout << "gdistance = " << gdistance << "\n";
  std::cout << "normy = " << normy << "\n";
  cv::Matx33d bRl, bRr, lRm, rRm;
  cv::Vec3d btl, btr, rtm, ltm;
  // computeCameraTransform(Rl, tl, Rb, tb, bRl, btl);
  // computeCameraTransform(Rr, tr, Rb, tb, bRr, btr);
  // [ x: -0.5481488, y: 0.0440343, z: -0.0084505 ]
  // TODO(gwb) Multiply these matrices together again and see if you get the original
  bRl = cv::Matx33d(
       1.000,  0.000, 0.000,
       0.000,  0.853, 0.521,
       0.000, -0.521, 0.853
      ); // original
  // btl = cv::Vec3d(0.6, -0.73, 0.02); // manually tuned
  btl = cv::Vec3d(0. - x , -0.03 - y, 0.02); // original
  std::cout << "btl = " << btl << "\n";
  bRr = cv::Matx33d(
      1.000,  0.000,  0.000,
      0.000,  0.853, -0.521,
      0.000,  0.521,  0.853
      ); // original
  // btr = cv::Vec3d(-0.6, 0.73, 0.02); // manually tuned
  btr = cv::Vec3d(-0. + x, 0.03 + y, 0.02); // original 
  std::cout << "btr = " << btr << "\n";
  cv::Vec3d normal = Rb * cv::Vec3d(0, 0, 1);
  // cv::Vec3d normal = normy; 
  // normal[1] = 0.;
  normal = cv::normalize(normal);
  std::cout << "normal = " << normal << "\n";
  // double const distance = 2.;
  cv::Matx33d const Kb(
       500.,    0., 900., // increasing fx stretches left-right, cx moves image left 
         0.,  500., 1700., // increasing fy zooms in, cy moves image down 
         0.,    0., 1.
      );
  cv::Matx33d homography_left = Kb * computeHomography(bRl, btl, gdistance, normal) * Kl.inv();
  homography_left /= homography_left(2, 2);
  cv::Matx33d homography_right = Kb * computeHomography(bRr, btr, gdistance, normal) * Kr.inv();
  homography_right /= homography_right(2, 2);
  cv::warpPerspective(left, warped_left, homography_left, cv::Size(left.cols + 1000, left.rows + 4000));
  cv::warpPerspective(right, warped_right, homography_right, cv::Size(right.cols + 1000, right.rows + 4000));
}

void on_x(int, void*) {
  x = static_cast<double>(x_slider) / x_max;
  cv::Mat image1 = imgs.at(0);
  cv::Mat image2 = imgs.at(1);
  cv::Mat warpedImage1, warpedImage2, result;
  mosaic(image2, image1, warpedImage1, warpedImage2);
  cv::addWeighted(warpedImage1, 0.5, warpedImage2, 0.5, 0., result);
  cv::resizeWindow("mosaic", result.cols / 2, result.rows / 2);
  cv::imshow("mosaic", result);
}
void on_y(int, void*) {
  y = static_cast<double>(y_slider) / y_max;
  cv::Mat image1 = imgs.at(0);
  cv::Mat image2 = imgs.at(1);
  cv::Mat warpedImage1, warpedImage2, result;
  mosaic(image2, image1, warpedImage1, warpedImage2);
  cv::addWeighted(warpedImage1, 0.5, warpedImage2, 0.5, 0., result);
  cv::resizeWindow("mosaic", result.cols / 2, result.rows / 2);
  cv::imshow("mosaic", result);
}
void on_d(int, void*) {
  gdistance = static_cast<double>(d_max) / d_slider;
  cv::Mat image1 = imgs.at(0);
  cv::Mat image2 = imgs.at(1);
  cv::Mat warpedImage1, warpedImage2, result;
  mosaic(image2, image1, warpedImage1, warpedImage2);
  cv::addWeighted(warpedImage1, 0.5, warpedImage2, 0.5, 0., result);
  cv::resizeWindow("mosaic", result.cols / 2, result.rows / 2);
  cv::imshow("mosaic", result);
}

int main(int argc, char* argv[])
{
    int retval = parseCmdArgs(argc, argv);
    if (retval) return EXIT_FAILURE;
     // Warp the images using the homography matrices
    cv::Mat image1 = imgs.at(0);
    cv::Mat image2 = imgs.at(1);
    cv::Mat warpedImage1, warpedImage2, result;
    mosaic(image2, image1, warpedImage1, warpedImage2);
    cv::namedWindow("mosaic", cv::WINDOW_NORMAL);
    cv::createTrackbar("x", "mosaic", &x_slider, x_max, on_x);
    cv::setTrackbarMin("x", "mosaic", -100);
    cv::createTrackbar("y", "mosaic", &y_slider, y_max, on_y);
    cv::setTrackbarMin("y", "mosaic", -100);
    cv::createTrackbar("d", "mosaic", &d_slider, d_max, on_d);
    cv::setTrackbarMin("d", "mosaic", -100);
    // cv::imwrite("image1.jpg", warpedImage1);
    // cv::imwrite("image2.jpg", warpedImage2);
    cv::addWeighted(warpedImage1, 0.5, warpedImage2, 0.5, 0., result);
    cv::resizeWindow("mosaic", result.cols / 2, result.rows / 2);
    cv::imshow("mosaic", result);
    cv::waitKey(0);
    // cv::imwrite("result.jpg", result);
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
