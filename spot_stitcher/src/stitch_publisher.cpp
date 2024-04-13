#include <opencv2/core/hal/interface.h>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/quaternion.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>
#include <iostream>
#include <chrono>
#include <opencv2/stitching/detail/blenders.hpp>
#include <opencv2/stitching/detail/camera.hpp>
#include <opencv2/stitching/detail/exposure_compensate.hpp>
#include <opencv2/stitching/detail/seam_finders.hpp>
#include <ratio>
using namespace std;
using namespace cv;
bool divide_images = false;
Stitcher::Mode mode = Stitcher::PANORAMA;
vector<Mat> imgs;
string result_name = "result.jpg";
void printUsage(char** argv);
int parseCmdArgs(int argc, char** argv);
// # Intrinsics
// Lionel/frontleft_fisheye
cv::Matx33d const Kl(          //
330.619,     0.0, 319.644, //
    0.0, 329.641, 241.036, //
    0.0,     0.0,     1.0);

cv::Matx33d const Kr(          //
330.956,     0.0, 312.064, //
    0.0, 329.804, 240.527, //
    0.0,     0.0,     1.0);

cv::Matx44d make_transform(cv::Quatd const& q, cv::Vec3d const& t) {
  // Initialize to identity for bottom row of homogeneous transform
  cv::Matx44d transform = cv::Matx44d::eye();
  // Copy in rotation
  auto const r = q.toRotMat3x3();
  transform(0, 0) = r(0, 0);
  transform(0, 1) = r(0, 1);
  transform(0, 2) = r(0, 2);
  transform(1, 0) = r(1, 0);
  transform(1, 1) = r(1, 1);
  transform(1, 2) = r(1, 2);
  transform(2, 0) = r(2, 0);
  transform(2, 1) = r(2, 1);
  transform(2, 2) = r(2, 2);
  // Copy in translation
  transform(0, 3) = t(0);
  transform(1, 3) = t(1);
  transform(2, 3) = t(2);

  return transform;
}
// # Extrinsics
cv::Vec3d const wtl(0.383425730433869, 0.035200391141172356, -0.046645597578404197);
cv::Quatd const wql(0.5254497615708897, 0.14428877676528717, 0.8083316709117128, -0.22289730093868648);
// Lionel/body Lionel/frontleft_fisheye
// cv::Matx44d const wTl(
//  -0.406,  0.468,  0.785,  0.383, 
//  -0.001,  0.859, -0.512,  0.035,
//  -0.914, -0.209, -0.348, -0.047,
//   0.000,  0.000,  0.000,  1.000);
cv::Matx44d const wTl = make_transform(wql, wtl);

cv::Vec3d const wtr(0.3857987361510829, -0.0353843606877599, -0.04788883099374454);
cv::Quatd const wqr(0.5188475944760547, -0.14970432380380572, 0.8067028035363543, 0.24003411404932312);
// Lionel/body Lionel/frontright_fisheye
// cv::Matx44d const wTr(
//  -0.417, -0.491,  0.765,  0.386,
//   0.008,  0.840,  0.543, -0.035,
//  -0.909,  0.232, -0.346, -0.048,
//   0.000,  0.000,  0.000,  1.000);
cv::Matx44d const wTr = make_transform(wqr, wtr);


cv::Mat draw_arrows(const cv::Vec3d& vector, int imageSize, int lineThickness) {
    // Create a blank canvas
    int canvasWidth = imageSize * 3;
    int canvasHeight = imageSize;
    cv::Mat canvas = cv::Mat::zeros(canvasHeight, canvasWidth, CV_8UC3);

    // Define colors for each projection
    cv::Scalar colorXY(255, 0, 0); // Blue for XY
    cv::Scalar colorXZ(0, 255, 0); // Green for XZ
    cv::Scalar colorYZ(0, 0, 255); // Red for YZ

    // Define the center points for drawing each projection
    cv::Point centerXY(imageSize / 2, canvasHeight / 2);
    cv::Point centerXZ(imageSize + imageSize / 2, canvasHeight / 2);
    cv::Point centerYZ(2 * imageSize + imageSize / 2, canvasHeight / 2);

    // Calculate the end points for each projection
    auto const scale = 80.;
    cv::Point endXY(centerXY.x + scale * vector[0], centerXY.y - scale * vector[1]);
    cv::Point endXZ(centerXZ.x + scale * vector[0], centerXZ.y - scale * vector[2]);
    cv::Point endYZ(centerYZ.x + scale * vector[1], centerYZ.y - scale * vector[2]);

    // Draw the XY projection on the first part of the canvas
    cv::line(canvas, centerXY, endXY, colorXY, lineThickness);
    cv::putText(canvas, "XY", cv::Point(centerXY.x - 20, centerXY.y - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, colorXY, 1);

    // Draw the XZ projection on the second part of the canvas
    cv::line(canvas, centerXZ, endXZ, colorXZ, lineThickness);
    cv::putText(canvas, "XZ", cv::Point(centerXZ.x - 20, centerXZ.y - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, colorXZ, 1);

    // Draw the YZ projection on the third part of the canvas
    cv::line(canvas, centerYZ, endYZ, colorYZ, lineThickness);
    cv::putText(canvas, "YZ", cv::Point(centerYZ.x - 20, centerYZ.y - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, colorYZ, 1);

    return canvas;
}

void mosaic(cv::Mat const& left, cv::Mat const& right, std::vector<cv::UMat>& warped_images, std::vector<cv::UMat>& warped_masks);

void refresh_mosaic() {
  auto const start_function = std::chrono::high_resolution_clock::now();
  cv::Mat image1 = imgs.at(0);
  cv::Mat image2 = imgs.at(1);
  thread_local std::vector<cv::UMat> warped_images(2);
  thread_local std::vector<cv::UMat> warped_masks(2);
  // warped_images.clear();
  // warped_masks.clear();
  // while image2 in testing is coming from the right camera, it sees the left side of the scene
  auto const start_mosaic = std::chrono::high_resolution_clock::now();
  mosaic(image2, image1, warped_images, warped_masks);
  auto const end_mosaic = std::chrono::high_resolution_clock::now();
  // Top left corners, should be a different way to get these numbers, probably by transforming 0, 0 by homography
  // std::vector<cv::Point> corners{cv::Point(0, 797), cv::Point(0, 0)};
  auto const start_gain = std::chrono::high_resolution_clock::now();
  std::vector<cv::Point> corners{cv::Point(0, 0), cv::Point(0, 0)};
  // In some datasets it is shown that the gain compensation is worth it.
  // And the cost can be non-linear as the seamer seems to have to work less hard
  thread_local std::vector<std::pair<UMat,uchar> > level_masks;
  level_masks.clear();
  for (size_t i = 0; i < warped_masks.size(); ++i) {
    level_masks.push_back(std::make_pair(warped_masks[i], (uchar)255));
  }
  // auto compensator = cv::detail::BlocksGainCompensator(); // 50 ms, great , Default in stitcher
  // auto compensator = cv::detail::BlocksChannelsCompensator(); // 96 ms, ok but visible seam in some places
  // auto compensator = cv::detail::ChannelsCompensator(); // 15 ms, ok but visible seam in a large area/everywhere 
  auto compensator = cv::detail::GainCompensator(); // 7 ms, ok ok but still a seam, though less than the (blocks)channels compensators
  compensator.feed(corners, warped_images, level_masks);
  // Top left corners
  for (size_t ndx = 0; ndx < warped_images.size(); ndx++){
    compensator.apply(ndx, corners[ndx], warped_images[ndx], warped_masks); 
  }
  auto const end_gain = std::chrono::high_resolution_clock::now();
  // Convert images for seaming after they've been compensated
  auto const start_seam = std::chrono::high_resolution_clock::now();
  thread_local std::vector<cv::UMat> warped_images_f(2);
  for (size_t ndx = 0; ndx < warped_images.size(); ndx++){
    warped_images[ndx].convertTo(warped_images_f[ndx], CV_32F);
  }
  // Find optimal seams to cut at
  auto seamer = cv::detail::DpSeamFinder(); // GraphCut is default in stitcher but they do it at a different scale
                                            // DpSeamFinder is faster and has better performance in this use case
  seamer.find(warped_images_f, corners, warped_masks);
  auto const end_seam = std::chrono::high_resolution_clock::now();

  auto const start_blend = std::chrono::high_resolution_clock::now();
  auto blender = cv::detail::MultiBandBlender();
  // Determine the size and ROI for blending based on the warped images
  std::vector<cv::Size> sizes(2);
  for (size_t ndx = 0; ndx < sizes.size(); ndx++) {
    sizes[ndx] = warped_images[ndx].size();
  }
  blender.prepare(cv::detail::resultRoi(corners, sizes)); // for some reason this adds a lot of heig
  // blender.prepare(cv::Rect(0, 0, sizes[0].width, sizes[0].height)); 

  // Feed the warped images and their masks to the blender
  thread_local std::vector<cv::UMat> warped_images_s(2);
  for (size_t ndx = 0; ndx < warped_images.size(); ndx++){
    warped_images[ndx].convertTo(warped_images_s[ndx], CV_16S);
  }
  blender.feed(warped_images_s[0], warped_masks[0], cv::Point(0, 0));
  blender.feed(warped_images_s[1], warped_masks[1], cv::Point(0, 0));
  // Blend the images
  cv::Mat blend_mask;
  cv::Mat result;
  blender.blend(result, blend_mask);
  result.convertTo(result, CV_8U);
  auto const end_blend = std::chrono::high_resolution_clock::now();
  cv::resizeWindow("mosaic", result.cols, result.rows);
  cv::resizeWindow("left", sizes[0]);
  cv::resizeWindow("right", sizes[1]);
  cv::imshow("mosaic", result);
  cv::imshow("left", warped_masks[0]);
  cv::imshow("right", warped_masks[1]);
  // cv::imwrite("result.png", result);
  auto const end_function = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> duration_mosaic = end_mosaic - start_mosaic;
  std::chrono::duration<double, std::milli> duration_gain = end_gain - start_gain;
  std::chrono::duration<double, std::milli> duration_seam = end_seam - start_seam;
  std::chrono::duration<double, std::milli> duration_blend = end_blend - start_blend;
  std::chrono::duration<double, std::milli> duration_function = end_function - start_function;

  std::cout << "function: " << duration_function.count() << "\n";
  std::cout << "mosaic: " << duration_mosaic.count() << "\n";
  std::cout << "gain: " << duration_gain.count() << "\n";
  std::cout << "seam: " << duration_seam.count() << "\n";
  std::cout << "blend: " << duration_blend.count() << "\n";
}

// maps an integer value from trackbar to -1:1
double to_normy(int x) {
  // values from zero to 200 with 100 being the center will map to -1:1
  x = std::clamp(x, 0, 200);
  return static_cast<double>(x - 100) / 100;
}

// maps an double value from normy to 0:200
int to_trackbar(double x) {
  // values from -1 to 1 with 0 being the center will map to 0:200
  x = std::clamp(x, -1., 1.);
  return static_cast<int>(x * 100 + 100);
}

// The range of the elements of normy can be [-1:1:0.1], default (0, 0, 1)
// but the whole vector will stay normalized.
// So as you pull on one slider, the other sliders will update
// cv::Vec3d normy(-0.159758, 0, 0.987156);
cv::Vec3d normy(0., 0, 1.);
int nx_slider = 84;
int const nx_max = 200;
int ny_slider = 100;
int const ny_max = 200;
int nz_slider = 198;
int const nz_max = 200;

void refresh_control() {
  normy = cv::normalize(normy);
  try {
  cv::setTrackbarPos("nx", "control", to_trackbar(normy[0])); 
  cv::setTrackbarPos("ny", "control", to_trackbar(normy[1])); 
  cv::setTrackbarPos("nz", "control", to_trackbar(normy[2]));
  } catch(cv::Exception const& e) {
    std::cout << e.what() << "\n";
  }
  cv::Mat normal_arrow = draw_arrows(normy, 200, 2);
  cv::imshow("control", normal_arrow);
}

void on_nx(int, void*) {
  normy[0] = to_normy(nx_slider);
  refresh_control();
  refresh_mosaic();
}

void on_ny(int, void*) {
  normy[1] = to_normy(ny_slider);
  refresh_control();
  refresh_mosaic();
}
void on_nz(int, void*) {
  normy[2] = to_normy(nz_slider);
  refresh_control();
  refresh_mosaic();
}

// These are producing the best results
// Some of these look similar the elements in the Rb and tb matrices.
// See if there is a permulation of them that will be acceptable
// gdistance = 10
// btl = [0.25, -0.43, 0.02]
// btr = [-0.25, 0.43, 0.02]
// normal = [-0.159758, 0, 0.987156]
// Don't forget to math out the values of btl and btr to see if they can come from normy

int cx_slider = 0;
int const cx_max = 1000;

void on_cx(int, void*) {
  refresh_mosaic();
}

int cy_slider = 0;
int const cy_max = 1000;

void on_cy(int, void*) {
  refresh_mosaic();
}

int ff_slider = 0;
int const ff_max = 1000;

void on_ff(int, void*) {
  refresh_mosaic();
}
int row_slider = 0;
int const row_max = 2000;

void on_row(int, void*) {
  refresh_mosaic();
}
// We want the range of x to be [-2:2:0.1], default 0
double x = 0.;
int x_slider = -1;
int const x_max = 100;

void on_x(int, void*) {
  x = static_cast<double>(x_slider) / x_max;
  refresh_mosaic();
}

// We want the range of y to be [-2:2:0.1], default 0
double y = 0.;
int y_slider = 0;
int const y_max = 100;

void on_y(int, void*) {
  y = static_cast<double>(y_slider) / y_max;
  refresh_mosaic();
}

// We want the range of gdistance to be [0.1:10:0.1], default 1
double gdistance = 10.;
int d_slider = 18;
int const d_max = 100;

void on_d(int, void*) {
  gdistance = 0.1 * static_cast<double>(d_max) / d_slider;
  refresh_mosaic();
}

cv::Matx33d between(cv::Matx33d const& R1, cv::Matx33d const& R2) {
  // Convert rotation matrices to quaternions
  cv::Quatd const q1 = cv::Quatd::createFromRotMat(R1);
  cv::Quatd const q2 = cv::Quatd::createFromRotMat(R2);

  // Average the quaternions and convert back to rotation matrix
  return cv::Quatd::slerp(q1, q2, 0.5).toRotMat3x3();
}

cv::Matx33d computeHomography(cv::Matx33d const& Km, cv::Matx33d const& Kc, cv::Matx44d const& cTm, double thedistance, cv::Vec3d const& normal) {
  double const d_inv = 1. / thedistance;
  //  Compute the amount to skew the rotation according to the plane definition (needs better explanation)
  cv::Matx33d const Tt = d_inv * cTm.get_minor<3, 1>(0, 3) * normal.t();
  // Compute the geometric homography
  cv::Matx33d const Hg = cTm.get_minor<3, 3>(0, 0) + Tt;
  // Compute the image space homography
  cv::Matx33d H = Km * Hg * Kc.inv();
  // Normalize on the scalar element
  H /= H(2, 2);
  return H;
};

void assign_translation(cv::Vec3d const& t, cv::Matx44d& T) {
  T(0, 3) = t(0);
  T(1, 3) = t(1);
  T(2, 3) = t(2);
}

void assign_translation(cv::Matx31d const& t, cv::Matx44d& T) {
  T(0, 3) = t(0);
  T(1, 3) = t(1);
  T(2, 3) = t(2);
}

void assign_rotation(cv::Matx33d const& R, cv::Matx44d& T) {
  T(0, 0) = R(0, 0);
  T(0, 1) = R(0, 1);
  T(0, 2) = R(0, 2);

  T(1, 0) = R(1, 0);
  T(1, 1) = R(1, 1);
  T(1, 2) = R(1, 2);

  T(2, 0) = R(2, 0);
  T(2, 1) = R(2, 1);
  T(2, 2) = R(2, 2);
}

cv::Matx44d middle(cv::Matx44d const& T1, cv::Matx44d const& T2) {
  cv::Matx44d T3 = cv::Matx44d::eye();
  assign_rotation(between(T1.get_minor<3, 3>(0, 0), T2.get_minor<3, 3>(0, 0)), T3);
  assign_translation(0.5 * (T1.get_minor<3, 1>(0, 3) + T2.get_minor<3, 1>(0, 3)), T3);
  return T3;
}

void mosaic(cv::Mat const& left, cv::Mat const& right, std::vector<cv::UMat>& warped_images, std::vector<cv::UMat>& warped_masks) {
  std::cout << "gdistance = " << gdistance << "\n";
  std::cout << "normy = " << normy << "\n";
  cv::Matx44d const wTb = middle(wTl, wTr);
  cv::Matx44d lTm = wTl.inv() * wTb;
  cv::Matx44d rTm = wTr.inv() * wTb;
  std::cout << "x: " << x << " y: " << y << "\n";
  lTm(0, 3) -= x;
  lTm(0, 4) -= y;
  std::cout << "lTm = " << lTm << "\n";
  rTm(0, 3) += x;
  rTm(0, 4) += y;
  std::cout << "rTm = " << rTm << "\n";
  cv::Vec3d normal = normy; 
  normal = cv::normalize(normal);
  std::cout << "normal = " << normal << "\n";
  cv::Matx33d const Kb(
       385. + ff_slider,    0., 315. + cx_slider, // increasing fx stretches left-right, cx moves image left 
         0.,  385. + ff_slider, 844. + cy_slider, // increasing fy zooms in, cy moves image down 
         0.,    0., 1.
      );
  cv::Matx33d const homography_left = computeHomography(Kb, Kl, lTm, gdistance, normal);  
  cv::Matx33d const homography_right = computeHomography(Kb, Kr, rTm, gdistance, normal);  
  cv::warpPerspective(left, warped_images[0], homography_left, cv::Size(left.cols, left.rows + row_slider + 1182));
  cv::warpPerspective(right, warped_images[1], homography_right, cv::Size(right.cols, right.rows + row_slider + 1182));
  
  cv::UMat mask_left(left.size(), CV_8U, 255);
  cv::UMat mask_right(right.size(), CV_8U, 255);
  cv::warpPerspective(mask_left, warped_masks[0], homography_left, cv::Size(left.cols, left.rows + row_slider + 1182));
  cv::warpPerspective(mask_right, warped_masks[1], homography_right, cv::Size(right.cols, right.rows + row_slider + 1182));
}


int main(int argc, char* argv[])
{
    int retval = parseCmdArgs(argc, argv);
    if (retval) return EXIT_FAILURE;
    cv::namedWindow("mosaic", cv::WINDOW_NORMAL);
    cv::namedWindow("left", cv::WINDOW_NORMAL);
    cv::namedWindow("right", cv::WINDOW_NORMAL);
    cv::namedWindow("control", cv::WINDOW_NORMAL);
    cv::createTrackbar("x", "control", &x_slider, x_max, on_x);
    cv::setTrackbarMin("x", "control", -100);
    cv::createTrackbar("y", "control", &y_slider, y_max, on_y);
    cv::setTrackbarMin("y", "control", -100);
    cv::createTrackbar("d", "control", &d_slider, d_max, on_d);
    cv::setTrackbarMin("d", "control", -100);
    cv::createTrackbar("nx", "control", &nx_slider, nx_max, on_nx);
    cv::createTrackbar("ny", "control", &ny_slider, ny_max, on_ny);
    cv::createTrackbar("nz", "control", &nz_slider, nz_max, on_nz);
    cv::createTrackbar("cx", "control", &cx_slider, cx_max, on_cx);
    cv::setTrackbarMin("cx", "control", -1000);
    cv::createTrackbar("cy", "control", &cy_slider, cy_max, on_cy);
    cv::setTrackbarMin("cy", "control", -1000);
    cv::createTrackbar("ff", "control", &ff_slider, ff_max, on_ff);
    cv::setTrackbarMin("ff", "control", -1000);
    cv::createTrackbar("rows", "control", &row_slider, row_max, on_row);
    cv::Mat normal_arrow = draw_arrows(normy, 200, 2);
    cv::imshow("control", normal_arrow);
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
