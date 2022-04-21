#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

int main()
{
 Mat image = Mat::zeros(1536, 2048, CV_8UC1 );
 circle(image, cv::Point2f(1024, 768), 800, Scalar(255), -1, 8, 0);
 imwrite("mask.jpg", image);
 imshow("circle", image);
 waitKey(0);
 return 0;
}
