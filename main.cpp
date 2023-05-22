#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;


class Color
{
    private:
        std::vector<double> color_ =  {0,0,0};
    public:
        Color(std::vector<double> color) : color_{color} 
        {
        }
        Color(double hue, double saturation, double value)
            : color_{hue, saturation, value}
        {
        }
        double &operator[](int index) { return color_[index]; }
};

cv::Mat TakeThresholdOfBlob(const cv::Mat &frameHSV, Color Color);
cv::Rect detectBlob(cv::Mat &frame, const cv::Mat &threshold);
cv::Point2i SteppedDetection(cv::Mat &frame, Color colorHSV);
cv::Point2i GetMassCenter(const cv::Rect &rectangle);
cv::Point2i GetMassCenter(const cv::Point2i &center1, const cv::Point2i &center2);
float findangle(const cv::Point2i &massCenter1, const cv::Point2i &massCenter2, const cv::Point2i &massCenter3);
std::string ChooseTheCommand(float cosOfAngle);

std::string RightChoice(int number)
{
    if(number == 1)
        return "right";
    if(number == 2)
        return "right";
    if(number == 3)
        return "left";
    if(number == 4)
        return "left";
    if(number == 5)
        return "left";
    if(number == 6)
        return "left";
    if(number == 7)
        return "right";
    if(number == 8)
        return "right";
    if(number == 9)
        return "right";
    if(number == 10)
        return "left";
    if(number == 11)
        return "left";
    if(number == 12)
        return "right";
    if(number == 13)
        return "left";
    if(number == 14)
        return "right";
    if(number == 15)
        return "right";
    if(number == 16)
        return "left";
    return "error";
}

Color tailColorHsv_{110, 140, 100};
Color headColorHsv_{160, 140, 100};
Color destColorHsv_{50,100,40};

    Point2i head, tail, dest;
int main()
{
    Mat frame;
    string fileBase = ".png";
    for(int i = 0; i < 16; i++)
    {
        string file = fileBase;
        file.insert(0,to_string(i+1));
        cout << file << ": ";
        frame = imread(file);
        
        head = SteppedDetection(frame, headColorHsv_);
        tail = SteppedDetection(frame, tailColorHsv_);
        dest = SteppedDetection(frame, destColorHsv_);
        float angle = findangle( tail,head, dest);
        cout << angle <<" " << ChooseTheCommand(angle).compare(RightChoice(i+1)) << endl;



        // resize(frame, frame, Size(frame.cols/3, frame.rows/3));
        // imshow("frame", frame);
        // waitKey(10);
    }

    return 0;
}

std::string ChooseTheCommand(float cosOfAngle)
{
    if (cosOfAngle > 0)
    {
        if (tail.x < head.x)
            if (tail.y < head.y)
                return "left";
            else
                return "right";
        else if (tail.y < head.y)
                return "left";
            else
                return "right";
    }
    else
    {
        if (tail.x < head.x)
            if (tail.y > head.y)
                return "left";
            else
                return "right";
        else if (tail.y < head.y)
            return "left";
        else
            return "right";
    }
}

float findangle(const cv::Point2i &massCenter1, const cv::Point2i &massCenter2,  const cv::Point2i &massCenter3)
{
    //normal vectors n(A,B)
    float A1, A2, B1, B2;
    A1 = massCenter1.x - massCenter2.x;
    A2 = massCenter2.x - massCenter3.x;
    B1 = massCenter1.y - massCenter2.y;
    B2 = massCenter2.y - massCenter3.y;

    float CosOfAngle = static_cast<float>(
        (A1 * A2 + B1 * B2) /
        (sqrt(pow(A1, 2) + pow(B1, 2)) * sqrt(pow(A2, 2) + pow(B2, 2))));
    return CosOfAngle;
}


cv::Point2i GetMassCenter(const cv::Rect &rectangle)
{
    cv::Point2i TopLeftPointRect = rectangle.tl();
    cv::Point2i BottomRightPointRect = rectangle.br();
    int X = (TopLeftPointRect.x + BottomRightPointRect.x)/2;
    int Y = (TopLeftPointRect.y + BottomRightPointRect.y)/2;
    return cv::Point2i(X, Y);
}

cv::Mat TakeThresholdOfBlob(const cv::Mat &frameHSV, Color color)
{
    cv::Mat threshold;
    //vectors of HSV color that need to find. Make a range using Offset
    uint8_t Offset = 15;
    std::vector<uint8_t> HsvMin = {static_cast<uint8_t>(color[0] - Offset), static_cast<uint8_t>(color[1] - Offset), static_cast<uint8_t>(color[2] - Offset)};
    std::vector<uint8_t> HsvMax = {static_cast<uint8_t>(color[0] + Offset), 255, 255};

    //Make result image matrix with found color
    inRange(frameHSV, HsvMin, HsvMax, threshold);
    //Creating the image in white&black with area of needed color 
    // cv::imshow( "FrameResult.jpg", threshold );
    // cv::waitKey(1000);
    return threshold;
}

cv::Rect detectBlob(cv::Mat &frame, const cv::Mat &threshold)
{
    cv::Rect rectangle;
    std::vector<std::vector<cv::Point>> contours;

    findContours(threshold, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    std::vector<cv::RotatedRect> minRect(contours.size());
    for(size_t i = 0; i < contours.size(); i++)
            minRect[i] = minAreaRect(contours[i]);
    
    for(size_t i = 0; i < contours.size(); i++)
    {
        int area = minRect[i].size.area();
        int minArea = 1500;
        if(area > minArea)
        {
            rectangle = minRect[i].boundingRect();
            //Uncomment if you want to see Rectangles on image
            cv::Point2f rect_points[4];
            minRect[i].points( rect_points );
            for ( int j = 0; j < 4; j++ )
            {
                line( frame, rect_points[j], rect_points[(j+1)%4], (0,0,0) );
            }
        }
    }
    // imshow("result", threshold);
    // waitKey();
    return rectangle;
}

cv::Point2i SteppedDetection(cv::Mat &frame, Color colorHSV)
{
    cv::Mat frameHSV, threshold;
    cv::Rect rectangleOfColor;

    cvtColor(frame, frameHSV, cv::COLOR_BGR2HSV);
    threshold = TakeThresholdOfBlob(frameHSV, colorHSV);
    rectangleOfColor = detectBlob(frame, threshold);

    return GetMassCenter(rectangleOfColor);
}