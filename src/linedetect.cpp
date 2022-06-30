/** MIT License
Copyright (c) 2017 Sudarshan Raghunathan
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*
*@copyright Copyright 2017 Sudarshan Raghunathan
*@file linedetect.cpp
*@author Sudarshan Raghunathan
*@brief  Class linedetect's function definitions
*/
#include "linedetect.hpp"
#include <cv_bridge/cv_bridge.h>
#include <cstdlib>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "ros/console.h"
#include "riki_line_follower/pos.h"
#include<vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <ctime>

void LineDetect::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img = cv_ptr->image;
        cv::waitKey(30);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

cv::Mat LineDetect::Gauss(cv::Mat input) {
    cv::Mat output;
    // Applying Gaussian Filter
    cv::GaussianBlur(input, output, cv::Size(11, 11), 0, 0); 
    /*OpenCV高斯平滑
    void GaussianBlur(const Mat &src, Mat &dst, Size ksize, double sigmaX, double sigmaY)

    src：輸入可以為多通道圖，會單獨處理各通道，但是通常使用單通道灰階圖，例如CV_8U或CV_16U。
    dst：輸出圖會和輸入圖尺寸、型態相同。
    ksize：模板大小，長寬可以不同，但是都必須為正的奇數。
    sigmaX：x方向的標準差。
    sigmaY：y方向的標準差。*/
    return output;
}

int LineDetect::colorthresh(cv::Mat input) {

    // 讀取 arduino 三個超聲波感測器的數值
    std::string file = "/home/rikirobot/putty.log"; 
    std::ifstream ifs(file.c_str(), std::fstream::in);

    if(!ifs)
        std::cout << "Read file failed!" << std::endl;   
    //obtain size of file in Byte
    ifs.seekg(0, ifs.end);
    int length = ifs.tellg();
    //std::cout << "length: " << length << std::endl; 

    // 將 putty.log 的字串做正規化處理，分別讀出三個超聲波感測器的數值
    std::string line;
    //check '\n' from second character because the last character is '\n'
    int index = -3; 
    int three =  0;
    int second = 0;
    int first = 0;
    while(length){   
        char c;
        ifs.seekg (index, ifs.end);
        ifs.get(c);
        //check '\n' from end to begin
        if(c == '\n'){
            //get the the last line when finding its corresponding beginning
            std::getline(ifs, line);
            //convet characters to int through istringstream class provided in c++
            std::istringstream iss(line);
            iss >> first;
            break;
        }else if(c == ' '){
            //get the the last line when finding its corresponding beginning
            std::getline(ifs, line);
            //convet characters to int through istringstream class provided in c++
            std::istringstream iss(line);
            iss >> second;
            //break;
        }else if(c == '!'){
            //get the the last line when finding its corresponding beginning
            std::getline(ifs, line);
            //convet characters to int through istringstream class provided in c++
            std::istringstream iss(line);
            iss >> three;
            //break;
        }
        length--;
        index--;
    }
    //std::cout << "line num: " << linenum << std::endl;
    char str[10];
    sprintf(str, "%d %d %d", first,second, three);


    // Initializaing variables
    cv::Size s = input.size();
    std::vector<std::vector<cv::Point> > v;
    auto w = s.width;
    auto h = s.height;
    auto c_x = 0.0;
  
    // Detect all objects within the HSV range
    cv::cvtColor(input, LineDetect::img_hsv, CV_BGR2HSV);
    LineDetect::LowerYellow = {30, 65, 98};
    LineDetect::UpperYellow = {51, 220, 255};
  
    cv::inRange(LineDetect::img_hsv, LowerYellow,UpperYellow, LineDetect::img_mask);//可以更準確的知道綠球位置
  
    cv::erode(LineDetect::img_mask, LineDetect::img_mask, cv::Mat(),cv::Point(-1,-1),2);
  
  
    /*OpenCV侵蝕
    erode(const Mat &src, Mat &dst, Mat kernel, Point anchor=Point(-1,-1), int iterations=1)

    src：輸入圖，可以多通道，深度可為CV_8U、CV_16U、CV_16S、CV_32F或CV_64F。
    dst：輸出圖，和輸入圖尺寸、型態相同。
    kernel：結構元素，如果kernel=Mat()則為預設的3×3矩形，越大侵蝕效果越明顯。
    anchor：原點位置，預設為結構元素的中央。
    iterations：執行次數，預設為1次，執行越多次侵蝕效果越明顯。*/

    cv::dilate(LineDetect::img_mask, LineDetect::img_mask, cv::Mat(),cv::Point(-1,-1),2);
  
    /*
    OpenCV膨脹
    dilate(const Mat &src, Mat &dst, Mat kernel, Point anchor=Point(-1,-1), int iterations=1)

    src：輸入圖，可以多通道，深度可為CV_8U、CV_16U、CV_16S、CV_32F或CV_64F。
    dst：輸出圖，和輸入圖尺寸、型態相同。
    kernel：結構元素，如果kernel=Mat()則為預設的3×3矩形，越大膨脹效果越明顯。
    anchor：原點位置，預設為結構元素的中央。
    iterations：執行次數，預設為1次，執行越多次膨脹效果越明顯。*/

    cv::findContours(LineDetect::img_mask, v, CV_RETR_LIST, CV_CHAIN_APPROX_NONE); //計算球的輪廓
    // Choosing contours with maximum area
  
    cv::Point2f center;
    float radius;
  
    if(v.size() != 0){
        auto area = 0;
        auto idx = 0;
        auto count = 0;
        while(count < v.size()){
            if(area < v[count].size()){
                idx = count;
                area = v[count].size();
            }
            count++;
        }
        cv::minEnclosingCircle(v[idx],center,radius);  //得到包含二维点集的最小圆
        /*InputArray points：输入的二维点集

        Point2f& center：表示输出的圆形的中心坐标，是float型
        float& radius：输出的最小圆的半径，是float型*/

        // Perform centroid detection of line
        // 利用 moments() 找出 img_mask 網球的質心(centroid)
        cv::Moments M = cv::moments(LineDetect::img_mask);
  
        if (radius > 10) {
            cv::circle(input, center, radius, cv::Scalar(0, 0, 255),2,8,0);
            /*
            void circle(Mat& img, Point center, int radius, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
            img：輸入圖，圓會畫在上面。
            center：圓心。
            radius：圓半徑。
            color：圓形的顏色。
            thickness：圓形的邊線寬度，輸入負值或CV_FILLED代表填滿圓形。
            lineType：通道型態，可輸入8、4、CV_AA： 8->8通道連結。 4->4通道連結。 CV_AA->消除鋸齒(antialiased line)，消除顯示器畫面線邊緣的凹凸鋸齒。*/
        }
  
        if (M.m00 > 0) {
            cv::Point p1(M.m10/M.m00, M.m01/M.m00);
            //cv::circle(LineDetect::img_mask, p1, 5, cv::Scalar(0, 0, 255), -1); 
        }
        /* 找出質心 X 軸在input image 中的位置
        Centroid X軸與Y軸的計算公式
        c_x = M.m10/M.m00
        c_y = M.m01/M.m00

                 c_x
              _________
             |         |
        c_y  |         |
             |_________|
        */
        c_x = M.m10/M.m00;
  
        /*OpenCV 畫文字字串
        void putText(Mat& img, const string& text, Point org, int fontFace, double fontScale, Scalar color, int thickness=1, int lineType=8, bool bottomLeftOrigin=false)

        img：輸入圖，字串會畫在上面。
        text：輸出字串，OpenCV目前沒有支援中文文字顯現。
        org：文字左下角位置。
        fontFace：字體樣式。
        fontScale：字體大小。
        color：字串顏色。
        thickness：構成字串的線寬度。
        lineType：通道型態，有以下三種可選： 8：8通道連結。 4：4通道連結。 CV_AA：消除鋸齒(antialiased line)，消除顯示器畫面橢圓邊緣的凹凸鋸齒。*/
        cv::putText(input, str, cv::Point(M.m10/M.m00, M.m01/M.m00),CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(0, 0, 0));
    }
    

    // Tolerance to chooise directions
    auto tol = 30;
    auto count = cv::countNonZero(img_mask);
    // Turn left if centroid is to the left of the image center minus tolerance
    // Turn right if centroid is to the right of the image center plus tolerance
    // Go straight if centroid is near image center

    if(first<=30 || second<=30 || three<=10){
        if(first < second && first<three){
            LineDetect::dir = 4;
        }else if(first > second && second<=three){
            LineDetect::dir = 5;
        }else if(three < 10){
            LineDetect::dir = 1;
        }
    }else{
        // 如果球的質心出現在畫面的左側，則車子輪子轉向左
        if(c_x < w/2-tol){
            LineDetect::dir = 0; // left
        // 如果球的質心出現在畫面的右側，則車子輪子轉向右
        }else if(c_x > w/2+tol){
            LineDetect::dir = 2; // right
        }else{
            LineDetect::dir = 1; // Straight
        }
        // Search if no ball detected
        if(count == 0){
            LineDetect::dir = 3;
        }
    }
    // Output images viewed by the turtlebot
    cv::namedWindow("Rikirobot View");
    imshow("Rikirobot View", input);
    return LineDetect::dir;
}
