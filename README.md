# Tennisball_follower

[![C++](https://img.shields.io/badge/language-C%2B%2B-%23f34b7d.svg?style=plastic)](https://zh.wikipedia.org/wiki/C%2B%2B) 

Author|Chieh-Ting Chung|Jhong-Ting Liang|Zhi-Jia Wen|
|---|---|---|---|
|E-mail|sksksk1748@gmail.com|s3a517045@gmail.com|wengk99@gmail.com|



## 目的
經由實驗室申請經費，購買機器人做軟體上的二次開發。透過OpenCV影像辨識，並把圖像獲取、處理和分析，使撿網球機能夠辨識當前的物體是否具有網球的特徵，如球形及亮綠色，實現追蹤網球，並搭配超聲波感測器實現自主避障，以及搭配自製撿球裝置，可於網球場取代撿球工作。

## 開發環境與技術
* Robot Operating System on Ubuntu
* Catkin
* roscpp package
* std_msgs package
* message_generation package
* OpenCV

## ROS重新編譯
退出到catkin_ws目錄下執行catkin_make命令進行編譯並設置環境
```
cd ..
catkin_make
source ~/catkin-ws/devel/setup.bash
```

## 開發專案參考出處
* Github : [sudrag/line_follower_turtlebot](https://github.com/sudrag/line_follower_turtlebot)
* [Ball Tracking with OpenCV](https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/)

![image](https://github.com/sksksk1748/Tennisball_follower/blob/master/ball-tracking-animated-02.gif)

## 購買機器人出處
[王道機器人](http://www.kinglyrobotics.com/about.html)


