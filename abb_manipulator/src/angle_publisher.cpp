#include "opencv2/opencv.hpp"
#include <vector>
#include <algorithm>
#include <chrono> 
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <sstream>
 
using namespace std;
using namespace cv;
using namespace std::chrono; 

Mat frame,src, erosion_dst, dilation_dst, dst, cdst,edges,hsv,mask,frame_original;
Mat1b gray,filtered;
vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

 
     float temp=0;
    int largest_area=0;
    int largest_contour_index=0;
    Rect bounding_rect;

  RNG rng(12345);
 
int main(int argc, char **argv)
{   
     ros::init(argc, argv, "angle_publisher");
     ros::NodeHandle n;
     ros::Publisher angle_pub = n.advertise<std_msgs::Float64>("/angle_topic", 1000);

    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
 
 /*while(ros::ok())
 {
cap >> frame;
    namedWindow( "Captured", CV_WINDOW_AUTOSIZE );
        imshow( "Captured", frame );
        waitKey(2);
 }
*/
    
    //namedWindow("edges",1);
    //for(;;)
    while (ros::ok())
    {
        auto start = high_resolution_clock::now();
        cap >> frame; // get a new frame from camera
        frame_original=frame;
        cvtColor(frame,hsv,COLOR_BGR2HSV);
        inRange(hsv,Scalar(70,70,70),Scalar(95,255,255),mask);
        //inRange(hsv,Scalar(40,0,0),Scalar(80,255,255),mask);

        frame.setTo(cv::Scalar(0,0,0));
        frame.setTo(Scalar(255,255,255),mask);

        // Convert to grayscale
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        // Get binary mask (remove jpeg artifacts)
        gray = gray > 200;
        
        //erosion
        Mat element1 = getStructuringElement(MORPH_RECT, Size(1, 4));
        erode(gray, erosion_dst, element1); // Erode with a 3 x 3 kernel 

        //fastNlMeansDenoising(gray, filtered, 3, 7, 21 ); //filter_strength,templatewindow size, search window size

        Mat element2 = getStructuringElement(MORPH_RECT, Size(6, 10));
        dilate(erosion_dst, dilation_dst, element2, Point(-1, -1), 2, 1, 1);
        Canny(dilation_dst, dst, 50, 200, 3); 
        /// Find contours
        findContours( dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
        
        
        double largest_area=0;
        for( size_t i = 0; i< contours.size(); i++ ) // iterate through each contour.
        {
            double area = contourArea( contours[i] );  //  Find the area of contour
            
            //cout<<contourArea( contours[i] )<<endl;
            if( area > largest_area )
        {
            largest_area = area;
            largest_contour_index = i;               //Store the index of largest contour
            //bounding_rect = boundingRect( contours[i] ); // Find the bounding rectangle for biggest contour
        }

    }
      //cout<<"Largest Contour Area is :"<<contourArea( contours[largest_contour_index] )<<endl;
    //cout<<largest_contour_index<<endl;

        
/*

        /// Draw contours
        Mat drawing = Mat::zeros( dst.size(), CV_8UC3 );
        for( int i = 0; i< contours.size(); i++ )
           {
             Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
             }

        /// Show in a window
        namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
        imshow( "Contours", drawing );
        */


        RotatedRect minRect;
        minRect = minAreaRect( Mat(contours[largest_contour_index]));

        Mat drawing = Mat::zeros( dst.size(), CV_8UC3 );
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        if(largest_contour_index < contours.size())
          drawContours( drawing, contours, largest_contour_index, color, 2, 8, hierarchy, 0, Point() );
         
         // rotated rectangle
       Point2f rect_points[4]; 
       minRect.points( rect_points );
       for( int j = 0; j < 4; j++ )
          line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );

        namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
        imshow( "Contours", drawing );
        if(minRect.angle<-50)
             temp=minRect.angle+90;
        else
             temp=minRect.angle;
        cout<< temp <<endl;
        std_msgs::Float64 angle_fin;
        angle_fin.data=round(temp);
        angle_pub.publish(angle_fin);


        //imshow("Result", frame);


      imshow("original", dilation_dst);


        //cvtColor(frame, edges, CV_BGR2GRAY);
        //GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
       // Canny(edges, edges, 0, 30, 3);
        //imshow("edges", frame);
        auto stop = high_resolution_clock::now(); 
        auto duration = duration_cast<microseconds>(stop - start); 
        //cout << duration.count() << endl; 
        if(waitKey(10) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}