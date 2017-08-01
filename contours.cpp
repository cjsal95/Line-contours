#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#define PI 3.1415926



//#if !defined LINEF
//#define LINEF
 
 
class LineFinder {
 
  private:
 
      // original image
      cv::Mat img;
 
      // vector containing the end points 
      // of the detected lines
      std::vector<cv::Vec4i> lines;
 
      // accumulator resolution parameters
      double deltaRho;
      double deltaTheta;
 
      // minimum number of votes that a line 
      // 선으로 간주되기 전에 받아야 하는 투표 최소 개수
      int minVote;
 
      // 선의 최소 길이
      double minLength;
 
      // 선을 따라가는 최대 허용 간격(gap)
      double maxGap;
 
  public:
 
      // Default accumulator resolution is 1 pixel by 1 degree
      // no gap, no mimimum length
      LineFinder() : deltaRho(1), deltaTheta(PI/180), minVote(10), minLength(0.), maxGap(0.) {}
 
      // Set the minimum number of votes
      void setMinVote(int minv) {
 
          minVote= minv;
      }
 
      // Set line length and gap
      void setLineLengthAndGap(double length, double gap) {
 
          minLength= length;
          maxGap= gap;
      }
 
      // Apply probabilistic Hough Transform
      std::vector<cv::Vec4i> findLines(cv::Mat& binary) {
 
          lines.clear();

          cv::HoughLinesP(binary,lines,deltaRho,deltaTheta,minVote, minLength, maxGap);
 
          return lines;
      }
 
      // Draw the detected lines on an image
      void drawDetectedLines(cv::Mat &image, cv::Scalar color=cv::Scalar(0,0,255)) {
    
          // Draw the lines
          std::vector<cv::Vec4i>::const_iterator it2= lines.begin();
    
          while (it2!=lines.end()) {
        
              cv::Point pt1((*it2)[0],(*it2)[1]);        
              cv::Point pt2((*it2)[2],(*it2)[3]);
              
              cv::line( image, pt1, pt2, color, 5); 
        
              ++it2;    
          }
      }
 
      
      std::vector<cv::Vec4i> removeLinesOfInconsistentOrientations(
          const cv::Mat &orientations, double percentage, double delta) {
 
              std::vector<cv::Vec4i>::iterator it= lines.begin();
    
              // check all lines
              while (it!=lines.end()) {
 
                  // end points
                  int x1= (*it)[0];
                  int y1= (*it)[1];
                  int x2= (*it)[2];
                  int y2= (*it)[3];
           
                  // line orientation + 90o to get the parallel line
                  double ori1= atan2(static_cast<double>(y1-y2),static_cast<double>(x1-x2))+PI/2;
                  if (ori1>PI) ori1= ori1-2*PI;
 
                  double ori2= atan2(static_cast<double>(y2-y1),static_cast<double>(x2-x1))+PI/2;
                  if (ori2>PI) ori2= ori2-2*PI;
    
                  // for all points on the line
                  cv::LineIterator lit(orientations,cv::Point(x1,y1),cv::Point(x2,y2));
                  int i,count=0;
                  for(i = 0, count=0; i < lit.count; i++, ++lit) { 
        
                      float ori= *(reinterpret_cast<float *>(*lit));
 
                      // is line orientation similar to gradient orientation ?
                      if (std::min(fabs(ori-ori1),fabs(ori-ori2))<delta)
                          count++;
        
                  }
 
                  double consistency= count/static_cast<double>(i);
 
                  // set to zero lines of inconsistent orientation
                  if (consistency < percentage) {
 
                      (*it)[0]=(*it)[1]=(*it)[2]=(*it)[3]=0;
 
                  }
 
                  ++it;
              }
 
              return lines;
      }
};
 
 
//#endif

//#if !defined SOBELEDGES
//#define SOBELEDGES
 
 
class EdgeDetector {
 
  private:
 
      // original image
      cv::Mat img;
 
      // 16-bit signed int image
      cv::Mat sobel;
 
      // Aperture size of the Sobel kernel
      int aperture;
 
      // Sobel magnitude
      cv::Mat sobelMagnitude;
 
      // Sobel orientation
      cv::Mat sobelOrientation;
 
  public:
 
      EdgeDetector() : aperture(3) {}
 
      // Compute the Sobel
      void computeSobel(const cv::Mat& image) {
 
          cv::Mat sobelX;
          cv::Mat sobelY;
 
          // Compute Sobel
          cv::Sobel(image, sobelX, CV_32F, 1, 0, aperture);
          cv::Sobel(image, sobelY, CV_32F, 0, 1, aperture);
 
          // Compute magnitude and orientation
          cv::cartToPolar(sobelX, sobelY, sobelMagnitude, sobelOrientation);
      }
 
      // Get Sobel orientation
      cv::Mat getOrientation() {
 
          return sobelOrientation;
      }
 
};
 
 
//#endif



int main()
{
	cv::VideoCapture cap(0);
	cv::Mat image;

	while(true) {

		//cap >> image;
		image = cv::imread("line2.jpg"); 

		cv::Mat ROI(image, cv::Rect(0 + image.cols/3, image.rows/2, image.cols/3, image.rows/2));

		cv::Mat img = ROI;

		

		cv::medianBlur(img, img, 5);

		// Compute Sobel
		EdgeDetector ed;
		ed.computeSobel(img);

		// Apply Canny algorithm
		cv::Mat contours;
		cv::Canny(img,contours,70,200);  //70 , 200

		cv::namedWindow("contours");
		cv::imshow("contours",contours);

		// Create LineFinder instance
		LineFinder ld;

		// 선 길이와 간격 설정
		ld.setLineLengthAndGap(100, 20);  // 100 , 20
		ld.setMinVote(60);   // 투표 최소 개수 설정 60

		// Detect lines
		std::vector<cv::Vec4i> li= ld.findLines(contours);

		// 조건에 맞지 않는 선 제거
		ld.removeLinesOfInconsistentOrientations(ed.getOrientation(),0.4,0.1);  // 0.4   0.1

		ld.drawDetectedLines(img);

		image.copyTo(img);

		cv::namedWindow("imgae");
		cv::imshow("imgae", image);

		if(cv::waitKey(10) == 27)
			break;
	}
}
