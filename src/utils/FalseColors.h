#pragma once

#include <iostream>
#include <string>
#include <vector>

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace Eigen;
#include <opencv2/core/eigen.hpp>

using namespace std;


class FalseColors
{
public:
    FalseColors() {
        cv::Mat colormap_gray = cv::Mat::zeros( 1, 256, CV_8UC1 );
        for( int i=0 ; i<256; i++ ) colormap_gray.at<uchar>(0,i) = i;
        cv::applyColorMap(colormap_gray, colormap_color, cv::COLORMAP_HOT	);
    }

    cv::Scalar getFalseColor( float f ) {
        int idx = (int) (f*255.);
        if( f<0 ) {
            idx=0;
        }
        if( f>255 ) {
            idx=255;
        }


        cv::Vec3b f_ = colormap_color.at<cv::Vec3b>(0,  (int)idx );
        cv::Scalar color_marker = cv::Scalar(f_[0],f_[1],f_[2]);
        return color_marker;
    }

    void getFalseColor( float f, int& red, int& green, int& blue )
    {
        int idx = (int) (f*255.);
        if( f<0 ) {
            idx=0;
        }
        if( f>255 ) {
            idx=255;
        }


        cv::Vec3b f_ = colormap_color.at<cv::Vec3b>(0,  (int)idx );
        red = (int)f_[2];
        green = (int)f_[1];
        blue = (int)f_[0];
    }

    cv::Mat getStrip( int nrows, int ncols ) {
        cv::Mat colormap_gray = cv::Mat::zeros( nrows, ncols, CV_8UC1 );

        for( int r=0; r<nrows; r++ ) {
            for( int c=0 ; c<ncols; c++ )
                colormap_gray.at<uchar>(r,c) = (uchar) ( (float(c)/ncols)*256 );
        }

        cv::Mat __dst;
        cv::applyColorMap(colormap_gray, __dst, cv::COLORMAP_HOT	);
        return __dst;
    }


  static cv::Scalar randomColor( int rng )
  {
      int icolor = (unsigned) rng;
      //TODO: to get even better colors use rng to generave a hue. With max saturation and brightness convert it to RGB
      return cv::Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
  }


private:
    cv::Mat colormap_color;


};
