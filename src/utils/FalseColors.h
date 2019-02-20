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
    //   int icolor = (unsigned) rng;

    #if 0
    srand( rng );
      int icolor = (unsigned) rand();
      //TODO: to get even better colors use rng to generave a hue. With max saturation and brightness convert it to RGB
      return cv::Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
     #endif



     cv::Mat ze = cv::Mat::zeros( 3,3, CV_8UC3 );
    //  ze.at<cv::Vec3b>(0,0)[0] = rand() % 255;
     ze.at<cv::Vec3b>(0,0)[0] = int( 5458. / (rng+1. ) ) % 255;
     ze.at<cv::Vec3b>(0,0)[1] = 255;
     ze.at<cv::Vec3b>(0,0)[2] = 255;

     cv::Mat rgb;
     cv::cvtColor(ze, rgb,cv::COLOR_HSV2BGR);
     int r, g, b;
     r = rgb.at<cv::Vec3b>(0,0)[0];
     g = rgb.at<cv::Vec3b>(0,0)[1];
     b = rgb.at<cv::Vec3b>(0,0)[2];
     return cv::Scalar( r,g,b );




  }

  static std::vector<std::string>
  split( std::string const& original, char separator )
  {
        std::vector<std::string> results;
        std::string::const_iterator start = original.begin();
        std::string::const_iterator end = original.end();
        std::string::const_iterator next = std::find( start, end, separator );
        while ( next != end ) {
            results.push_back( std::string( start, next ) );
            start = next + 1;
            next = std::find( start, end, separator );
        }
        results.push_back( std::string( start, next ) );
        return results;
    }

  // append a status image . ';' separated
  static void append_status_image( cv::Mat& im, const string& msg, float txt_size=0.4, cv::Scalar bg_color=cv::Scalar(0,0,0), cv::Scalar txt_color=cv::Scalar(255,255,255) )
  {
        bool is_single_channel = (im.channels()==1)?true:false;
        txt_size = (txt_size<0.1 || txt_size>2)?0.4:txt_size;

        std::vector<std::string> msg_tokens = split(msg, ';');
        int status_im_height = 50+20*msg_tokens.size();

        cv::Mat status;
        if( is_single_channel )
            status = cv::Mat(status_im_height, im.cols, CV_8UC1, cv::Scalar(0,0,0) );
        else
            status = cv::Mat(status_im_height, im.cols, CV_8UC3, bg_color );


        for( int h=0 ; h<msg_tokens.size() ; h++ )
            cv::putText( status, msg_tokens[h].c_str(), cv::Point(10,20+20*h),
                    cv::FONT_HERSHEY_SIMPLEX,
                    txt_size, txt_color, 1.5 );


        cv::vconcat( im, status, im );


    }


private:
    cv::Mat colormap_color;


};
