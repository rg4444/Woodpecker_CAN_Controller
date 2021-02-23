#include "bumble_grabber.hpp"

#ifdef USE_BB_CAMERA
    #include <flycapture/FlyCapture2.h>
    #include <opencv2/core/core.hpp>
    #include <opencv2/imgproc/imgproc.hpp>
    #include <opencv2/highgui/highgui.hpp>
#endif

#include <stdlib.h>
#include <string>

#include <iostream>

#ifdef USE_BB_CAMERA
    using namespace FlyCapture2;
#endif
 
#ifdef USE_BB_CAMERA
    static Camera camera;
    static CameraInfo camInfo;
    static Error error;
#endif
static char  file_name[30];

#ifdef USE_BB_CAMERA
    cv::Mat offsetImageWithPadding(const cv::Mat& originalImage, int offsetX, int offsetY, cv::Scalar backgroundColour){
        cv::Mat padded = cv::Mat(originalImage.rows + 2 * abs(offsetY), originalImage.cols + 2 * abs(offsetX), CV_8U, backgroundColour);
        originalImage.copyTo(padded(cv::Rect(abs(offsetX), abs(offsetY), originalImage.cols, originalImage.rows)));
        return cv::Mat(padded,cv::Rect(abs(offsetX) + offsetX, abs(offsetY) + offsetY, originalImage.cols, originalImage.rows));
        }

    cv::Mat cropResize(const cv::Mat& originalImage, int offsetX, int offsetY){
        return cv::Mat(originalImage, cv::Rect((abs(offsetX) - offsetX) / 2, (abs(offsetY) - offsetY) / 2, originalImage.cols - abs(offsetX), originalImage.rows - abs(offsetY)));
        }
#endif

bool create_camera(void){

    #ifdef USE_BB_CAMERA
        error = camera.Connect( 0 );
        if ( error != PGRERROR_OK )
        {
            std::cout << "Failed to connect to camera" << std::endl;     
            return false;
        }
        
        // Get the camera info and print it out
        error = camera.GetCameraInfo( &camInfo );
        if ( error != PGRERROR_OK )
        {
            std::cout << "Failed to get camera info from camera" << std::endl;     
            return false;
        }
        std::cout << camInfo.vendorName << " "
        		  << camInfo.modelName << " " 
        		  << camInfo.serialNumber << std::endl;
    	
    	error = camera.StartCapture();
        if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
        {
            std::cout << "Bandwidth exceeded" << std::endl;     
            return false;
        }
        else if ( error != PGRERROR_OK )
        {
            std::cout << "Failed to start image capture" << std::endl;     
            return false;
        } 
    #endif

}

bool destroy_camera(void){

	
    #ifdef USE_BB_CAMERA
    	error = camera.StopCapture();
        if ( error != PGRERROR_OK )
        {
            // This may fail when the camera was removed, so don't show 
            // an error message
            return false;
        }  
    	
    	camera.Disconnect();

    #endif

    return true;

}

char* save_frame_png(unsigned int num){
        
    #ifdef USE_BB_CAMERA
    
        std::string filename_str; 
        const int x_offset = -32; 
        const int y_offset = -25;

        filename_str = std::string("tri_frame_") + std::to_string(num) + std::string(".png");
        cv::Mat in_channels[3];
        cv::Mat out_channels[3];
        
        Image rawImage;
		Error error = camera.RetrieveBuffer( &rawImage );
		if ( error != PGRERROR_OK )
		{
			std::cout << "capture error" << std::endl;
		}
		
		// convert to rgb
	    Image rgbImage;
        rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );
       
		// convert to OpenCV Mat
		unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();       
		cv::Mat image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
		
		
        cv::split(image, in_channels);
        cv::Mat temp = offsetImageWithPadding(in_channels[2], x_offset,y_offset, cv::Scalar(0,0,0));
        
        in_channels[2] = temp.clone();
        cv::merge(in_channels,3, image);
        
        image = cropResize(image, x_offset, y_offset);

        //cv::split(image, out_channels);
        
        //for(int i = 0; i < 3; i++){
                  
            //cv::resize(out_channels[i], out_channels[i], cv::Size(640,480),0,0,cv::INTER_AREA);
        
        
        //}

        //cv::merge(out_channels,3, image);
        
        
        cv::resize(image, image, cv::Size(640,480),0,0,cv::INTER_AREA);
        cv::imshow("image", image);
       
        cv::imwrite(filename_str, image);
    
        
        
        
        strcpy(file_name,filename_str.c_str());
    
    #endif

    //return (char *) &filename_str[0]; //return char pointer of string begin
    return  file_name; 

}
