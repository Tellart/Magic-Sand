/***********************************************************************
KinectV2Grabber - KinectV2Grabber takes care of communication with
the KinectV2 and the filtering of the data

--- Based on ofxKinectforWindows2 by Elliot Woods
Copyright (c) 2016-2017 Elliot Woods

--- Adapted from Magic-Sand by Thomas Wolf and Rasmus R. Paulsen
Copyright (c) 2016-2017 Thomas Wolf and Rasmus R. Paulsen (people.compute.dtu.dk/rapa)
***********************************************************************/

#pragma once
#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxCv.h"
#include "ofxKinectForWindows2.h"

// Define Kinect V2 resolutions
#define DEPTH_WIDTH 512
#define DEPTH_HEIGHT 424
#define DEPTH_SIZE DEPTH_WIDTH * DEPTH_HEIGHT

#define COLOR_WIDTH 1920
#define COLOR_HEIGHT 1080
#define COLOR_SIZE COLOR_WIDTH * COLOR_HEIGHT

// You can choose to use the color feed or the depth feed as the source for depth calculations
// define USE_COLOR to use the color feed
//#define USE_COLOR

#ifdef USE_COLOR
	#define _WIDTH COLOR_WIDTH
	#define _HEIGHT COLOR_HEIGHT
	#define _SIZE COLOR_SIZE
#else
	#define _WIDTH DEPTH_WIDTH
	#define _HEIGHT DEPTH_HEIGHT
	#define _SIZE DEPTH_SIZE
#endif

class KinectV2Grabber: public ofThread{

	public:
		typedef float Filtered; // Data type for filtered values

		KinectV2Grabber();
		~KinectV2Grabber();
		void start();
		void stop();
		void performInThread(std::function<void(KinectV2Grabber&)> action);
		bool setup();
		bool openKinect();
		void setupFramefilter(float newMaxOffset, ofRectangle ROI, bool spatialFilter, bool followBigChange, int numAveragingSlots);
		void initiateBuffers(void); // Reinitialise Buffers
		void resetBuffers(void);

		bool bInteraction;
	
		ofVec3f getStatDepthBuffer(int x, int y);
		float getAveragingDepthBuffer(int x, int y, int slotNum);
		float getValidDepthBuffer(int x, int y);

		void setFollowBigChange(bool newfollowBigChange);
		void setKinectROI(ofRectangle skinectROI);
		void setAveragingSlotsNumber(int snumAveragingSlots);

		void decStoredframes(){
			storedframes -= 1;
		}

		bool isImageStabilized(){
			return firstImageReady;
		}

		bool isFrameNew(){
			return newFrame;
		}

		ofVec2f getKinectSize(){  // DEPTH
			return ofVec2f(DEPTH_WIDTH, DEPTH_HEIGHT);
		}

		ofVec2f getKinectColorSize(){
			return ofVec2f(COLOR_WIDTH, COLOR_HEIGHT);
		}

#ifndef USE_COLOR
		ofRectangle getdkinectROI(){
			return dkinectROI;
		}
#endif // !USE_COLOR

		ofVec2f getDepthCoordAt(int x, int y){
			// COLOR => DEPTH 
			int index = (y * COLOR_WIDTH) + (COLOR_WIDTH - x); // because we mirrored the feeds
			ofVec2f depthCoord(0, 0);
			if(getDepthCoordLookUp()){
				depthCoord = ofVec2f(int(depthCoordLookUp[index].x), int(depthCoordLookUp[index].y));
			}
			// NOTE: casting to integer
			return depthCoord;
		}

#ifndef USE_COLOR
		ofVec3f getDepth2WorldCoordAt(int x, int y){
			// DEPTH => WORLD in meters
			int index = (y * DEPTH_WIDTH) + (DEPTH_WIDTH - x); // because we mirrored the feeds
			return ofVec3f(depthMapper[index*3], depthMapper[index*3+1], depthMapper[index*3+2]);
		}
#endif

		ofVec3f getColor2WorldCoordAt(int x, int y){
			// COLOR => WORLD in meters
			int index = (y * COLOR_WIDTH) + (COLOR_WIDTH - x); // because we mirrored the feeds
			return ofVec3f(colorMapper[index*3], colorMapper[index*3+1], colorMapper[index*3+2]);
		}

		ofVec2f cameraSpaceTable;

		void setMaxOffset(float newMaxOffset){
			maxOffset = newMaxOffset;
		}

		void setSpatialFiltering(bool newspatialFilter){
			spatialFilter = newspatialFilter;
		}

		void setInPainting(bool inp){
			doInPaint = inp;
		}

		// Should the entire frame be filtered and thereby ignoring the KinectROI
		void setFullFrameFiltering(bool ff, ofRectangle ROI);

		ofThreadChannel<ofFloatPixels> filteredDepth; 
		ofThreadChannel<ofFloatPixels> filteredX;
		ofThreadChannel<ofFloatPixels> filteredY;
		ofThreadChannel<ofPixels> colored;

	private:
		void threadedFunction() override;

		void filterDepth();
		void filterXY();

		bool isInsideROI(int x, int y); // test is x, y is inside ROI
		bool isInsideDepthVOF(int x, int y); // test is x, y is inside DEPTH FOV
		void applySpaceFilter();

		// A simple inpainting algorithm to remove outliers in the depth
		// Since the shader has no way of filtering outliers (0 and 4000 values mainly) it creates visual artifacts if they are not 
		// removed prior to the shader pass
		void applySimpleOutlierInpainting();
		float findInpaintValue(float *data, int x, int y);
		float findInpaintValueXY(float *data, int x, int y);
		double ROIAverageValue = 0;
		int setToLocalAvg = 0;
		int setToGlobalAvg = 0;

		bool newFrame;
		bool buffersInitiated;
		bool firstImageReady;
		int storedframes;

		// Thread lambda functions (actions)
		vector<std::function<void(KinectV2Grabber&)> > actions;
		ofMutex actionsLock;

		// Kinect parameters
		bool kinectOpened;
		ofxKFW2::Device kinect;
		ICoordinateMapper* coordinateMapper; // from ofxKinectforWindows2

		// Table's real world parameters in mm (rough)
		int tableSizeX, tableSizeY;

		// based on color coordinates or depth coordinates
		int ROIMinX, ROIMaxX; // Region Of Interest (the table outlines) 
		int ROIMinY, ROIMaxY;

#ifndef USE_COLOR
		ofRectangle dkinectROI; // alternative ROI for depth coordinates
#endif

		// ofxKinectforWindows2 functions
		bool getDepth2WorldCoordinates();
		bool getColor2WorldCoordinates();
		bool getDepthCoordLookUp();

		// Buffers
		ofImage kinectColorImage;
		vector<ofVec2f> depthCoordLookUp;

		// Color 2 World Coordinate Mapping
		ofFloatPixels colorMapper;
		ofFloatPixels colorMapperX;
		ofFloatPixels colorMapperY;
		ofFloatPixels colorMapperDepth;

		// Depth 2 World Coordinate Mapping
		ofFloatPixels depthMapper;
		ofFloatPixels depthMapperX;
		ofFloatPixels depthMapperY;
		ofFloatPixels depthMapperDepth;

		// Filtered/averaged x, y and depth data based on either depth or color coordinates
		ofFloatPixels filteredDepthframe;
		ofFloatPixels filteredXframe;
		ofFloatPixels filteredYframe;

		// Filtereing Map buffers, to work with either colorMapper or depthMapper
		float* averagingDepthBuffer; // Buffer to calculate running averages of each pixel's depth value
		float* statDepthBuffer; // Buffer retaining the running means and variances of each pixel's depth value
		float* validDepthBuffer; // Buffer holding the most recent stable depth value for each pixel

		float* averagingXBuffer; // Buffer to calculate running averages of each pixel's x value
		float* statXBuffer; // Buffer retaining the running means and variances of each pixel's x value
		float* validXBuffer; // Buffer holding the most recent stable depth value for each pixel

		float* averagingYBuffer; // Buffer to calculate running averages of each pixel's y value
		float* statYBuffer; // Buffer retaining the running means and variances of each pixel's y value
		float* validYBuffer; // Buffer holding the most recent stable depth value for each pixel

		// Frame filter parameters
		int numAveragingSlots; // Number of slots in each pixel's averaging buffer
		int averagingSlotIndex; // Index of averaging slot in which to store the next frame's depth values
		unsigned int minNumSamples; // Minimum number of valid samples needed to consider a pixel stable
		float maxVariance; // Maximum variance to consider a pixel stable
		float maxVarianceXY;
		float initialValue;
		float initialValueXY;
		float hysteresis; // Amount by which a new filtered value has to differ from the current value to update the display
		float hysteresisXY;

		bool followBigChange;
		float bigChange; // Amount of change over which the averaging slot is reset to new value

		bool spatialFilter; // Flag whether to apply a spatial filter to time-averaged depth values
		float maxOffset;

		int minInitFrame; // Minimal number of frame to consider the kinect initialized
		int currentInitFrame;

		bool doInPaint;
		bool doFullFrameFiltering;

};

