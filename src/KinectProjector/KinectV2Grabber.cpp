/***********************************************************************
KinectV2Grabber - KinectV2Grabber takes care of communication with
the KinectV2 and the filtering of the data

--- Based on ofxKinectforWindows2 by Elliot Woods
Copyright (c) 2016-2017 Elliot Woods

--- Adapted from Magic-Sand by Thomas Wolf and Rasmus R. Paulsen
Copyright (c) 2016-2017 Thomas Wolf and Rasmus R. Paulsen (people.compute.dtu.dk/rapa)
***********************************************************************/

#include "KinectV2Grabber.h"
#include "ofConstants.h"

KinectV2Grabber::KinectV2Grabber()
:newFrame(true),
buffersInitiated(false),
kinectOpened(false){

}

KinectV2Grabber::~KinectV2Grabber(){
	waitForThread(true);
}

void KinectV2Grabber::start(){
	startThread(true);
}

void KinectV2Grabber::stop(){
	stopThread();
}

bool KinectV2Grabber::setup(){
	// Settings and defaults
	storedframes = 0;
	ROIAverageValue = 0;
	setToGlobalAvg = 0;
	setToLocalAvg = 0;
	doInPaint = 0;
	doFullFrameFiltering = false;
	bInteraction = false;

	kinectColorImage.allocate(COLOR_WIDTH, COLOR_HEIGHT, OF_IMAGE_COLOR);
	kinectColorImage.setUseTexture(false);

#ifdef USE_COLOR
	colorMapperX.allocate(COLOR_WIDTH, COLOR_HEIGHT, 1);
	colorMapperY.allocate(COLOR_WIDTH, COLOR_HEIGHT, 1);
	colorMapperDepth.allocate(COLOR_WIDTH, COLOR_HEIGHT, 1);
#else
	depthMapperX.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, 1);
	depthMapperY.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, 1);
	depthMapperDepth.allocate(DEPTH_WIDTH, DEPTH_HEIGHT, 1);

	dkinectROI = ofRectangle(0, 0, DEPTH_WIDTH, DEPTH_HEIGHT);
#endif

	filteredDepthframe.allocate(_WIDTH, _HEIGHT, 1);
	filteredXframe.allocate(_WIDTH, _HEIGHT, 1);
	filteredYframe.allocate(_WIDTH, _HEIGHT, 1);

	depthCoordLookUp.resize(COLOR_SIZE); // for each coordinate in the color frame, get the coordinate in the depth frame

	return openKinect();
}

bool KinectV2Grabber::openKinect(){
	kinect.open();
	kinect.initDepthSource();
	kinect.initColorSource();

	if(kinect.getSensor()->get_CoordinateMapper(&coordinateMapper) < 0){
		ofLogError("KinectGrabber") << "openKinect: Could not acquire CoordinateMapper!";
	}

	kinectOpened = true;
	return kinectOpened;
}

void KinectV2Grabber::setupFramefilter(float newMaxOffset, ofRectangle ROI, bool sspatialFilter, bool sfollowBigChange, int snumAveragingSlots){

	spatialFilter = sspatialFilter;
	followBigChange = sfollowBigChange;
	numAveragingSlots = snumAveragingSlots;
	minNumSamples = (numAveragingSlots + 1) / 2;
	maxOffset = newMaxOffset;

	// Framefilter default parameters
	maxVariance = 20; // 4 (original values)
	maxVarianceXY = 16; // 4   16 seemed to work
	hysteresis = 0.5f; // 0.5f
	hysteresisXY = 0.5f; // 0.5f
	bigChange = 20.0f; // 10.0f 
	// TIP: play with bigChange value if the 'quick reaction' flickers too much

	initialValue = 4000;
	initialValueXY = 4000;
	minInitFrame = 60;

	// Setup ROI
	setKinectROI(ROI);

	// Setup buffers
	initiateBuffers();
}

void KinectV2Grabber::initiateBuffers(void){

	averagingSlotIndex = 0;

	filteredDepthframe.set(0);
	filteredXframe.set(0);
	filteredYframe.set(0);

	// Initialize the averaging buffers:
	// Buffers to calculate running averages of each pixel's depth value
	averagingDepthBuffer = new float[numAveragingSlots*_HEIGHT*_WIDTH];
	averagingXBuffer = new float[numAveragingSlots*_HEIGHT*_WIDTH];
	averagingYBuffer = new float[numAveragingSlots*_HEIGHT*_WIDTH];
	// @sabrina: I copied these and put them in scopes so that the variable names can stay the same
	{
		float* averagingBufferPtr = averagingDepthBuffer;
		for(int i = 0; i < numAveragingSlots; ++i)
			for(unsigned int y = 0; y < _HEIGHT; ++y)
				for(unsigned int x = 0; x < _WIDTH; ++x, ++averagingBufferPtr)
					*averagingBufferPtr = initialValue;
	}
	{
		float* averagingBufferPtr = averagingXBuffer;
		for(int i = 0; i < numAveragingSlots; ++i)
			for(unsigned int y = 0; y < _HEIGHT; ++y)
				for(unsigned int x = 0; x < _WIDTH; ++x, ++averagingBufferPtr)
					*averagingBufferPtr = initialValueXY;
	}
	{
		float* averagingBufferPtr = averagingYBuffer;
		for(int i = 0; i < numAveragingSlots; ++i)
			for(unsigned int y = 0; y < _HEIGHT; ++y)
				for(unsigned int x = 0; x < _WIDTH; ++x, ++averagingBufferPtr)
					*averagingBufferPtr = initialValueXY;
	}

	// Initialize the statistics buffers:
	// Buffer retaining the running means and variances of each pixel's depth value
	statDepthBuffer = new float[_HEIGHT*_WIDTH * 3];
	statXBuffer = new float[_HEIGHT*_WIDTH * 3];
	statYBuffer = new float[_HEIGHT*_WIDTH * 3];
	// @sabrina: I copied these and put them in scopes so that the variable names can stay the same
	{
		float* sbPtr = statDepthBuffer;
		for(unsigned int y = 0; y < _HEIGHT; ++y)
			for(unsigned int x = 0; x < _WIDTH; ++x)
				for(int i = 0; i < 3; ++i, ++sbPtr)
					*sbPtr = 0.0;
	}
	{
		float* sbPtr = statXBuffer;
		for(unsigned int y = 0; y < _HEIGHT; ++y)
			for(unsigned int x = 0; x < _WIDTH; ++x)
				for(int i = 0; i < 3; ++i, ++sbPtr)
					*sbPtr = 0.0;
	}
	{
		float* sbPtr = statYBuffer;
		for(unsigned int y = 0; y < _HEIGHT; ++y)
			for(unsigned int x = 0; x < _WIDTH; ++x)
				for(int i = 0; i < 3; ++i, ++sbPtr)
					*sbPtr = 0.0;
	}

	// Initialize the valid buffers: 
	// Buffer holding the most recent stable depth value for each pixel
	validDepthBuffer = new float[_HEIGHT*_WIDTH];
	validXBuffer = new float[_HEIGHT*_WIDTH];
	validYBuffer = new float[_HEIGHT*_WIDTH];
	// @sabrina: I copied these and put them in scopes so that the variable names can stay the same
	{
		float* vbPtr = validDepthBuffer;
		for(unsigned int y = 0; y < _HEIGHT; ++y)
			for(unsigned int x = 0; x < _WIDTH; ++x, ++vbPtr)
				*vbPtr = initialValue;
	}
	{
		float* vbPtr = validXBuffer;
		for(unsigned int y = 0; y < _HEIGHT; ++y)
			for(unsigned int x = 0; x < _WIDTH; ++x, ++vbPtr)
				*vbPtr = initialValueXY;
	}
	{
		float* vbPtr = validYBuffer;
		for(unsigned int y = 0; y < _HEIGHT; ++y)
			for(unsigned int x = 0; x < _WIDTH; ++x, ++vbPtr)
				*vbPtr = initialValueXY;
	}

	buffersInitiated = true;
	currentInitFrame = 0;
	firstImageReady = false;
}

void KinectV2Grabber::resetBuffers(void){
	if(buffersInitiated){
		buffersInitiated = false;

		delete[] averagingDepthBuffer;
		delete[] statDepthBuffer;
		delete[] validDepthBuffer;

		delete[] averagingXBuffer;
		delete[] statXBuffer;
		delete[] validXBuffer;

		delete[] averagingYBuffer;
		delete[] statYBuffer;
		delete[] validYBuffer;

	}
	initiateBuffers();
}

void KinectV2Grabber::threadedFunction(){
	while(isThreadRunning()){
		this->actionsLock.lock(); // Update the grabber state if needed
		for(auto & action : this->actions){
			action(*this);
		}
		this->actions.clear();
		this->actionsLock.unlock();

		kinect.update();
		if(kinect.isFrameNew()){

			getColor2WorldCoordinates(); // get world coordinates from color frame
#ifndef USE_COLOR
			getDepth2WorldCoordinates(); // get world coordinates from depth frame
#endif // END !USE_COLOR
	
			filterXY(); // filter xy coordinates
			filterDepth(); // filter depth coordinates

			// Mirror all frames for Kinect V2
			// NOTE: this has to be done every frame
			filteredDepthframe.mirror(0, 1);
			filteredXframe.mirror(0, 1);
			filteredYframe.mirror(0, 1);

			if(kinect.getColorSource()->isFrameNew()){
				kinectColorImage.setFromPixels(kinect.getColorSource()->getPixels()); // gets RGBA
				kinectColorImage.mirror(0, 1);
				kinectColorImage.setImageType(OF_IMAGE_COLOR); // force to RGB (instead of RGBA) for ofxCvColorImage in KinectProjector class
			}
		}

		if(storedframes == 0){

			filteredDepth.send(std::move(filteredDepthframe));
			filteredX.send(std::move(filteredXframe));
			filteredY.send(std::move(filteredYframe));
			colored.send(std::move(kinectColorImage.getPixels()));
			
			lock();
			storedframes += 1;
			unlock();
		}
	}

	kinect.close();

	delete[] averagingDepthBuffer;
	delete[] statDepthBuffer;
	delete[] validDepthBuffer;

	delete[] averagingXBuffer;
	delete[] statXBuffer;
	delete[] validXBuffer;

	delete[] averagingYBuffer;
	delete[] statYBuffer;
	delete[] validYBuffer;

}

void KinectV2Grabber::performInThread(std::function<void(KinectV2Grabber&)> action){
	this->actionsLock.lock();
	this->actions.push_back(action);
	this->actionsLock.unlock();
}

// TODO: make a filter function with arguments
// inputs: width & height; buffer variables; 

void KinectV2Grabber::filterDepth(){

	if(buffersInitiated && numAveragingSlots < 2){

		// Filter depth
#ifdef USE_COLOR
		const Filtered* inputFramePtr = static_cast<const Filtered*>(colorMapperDepth.getData());
#else
		const Filtered* inputFramePtr = static_cast<const Filtered*>(depthMapperDepth.getData());
#endif
		float* filteredFramePtr = filteredDepthframe.getData();
		inputFramePtr += ROIMinY*_WIDTH;
		filteredFramePtr += ROIMinY*_WIDTH;

		for(unsigned int y = ROIMinY; y < ROIMaxY; ++y){

			inputFramePtr += ROIMinX;
			filteredFramePtr += ROIMinX;

			for(unsigned int x = ROIMinX; x < ROIMaxX; ++x, ++inputFramePtr, ++filteredFramePtr){
				float newVal = static_cast<float>(*inputFramePtr * 1000);
				//ofLogVerbose("KinectGrabber:") << "DEBUG:" << newVal;
				*filteredFramePtr = newVal;
			}
			inputFramePtr += _WIDTH - ROIMaxX;
			filteredFramePtr += _WIDTH - ROIMaxX;
		}
	}else if(buffersInitiated){

#ifdef USE_COLOR
		const Filtered* inputFramePtr = static_cast<const Filtered*>(colorMapperDepth.getData());
#else
		const Filtered* inputFramePtr = static_cast<const Filtered*>(depthMapperDepth.getData());
#endif
		float* averagingBufferPtr = averagingDepthBuffer + averagingSlotIndex*_HEIGHT*_WIDTH;
		float* statBufferPtr = statDepthBuffer;
		float* validBufferPtr = validDepthBuffer;
		float* filteredFramePtr = filteredDepthframe.getData();

		inputFramePtr += ROIMinY*_WIDTH;
		averagingBufferPtr += ROIMinY*_WIDTH;
		statBufferPtr += ROIMinY*_WIDTH * 3;
		validBufferPtr += ROIMinY*_WIDTH;
		filteredFramePtr += ROIMinY*_WIDTH;

		int countChange = 0;

		for(unsigned int y = ROIMinY; y<ROIMaxY; ++y){

			inputFramePtr += ROIMinX;
			averagingBufferPtr += ROIMinX;
			statBufferPtr += ROIMinX * 3;
			validBufferPtr += ROIMinX;
			filteredFramePtr += ROIMinX;

			for(unsigned int x = ROIMinX; x<ROIMaxX; ++x, ++inputFramePtr, ++averagingBufferPtr, statBufferPtr += 3, ++validBufferPtr, ++filteredFramePtr){

				float newVal = static_cast<float>(*inputFramePtr * 1000);
				float oldVal = *averagingBufferPtr;

				if(newVal > maxOffset){ // We are under the ceiling plane
					*averagingBufferPtr = newVal; // Store the value

					if(statBufferPtr[0] > 0){ 
						float oldFiltered = statBufferPtr[1] / statBufferPtr[0]; // Compare newVal with average
						if(oldFiltered - newVal >= bigChange || newVal - oldFiltered >= bigChange){

							countChange++; // Way to keep track of change occuring or not

							if(followBigChange){ // Follow big changes

								float* averagingBufferPtr;
								for(int i = 0; i < numAveragingSlots; i++){ // Update all averaging slots
									averagingBufferPtr = averagingDepthBuffer + i*_HEIGHT*_WIDTH + y*_WIDTH + x;
									*averagingBufferPtr = newVal;
								}
								statBufferPtr[0] = numAveragingSlots; // Update statistics
								statBufferPtr[1] = newVal*numAveragingSlots;
								statBufferPtr[2] = newVal*newVal*numAveragingSlots;
							}
						}
					}

					// Update the pixel's statistics: 
					++statBufferPtr[0]; // Number of valid samples
					statBufferPtr[1] += newVal; // Sum of valid samples
					statBufferPtr[2] += newVal * newVal; // Sum of squares of valid samples

					// Check if the previous value in the averaging buffer was not initiated 
					if(oldVal != initialValue){

						--statBufferPtr[0]; // Number of valid samples
						statBufferPtr[1] -= oldVal; // Sum of valid samples
						statBufferPtr[2] -= oldVal * oldVal; // Sum of squares of valid samples
					}

				} // end > maxOffset


				// Check if the pixel is "stable": 
				if(statBufferPtr[0] >= minNumSamples &&
					statBufferPtr[2] * statBufferPtr[0] <= maxVariance*statBufferPtr[0] * statBufferPtr[0] + statBufferPtr[1] * statBufferPtr[1]) {

					// Check if the new running mean is outside the previous value's envelope: 
					float newFiltered = statBufferPtr[1] / statBufferPtr[0];
					if(abs(newFiltered - *validBufferPtr) >= hysteresis){
						// Set the output pixel value to the depth-corrected running mean: 
						*filteredFramePtr = *validBufferPtr = newFiltered;
					}else{
						// Leave the pixel at its previous value: 
						*filteredFramePtr = *validBufferPtr;
					}
				}

				*filteredFramePtr = *validBufferPtr; // save this as 'valid' frame
			}

			inputFramePtr += _WIDTH - ROIMaxX;
			averagingBufferPtr += _WIDTH - ROIMaxX;
			statBufferPtr += (_WIDTH - ROIMaxX) * 3;
			validBufferPtr += _WIDTH - ROIMaxX;
			filteredFramePtr += _WIDTH - ROIMaxX;
		}

		//ofLogVerbose("KinectGrabber") << "dfilterDepth:" << "count change:" << countChange;
		if(countChange > 410){
			//ofLogVerbose("KinectGrabber") << "dfilterDepth: WE HAVE INTERACTION!" << countChange;
			bInteraction = true;
		}else{
			bInteraction = false;
		}

		// Go to the next averaging slot: 
		if(++averagingSlotIndex == numAveragingSlots){
			averagingSlotIndex = 0;
		}

		if(!firstImageReady){
			currentInitFrame++;
			if(currentInitFrame > minInitFrame){
				firstImageReady = true;
			}
		}
	}

	if(buffersInitiated){
		// Apply outlier inpainting if requested: 
		if(doInPaint){
			applySimpleOutlierInpainting();
		}

		// Apply a spatial filter if requested: 
		if(spatialFilter){
			applySpaceFilter();
		}
	}
}

void KinectV2Grabber::filterXY(){

	if(buffersInitiated && numAveragingSlots < 2){
		// @sabrina: I copied these and put them in scopes so that the variable names can stay the same
		{

			// Filter x 
#ifdef USE_COLOR
			const Filtered* inputFramePtr = static_cast<const Filtered*>(colorMapperX.getData());
#else
			const Filtered* inputFramePtr = static_cast<const Filtered*>(depthMapperX.getData());
#endif
			float* filteredFramePtr = filteredXframe.getData();
			inputFramePtr += ROIMinY*_WIDTH; 
			filteredFramePtr += ROIMinY*_WIDTH;

			for(unsigned int y = ROIMinY; y < ROIMaxY; ++y){

				inputFramePtr += ROIMinX;
				filteredFramePtr += ROIMinX;

				for(unsigned int x = ROIMinX; x < ROIMaxX; ++x, ++inputFramePtr, ++filteredFramePtr) {
					float newVal = static_cast<float>(*inputFramePtr * 1000);
					*filteredFramePtr = newVal;
				}
				inputFramePtr += _WIDTH - ROIMaxX;
				filteredFramePtr += _WIDTH - ROIMaxX;
			}
		}

		{
			// Filter y
#ifdef USE_COLOR
			const Filtered* inputFramePtr = static_cast<const Filtered*>(colorMapperX.getData());
#else
			const Filtered* inputFramePtr = static_cast<const Filtered*>(depthMapperY.getData());
#endif
			float* filteredFramePtr = filteredYframe.getData();
			inputFramePtr += ROIMinY*_WIDTH; 
			filteredFramePtr += ROIMinY*_WIDTH;

			for(unsigned int y = ROIMinY; y < ROIMaxY; ++y){

				inputFramePtr += ROIMinX;
				filteredFramePtr += ROIMinX;

				for(unsigned int x = ROIMinX; x < ROIMaxX; ++x, ++inputFramePtr, ++filteredFramePtr){
					float newVal = static_cast<float>(*inputFramePtr * 1000);
					*filteredFramePtr = newVal;
				}
				inputFramePtr += _WIDTH - ROIMaxX;
				filteredFramePtr += _WIDTH - ROIMaxX;
			}
		}


	}else if(buffersInitiated){

		// @sabrina: I copied these and put them in scopes so that the variable names can stay the same
		{

			// Filter x
#ifdef USE_COLOR
			const Filtered* inputFramePtr = static_cast<const Filtered*>(colorMapperX.getData());
#else
			const Filtered* inputFramePtr = static_cast<const Filtered*>(depthMapperX.getData());
#endif
			float* averagingBufferPtr = averagingXBuffer + averagingSlotIndex*_HEIGHT*_WIDTH;
			float* statBufferPtr = statXBuffer;
			float* validBufferPtr = validXBuffer;
			float* filteredFramePtr = filteredXframe.getData();

			// Get depth to check if x coordinate is out of range of the depth view
#ifdef USE_COLOR
			const Filtered* inputDephtFramePtr = static_cast<const Filtered*>(colorMapperDepth.getData());
#else
			const Filtered* inputDephtFramePtr = static_cast<const Filtered*>(depthMapperDepth.getData());
#endif
			inputDephtFramePtr += ROIMinY*_WIDTH;

			inputFramePtr += ROIMinY*_WIDTH;
			averagingBufferPtr += ROIMinY*_WIDTH;
			statBufferPtr += ROIMinY*_WIDTH * 3;
			validBufferPtr += ROIMinY*_WIDTH;
			filteredFramePtr += ROIMinY*_WIDTH;

			for(unsigned int y = ROIMinY; y < ROIMaxY; ++y){

				inputDephtFramePtr += ROIMinX;

				inputFramePtr += ROIMinX;
				averagingBufferPtr += ROIMinX;
				statBufferPtr += ROIMinX * 3;
				validBufferPtr += ROIMinX;
				filteredFramePtr += ROIMinX;

				for(unsigned int x = ROIMinX; x < ROIMaxX; ++x, ++inputDephtFramePtr, ++inputFramePtr, ++averagingBufferPtr, statBufferPtr += 3, ++validBufferPtr, ++filteredFramePtr) {

					float newVal = static_cast<float>(*inputFramePtr * 1000);
					float newDepthVal = static_cast<float>(*inputDephtFramePtr * 1000);
					float oldVal = *averagingBufferPtr;

					if(newDepthVal > maxOffset){ // We are under the ceiling plane

						*averagingBufferPtr = newVal; // Store the value

						if(followBigChange && statBufferPtr[0] > 0){ // Follow big changes
							float oldFiltered = statBufferPtr[1] / statBufferPtr[0]; // Compare newVal with average
							if (oldFiltered - newVal >= bigChange || newVal - oldFiltered >= bigChange) {

								float* averagingBufferPtr;
								for(int i = 0; i < numAveragingSlots; i++){ // Update all averaging slots
									averagingBufferPtr = averagingXBuffer + i*_HEIGHT*_WIDTH + y*_WIDTH + x;
									*averagingBufferPtr = newVal;
								}
								statBufferPtr[0] = numAveragingSlots; // Update statistics
								statBufferPtr[1] = newVal*numAveragingSlots;
								statBufferPtr[2] = newVal*newVal*numAveragingSlots;
							}
						}

						// Update the pixel's statistics: 
						++statBufferPtr[0]; // Number of valid samples
						statBufferPtr[1] += newVal; // Sum of valid samples
						statBufferPtr[2] += newVal * newVal; // Sum of squares of valid samples

						// Check if the previous value in the averaging buffer was not initiated 
						if (oldVal != initialValueXY) {

							--statBufferPtr[0]; // Number of valid samples
							statBufferPtr[1] -= oldVal; // Sum of valid samples
							statBufferPtr[2] -= oldVal * oldVal; // Sum of squares of valid samples
						}

					} // end > maxOffset

					// Check if the pixel is "stable": 
					if (statBufferPtr[0] >= minNumSamples &&
						statBufferPtr[2] * statBufferPtr[0] <= maxVarianceXY*statBufferPtr[0] * statBufferPtr[0] + statBufferPtr[1] * statBufferPtr[1]) {

						// Check if the new running mean is outside the previous value's envelope: 
						float newFiltered = statBufferPtr[1] / statBufferPtr[0];
						if (abs(newFiltered - *validBufferPtr) >= hysteresisXY) {
							// Set the output pixel value to the depth-corrected running mean: 
							*filteredFramePtr = *validBufferPtr = newFiltered;
						}
						else {
							// Leave the pixel at its previous value: 
							*filteredFramePtr = *validBufferPtr;
						}
					}

					// Leave the pixel at its previous value: 
					*filteredFramePtr = *validBufferPtr;
				}

				inputDephtFramePtr += _WIDTH - ROIMaxX;
				inputFramePtr += _WIDTH - ROIMaxX;
				averagingBufferPtr += _WIDTH - ROIMaxX;
				statBufferPtr += (_WIDTH - ROIMaxX) * 3;
				validBufferPtr += _WIDTH - ROIMaxX;
				filteredFramePtr += _WIDTH - ROIMaxX;
			}
		}

		{
			// Filter y
#ifdef USE_COLOR
			const Filtered* inputFramePtr = static_cast<const Filtered*>(colorMapperY.getData());
#else
			const Filtered* inputFramePtr = static_cast<const Filtered*>(depthMapperY.getData());
#endif
			float* averagingBufferPtr = averagingYBuffer + averagingSlotIndex*_HEIGHT*_WIDTH;
			float* statBufferPtr = statYBuffer;
			float* validBufferPtr = validYBuffer;
			float* filteredFramePtr = filteredYframe.getData();

			// Get depth to check if x coordinate is out of range of the depth view
#ifdef USE_COLOR
			const Filtered* inputDephtFramePtr = static_cast<const Filtered*>(colorMapperDepth.getData());
#else
			const Filtered* inputDephtFramePtr = static_cast<const Filtered*>(depthMapperDepth.getData());
#endif
			inputDephtFramePtr += ROIMinY*_WIDTH;

			inputFramePtr += ROIMinY*_WIDTH;
			averagingBufferPtr += ROIMinY*_WIDTH;
			statBufferPtr += ROIMinY*_WIDTH * 3;
			validBufferPtr += ROIMinY*_WIDTH;
			filteredFramePtr += ROIMinY*_WIDTH;

			for(unsigned int y = ROIMinY; y < ROIMaxY; ++y){

				inputDephtFramePtr += ROIMinX;

				inputFramePtr += ROIMinX;
				averagingBufferPtr += ROIMinX;
				statBufferPtr += ROIMinX * 3;
				validBufferPtr += ROIMinX;
				filteredFramePtr += ROIMinX;

				for(unsigned int x = ROIMinX; x < ROIMaxX; ++x, ++inputDephtFramePtr, ++inputFramePtr, ++averagingBufferPtr, statBufferPtr += 3, ++validBufferPtr, ++filteredFramePtr){

					float newVal = static_cast<float>(*inputFramePtr * 1000);
					float newDepthVal = static_cast<float>(*inputDephtFramePtr * 1000);
					float oldVal = *averagingBufferPtr;

					if(newDepthVal > maxOffset){ // We are under the ceiling plane
						*averagingBufferPtr = newVal; // Store the value

						if(followBigChange && statBufferPtr[0] > 0){ // Follow big changes
							float oldFiltered = statBufferPtr[1] / statBufferPtr[0]; // Compare newVal with average
							if(oldFiltered - newVal >= bigChange || newVal - oldFiltered >= bigChange){

								float* averagingBufferPtr;
								for(int i = 0; i < numAveragingSlots; i++){ // Update all averaging slots
									averagingBufferPtr = averagingYBuffer + i*_HEIGHT*_WIDTH + y*_WIDTH + x;
									*averagingBufferPtr = newVal;
								}
								statBufferPtr[0] = numAveragingSlots; // Update statistics
								statBufferPtr[1] = newVal*numAveragingSlots;
								statBufferPtr[2] = newVal*newVal*numAveragingSlots;
							}
						}

						// Update the pixel's statistics: 
						++statBufferPtr[0]; // Number of valid samples
						statBufferPtr[1] += newVal; // Sum of valid samples
						statBufferPtr[2] += newVal * newVal; // Sum of squares of valid samples

						// Check if the previous value in the averaging buffer was not initiated 
						if(oldVal != initialValueXY){

							--statBufferPtr[0]; // Number of valid samples
							statBufferPtr[1] -= oldVal; // Sum of valid samples
							statBufferPtr[2] -= oldVal * oldVal; // Sum of squares of valid samples
						}
					} // end > maxOffset

					// Check if the pixel is "stable": 
					if (statBufferPtr[0] >= minNumSamples &&
						statBufferPtr[2] * statBufferPtr[0] <= maxVarianceXY*statBufferPtr[0] * statBufferPtr[0] + statBufferPtr[1] * statBufferPtr[1]) {

						// Check if the new running mean is outside the previous value's envelope: 
						float newFiltered = statBufferPtr[1] / statBufferPtr[0];
						if (abs(newFiltered - *validBufferPtr) >= hysteresisXY) {
							// Set the output pixel value to the depth-corrected running mean: 
							*filteredFramePtr = *validBufferPtr = newFiltered;
						}
						else {
							// Leave the pixel at its previous value: 
							*filteredFramePtr = *validBufferPtr;
						}
					}

					*filteredFramePtr = *validBufferPtr;
				}

				inputDephtFramePtr += _WIDTH - ROIMaxX;
				inputFramePtr += _WIDTH - ROIMaxX;
				averagingBufferPtr += _WIDTH - ROIMaxX;
				statBufferPtr += (_WIDTH - ROIMaxX) * 3;
				validBufferPtr += _WIDTH - ROIMaxX;
				filteredFramePtr += _WIDTH - ROIMaxX;
			}
		}
	}
}


void KinectV2Grabber::setFullFrameFiltering(bool ff, ofRectangle ROI){
	doFullFrameFiltering = ff;
	if(ff){
		setKinectROI(ofRectangle(0, 0, _WIDTH, _HEIGHT));

	}else{
		setKinectROI(ROI);

		// @sabrina: I copied these and put them in scopes so that the variable names can stay the same
		{
			float *Mapdata = filteredDepthframe.getData();

			// clear all pixels outside ROI
			for(unsigned int y = 0; y < _HEIGHT; y++){
				for(unsigned int x = 0; x < _WIDTH; x++){
					if(y < ROIMinY || y >= ROIMaxY || x < ROIMinX || x >= ROIMaxX){
						int idx = y * _WIDTH + x;
						Mapdata[idx] = 0;
					}
				}
			}
		}
		{
			float *Mapdata = filteredXframe.getData();

			// clear all pixels outside ROI
			for(unsigned int y = 0; y < _HEIGHT; y++){
				for(unsigned int x = 0; x < _WIDTH; x++){
					if(y < ROIMinY || y >= ROIMaxY || x < ROIMinX || x >= ROIMaxX){
						int idx = y * _WIDTH + x;
						Mapdata[idx] = 0;
					}
				}
			}
		}

		{
			float *Mapdata = filteredYframe.getData();

			// clear all pixels outside ROI
			for(unsigned int y = 0; y < _HEIGHT; y++){
				for(unsigned int x = 0; x < _WIDTH; x++){
					if(y < ROIMinY || y >= ROIMaxY || x < ROIMinX || x >= ROIMaxX){
						int idx = y * _WIDTH + x;
						Mapdata[idx] = 0;
					}
				}
			}
		}
	}
}

void KinectV2Grabber::applySpaceFilter(){

	// @sabrina: I copied these and put them in scopes so that the variable names can stay the same
	{
		for(int filterPass = 0; filterPass < 2; ++filterPass){
			// Pointer to first pixel of ROI
			float *ptrOffset = filteredDepthframe.getData() + ROIMinY * _WIDTH + ROIMinX;

			// Low-pass filter the values in the ROI
			// First a horizontal pass
			for(unsigned int x = 0; x < _WIDTH; x++){

				// Pointer to current pixel
				float* colPtr = ptrOffset + x;
				float lastVal = *colPtr;

				// Top border pixels 
				*colPtr = (colPtr[0] * 2.0f + colPtr[_WIDTH]) / 3.0f;
				colPtr += _WIDTH;

				// Filter the interior pixels in the column
				for(unsigned int y = ROIMinY + 1; y < ROIMaxY - 1; ++y, colPtr += _WIDTH){

					float nextLastVal = *colPtr;
					*colPtr = (lastVal + colPtr[0] * 2.0f + colPtr[_WIDTH])*0.25f;
					lastVal = nextLastVal; // To avoid using already updated pixels
				}

				// Filter the last pixel in the column: 
				*colPtr = (lastVal + colPtr[0] * 2.0f) / 3.0f;
			}

			// then a vertical pass
			for(unsigned int y = 0; y < _HEIGHT; y++){

				// Pointer to current pixel
				float* rowPtr = ptrOffset + y * _WIDTH;

				// Filter the first pixel in the row: 
				float lastVal = *rowPtr;
				*rowPtr = (rowPtr[0] * 2.0f + rowPtr[1]) / 3.0f;
				rowPtr++;

				// Filter the interior pixels in the row: 
				for(unsigned int x = ROIMinX + 1; x < ROIMaxX - 1; ++x, ++rowPtr){

					float nextLastVal = *rowPtr;
					*rowPtr = (lastVal + rowPtr[0] * 2.0f + rowPtr[1])*0.25f;
					lastVal = nextLastVal;
				}

				// Filter the last pixel in the row: 
				*rowPtr = (lastVal + rowPtr[0] * 2.0f) / 3.0f;
			}
		}
	} // end of depth scope

	{ // xscope
		for(int filterPass = 0; filterPass < 2; ++filterPass){
			// Pointer to first pixel of ROI
			float *ptrOffset = filteredXframe.getData() + ROIMinY * _WIDTH + ROIMinX;

			// Low-pass filter the values in the ROI
			// First a horizontal pass
			for(unsigned int x = 0; x < _WIDTH; x++){

				// Pointer to current pixel
				float* colPtr = ptrOffset + x;
				float lastVal = *colPtr;

				// Top border pixels 
				*colPtr = (colPtr[0] * 2.0f + colPtr[_WIDTH]) / 3.0f;
				colPtr += _WIDTH;

				// Filter the interior pixels in the column
				for(unsigned int y = ROIMinY + 1; y < ROIMaxY - 1; ++y, colPtr += _WIDTH){

					float nextLastVal = *colPtr;
					*colPtr = (lastVal + colPtr[0] * 2.0f + colPtr[_WIDTH])*0.25f;
					lastVal = nextLastVal; // To avoid using already updated pixels
				}

				// Filter the last pixel in the column: 
				*colPtr = (lastVal + colPtr[0] * 2.0f) / 3.0f;
			}

			// then a vertical pass
			for(unsigned int y = 0; y < _HEIGHT; y++){

				// Pointer to current pixel
				float* rowPtr = ptrOffset + y * _WIDTH;

				// Filter the first pixel in the row: 
				float lastVal = *rowPtr;
				*rowPtr = (rowPtr[0] * 2.0f + rowPtr[1]) / 3.0f;
				rowPtr++;

				// Filter the interior pixels in the row: 
				for(unsigned int x = ROIMinX + 1; x < ROIMaxX - 1; ++x, ++rowPtr){

					float nextLastVal = *rowPtr;
					*rowPtr = (lastVal + rowPtr[0] * 2.0f + rowPtr[1])*0.25f;
					lastVal = nextLastVal;
				}

				// Filter the last pixel in the row: 
				*rowPtr = (lastVal + rowPtr[0] * 2.0f) / 3.0f;
			}
		}
	}

	{ // y scope
		for(int filterPass = 0; filterPass < 2; ++filterPass){
			// Pointer to first pixel of ROI
			float *ptrOffset = filteredYframe.getData() + ROIMinY * _WIDTH + ROIMinX;

			// Low-pass filter the values in the ROI
			// First a horizontal pass
			for(unsigned int x = 0; x < _WIDTH; x++){

				// Pointer to current pixel
				float* colPtr = ptrOffset + x;
				float lastVal = *colPtr;

				// Top border pixels 
				*colPtr = (colPtr[0] * 2.0f + colPtr[_WIDTH]) / 3.0f;
				colPtr += _WIDTH;

				// Filter the interior pixels in the column
				for(unsigned int y = ROIMinY + 1; y < ROIMaxY - 1; ++y, colPtr += _WIDTH){

					float nextLastVal = *colPtr;
					*colPtr = (lastVal + colPtr[0] * 2.0f + colPtr[_WIDTH])*0.25f;
					lastVal = nextLastVal; // To avoid using already updated pixels
				}

				// Filter the last pixel in the column: 
				*colPtr = (lastVal + colPtr[0] * 2.0f) / 3.0f;
			}

			// then a vertical pass
			for(unsigned int y = 0; y < _HEIGHT; y++){

				// Pointer to current pixel
				float* rowPtr = ptrOffset + y * _WIDTH;

				// Filter the first pixel in the row: 
				float lastVal = *rowPtr;
				*rowPtr = (rowPtr[0] * 2.0f + rowPtr[1]) / 3.0f;
				rowPtr++;

				// Filter the interior pixels in the row: 
				for(unsigned int x = ROIMinX + 1; x < ROIMaxX - 1; ++x, ++rowPtr){

					float nextLastVal = *rowPtr;
					*rowPtr = (lastVal + rowPtr[0] * 2.0f + rowPtr[1])*0.25f;
					lastVal = nextLastVal;
				}

				// Filter the last pixel in the row: 
				*rowPtr = (lastVal + rowPtr[0] * 2.0f) / 3.0f;
			}
		}
	}

}

float KinectV2Grabber::findInpaintValue(float *data, int x, int y) {

	int sideLength = 4; // @sabrina: this is size of box of values to look around in?

	// We do not search outside ROI
	int tminx = max(ROIMinX, x - sideLength);
	int tmaxx = min(ROIMaxX, x + sideLength);
	int tminy = max(ROIMinY, y - sideLength);
	int tmaxy = min(ROIMaxY, y + sideLength);

	int samples = 0;
	double sumval = 0;
	for(int y = tminy; y < tmaxy; y++){
		for(int x = tminx; x < tmaxy; x++){
			int idx = y * _WIDTH + x;
			float val = abs(data[idx]);
			if(val != 0 && val != initialValue){
				samples++;
				sumval += val;
			}
		}
	}

	// No valid samples found in neighboorhood
	if (samples == 0)
		return 0;

	return sumval / samples;
}

float KinectV2Grabber::findInpaintValueXY(float *data, int x, int y) {

	int sideLength = 4; // @sabrina: this is size of box of values to look around in?

	// We do not search outside ROI
	int tminx = max(ROIMinX, x - sideLength);
	int tmaxx = min(ROIMaxX, x + sideLength);
	int tminy = max(ROIMinY, y - sideLength);
	int tmaxy = min(ROIMaxY, y + sideLength);

	int idx = y * _WIDTH + x;
	float thisval = data[idx];

	int samples = 0;
	double sumval = 0;
	for(int y = tminy; y < tmaxy; y++){
		for(int x = tminx; x < tmaxy; x++){
			int idx = y * _WIDTH + x;
			float val = abs(data[idx]);
			if(val != initialValueXY){
				samples++;
				sumval += val;
			}
		}
	}

	// No valid samples found in neighboorhood
	if(samples == 0){
		return initialValueXY; // because for XY 0 can be a valid value
	}

	return sumval / samples;
}

void KinectV2Grabber::applySimpleOutlierInpainting(){

	float *data = filteredDepthframe.getData();

	// Estimate overall average inside ROI
	int samples = 0;
	ROIAverageValue = 0;
	for(unsigned int y = ROIMinY; y < ROIMaxY; y++){
		for(unsigned int x = ROIMinX; x < ROIMaxX; x++){
			int idx = y * _WIDTH + x;
			float val = data[idx];

			if(val != 0 && val != initialValue){ 
				samples++;
				ROIAverageValue += val;
			}
		}
	}

	// No valid samples found in ROI - strange situation
	if(samples == 0){
		ROIAverageValue = initialValue;
	}

	ROIAverageValue /= samples;

	setToLocalAvg = 0;
	setToGlobalAvg = 0;

	// Filter ROI
	// @sabrina: why -2 & + 2?
	for(unsigned int y = max(0, ROIMinY - 2); y < min((int)_HEIGHT, ROIMaxY + 2); y++){
		for(unsigned int x = max(0, ROIMinX - 2); x < min((int)_WIDTH, ROIMaxX + 2); x++){

			int idx = y * _WIDTH + x;
			float val = data[idx];

			if(val == 0 || val == initialValue){ // == 0
				float newval = findInpaintValue(data, x, y);
				if(newval == 0){
					newval = ROIAverageValue;
					setToGlobalAvg++;
				}else{
					setToLocalAvg++;
				}
				data[idx] = newval;
			}
		}
	}
}

bool KinectV2Grabber::isInsideROI(int x, int y) {

	if(x<ROIMinX || x>ROIMaxX || y<ROIMinY || y>ROIMaxY){
		return false;
	}
	return true;
}

bool KinectV2Grabber::isInsideDepthVOF(int x, int y){
	// check if color coordinate is inside the depth image VOF
	ofVec2f coord = getDepthCoordAt(x, y);
	if((coord.x != -INFINITY && coord.y != -INFINITY) &&
		(coord.x >= 0 && coord.x <= DEPTH_WIDTH) &&
		(coord.y >= 0 && coord.y <= DEPTH_HEIGHT)){
		return true;
	}else{
		return false;
	}
}

void KinectV2Grabber::setKinectROI(ofRectangle ROI){

	// Here we set the Region Of Interest (ROI) which corresponds with the sand surface
	if(doFullFrameFiltering){

		ROIMinX = 0;
		ROIMaxX = _WIDTH;
		ROIMinY = 0;
		ROIMaxY = _HEIGHT;

	}else{

		ofVec2f tempMin;
		ofVec2f tempMax;

#ifdef USE_COLOR
		ROIMinX = static_cast<int>(roundf(ROI.getMinX()));
		ROIMaxX = static_cast<int>(roundf(ROI.getMaxX()));
		ROIMinY = static_cast<int>(roundf(ROI.getMinY()));
		ROIMaxY = static_cast<int>(roundf(ROI.getMaxY()));
#endif

		// Check if these depth coordinates exist, since the color VOF is bigger than the depth VOF
		// if they don't exist we can't make the ROI

		if(getDepthCoordLookUp() &&
			isInsideDepthVOF(roundf(ROI.getMinX()), roundf(ROI.getMinY())) && isInsideDepthVOF(roundf(ROI.getMaxX()), roundf(ROI.getMaxY())) ){
			
			ofVec2f ttempMin(getDepthCoordAt(roundf(ROI.getMinX()), roundf(ROI.getMinY())));
			ofVec2f ttempMax(getDepthCoordAt(roundf(ROI.getMaxX()), roundf(ROI.getMaxY())));
			// steps that account for consequences of mirroring 
			tempMin.set(ttempMax.x, ttempMin.y);
			tempMax.set(ttempMin.x, ttempMax.y);
			ofLogVerbose("KinectGrabber") << "setKinectROI: " << tempMin << ", " << tempMax;

		}else{
			tempMin.set(0, 0);
			tempMax.set(_WIDTH, _HEIGHT);
		}

#ifdef USE_COLOR
		// steps that account for consequences of mirroring 
		ofVec2f tempROIX = ofVec2f(COLOR_WIDTH - (ROI.getMaxX() - 2), COLOR_WIDTH - (ROI.getMinX() + 2));
		
		ROIMinX = max(0, static_cast<int>(tempROIX.x));
		ROIMaxX = min(static_cast<int>(tempROIX.y), (int)COLOR_WIDTH);
		ROIMinY = max(0, ROIMinY - 2);
		ROIMaxY = min(ROIMaxY + 2, (int)COLOR_HEIGHT);
		ofLogVerbose("KinectGrabber") << "setKinectROI: ROI Color Coordinates are " << ROIMinX << " " << ROIMinY << " & " << ROIMaxX << " " << ROIMaxY;
#else

		// Because we couldn't draw KinectROI on the reflective edges of the sand table
		// we're using the inside of the sand table as ROI and making it bigger with -val & +val

		ROIMinX = max(0, static_cast<int>(tempMin.x) - 50);
		ROIMaxX = min(static_cast<int>(tempMax.x) + 50, (int)DEPTH_WIDTH);
		
		ROIMinY = max(0, static_cast<int>(tempMin.y) - 50);
		ROIMaxY = min(static_cast<int>(tempMax.y) + 50, (int)DEPTH_HEIGHT);
		ofLogVerbose("KinectGrabber") << "setKinectROI: ROI Depth Coordinates are " << ROIMinX << " " << ROIMinY << " & " << ROIMaxX << " " << ROIMaxY;

		// We are assigning a new variable dkinectROI because ?
		// steps that account for consequences of mirroring 
		int ROIwidth = (ROIMaxX-2) - (ROIMinX+2);
		int ROIheigt = (ROIMaxY-2) - (ROIMinY+2);
		dkinectROI = ofRectangle(DEPTH_WIDTH-(ROIMaxX-2), (ROIMinY+2), ROIwidth, ROIheigt);
		//ofLogVerbose("KinectGrabber:") << "setKinectROI: new ROI = " << dkinectROI;
#endif

	}

	resetBuffers();

}

void KinectV2Grabber::setAveragingSlotsNumber(int snumAveragingSlots){
	if(buffersInitiated){
		buffersInitiated = false;

		delete[] averagingDepthBuffer;
		delete[] statDepthBuffer;
		delete[] validDepthBuffer;

		delete[] averagingXBuffer;
		delete[] statXBuffer;
		delete[] validXBuffer;

		delete[] averagingYBuffer;
		delete[] statYBuffer;
		delete[] validYBuffer;

	}
	numAveragingSlots = snumAveragingSlots;
	minNumSamples = (numAveragingSlots + 1) / 2;
	initiateBuffers();
}

void KinectV2Grabber::setFollowBigChange(bool newfollowBigChange){
	if(buffersInitiated){
		buffersInitiated = false;

		delete[] averagingDepthBuffer;
		delete[] statDepthBuffer;
		delete[] validDepthBuffer;

		delete[] averagingXBuffer;
		delete[] statXBuffer;
		delete[] validXBuffer;

		delete[] averagingYBuffer;
		delete[] statYBuffer;
		delete[] validYBuffer;

	}
	followBigChange = newfollowBigChange;
	initiateBuffers();
}

#ifdef USE_COLOR
// @sabrina: currently not using these 
ofVec3f KinectV2Grabber::getStatDepthBuffer(int x, int y){
	float* statBufferPtr = statDepthBuffer + 3 * (x + y*COLOR_WIDTH);
	return ofVec3f(statBufferPtr[0], statBufferPtr[1], statBufferPtr[2]);
}

float KinectV2Grabber::getAveragingDepthBuffer(int x, int y, int slotNum){
	float* averagingBufferPtr = averagingDepthBuffer + slotNum*COLOR_HEIGHT*COLOR_WIDTH + (x + y*COLOR_WIDTH);
	return *averagingBufferPtr;
}

float KinectV2Grabber::getValidDepthBuffer(int x, int y){
	float* validBufferPtr = validDepthBuffer + (x + y*COLOR_WIDTH);
	return *validBufferPtr;
}
#endif

bool KinectV2Grabber::getDepthCoordLookUp(){
	if(kinect.getDepthSource()->isFrameNew()){
		coordinateMapper->MapColorFrameToDepthSpace(DEPTH_SIZE, (UINT16*)kinect.getDepthSource()->getPixels(), COLOR_SIZE, (DepthSpacePoint*)depthCoordLookUp.data());
		return true;
	}else{
		return false;
	}
}

#ifndef USE_COLOR
bool KinectV2Grabber::getDepth2WorldCoordinates(){
	if(kinect.getDepthSource()->isFrameNew()){
		kinect.getDepthSource()->getWorldInDepthFrame(depthMapper);
		depthMapperX.setFromPixels(depthMapper.getChannel(0), DEPTH_WIDTH, DEPTH_HEIGHT, 1);
		depthMapperY.setFromPixels(depthMapper.getChannel(1), DEPTH_WIDTH, DEPTH_HEIGHT, 1);
		depthMapperDepth.setFromPixels(depthMapper.getChannel(2), DEPTH_WIDTH, DEPTH_HEIGHT, 1);
		return true;
	}else{
		return false;
	}
}
#endif

bool KinectV2Grabber::getColor2WorldCoordinates(){
	if (kinect.getDepthSource()->isFrameNew()){
		kinect.getDepthSource()->getWorldInColorFrame(colorMapper);
#ifdef USE_COLOR
		colorMapperX.setFromPixels(colorMapper.getChannel(0), COLOR_WIDTH, COLOR_HEIGHT, 1);
		colorMapperY.setFromPixels(colorMapper.getChannel(1), COLOR_WIDTH, COLOR_HEIGHT, 1);
		colorMapperDepth.setFromPixels(colorMapper.getChannel(2), COLOR_WIDTH, COLOR_HEIGHT, 1);
#endif
		return true;
	}else{
		return false;
	}
}

