// HandTracker.cpp
// David Kerr - Master of Science

#include "HandTracker.hpp"

HandTracker::HandTracker( const char* filePath, DisplayType display = PREDICTIVE, bool saveFrames = false ){
    
    mTrackerDisplay = display;
    mSaveFrames     = saveFrames;
    mFrames         = getFrames ( filePath );
    
    assert(mFrames.size() > 0);
    
    processFrames();
    
    for ( int i = 0; i < mFrames.size(); i++ ) {
        
        cout << i << SLASH << mFrames.size()-1 << endl;
        
        if ( mTrackerDisplay == HISTORICAL )
            drawHistoricalTracker( i, MAX_FLOW_FRAMES, RED );
        else
            drawPredictiveTracker( i, GREEN );
        
  //      Mat binarySkinFrame = getSkin ( i );
  //      imshow ( "skin frame", binarySkinFrame );
        
        imshow ( filePath, mFrames[i] );
        
        if (mSaveFrames)
            saveFrame(i);
        
        i = checkKey(i);
    }
}

HandTracker::~HandTracker(){
    mFrames.clear();
    mKeypoints.clear();
    mIndexVector.clear();
}

int HandTracker::processFrames(){
    
    Mat binarySkinFrame;
    
    assert ( mFrames.size() > 0 );
    
    for ( int i = 0; i < mFrames.size(); i++ ) {

        cout << i << SLASH << mFrames.size()-1 << endl;
        
        binarySkinFrame = getSkin ( i );
        
        mKeypoints.push_back( getKeyPoints(binarySkinFrame) );
        
        setIndexVector( );
        
    }
    
    
    return EXIT_SUCCESS;
}

Mat HandTracker::getSkin ( int frameNumber ){
    
    Vec3b intensity;
    uchar blue, green, red;
    
    uchar maxBG, maxAll, minBG, minAll;;
    bool maxMinThr, absRGThr;
    
    Mat frame = mFrames[frameNumber];
    Size frameSize = frame.size();
    Mat binarySkinFrame(Mat::zeros(frameSize.height, frameSize.width, DataType<uchar>::type));
    
    for ( int x = 0; x < frameSize.width; x++ ){
        for (int y = 0; y < frameSize.height; y++ ){
            
            intensity = frame.at<Vec3b>(y, x);
            blue = intensity.val[BLUE_INDEX];
            green = intensity.val[GREEN_INDEX];
            red = intensity.val[RED_INDEX];
            
            //rgb skin thresholding
            maxBG = max(blue, green);
            maxAll = max(red, maxBG);
            minBG = min(blue, green);
            minAll = min(red, minBG);
            maxMinThr = (maxAll - minAll) > MAX_MIN_THR;
            absRGThr = abs(red-green) > ABS_RED_GREEN_THR;
            
            if (red > RED_THR and green > GREEN_THR and blue > BLUE_THR and
                maxMinThr and absRGThr and red > green and red > blue)
                
                binarySkinFrame.at<uchar>(y,x) = SKIN_PIXEL;
            
        }
    }
    
    return binarySkinFrame;
}

vector<cv::KeyPoint> HandTracker::getKeyPoints( Mat binarySkinFrame ){
    SimpleBlobDetector::Params params;
    Ptr<cv::FeatureDetector> blob_detector;
    vector<cv::KeyPoint> keypoints;
    
    params.minDistBetweenBlobs  = MIN_DIST_BETWEEN_BLOBS;
    params.filterByInertia      = FILTER_BY_INTERTIA;
    params.filterByConvexity    = FILTER_BY_CONVEXITY;
    params.filterByColor        = FILTER_BY_COLOR;
    params.filterByCircularity  = FILTER_BY_CIRCULARITY;
    params.filterByArea         = FILTER_BY_AREA;
    params.minArea              = MIN_AREA;
    params.maxArea              = MAX_AREA;
    
    blob_detector = new cv::SimpleBlobDetector(params);
    blob_detector->create( BLOB_DET_TYPE );
    blob_detector->detect(binarySkinFrame, keypoints);
    
    return keypoints;
}

vector<Mat> HandTracker::getFrames ( const char* filePath ) {
	
	int frameCount;
	Mat tempFrame;
	vector<Mat> frames;
	VideoCapture capture ( filePath );
    
    cout << filePath << endl;
	
	assert ( capture.isOpened() );
        
    frameCount = capture.get ( CV_CAP_PROP_FRAME_COUNT )-1;
    
    for ( int i = 0; i < frameCount; i++ ) {
        cout << i << SLASH << frameCount << endl;
        capture >> tempFrame;
        tempFrame = tempFrame ( Rect(0,0, WIDTH_ROI, HEIGHT_ROI));
        frames.push_back ( tempFrame.clone() );
        
    }
	
    cout << FPS << capture.get( CV_CAP_PROP_FPS ) << endl;
    cout << capture.get( CV_CAP_PROP_FORMAT ) << endl;
    
    
	return frames;
	
}

int HandTracker::drawHistoricalTracker( int frameNumber, int maxFrames, CvScalar color ){
    
    Mat frame = mFrames[frameNumber];
    int startKeypointFrame = max( 0, frameNumber - maxFrames );
    
    for (int i = startKeypointFrame; i < frameNumber; i++){
        
        for (int j=0; j < mKeypoints[i].size(); j++){
            int index = mIndexVector[i][j];
            
            if (index == EMPTY_INDEX)
                circle(frame, mKeypoints[i][j].pt, CIRCLE_RAD, color, CIRCLE_THICKNESS );
            else
                line(frame, mKeypoints[i-1][index].pt, mKeypoints[i][j].pt, color, CIRCLE_THICKNESS );
        
        }
        
    }
    return EXIT_SUCCESS;
}

int HandTracker::drawPredictiveTracker( int frameNumber, CvScalar color ) {
    
    Mat frame;
    
    if ( frameNumber < mFrames.size()-1 ) {
        
        frame = mFrames[frameNumber];
    
        for ( int i = 0; i < mKeypoints[frameNumber+1].size(); i++ ) {
            
            int index = mIndexVector[frameNumber+1][i];
            double distance = getDistance( mKeypoints[frameNumber][index], mKeypoints[frameNumber+1][i] );
            
            if (index == EMPTY_INDEX || distance < MIN_PRED_DISTANCE )
                circle(frame, mKeypoints[frameNumber+1][i].pt, CIRCLE_RAD, color, CIRCLE_THICKNESS );
            else
                drawPredictiveVector( frame, frameNumber, mKeypoints[frameNumber][index].pt, mKeypoints[frameNumber+1][i].pt, GREEN );
        }
    
    }
    
    return EXIT_SUCCESS;
}


int HandTracker::drawPredictiveVector( Mat frame, int index, CvPoint p, CvPoint q, CvScalar color ) {
    
    double angle;
	double hypotenuse;
    
    
    angle = getAngle ( p, q );
    hypotenuse = sqrt( SQUARE(p.y - q.y) + SQUARE(p.x - q.x) );
    
    q.x = (int) ( p.x -  SCALE_FACTOR * hypotenuse * cos( angle ) );
    q.y = (int) ( p.y -  SCALE_FACTOR * hypotenuse * sin( angle ) );
    
    line( frame, p, q, color, LINE_THICKNESS, CV_AA );
    
    p.x = (int) (q.x + SCALE_FACTOR * cos(angle + PI_OVER_FOUR));
    p.y = (int) (q.y + SCALE_FACTOR * sin(angle + PI_OVER_FOUR));
    line( frame, p, q, color, LINE_THICKNESS, CV_AA );
    
    p.x = (int) (q.x + SCALE_FACTOR * cos(angle - PI_OVER_FOUR));
    p.y = (int) (q.y + SCALE_FACTOR * sin(angle - PI_OVER_FOUR));
    line( frame, p, q, color, LINE_THICKNESS, CV_AA );
    
    return EXIT_SUCCESS;
}


int HandTracker::setIndexVector( ) {
 
    int kpIndex = mKeypoints.size() - 1;
    vector <int> tmp;
    mIndexVector.push_back( tmp );
    
    if ( mKeypoints.size() == 1 ){
        
        for (int i=0; i < mKeypoints[START_INDEX].size(); i++){
            
            mIndexVector[START_INDEX].push_back(EMPTY_INDEX);
        }
    }else{
        
        float min;
        
        for (int i=0; i < mKeypoints[kpIndex].size(); i++){
            min = MIN_DISTANCE;
            mIndexVector[kpIndex].push_back(EMPTY_INDEX);
            
            for (int j = 0; j < mKeypoints[kpIndex - 1].size(); j++) {
             
                int distance = getDistance( mKeypoints[kpIndex][i], mKeypoints[kpIndex-1][j] );
                if ( min > distance ){
                    min = distance;
                    mIndexVector[kpIndex][i] = j;
                }
            }
            
        }
    }
    
    return EXIT_SUCCESS;
}

int HandTracker::saveFrame( int frameNumber ){
    stringstream outputFilePath;
    Mat frame = mFrames[frameNumber];
    
    outputFilePath.str( EMPTY_STRING );
    outputFilePath << OUT_PATH << frameNumber  << JPG_EXT;
    imwrite ( outputFilePath.str(), frame );
    
    return EXIT_SUCCESS;
}

int HandTracker::checkKey(int i){
    
    int key = cvWaitKey(WAIT_KEY_DURATION);
    switch (key) {
        case ESC_KEY:
            i = mFrames.size()+1;
        case LEFT_ARROW:
            if (i > 0 )
                i -= 2;
    }
    
    return i;
}