// OpticalFlow.cpp
// David Kerr - Master of Science

#include "OpticalFlow.hpp"
#include "Math.h"

OpticalFlow::OpticalFlow ( const char* filePath, DisplayType display = PREDICTIVE, bool saveFrames = false ) {
    
    mFlowDisplay = display;
    mSaveFrames = saveFrames;
    
    Mat frame1, bwFrame1;
	Mat frame2, bwFrame2;
	vector<Point2f> corners1, corners2;
	vector<uchar> status;
	vector<float> error;
	
	namedWindow ( filePath, CV_WINDOW_AUTOSIZE );
	cvMoveWindow ( filePath, WIN_POS, WIN_POS );
	
	mFrames = getFrames ( filePath );
	
	assert ( mFrames.size() > 0 );
        
    for ( int i = 0; i < mFrames.size()-1; i++ ) {
        cout << i << SLASH << mFrames.size()-2 << endl;
        
        frame1 = mFrames[i].clone();
        cvtColor( frame1, bwFrame1, CV_BGR2GRAY );
        
        goodFeaturesToTrack ( bwFrame1, corners1, NUM_OF_FEAT, FEAT_QUA_LEV, FEAT_MIN_DIS, Mat() );
        
        cvtColor( mFrames[i+1], bwFrame2, CV_BGR2GRAY );
        
        calcOpticalFlowPyrLK ( bwFrame1, bwFrame2, corners1, corners2, status, error);
        
        drawOpticalFlow ( frame1, corners1, corners2, status, i );
   
        imshow ( filePath, frame1 );
        
        if (mSaveFrames)
            saveFrame ( i, frame1 );
        
        i = checkKey ( i );
        
		}
}

OpticalFlow::~OpticalFlow () {
    mFrames.clear();
    mHistoricalFlowVectorList.clear();
}


vector<Mat>  OpticalFlow::getFrames ( const char* filePath ) {
	
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
	return frames;
	
}

int OpticalFlow::drawHistoricalFlow( Mat frame, int frameNumber, CvScalar color ){
    
    int frameDifference;
    
    for (vector<HistoricalFlowVector*>::iterator it = mHistoricalFlowVectorList.begin(); it != mHistoricalFlowVectorList.end(); it++ ){
        frameDifference = frameNumber - (*it)->mFrameNumber;
        
        if ( frameDifference < HISTORICAL_FRM_THR ){
            line( frame, (*it)->mPoint1, (*it)->mPoint2, color, LINE_THICKNESS, CV_AA, LINE_SHIFT );
        }
    }
    
    return EXIT_SUCCESS;
}

int OpticalFlow::drawPredictiveVector( Mat frame, int index, vector<Point2f> corners1, vector<Point2f> corners2, CvScalar color ) {
    
    double angle;
	double hypotenuse;
    CvPoint p,q;
    
    p.x = (int) corners1[index].x;
    p.y = (int) corners1[index].y;
    q.x = (int) corners2[index].x;
    q.y = (int) corners2[index].y;
    
    angle = getAngle ( p, q );
    hypotenuse = sqrt( SQUARE(p.y - q.y) + SQUARE(p.x - q.x) );
    
    q.x = (int) ( p.x -  SCALE_FACTOR * hypotenuse * cos( angle ) );
    q.y = (int) ( p.y -  SCALE_FACTOR * hypotenuse * sin( angle ) );
    
    line( frame, p, q, color, LINE_THICKNESS, CV_AA, LINE_SHIFT );
    
    p.x = (int) (q.x + SCALE_FACTOR * cos(angle + PI_OVER_FOUR));
    p.y = (int) (q.y + SCALE_FACTOR * sin(angle + PI_OVER_FOUR));
    line( frame, p, q, color, LINE_THICKNESS, CV_AA, LINE_SHIFT );
    
    p.x = (int) (q.x + SCALE_FACTOR * cos(angle - PI_OVER_FOUR));
    p.y = (int) (q.y + SCALE_FACTOR * sin(angle - PI_OVER_FOUR));
    line( frame, p, q, color, LINE_THICKNESS, CV_AA, LINE_SHIFT );
    
    return EXIT_SUCCESS;
}

int OpticalFlow::printHistogramBin( int numVectorsInBin ){
    
    if ( numVectorsInBin == 0 ){
        cout << NO_VALUE;
    }
    else {
        cout << numVectorsInBin << SPACE;
    }
    
    return EXIT_SUCCESS;
}

int OpticalFlow::drawHistogram ( Mat frame, vector<int> hist[ HIST_SCL_BINS ][ HIST_ANG_BINS ], vector<Point2f> corners1, vector<Point2f> corners2, int frameNumber ) {

    
    if (mFlowDisplay == HISTORICAL) {
        drawHistoricalFlow( frame, frameNumber, RED );
    }
    
	
	for (int i = 0; i < HIST_SCL_BINS; i++ ) {
		for ( int j = 0; j < HIST_ANG_BINS; j++ ) {
			
			printHistogramBin( hist [ i ] [ j ].size() );
			
			if ( hist [ i ][ j ].size() > HIST_THRESHOLD ){
				for (int k = 0; k < hist [ i ][ j ].size(); k++ ) {
					
					
                    int index = hist [ i ][ j ][ k ];
                    
                    if (mFlowDisplay == PREDICTIVE)
                        drawPredictiveVector( frame, index, corners1, corners2, GREEN );
                    else
                        mHistoricalFlowVectorList.push_back( new HistoricalFlowVector(corners1[index], corners2[index], frameNumber) );
				}
            }
		}
		cout << endl;
	}
	
	return EXIT_SUCCESS;
}


int OpticalFlow :: drawOpticalFlow ( Mat frame, vector<Point2f> corners1, vector<Point2f> corners2,
															 vector<uchar> status, int frameNumber ) {

	vector<int> hist[ HIST_SCL_BINS ][ HIST_ANG_BINS ];
	double angle;
	double length;
		
	for (int i = 0; i < corners1.size(); i++ ) {
		
		if ( status[i] ) {
			
			CvPoint p,q;
			p.x = (int) corners1[i].x;
			p.y = (int) corners1[i].y;
			q.x = (int) corners2[i].x;
			q.y = (int) corners2[i].y;
			
			length = getLength ( p, q );
			
			if ( length > MIN_FLW_LEN and length < MAX_FLW_LEN) {
				double dy = q.y - p.y;
                double dx = q.x - p.x;
				angle = atanRadToDeg ( getAngle ( p, q ) );
				
				hist [ (int)length ] [ (int)ANG_2_HISBIN ( angle ) ].push_back( i );
	
			}
		}
		
	}
    
	drawHistogram ( frame, hist, corners1, corners2, frameNumber );
	
	return EXIT_SUCCESS;
}

int OpticalFlow::checkKey ( int i ) {
    int key = cvWaitKey(WAIT_KEY_DURATION);
    switch (key) {
        case ESC_KEY:
            i = mFrames.size();
        case LEFT_ARROW:
            if (i > 0 )
                i -= 2;
            
    }
    
    return i;
}

