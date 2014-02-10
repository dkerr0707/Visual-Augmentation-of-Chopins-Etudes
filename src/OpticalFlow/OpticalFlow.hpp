// OpticalFlow.hpp
// David Kerr - Master of Science

#include <iostream>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "constants.hpp"
#include "assert.h"

using namespace std;
using namespace cv;

class OpticalFlow {

public:
    enum DisplayType { PREDICTIVE, HISTORICAL };
	OpticalFlow( const char*, DisplayType, bool );
    ~OpticalFlow();
	
private:
    class HistoricalFlowVector {
    public:
        HistoricalFlowVector(Point2f p1, Point2f p2, int frame) : mPoint1(p1), mPoint2(p2), mFrameNumber(frame) {};
        ~HistoricalFlowVector();
    private:
        Point2f mPoint1;
        Point2f mPoint2;
        int     mFrameNumber;
        friend class OpticalFlow;
    };
    
    vector<Mat>                     mFrames;
    vector<HistoricalFlowVector*>   mHistoricalFlowVectorList;
    DisplayType                     mFlowDisplay;
    bool                            mSaveFrames;
    
    // Member Functions
	
	int drawOpticalFlow ( Mat, vector<Point2f>, vector<Point2f>, vector<uchar>, int );
	
    int drawHistogram ( Mat, vector<int>[ HIST_SCL_BINS ][ HIST_ANG_BINS ], vector<Point2f>, vector<Point2f>, int );
    
    int drawHistoricalFlow( Mat, int, CvScalar );
    
    int drawPredictiveVector( Mat, int, vector<Point2f>, vector<Point2f>, CvScalar );
    
    int printHistogramBin( int );
	
	vector<Mat> getFrames ( const char* filePath );
    
    int checkKey ( int );
	
	inline double getLength ( CvPoint p, CvPoint q ) { return sqrt ( SQUARE(p.x - q.x) + SQUARE(p.y - q.y) ); }

	inline double getAngle ( CvPoint p, CvPoint q ) { return atan2( (double) p.y - q.y, (double) p.x - q.x ); }
    
    inline int atanRadToDeg ( double a ) { return (int) - ( ( a * HALF_CIRCLE_DEG / PI ) - HALF_CIRCLE_DEG ); }
	
	inline void saveFrame ( int frameIndex, Mat frame ) { stringstream outputFilePath; outputFilePath << OUT_PATH << frameIndex  << JPG_EXT; imwrite( outputFilePath.str() , frame ); }
    
    template <class T> T SQUARE( T a ) { return a * a; }
    
    template <class T> T ANG_2_HISBIN( T a ) { return a / (CIRCLE_DEG / HIST_ANG_BINS ); }
			
};