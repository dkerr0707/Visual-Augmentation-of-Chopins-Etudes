// HandTracker.hpp
// David Kerr - Master of Science

#include "iostream"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "constants.hpp"
#include "assert.h"

using namespace std;
using namespace cv;

class HandTracker {

public:
    enum DisplayType { PREDICTIVE, HISTORICAL };
    HandTracker( const char*, DisplayType, bool );
    ~HandTracker();
	
private:
    vector<Mat>                     mFrames;
    vector< vector<cv::KeyPoint> >  mKeypoints;
    vector< vector<int> >           mIndexVector;
    DisplayType                     mTrackerDisplay;
    bool                            mSaveFrames;
    
    vector<Mat> getFrames ( const char* );
    int processFrames();
    Mat getSkin ( int );
    vector<cv::KeyPoint> getKeyPoints( Mat );
    int setIndexVector( );
    int drawHistoricalTracker( int, int, CvScalar );
    int drawPredictiveTracker( int, CvScalar );
    int drawPredictiveVector( Mat, int, CvPoint, CvPoint, CvScalar );
    int saveFrame( int);
    int checkKey( int );
    inline float getDistance( KeyPoint a, KeyPoint b ) { return sqrt( SQUARE(b.pt.x - a.pt.x) + SQUARE(b.pt.y - a.pt.y) ); }
    inline double getAngle ( CvPoint p, CvPoint q ) { return atan2( (double) p.y - q.y, (double) p.x - q.x ); }
    template <class T> T SQUARE( T a ) { return a * a; }
			
};