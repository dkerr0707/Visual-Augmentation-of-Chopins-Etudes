// ObjectDetector.h
// David Kerr - Master of Science

#include <cv.h>
#include <highgui.h>

using std::vector;

#define ASCII_ESC								27
#define MIN_WAIT_TIME						1

#define IMAGE_RESOLUTION				640, 480 // native 1280 1024
#define DEF_WIN_POS							50
#define SHAPE_THICKNESS					2
#define RED											0, 0, 255
#define GREEN										0, 255, 0

#define IPL_THREE_CHANNEL				3
#define SURF_RAD_SCALE					0.2
#define SURF_PAR_HESSIAN_THR    500
#define SURF_PAR_EXTENDED_DESC  0	
#define SURF_STOR_BLK_SIZE			0
#define SURF_MASK								0

#define EMPTY_STRING						""

class ObjectDetect {
	
	template<class T> struct index_cmp {
		index_cmp(const T arr) : arr(arr) {}
		bool operator()(const size_t a, const size_t b) const
		{ return arr[a] < arr[b]; }
		const T arr;
	};
	
	
public:
	int createSurfVideoFrames ( const char* imagePath, const char* tempImagePath );
	int realTimeSURF ( const char* modelPath );
	
private:
	int drawSURFPoints ( IplImage* frame, CvSeq* keypoints, CvSeq* descriptors );
	int drawKeypoints ( IplImage* frame, CvSeq* frameKeypoints, CvSeq* modelKeypoints, 
										 vector<int>& pointsList, int offset );
	void findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
								 const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, 
								 vector<int>& ptpairs );
	int naiveNearestNeighbor( const float* vec, int laplacian, const CvSeq* model_keypoints,
													 const CvSeq* model_descriptors );
	double compareSURFDescriptors( const float* d1, const float* d2, 
																double best, int length );
	void validatePairsLC(vector<int>& ptPairs, vector<int>& validPtPairs,
											 vector<int>& invalidPtPairs,
											 const CvSeq* currentFrameKeypoints, 
											 const CvSeq* modelKeypoints);
	float getEuclideanDistance(int x1, int y1, int x2, int y2);
	float getRatio(float a, float b);
	float getAngleDiff(float a, float b);
	void euclideanNearestNeibour(vector<size_t>& euclIndex,
															 const CvSURFPoint* featPt, 
															 const CvSeq* currentFrameKeypoints,
															 vector<int>& ptPairs,
															 int N, int featIndex);
	
	
}; 


