// ObjectDetector.cxx
// David Kerr - Master of Science

#include <sys/stat.h>
#include "ObjectDetector.h"

using namespace std;

int ObjectDetect::realTimeSURF( const char* modelPath ) {

	CvCapture* capture;
	IplImage* frame, *model, *resizedFrame, *correspond;
	CvSeq *frameKeypoints, *frameDescriptors;
	CvSeq *modelKeypoints, *modelDescriptors;
	CvMemStorage* storage = cvCreateMemStorage( SURF_STOR_BLK_SIZE );
	CvSURFParams params = cvSURFParams( SURF_PAR_HESSIAN_THR, SURF_PAR_EXTENDED_DESC );
	vector<int> ptPairs, validPtPairs, invalidPtPairs;
	clock_t start;
	int offset;
	double duration, imgProDur, surfDur, pairsDur, valDur, drawDur;
	
	try {
		model = cvLoadImage ( modelPath );
		cvExtractSURF( model, SURF_MASK, &modelKeypoints, &modelDescriptors, storage, params );
		
	}
	catch ( int error ) {
		cout << error << endl;
	}
	
	try {
		capture = cvCaptureFromCAM ( CV_CAP_ANY );
	}
	catch( int error ) {
		cout << error << endl;
	}
	
	if ( capture and model ) {
		cvNamedWindow ( modelPath, CV_WINDOW_AUTOSIZE );
		cvMoveWindow ( modelPath, DEF_WIN_POS, DEF_WIN_POS );
		
		resizedFrame = cvCreateImage ( cvSize ( IMAGE_RESOLUTION ), IPL_DEPTH_8U, IPL_THREE_CHANNEL );
		
		while( TRUE ) {
			
			start = clock();
			
			validPtPairs.clear();
			invalidPtPairs.clear();
			
			try {
				frame = cvQueryFrame ( capture );
				cvResize ( frame, resizedFrame );
				frame = resizedFrame;
				
			}
			catch( int error ) {
				cout << error << endl;
			}
			
			//debug time
			imgProDur = ( clock() - start ) / (double) CLOCKS_PER_SEC;
			
			if ( frame ) {
				
				cvExtractSURF( frame, SURF_MASK, &frameKeypoints, &frameDescriptors, storage, params );
				
				//debug time
				surfDur = ( ( clock() - start ) / (double)CLOCKS_PER_SEC ) - imgProDur;
				
				findPairs( modelKeypoints, modelDescriptors, frameKeypoints, frameDescriptors, ptPairs );
				
				//debug time
				pairsDur = (( clock() - start ) / ( double ) CLOCKS_PER_SEC ) - surfDur - imgProDur;
				
				validatePairsLC(ptPairs, validPtPairs, invalidPtPairs,
												frameKeypoints, modelKeypoints);
				
				//debug time
				valDur = (( clock() - start ) / ( double ) CLOCKS_PER_SEC ) 
					- surfDur - imgProDur - pairsDur;
				
				correspond = cvCreateImage( cvSize(frame->width+model->width,
																										 max(frame->height, model->height))
																						 ,IPL_DEPTH_8U,IPL_THREE_CHANNEL);
				
				cvSetImageROI( correspond, cvRect(0,0,frame->width, frame->height));
				cvCopy(frame, correspond);
				cvSetImageROI( correspond, cvRect(frame->width, 0, correspond->width,
																					model->height));
				
				cvCopy(model, correspond);
				cvResetImageROI(correspond);
				
				offset = frame->width;
				
				frame = correspond;
				
				drawKeypoints ( frame, frameKeypoints, modelKeypoints, validPtPairs, offset);
				
				cvShowImage( modelPath, frame );
				
				cvReleaseImage ( &correspond );
				
				//debug time
				drawDur = (( clock() - start ) / ( double ) CLOCKS_PER_SEC ) 
					- surfDur - imgProDur - pairsDur - valDur;
			}
			
			//debug time
			duration = ( clock() - start ) / (double) CLOCKS_PER_SEC;
			cout << "i " << imgProDur << " s " << surfDur << " p " << pairsDur 
				<< " v " << valDur << " d " << drawDur << " t " << duration << endl;
			
			if ( cvWaitKey ( MIN_WAIT_TIME ) == ASCII_ESC )
				break;
			
		}
		
		cvReleaseImage( &resizedFrame );
		cvReleaseCapture( &capture );
		
	}
	return EXIT_SUCCESS;
	
}

void ObjectDetect::euclideanNearestNeibour(vector<size_t>& euclIndex,
														 const CvSURFPoint* featPt, 
														 const CvSeq* currentFrameKeypoints,
														 vector<int>& ptPairs,
														 int N, int featIndex){
	
	int x1, x2, y1, y2;
	CvSURFPoint* currFeatPt; //pt, laplacian, size, dir, hessian
	vector<float> euclNearNeib;
	
	x1 = featPt->pt.x;
	y1 = featPt->pt.y;
	
	//	printf("pairs: %d\n", ptPairs.size());
	
	
	for (int i = 1; i < ptPairs.size(); i += 2){
		
		//dont check the same feature it will always be valid
		if (i != featIndex){
			currFeatPt = (CvSURFPoint*)cvGetSeqElem( currentFrameKeypoints, 
																							ptPairs[i] );
			x2 = currFeatPt->pt.x;
			y2 = currFeatPt->pt.y;
			
			euclNearNeib.push_back(getEuclideanDistance(x1, y1, x2, y2));
		}
		
	}
	
	//make the index so we can sort and get index values 
	for (unsigned i=0; i < euclNearNeib.size(); i++)
		euclIndex.push_back(i);
	
	sort(euclIndex.begin(), euclIndex.end(), 
			 index_cmp<vector<float>&>(euclNearNeib));
	
	//keep the top N matches
	euclIndex.erase(euclIndex.begin() + N, euclIndex.end() );
	
	
}

float ObjectDetect::getAngleDiff(float a, float b){
	
	return max(a,b) - min(a,b);
}


float ObjectDetect::getRatio(float a, float b){
	
	if (a == 0.0f && b == 0.0f) {
		return 1.0f;
	}
	if (a == 0.0f) {
		a = 0.001f;
	}
	if (b == 0.0f) {
		b = 0.001f;
	}
	
	return a / b;
}

void ObjectDetect::validatePairsLC(vector<int>& ptPairs, vector<int>& validPtPairs,
										 vector<int>& invalidPtPairs,
										 const CvSeq* currentFrameKeypoints, 
										 const CvSeq* modelKeypoints){
	
	int N = 20;
	int K = 4;
	
	float angleThreshold = 22.5f;
	float scaleRatioFactor = 1.5f;
	float distanceRatioFactor = 1.2f;
	
	int count;
	
	vector<size_t> euclIndex;
	
	CvSURFPoint* currFeatPt;
	CvSURFPoint* currModelPt;
	CvSURFPoint* nnCurFramFeatPt; //pt, laplacian, size, dir, hessian
	CvSURFPoint* nnModelFeatPt;
	
	float currAngDiff, nnAngDiff, angleDiff,
	currScaleRatio, nnScaleRatio,
	cf2nnDist, mk2nnDist, distanceRatio;
	
	
	
	for(int i = 1; i < (int)ptPairs.size(); i += 2)
	{
		count = 0;
		
		currModelPt = (CvSURFPoint*)cvGetSeqElem( modelKeypoints, 
																						 ptPairs[i-1] );
		
		currFeatPt = (CvSURFPoint*)cvGetSeqElem( currentFrameKeypoints, 
																						ptPairs[i] );
		
		currAngDiff = getAngleDiff(currModelPt->dir, currFeatPt->dir);		
		currScaleRatio = getRatio(currModelPt->size, currFeatPt->size);
		
		
		euclIndex.erase(euclIndex.begin(), euclIndex.end() );
		
		euclideanNearestNeibour(euclIndex, currFeatPt, currentFrameKeypoints,
														ptPairs, N, i);
		
		for (int j = 0; j < euclIndex.size(); j++){
			
			nnModelFeatPt = (CvSURFPoint*)cvGetSeqElem( modelKeypoints, 
																								 ptPairs[euclIndex[j]*2] );
			
			// *2+1 to get the correct indexs
			nnCurFramFeatPt = (CvSURFPoint*)cvGetSeqElem( currentFrameKeypoints, 
																									 ptPairs[euclIndex[j]*2+1] );
			
			
			//calculate the parameters for the current nn match
			nnAngDiff = getAngleDiff(nnModelFeatPt->dir, nnCurFramFeatPt->dir);
			nnScaleRatio = getRatio(nnModelFeatPt->size, nnCurFramFeatPt->size);
			mk2nnDist = getEuclideanDistance(currModelPt->pt.x, currModelPt->pt.y,
																			 nnModelFeatPt->pt.x, nnModelFeatPt->pt.y);
			cf2nnDist = getEuclideanDistance(currFeatPt->pt.x, currFeatPt->pt.y,
																			 nnCurFramFeatPt->pt.x, nnCurFramFeatPt->pt.y);
			
			
			//compare the current feature to its nn feature
			angleDiff = getAngleDiff(currAngDiff, nnAngDiff);
			distanceRatio = getRatio(mk2nnDist, cf2nnDist);
			
			if (angleDiff < angleThreshold &&
					currScaleRatio > nnScaleRatio / scaleRatioFactor && 
					currScaleRatio < nnScaleRatio * scaleRatioFactor &&
					currScaleRatio > distanceRatio / distanceRatioFactor &&
					currScaleRatio < distanceRatio * distanceRatioFactor){
				
				count++;
				
			}
		}
		//feature is valid
		if (count >= K) {
			validPtPairs.push_back(ptPairs[i-1]);
			validPtPairs.push_back(ptPairs[i]);
			
			//feature is invalid
		}else {
			invalidPtPairs.push_back(ptPairs[i-1]);
			invalidPtPairs.push_back(ptPairs[i]);
		}
		
		
	}	
	
	//	delete euclIndex;
	
	//	delete currFeatPt;
	//	delete currModelPt;
	//	delete nnCurFramFeatPt;
	//	delete nnModelFeatPt;
	
}


float ObjectDetect::getEuclideanDistance(int x1, int y1, int x2, int y2){
	
	float xSquared = x2-x1; 
	xSquared *= xSquared;
	float ySquared = y2-y1; 
	ySquared *= ySquared;
	
	return sqrt(xSquared + ySquared);
	
}

double
ObjectDetect::compareSURFDescriptors( const float* d1, const float* d2, 
											 double best, int length )
{
	double total_cost = 0;
	assert( length % 4 == 0 );
	for( int i = 0; i < length; i += 4 )
	{
		double t0 = d1[i] - d2[i];
		double t1 = d1[i+1] - d2[i+1];
		double t2 = d1[i+2] - d2[i+2];
		double t3 = d1[i+3] - d2[i+3];
		total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
		if( total_cost > best )
			break;
	}
	return total_cost;
}


int
ObjectDetect::naiveNearestNeighbor( const float* vec, int laplacian,
										 const CvSeq* model_keypoints,
										 const CvSeq* model_descriptors )
{
	int length = (int)(model_descriptors->elem_size/sizeof(float));
	int i, neighbor = -1;
	double d, dist1 = 1e6, dist2 = 1e6;
	CvSeqReader reader, kreader;
	cvStartReadSeq( model_keypoints, &kreader, 0 );
	cvStartReadSeq( model_descriptors, &reader, 0 );
	
	//cout << length << endl;
	
	for( i = 0; i < model_descriptors->total; i++ )
	{
		const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
		const float* mvec = (const float*)reader.ptr;
		CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
		CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
		if( laplacian != kp->laplacian )
			continue;
		d = compareSURFDescriptors( vec, mvec, dist2, length );
		if( d < dist1 )
		{
			dist2 = dist1;
			dist1 = d;
			neighbor = i;
		}
		else if ( d < dist2 )
			dist2 = d;
	}
 // if ( dist1 < 0.8*dist2 )//0.6
		return neighbor;
	
 // return -1;
}

void
ObjectDetect::findPairs( const CvSeq* objectKeypoints, const CvSeq* objectDescriptors,
					const CvSeq* imageKeypoints, const CvSeq* imageDescriptors, 
					vector<int>& ptpairs )
{
	int i;
	CvSeqReader reader, kreader;
	cvStartReadSeq( objectKeypoints, &kreader );
	cvStartReadSeq( objectDescriptors, &reader );
	ptpairs.clear();
	
	for( i = 0; i < objectDescriptors->total; i++ )
	{
		const CvSURFPoint* kp = (const CvSURFPoint*)kreader.ptr;
		const float* descriptor = (const float*)reader.ptr;
		CV_NEXT_SEQ_ELEM( kreader.seq->elem_size, kreader );
		CV_NEXT_SEQ_ELEM( reader.seq->elem_size, reader );
		int nearest_neighbor = naiveNearestNeighbor( descriptor, 
																								kp->laplacian, 
																								imageKeypoints, 
																								imageDescriptors );
		if( nearest_neighbor >= 0 )
		{
			ptpairs.push_back(i);
			ptpairs.push_back(nearest_neighbor);
		}
	}
}

int ObjectDetect::drawKeypoints ( IplImage* frame, CvSeq* frameKeypoints, CvSeq* modelKeypoints, vector<int>& pointsList, int offset ) {

	CvSURFPoint* point;
	int x, y, x1, y1, r, d;
	CvScalar color = cvScalar( GREEN );
	
	for ( int j = 1; j < (int)pointsList.size(); j += 2 ) {//pointsList.size()
		//cout << j << " " << pointsList.size() << endl;
		point = ( CvSURFPoint* ) cvGetSeqElem ( frameKeypoints, pointsList[j] );
		
		x = cvRound( point->pt.x );
		y = cvRound( point->pt.y );
		r = cvRound( point->size * SURF_RAD_SCALE );
		d = cvRound( point->dir );
		
		cvLine( frame, cvPoint ( x, y ),
					 cvPoint ( x + cos(d) * r, y + sin(d) * r ), 
					 color, SHAPE_THICKNESS );
		
		cvCircle( frame, cvPoint ( x, y ), r, color, SHAPE_THICKNESS );
		
		point = (CvSURFPoint*)cvGetSeqElem( modelKeypoints, pointsList[j-1] );
		x1 = cvRound(offset + point->pt.x);
		y1 = cvRound(point->pt.y);
		r = cvRound( point->size * SURF_RAD_SCALE );
		d = cvRound( point->dir );
		
	//	cout << point->pt.x << " " << point->pt.y << endl;
		
		//circle line
		cvLine( frame, cvPoint ( x1, y1 ),
					 cvPoint ( x1 + cos(d) * r, y1 + sin(d) * r ), 
					 color, SHAPE_THICKNESS );
		
		cvCircle( frame, cvPoint ( x1, y1 ), r, color, SHAPE_THICKNESS );
		
		cvLine(frame, cvPoint(x1,y1), cvPoint(x, y), cvScalar(RED));
	 
	}
	
	return EXIT_SUCCESS;
	
}

int ObjectDetect::drawSURFPoints ( IplImage* frame, CvSeq* keypoints, CvSeq* descriptors ) {
	
	CvMemStorage* storage = cvCreateMemStorage( SURF_STOR_BLK_SIZE );
	CvSURFParams params = cvSURFParams( SURF_PAR_HESSIAN_THR, SURF_PAR_EXTENDED_DESC );
	CvSURFPoint* point;
	CvScalar color = cvScalar( RED );
	int x, y, r, d;	
	
	cvExtractSURF( frame, SURF_MASK, &keypoints, &descriptors, storage, params );
	
	for( int j = 0; j < keypoints->total; j++ ) {
		
		point = ( CvSURFPoint* ) cvGetSeqElem ( keypoints, j );
		
		x = cvRound( point->pt.x );
		y = cvRound( point->pt.y );
		r = cvRound( point->size * SURF_RAD_SCALE );
		d = cvRound( point->dir );
		
		cvLine( frame, cvPoint ( x, y ),
						cvPoint ( x + cos(d) * r, y + sin(d) * r ), 
						color, SHAPE_THICKNESS );
		
		cvCircle( frame, cvPoint ( x, y ), r, color, SHAPE_THICKNESS );
	}
	
	cvRelease( (void **) &keypoints );
	cvRelease( (void **) &descriptors );
	cvReleaseMemStorage( &storage );
	
	return EXIT_SUCCESS;
}
