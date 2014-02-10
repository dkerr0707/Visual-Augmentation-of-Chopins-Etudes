// OpticalFlow.cxx
// David Kerr - Master of Science

#include <iostream>
#include <sys/stat.h>
#include "OpticalFlow.hpp"

using namespace std;

const int       FILE_DOES_NOT_EXIST     = -1;
const int       MIN_ARGS                = 3;
const int       PATH_ARG                = 1;
const int       DISPLAY_ARG             = 2;
const int       SAVE_ARG                = 3;

const string    PARAM_PREDICTIVE        = "-p";
const string    PARAM_HISTORICAL        = "-h";
const string    PARAM_SAVE_FRAMES       = "-s";
const string    ERROR                   = "ERROR - ";
const string    DOES_NOT_EXIST          = " does not exist.";
const string    USAGE_MSG               = "Usage: ./OpticalFlow [ FILE_PATH ] -p [predictive] or -h [historical] -s [save frames]";

int main ( int argc, char *argv[] ) {
	
    OpticalFlow * opticalFlow;
	const char* filePath;
	struct stat buf;
    bool saveFrames = false;
    string displayType;
    
	if ( argc < MIN_ARGS or (std::string(argv[DISPLAY_ARG]) != PARAM_PREDICTIVE and std::string(argv[DISPLAY_ARG]) != PARAM_HISTORICAL) )
		cout << USAGE_MSG << endl;
	
	else {
        
        displayType = string(argv[DISPLAY_ARG]);
        
        if ( argc > MIN_ARGS and std::string(argv[SAVE_ARG]) == PARAM_SAVE_FRAMES )
            saveFrames = true;
		
		filePath = argv[PATH_ARG];
        
		// does the file exist
		if ( stat( filePath, &buf ) == FILE_DOES_NOT_EXIST )
			cout << ERROR << filePath << DOES_NOT_EXIST << endl;
		else{
            if ( displayType == PARAM_PREDICTIVE )
                opticalFlow = new OpticalFlow( filePath, OpticalFlow::PREDICTIVE, saveFrames );
            else
                opticalFlow = new OpticalFlow( filePath, OpticalFlow::HISTORICAL, saveFrames );
        }
	}
    
	if (opticalFlow)
        delete opticalFlow;
    
    return EXIT_SUCCESS;
	
}