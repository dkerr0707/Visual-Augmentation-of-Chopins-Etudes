// constants.hpp
// David Kerr - Master of Science

//color indexs
enum { BLUE_INDEX, GREEN_INDEX, RED_INDEX };

//keys
const int           ESC_KEY                 = 27;
const int           RIGHT_ARROW             = 63235;
const int           LEFT_ARROW              = 63234;
const int           WAIT_KEY_DURATION       = 0;

const int           WIDTH_ROI               = 640;
const int           HEIGHT_ROI              = 440;

const std::string   EMPTY_STRING            = "";
const std::string   SLASH                   = " / ";
const std::string   FPS                     = "FPS - ";
const std::string   OUT_PATH                = "tempImages/";
const std::string   JPG_EXT                 = ".jpg";

const CvScalar      RED                     = CV_RGB( 255, 0, 0 );
const CvScalar      GREEN                   = CV_RGB( 0, 255, 0 );

//circle values
const int           CIRCLE_RAD              = 2;
const int           CIRCLE_THICKNESS        = 2;
const int           SCALE_FACTOR            = 3;
const int           LINE_THICKNESS          = 2;
const double        PI                      = 3.14159265358979323846;
const double        PI_OVER_FOUR            = PI / 4;

//blob detection params
const std::string   BLOB_DET_TYPE           = "SimpleBlob";
const int           MIN_DIST_BETWEEN_BLOBS  = 30.0f;
const bool          FILTER_BY_INTERTIA      = false;
const bool          FILTER_BY_CONVEXITY     = false;
const bool          FILTER_BY_COLOR         = false;
const bool          FILTER_BY_CIRCULARITY   = false;
const bool          FILTER_BY_AREA          = true;
const float         MIN_AREA                = 50.0f;
const float         MAX_AREA                = 99999.0f;

//rgb skin thesholds
const int           MAX_MIN_THR             = 5;
const int           ABS_RED_GREEN_THR       = 5;
const int           RED_THR                 = 80;
const int           GREEN_THR               = 25;
const int           BLUE_THR                = 5;
const int           SKIN_PIXEL              = 255;

const int           MAX_FLOW_FRAMES         = 20;

const int           EMPTY_INDEX             = -1;
const int           MIN_DISTANCE            = 25;
const int           START_INDEX             = 0;

const int           MIN_PRED_DISTANCE       = 1;//2