#include <vector>
#include <assert.h>
#include "Point2DAbhishek.h"
#include "HOG.h"
#include "HOGFeaturesOfBlock.h"

using namespace std;

enum BodyPart { HEAD, TORSO, LEFTARM, RIGHTARM, LEFTHAND, RIGHTHAND, FULLBODY };

class FeaturesRGBD {
public:
    FILE* pRecFile;
    bool mirrored;
    static const int BLOCK_SIDE = 8;
    static const int width = 320;
    static const int WIDTH = 320;
    static const int height = 240;

    // just keep attaching, don't put newline character (\n)..
    void outputFeature(double a) {
        fprintf(pRecFile, "%.3f,", a);
    }

  // Given (x,y,z) coordinates, converts that point into its x pixel number in the 2D image.
  int xPixelFromCoords(double x, double y, double z)
  {
    return (int) (156.8584456124928 + 0.0976862095248 * x - 0.0006444357104 * y + 0.0015715946682 * z);
  }

  // Given (x,y,z) coordinates, converts that point into its y pixel number in the 2D image.
  int yPixelFromCoords(double x, double y, double z)
  {
    return (int) (125.5357201011431 + 0.0002153447766 * x - 0.1184874093530 * y - 0.0022134485957 * z);
  }

  /* Given an image IMAGE and skeleton data, as well as sets of indices into the data and pos_data
     arrays, and integers telling us how long the ori_inds and pos_inds arrays are, computes the 
     bounding box around the set of joints specified. The corners array is then populated with
     the two points representing the corners of the bounding box. */
  void findBoundingBox(int ***IMAGE, double **data, double **pos_data, int *ori_inds, int num_ori_inds,
		       int *pos_inds, int num_pos_inds, Point2DAbhishek * corners)
  {
    const int NJOINTS = 15; // number of joints in the skeleton
    const int NORI_JOINTS = 11; // number of joints that have orientation data (data array)
    const int NPOS_JOINTS = 4; // number of joints without orientations (pos_data array)

    // Will contain pixel values of each joint projected onto the image plane.
    int **joint_pos = new int*[num_ori_inds+num_pos_inds];
    for (int i = 0; i < num_ori_inds+num_pos_inds; i++){
      joint_pos[i] = new int[2];
   }
    
    // Compute the pixel values for each joint.
    for (int i = 0; i < num_ori_inds; i++)
    {
      int ind = ori_inds[i];
      joint_pos[i][0] = xPixelFromCoords(data[ind][9], data[ind][10], data[ind][11]);
      joint_pos[i][1] = yPixelFromCoords(data[ind][9], data[ind][10], data[ind][11]);
    }
    for (int i = 0; i < num_pos_inds; i++)
    {
      int ind = pos_inds[i];
      joint_pos[num_ori_inds+i][0] = xPixelFromCoords(pos_data[ind][0], pos_data[ind][1], pos_data[ind][2]);
      joint_pos[num_ori_inds+i][1] = yPixelFromCoords(pos_data[ind][0], pos_data[ind][1], pos_data[ind][2]);
    }

    // Find the most extreme x and y values for the bounding box.
    int minX = 100000000;
    int maxX = 0;
    int minY = 100000000;
    int maxY = 0;
    for (int i = 0; i < num_ori_inds+num_pos_inds; i++)
    {
      if (joint_pos[i][0] < minX)
	minX = joint_pos[i][0];

      if (joint_pos[i][0] > maxX)
	maxX = joint_pos[i][0];

      if (joint_pos[i][1] < minY)
	minY = joint_pos[i][1];

      if (joint_pos[i][1] > maxY)
	maxY = joint_pos[i][1];
    } 

    // Populate the corners array with the bounding box corners.
    corners[0] = Point2DAbhishek(minX, minY);
    corners[1] = Point2DAbhishek(maxX, maxY);

    // Tear down
    for (int i = 0; i < num_ori_inds+num_pos_inds; i++){
      delete [] joint_pos[i];
    }
    delete [] joint_pos;
  }

  void findHeadBoundingBox(int ***IMAGE, double **data, double **pos_data, Point2DAbhishek *corners){
    int ori_inds [2] = {0, 1};
    int num_ori_inds = 2;
    int num_pos_inds = 0;

    findBoundingBox(IMAGE, data, pos_data, ori_inds, num_ori_inds, 0, num_pos_inds, corners);
  }

  void findTorsoBoundingBox(int ***IMAGE, double **data, double **pos_data, Point2DAbhishek *corners){
    int ori_inds [5] = {2, 3, 5, 7, 9};
    int num_ori_inds = 5;
    int num_pos_inds = 0;

    findBoundingBox(IMAGE, data, pos_data, ori_inds, num_ori_inds, 0, num_pos_inds, corners);
  }

  void findLeftArmBoundingBox(int ***IMAGE, double **data, double **pos_data, Point2DAbhishek *corners){
    int ori_inds [2] = {3, 4};
    int num_ori_inds = 2;
    int pos_inds [1] = {0};
    int num_pos_inds = 1;

    findBoundingBox(IMAGE, data, pos_data, ori_inds, num_ori_inds, pos_inds, num_pos_inds, corners);
  }

  void findRightArmBoundingBox(int ***IMAGE, double **data, double **pos_data, Point2DAbhishek *corners){
    int ori_inds [2] = {5, 6};
    int num_ori_inds = 2;
    int pos_inds [1] = {1};
    int num_pos_inds = 1;

    findBoundingBox(IMAGE, data, pos_data, ori_inds, num_ori_inds, pos_inds, num_pos_inds, corners);
  }

  void findLeftHandBoundingBox(int ***IMAGE, double **data, double **pos_data, Point2DAbhishek *corners){
    int num_ori_inds = 0;
    int pos_inds [1] = {0};
    int num_pos_inds = 1;

    findBoundingBox(IMAGE, data, pos_data, pos_inds, num_ori_inds, pos_inds, num_pos_inds, corners);
    corners[1] = Point2DAbhishek(min(corners[0].x+BLOCK_SIDE, 320), corners[0].y);
  }

  void findRightHandBoundingBox(int ***IMAGE, double **data, double **pos_data, Point2DAbhishek *corners){
    int num_ori_inds = 0;
    int pos_inds [1] = {0};
    int num_pos_inds = 1;

    findBoundingBox(IMAGE, data, pos_data, pos_inds, num_ori_inds, pos_inds, num_pos_inds, corners);
    corners[0] = Point2DAbhishek(max(corners[1].x-BLOCK_SIDE, 0), corners[1].y);
  }

  void findFullBodyBoundingBox(int ***IMAGE, double **data, double **pos_data, Point2DAbhishek *corners){
    int ori_inds [11] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    int num_ori_inds = 11;
    int pos_inds [4] = {0, 1, 2, 3};
    int num_pos_inds = 4;

    findBoundingBox(IMAGE, data, pos_data, ori_inds, num_ori_inds, pos_inds, num_pos_inds, corners);
  }

  /* This function takes a HOG object and aggregates the HOG features for each stripe in the chunk.
     It populates aggHogVec with one HOGFeaturesOfBlock object for each stripe in the image. */
  void computeAggHogBlock(HOG & hog, int numStripes, int minXBlock, int maxXBlock, int minYBlock, 
			  int maxYBlock, std::vector<HOGFeaturesOfBlock> & aggHogVec){
    // The number of blocks in a single column of blocks in one stripe in the image.
    double stripeSize = ((double)(maxYBlock - minYBlock)) / numStripes;
    
    for (int n = 0; n < numStripes; n++)
      {
	// For each stripe, create a new HOGFeaturesOfBlock vector hogvec and fill it with the HOGFeaturesOfBlocks
	// in this stripe.
	std::vector<HOGFeaturesOfBlock> hogvec;
	for (int j = (int)(minYBlock + n * stripeSize); j <= (int)(minYBlock + (n+1) * stripeSize); j++){
	  for (int i = minXBlock; i <= maxXBlock; i++){
	    HOGFeaturesOfBlock hfob;
	    hog.getFeatVec(j, i, hfob);
	    hogvec.push_back(hfob);
	  }
	}
	// Now compute the aggregate features for this stripe, and store the aggregate as its own
	// HOGFeaturesOfBlock in the aggHogVec vector.
	HOGFeaturesOfBlock agg_hfob;
	HOGFeaturesOfBlock::aggregateFeatsOfBlocks(hogvec, agg_hfob);
	aggHogVec.push_back(agg_hfob);
      }
  }

  /* This function take a bunch of data and an enum specifying the body part, and computes the HOG features
     for the bounding box surrounding that body part. The result is pushed into aggHogVec. */
  void computeBodyPartHOGFeatures(int ***IMAGE, HOG & hog, double **data, double **pos_data, enum BodyPart bodyPart,
				  std::vector<HOGFeaturesOfBlock> & aggHogVec){
    Point2DAbhishek corners [2];
    const int numStripes = 1;
    if (bodyPart == HEAD) findHeadBoundingBox(IMAGE, data, pos_data, corners);
    else if (bodyPart == TORSO) findTorsoBoundingBox(IMAGE, data, pos_data, corners);
    else if (bodyPart == LEFTARM) findLeftArmBoundingBox(IMAGE, data, pos_data, corners);
    else if (bodyPart == RIGHTARM) findRightArmBoundingBox(IMAGE, data, pos_data, corners);
    else if (bodyPart == LEFTHAND) findLeftHandBoundingBox(IMAGE, data, pos_data, corners);
    else if (bodyPart == RIGHTHAND) findRightHandBoundingBox(IMAGE, data, pos_data, corners);
    else if (bodyPart == FULLBODY) findFullBodyBoundingBox(IMAGE, data, pos_data, corners);
    else assert(false);
    
    int minXBlock = (int)(corners[0].x / BLOCK_SIDE);
    int minYBlock = (int)(corners[0].y / BLOCK_SIDE);
    int maxXBlock = (int)(corners[1].x / BLOCK_SIDE);
    int maxYBlock = (int)(corners[1].y / BLOCK_SIDE);

    computeAggHogBlock(hog, numStripes, minXBlock, maxXBlock, minYBlock, maxYBlock, aggHogVec);
  }

  /* Take all the features in the vector aggHogVec, and compile them into a single double array. */
  double* aggregateFeaturesIntoArray(std::vector<HOGFeaturesOfBlock> & aggHogVec, int *numFeats){
    // First, iterate through and just count the number of features we'll need.
    std::vector<HOGFeaturesOfBlock>::iterator block_iterator = aggHogVec.begin();
    *numFeats = 0;
    while (block_iterator != aggHogVec.end()) {
      *numFeats += HOGFeaturesOfBlock::numFeats;
      block_iterator++;
    }

    double *feats = new double[*numFeats];
    // Now iterator through again and add the features in.
    int ind = 0;
    block_iterator = aggHogVec.begin();
    while (block_iterator != aggHogVec.end()) {
      for (int i = 0; i < HOGFeaturesOfBlock::numFeats; i++){
	feats[ind] = block_iterator->feats[i];
	ind++;
      }
      block_iterator++;
    }
    return feats;
  }

  void mirrorData(int ***IMAGE, int width, int height){
    const int NCHANNELS = 4;
    int tmp;
    for (int x = 0; x < width/2; x++){
      for (int y = 0; y < height; y++){
	for (int ch = 0; ch < NCHANNELS; ch++){
	  tmp = IMAGE[width-1-x][y][ch];
	  IMAGE[width-1-x][y][ch] = IMAGE[x][y][ch];
	  IMAGE[x][y][ch] = tmp;
	}
      }
    }
  }

  void populateDepthImage(int ***IMAGE, int ***depthIMAGE, const int width, const int height){
    double scaleFactor = 255.0 / 10000.0;
    int maxValue = 255;
    for (int i = 0; i < width; i++){
      for (int j = 0; j < height; j++){
	for (int k = 0; k < 4; k++){
	  depthIMAGE[i][j][k] = min((int)(IMAGE[i][j][3] * scaleFactor), maxValue);
	}
      }
    }
  }

  /* Compute the HOG features for the images in the bounding boxes of various body parts.
     Computes both image HOG featurs and depth HOG features.
     Return a pointer to a double array with those features, and popualte the numFeats
     integer with the length of the returned doubled array. Note that the first half 
     of the returned array is image features, while the second half is depth features. */
  double* computeFeatures(int ***IMAGE, double **data, double **pos_data, int *numFeats,
			  bool useHead, bool useTorso, bool useLeftArm,
			  bool useRightArm, bool useLeftHand, bool useRightHand,
			  bool useFullBody, bool useImage, bool useDepth) {
    
    int ***depthIMAGE;
    if (useDepth){
      depthIMAGE = new int**[width];
      for (int i = 0; i < width; i++){
	depthIMAGE[i] = new int*[height];
	for (int j = 0; j < height; j++){
	  depthIMAGE[i][j] = new int[4];
	}
      }
      
      populateDepthImage(IMAGE, depthIMAGE, width, height);
    }
    
    if (this->mirrored) mirrorData(IMAGE, width, height);

    HOG hog, depthHog;
    
    if (useImage)
      hog.computeHOG(IMAGE, width, height);

    if (useDepth)
      depthHog.computeHOG(depthIMAGE, width, height);
      
    std::vector<HOGFeaturesOfBlock> aggHogVec;
    if (useImage){
      if (useHead)
	computeBodyPartHOGFeatures(IMAGE, hog, data, pos_data, HEAD, aggHogVec);

      if (useTorso)
	computeBodyPartHOGFeatures(IMAGE, hog, data, pos_data, TORSO, aggHogVec);

      if (useLeftArm)
	computeBodyPartHOGFeatures(IMAGE, hog, data, pos_data, LEFTARM, aggHogVec);

      if (useRightArm)
	computeBodyPartHOGFeatures(IMAGE, hog, data, pos_data, RIGHTARM, aggHogVec);

      if (useLeftHand)
	computeBodyPartHOGFeatures(IMAGE, hog, data, pos_data, LEFTHAND, aggHogVec);

      if (useRightHand)
	computeBodyPartHOGFeatures(IMAGE, hog, data, pos_data, RIGHTHAND, aggHogVec);

      if (useFullBody)
	computeBodyPartHOGFeatures(IMAGE, hog, data, pos_data, FULLBODY, aggHogVec);
    }
    
    if (useDepth){
      if (useHead)
	computeBodyPartHOGFeatures(depthIMAGE, depthHog, data, pos_data, HEAD, aggHogVec);

      if (useTorso)
	computeBodyPartHOGFeatures(depthIMAGE, depthHog, data, pos_data, TORSO, aggHogVec);

      if (useLeftArm)
	computeBodyPartHOGFeatures(depthIMAGE, depthHog, data, pos_data, LEFTARM, aggHogVec);
      
      if (useRightArm)
	computeBodyPartHOGFeatures(depthIMAGE, depthHog, data, pos_data, RIGHTARM, aggHogVec);

      if (useLeftHand)
	computeBodyPartHOGFeatures(depthIMAGE, depthHog, data, pos_data, LEFTHAND, aggHogVec);

      if (useRightHand)
	computeBodyPartHOGFeatures(depthIMAGE, depthHog, data, pos_data, LEFTHAND, aggHogVec);

      if (useFullBody)
	computeBodyPartHOGFeatures(depthIMAGE, depthHog, data, pos_data, FULLBODY, aggHogVec);

      // Tear down
      for (int i = 0; i < width; i++){
	for (int j = 0; j < height; j++){
	  delete [] depthIMAGE[i][j];
	}
	delete [] depthIMAGE[i];
      }
      delete [] depthIMAGE;
    }

      return aggregateFeaturesIntoArray(aggHogVec, numFeats);
    }

    // MIRRORED means skeleton is mirrored; RGBD comes in non mirrored form
    // but mirroring should be easy for RGBD
    FeaturesRGBD(FILE* pRecFile, bool mirrored) {
        this->pRecFile=pRecFile;
        this->mirrored=mirrored;
    }
    
    ~FeaturesRGBD() {
    
    }    

};

/** 
there are 11 joints that have both orientation (3x3) and position (x,y,z) data
        XN_SKEL_HEAD,
        XN_SKEL_NECK,
        XN_SKEL_TORSO,
        XN_SKEL_LEFT_SHOULDER,
        XN_SKEL_LEFT_ELBOW,
        XN_SKEL_RIGHT_SHOULDER,
        XN_SKEL_RIGHT_ELBOW,
        XN_SKEL_LEFT_HIP,
        XN_SKEL_LEFT_KNEE,
        XN_SKEL_RIGHT_HIP,
        XN_SKEL_RIGHT_KNEE
	
there are 4 joints that have only position (x,y,z) data
        XN_SKEL_LEFT_HAND,
        XN_SKEL_RIGHT_HAND,
        XN_SKEL_LEFT_FOOT,
        XN_SKEL_RIGHT_FOOT

 data[][0~8]    -> orientation (3x3 matrix)
                     3x3 matrix is stored as 
                        0 1 2
                        3 4 5
                        6 7 8
                     read PDF for description about 3x3 matrix 
 data[][9~11]   -> x,y,z position for eleven joints
 
 data_CONF[][0]   -> confidence value of orientation  (data[][0~8]) 
 data_CONF[][1]   -> confidence value of xyz position (data[][9~11])
 
 data_pos[][0~2] -> x,y,z position for four joints
 data_pos_CONF[]  -> confidence value of xyz position (data_pos[][0~2])
 
X_RES and Y_RES are in constants.h, so just use them.
 IMAGE[X_RES][Y_RES][0~2]   -> RGB values
 IMAGE[X_RES][Y_RES][3]     -> depth values

 
 */
