#ifndef HOGFEATURESOFBLOCK_H
#define HOGFEATURESOFBLOCK_H

#include <math.h>
#include <stdlib.h>
#include <vector>

using namespace std;

class HOGFeaturesOfBlock
{
public:
  static int const numDirections=9;
  static int const numFeats=27+4+1;
  double feats[numFeats];

  static void aggregateFeatsOfBlocks(std::vector<HOGFeaturesOfBlock> & featsOfBlocks, HOGFeaturesOfBlock & aggFeats);
  void pushBackAllFeats(std::vector<float> & featureVector);
  void pushNonContrastFeats(std::vector<float> & featureVector);
  void pushTextureFeats(std::vector<float> & featureVector);
  
};

#endif
