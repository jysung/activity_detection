#include "HOGFeaturesOfBlock.h"

void HOGFeaturesOfBlock::aggregateFeatsOfBlocks(std::vector<HOGFeaturesOfBlock> & featsOfBlocks, HOGFeaturesOfBlock & aggFeats)
{
  size_t numBlocks=featsOfBlocks.size();
  for(size_t f=0;f<numFeats;f++)
    {
      aggFeats.feats[f]=0;
      for(size_t b=0;b<numBlocks;b++)
	aggFeats.feats[f]+=featsOfBlocks[b].feats[f];
      aggFeats.feats[f]/=numBlocks;
    } 
}

void HOGFeaturesOfBlock::pushBackAllFeats(std::vector<float> & featureVector)
{
  for(size_t i=0;i<numFeats-1;i++) // the last one always seems to be 0
    featureVector.push_back(feats[i]);
}

void HOGFeaturesOfBlock::pushNonContrastFeats(std::vector<float> & featureVector) // the last one always seems to be 0
{
  for(size_t i=numDirections*2;i<numFeats-1;i++)
    featureVector.push_back(feats[i]);
}

void HOGFeaturesOfBlock::pushTextureFeats(std::vector<float> & featureVector) // the last one always seems to be 0
{
  for(size_t i=numDirections*3;i<numFeats-1;i++)
    featureVector.push_back(feats[i]);
}
