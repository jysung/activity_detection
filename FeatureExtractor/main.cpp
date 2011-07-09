#include "cv.h"
#include "highgui.h"
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <vector>
#include <cmath>
#include <dirent.h>
#include <algorithm>
#include <string.h>

using namespace std;

#include "constants.h"
#include "readData.cpp"
#include "features.cpp"
#include "featuresRGBD.cpp"

bool USE_HOG = true;

// print error message
void errorMsg(string message) {
    cout << "ERROR! " << message << endl;
    exit(1);
}

void parseChk(bool chk) {
    if (!chk) {
        errorMsg("parsing error.");
    }
}

map<string, string> data_act_map;
string dataLocation;

// read file that maps data and activity
void readDataActMap() {
    const string mapfile = dataLocation + "activityLabel.txt";

    printf("Opening map of data to activity: \"%s\"\n", (char*)mapfile.c_str());
    ifstream file((char*)mapfile.c_str(), ifstream::in);    
    
    string line;
    int count = 0;
    while(getline(file,line)) {
        stringstream lineStream(line);
        string element1, element2;
        parseChk(getline(lineStream, element1, ','));
        
        if (element1.compare("END") == 0) {
            break;
        }
        parseChk(getline(lineStream, element2, ','));
        if (element1.length() != 10) {
            errorMsg("Data Act Map file format mismatch..");
        }
        data_act_map[element1] = element2;
        cout << "\t" << element1  << " -> \"" << data_act_map[element1] << "\"" << endl;
        count++;
    } 
    file.close();
    
    if(count == 0) {
        errorMsg("File does not exist or is empty!\n");
    }
    printf("\tcount = %d\n\n", count);
}

void printData(FILE* pRecFile, double * feats, int numFeats){
    for (int i = 0; i < numFeats; i++){
        fprintf(pRecFile, "%.7f,", feats[i]);
    }
}

void printData(FILE* pRecFile, int * feats, int numFeats){
    for (int i = 0; i < numFeats; i++){
        fprintf(pRecFile, "%d,", feats[i]);
    }
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        errorMsg("USAGE ERROR!\nUSAGE: "+(string)argv[0]+" (data location) (output file)");
    } 

    string dataLoc = argv[1];

    bool useHead = true; 
    bool useTorso = true;
    bool useLeftArm = true;
    bool useRightArm = true;
    bool useLeftHand = true;
    bool useRightHand = true;
    bool useFullBody = true;
    bool useImage = true;
    bool useDepth = true;
    bool useSkeleton = true;
    
    dataLocation = dataLoc;
    if (dataLoc[dataLoc.length()-1]!='/') { 
        dataLocation = dataLoc + "/";
    }
    string mirrored_dataLocation = dataLocation;
    string outputFile = argv[2];
    
    printf("Data Location: \"%s\"\n", (char*)dataLocation.c_str());
    printf("Output Location: \"%s\"\n\n", (char*)outputFile.c_str());
    
    // read (data -> activity) map
    // which are stored in (activityLabel.txt) in each data folder
    readDataActMap();

    // get all names of file from the map
    vector<string> all_files;
    map<string,string>::iterator it = data_act_map.begin();
    while(it != data_act_map.end()) {
        all_files.push_back(it->first);
        it++;
    }
    printf("Number of Files to be processed = %d\n", all_files.size());
    printf("Processed data goes to (%s)\n\n", (char*)outputFile.c_str());
    
    if (all_files.size() == 0) {
        errorMsg("NO FILE TO PROCESS!!");    
    }
    // read skeleton data
    FILE* pRecFile;
    pRecFile = fopen((char*)outputFile.c_str(), "w");
    
    double **data;          //[JOINT_NUM][JOINT_DATA_NUM];
    int **data_CONF;        //[JOINT_NUM][JOINT_DATA_TYPE_NUM]
    double **pos_data;      //[POS_JOINT_NUM][POS_JOINT_DATA_NUM];
    int *pos_data_CONF;     //[POS_JOINT_NUM]
    data = new double*[JOINT_NUM];
    data_CONF = new int*[JOINT_NUM];
    for(int i=0;i<JOINT_NUM;i++) {
        data[i] = new double[JOINT_DATA_NUM];
        data_CONF[i] = new int[JOINT_DATA_TYPE_NUM];
    }
    pos_data = new double*[POS_JOINT_NUM];
    pos_data_CONF = new int[POS_JOINT_NUM];
    for(int i=0;i<POS_JOINT_NUM;i++) {
        pos_data[i] = new double[POS_JOINT_DATA_NUM];
    }
    
    int ***IMAGE;    // [X_RES][Y_RES]
    IMAGE = new int**[X_RES];
    for(int i=0;i<X_RES;i++) {
        IMAGE[i] = new int*[Y_RES];
        for (int j=0;j<Y_RES;j++) {
            IMAGE[i][j] = new int[RGBD_data];
        }
    }
    
    for (int i=0;i<all_files.size();i++) {
        //for (int j=0;j<2;j++) {
        bool mirrored = false; //(j==0)?false:true;

        readData* DATA = new readData(dataLocation, all_files[i], data_act_map, i+1, mirrored, mirrored_dataLocation);
        bool status = DATA->readNextFrame(data, pos_data, data_CONF, pos_data_CONF, IMAGE);

        Features* features_skeleton;
        if (useSkeleton)
            features_skeleton = new Features((char*)all_files[i].c_str(), pRecFile, mirrored);

        FeaturesRGBD* features_rgbd = new FeaturesRGBD(pRecFile, mirrored);

        while (status) {
            // extract skeleton features (features used in AAAI paper)
            bool started;
            if (useSkeleton)
                started =features_skeleton->extractSkeletonFeature(data, pos_data);
            else 
                started = true;

            if (started) {
                int numSurFeat = 0;

                if (USE_HOG) {
                    int numImageFeats = 0;

                    double * imageFeats = features_rgbd->computeFeatures(IMAGE, data, pos_data, &numImageFeats,
                    useHead, useTorso, useLeftArm, useRightArm,
                    useLeftHand, useRightHand, useFullBody, useImage, useDepth);

                    printData(pRecFile, imageFeats, numImageFeats);
                }

                fprintf(pRecFile, "\n");
            }

            // read next frame
            status = DATA->readNextFrame(data, pos_data, data_CONF, pos_data_CONF, IMAGE);
        }
        if (useSkeleton)
            delete features_skeleton;

        delete DATA;
        //}
    }
    fclose(pRecFile);
    
    printf("ALL DONE.\n\n");
} // end main

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
 
 IMAGE[X_RES][Y_RES][0~2]   -> RGB values
 IMAGE[X_RES][Y_RES][3]     -> depth values
 */
