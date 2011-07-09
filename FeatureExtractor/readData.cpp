bool READ_FROM_PNG = true;

class readData {
private:
    
    int currentFrameNum;
    int currentFrameNum_RGBD;
    int lastFrame;
    
    string dataLocation;
    string dataLocation_mirrored;
    string fileName;
    string fileName_skeleton;
    string fileName_RGBD;
    string curActivity;
    map<string, string> data_act_map;
    ifstream* file;
    ifstream* file_RGBD;
    
    bool mirrored;

    // print error message
    void errorMsg(string message, bool exitProgram) {
        cout << "ERROR! " << message << endl;
        printf("\tcurrentFrameNum = %d\n", currentFrameNum);
        printf("\tcurrentFrameNum_RGBD = %d\n", currentFrameNum_RGBD);
        
        if (exitProgram) {
            exit(1);
        }
    }    
    
    void errorMsg(string message) {
        errorMsg(message, true);
    }
    

    
    bool parseChk(bool chk, bool skeleton) {
        if (!chk) {
            if (skeleton) {
                errorMsg("parsing error. (skeleton)", true); 
            } else {
                errorMsg("parsing error. (RGBD) - IGNORE THIS ERROR!! (all random dataset will hit this error)", false);
            }
            return false;
        }
        return true;
    }

    // read skeleton data file
    void prepareSkeletonData() {
        curActivity = data_act_map[fileName];
        
        if (!mirrored) {
            fileName_skeleton = dataLocation + fileName + ".txt";
        } else {
            fileName_skeleton = dataLocation_mirrored + fileName + ".txt";
        }
        
        printf("\tOpening \"%s\" (%s)\n", (char*)fileName_skeleton.c_str(), (char*)curActivity.c_str());
        file = new ifstream((char*)fileName_skeleton.c_str(), ifstream::in);
        currentFrameNum = -99;
    }
    
    void closeSkeletonData() {
        file->close();
        printf("\tskeleton file closed\n");
    }
    
    // return true if data retrieving was successful
    bool readNextLine_skeleton(double **data, double **pos_data, int **data_CONF, int *data_pos_CONF) {
        string line;        
        bool file_ended = true;
        
        if (getline(*file,line)) {
            file_ended=false;
            stringstream lineStream(line);
            string element;
            
            int jointCount=0;  
            int joint_dataCount = 0;
            
            int pos_jointCount = 0;      
            int pos_joint_dataCount = 0;
            
            parseChk(getline(lineStream, element, ','), true);
            currentFrameNum = atoi((char*)element.c_str());
            
            if (element.compare("END") == 0) {
                file_ended = true;
                return false;
            }
            
            while (getline(lineStream, element, ',')) {
                double e = strtod((char*)element.c_str(), NULL);

                if (jointCount < JOINT_NUM) {
                    data[jointCount][joint_dataCount] = e;
                    joint_dataCount++;
                    
                    if (joint_dataCount == JOINT_DATA_ORI_NUM) {
                        parseChk(getline(lineStream, element, ','), true); // ori conf value
                        data_CONF[jointCount][0] = atoi((char*)element.c_str());
                    } else if (joint_dataCount >= JOINT_DATA_NUM) {
                        parseChk(getline(lineStream, element, ','), true); // pos conf value
                        data_CONF[jointCount][1] = atoi((char*)element.c_str());
                        jointCount++;
                        joint_dataCount = 0;
                    }
                    
                } else {
                    // pos only joints
                    if (pos_jointCount >= POS_JOINT_NUM) {
                        errorMsg("PARSING ERROR!!!!!");
                    }
                    pos_data[pos_jointCount][pos_joint_dataCount] = e;
                    pos_joint_dataCount++;
                    if (pos_joint_dataCount >= POS_JOINT_DATA_NUM) {
                        parseChk(getline(lineStream, element, ','), true); // pos conf value
                        data_pos_CONF[pos_jointCount] = atoi((char*)element.c_str());
                        
                        pos_jointCount++;
                        pos_joint_dataCount = 0;
                    }
                }
            }
            
            // check if there is more data in current frame..
            if (getline(lineStream, element,',')) {
                errorMsg("more data exist in skeleton data ..\n");
            }
            
        } 
        
        if (currentFrameNum == -99) {
            errorMsg("file does not exist or empty!!");
        }
        
        return !file_ended;
    }

    // read RGBD data file
    void prepareRGBDData() {
        fileName_RGBD = dataLocation + fileName + "_rgbd.txt";
        printf("\tOpening \"%s\" (%s)\n", (char*)fileName_RGBD.c_str(), (char*)curActivity.c_str());
        file_RGBD = new ifstream((char*)fileName_RGBD.c_str(), ifstream::in);
        currentFrameNum = -99;
    }
    
    void closeRGBDData() {
        file_RGBD->close();
        printf("\tRGBD file closed\n");
    }

    // return true if data retrieving was successful
    bool readNextPNG(int ***data) {
        stringstream ss;
        ss << currentFrameNum;
        fileName_RGBD = dataLocation + fileName + "/RGB_" + ss.str() +".png";
        string fileName_Depth = dataLocation + fileName + "/Depth_" + ss.str() +".png";
        if (currentFrameNum == 1) {
            printf("\tOpening \"%s\" and so forth..\n", 
                    (char*)fileName_RGBD.c_str());
            printf("\tOpening \"%s\" and so forth..\n", 
                    (char*)fileName_Depth.c_str());
        }

        // Load an image from file
        cv::Mat rgbImage = cv::imread((char*)fileName_RGBD.c_str(),1);
        cv::Mat colorArr[3];
        cv::split(rgbImage, colorArr);
        
        cv::Mat depthImage = cv::imread((char*)fileName_Depth.c_str(),-1);
        
        if (rgbImage.data == NULL) {
            printf("ERROR! Unable to open file %s.\n", (char*)fileName_RGBD.c_str());
            exit(1);
        }
        if (depthImage.data == NULL) {
            printf("ERROR! Unable to open file %s.\n", (char*)fileName_Depth.c_str());
            exit(1);
        }
        
        for (int y=0; y<Y_RES; y++){ 
            // opencv uses BGR order
            uchar* Bptr = colorArr[0].ptr<uchar>(y); 
            uchar* Gptr = colorArr[1].ptr<uchar>(y); 
            uchar* Rptr = colorArr[2].ptr<uchar>(y); 
            ushort* IRptr = depthImage.ptr<ushort>(y); 
            for(int x=0;x<X_RES; x++){
                // our data is stored in RGB order
                data[x][y][0] = Rptr[x]; 
                data[x][y][1] = Gptr[x]; 
                data[x][y][2] = Bptr[x]; 
                data[x][y][3] = IRptr[x];
            } 
        } 
        
        return true;
    }
    
    // return true if data retrieving was successful
    bool readNextLine_RGBD(int ***IMAGE) {
        string line;        
        char* line_c;
        bool file_ended = true;
        
        if (getline(*file_RGBD,line)) {
            file_ended = false;

            line_c = (char*)line.c_str();
            char* element = strtok(line_c, ",");
            if (element == NULL || strcmp(element,"END") == 0) {
                file_ended = true;
                return false;
            }
            currentFrameNum_RGBD = atoi(element);
            if (currentFrameNum != currentFrameNum_RGBD) {
                printf("skeleton: %d rgbd: %d\n", currentFrameNum, currentFrameNum_RGBD);
                errorMsg("FRAME NUMBER BETWEEN SKELETON AND RGBD DOES NOT MATCH!!!!!!!!! (READING RGBD)");
            }
            
            for (int y=0;y<Y_RES;y++) {
                for (int x=0;x<X_RES;x++) {   
                    for (int d = 0; d<RGBD_data; d++) {     
                        
                        element = strtok(NULL, ",");  // passing NULL keeps tokenizing previous call
                        if (element == NULL) {
                            file_ended = true;
                            return false;
                        }
                        int e = atoi(element);
                        
                        if (!mirrored) {
                            IMAGE[x][y][d] = e;
                        } else {
                            IMAGE[x][(Y_RES-1)-y][d] = e; 
                        }
                    }
                }
            }            
            
            // check if there is more data in current frame..
            element = strtok(NULL, ",");  
            if (element != NULL) {
                printf("line_c = %s\n", line_c);
                errorMsg("more data exist in RGBD data ..\n");
                
            }

        } 
        
        return !file_ended;
    }

public:

    

    // return true if data retrieving was successful
    bool readNextFrame(double **data, double **pos_data, int **data_CONF, int *data_pos_CONF, int ***IMAGE) {
        if (currentFrameNum % 100 == 0) {
            printf("\t\t(progress..) frame num = %d\n", currentFrameNum);
        }
        bool status = readNextLine_skeleton(data,pos_data, data_CONF, data_pos_CONF);
        if (!status) {
            printf("\t\ttotal number of frames = %d\n", lastFrame);
            return false;
        }
        
        bool status_RGBD;
        if (!READ_FROM_PNG) {
            status_RGBD = readNextLine_RGBD(IMAGE);
        } else {
            status_RGBD = readNextPNG(IMAGE);    
        }
        if (status_RGBD) {
            lastFrame = currentFrameNum;
        } else {
            printf("\t\ttotal number of frames = %d\n", lastFrame);
        }
        return status_RGBD;
    }

    readData(string dataLoc, string fileN, map<string, string> d_a_map, int i, bool mirrored, string dataLoc_mirrored) {
        if (!mirrored) {
            printf("%d. ", i);    
        } else {
            printf("%d(M). ", i);
        }
        dataLocation = dataLoc;
        dataLocation_mirrored = dataLoc_mirrored;
        fileName = fileN;
        data_act_map = d_a_map;
        this->mirrored = mirrored;
        
        prepareSkeletonData();
        if (!READ_FROM_PNG) {
            prepareRGBDData();
        }
    }
    
    readData() {
    
    }
    
    ~readData(){
        closeSkeletonData();
        if (!READ_FROM_PNG) {
            closeRGBDData();
        }
        printf("\n");
    }

};


