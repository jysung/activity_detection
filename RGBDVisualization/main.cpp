// Author: Jae Yong Sung

// opencv
#include <cv.h>
#include <highgui.h>

#include <cstdio>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <dirent.h>

using namespace std;

const int MAX_DEPTH = 20000; // in milimeters

const int X_RES = 320;
const int Y_RES = 240;
const int DATAPP = 4;   // Data Per Pixel: R,G,B,D

char* dataFile;         // includes path 
string dataFile_name;   // just name extracted (also does not include extension)

int curFile_frameNum;

// output to PNG file
void storeRGBDImage(unsigned int data[X_RES][Y_RES][DATAPP]) {
    cv::Mat colorArr[3]; 
    cv::Mat colorImage; 
    cv::Mat irImage; 
    
    colorArr[0] = cv::Mat(Y_RES,X_RES,CV_8U);
    colorArr[1] = cv::Mat(Y_RES,X_RES,CV_8U);
    colorArr[2] = cv::Mat(Y_RES,X_RES,CV_8U);    
    irImage = cv::Mat(Y_RES,X_RES,CV_8U);
    
    for (int y=0; y<Y_RES; y++){ 
        // opencv uses BGR order
        uchar* Bptr = colorArr[0].ptr<uchar>(y); 
        uchar* Gptr = colorArr[1].ptr<uchar>(y); 
        uchar* Rptr = colorArr[2].ptr<uchar>(y); 
        uchar* IRptr = irImage.ptr<uchar>(y); 
        for(int x=0;x<X_RES; x++){
            // our data is stored in RGB order
            Rptr[x] = data[x][y][0]; 
            Gptr[x] = data[x][y][1]; 
            Bptr[x] = data[x][y][2]; 
            IRptr[x] = data[x][y][3]*255/MAX_DEPTH;
            if (IRptr[x]>255) {
                IRptr[x]=255;
            }
        } 
    } 
    cv::merge(colorArr,3,colorImage); 

    std::stringstream ss;
    ss << curFile_frameNum;
    string filename = "out/" + dataFile_name + "/" + dataFile_name + "_" + ss.str() + ".png";
    string filename_ir = "out/" + dataFile_name + "/" + dataFile_name + "_" + ss.str() + "_ir.png";
    cv::imwrite(filename, colorImage);
    cv::imwrite(filename_ir, irImage);
}

// read RGBD CSV file
void readDataFile(const char* fileName) {
    printf("reading and converting to PNG files ..\n");
    ifstream file(fileName, ifstream::in);
    
    string line;
    bool file_ended = false;
    curFile_frameNum = 0;
    
    while(!file_ended && getline(file,line)) {
        stringstream lineStream(line);
        string element;
        unsigned int data[X_RES][Y_RES][DATAPP];
        
        if (getline(lineStream, element, ',')) {
            if (element.compare("END") == 0) {
                file_ended = true;
                break;
            }
            curFile_frameNum = atoi((char*)element.c_str());
            
            for (int y=0;y<Y_RES;y++) {
                for (int x=0;x<X_RES;x++) {
                    for (int z=0;z<DATAPP;z++) {
                        if (getline(lineStream, element, ',')) {
                            data[x][y][z] = (unsigned int) atoi((char*)element.c_str());
                            if (z != 3 && (data[x][y][z] > 255 || data[x][y][z] < 0)) {
                                printf("ERROR: DATA CORRUPTED??  \tx:%d y:%d z:%d  data: %d(%s)\n", x,y,z, data[x][y][z], (char*) element.c_str());
                            } 
                        } else {
                            printf("ERROR: THIS SHOULDN'T HAPPEN... SOMETHING IS WRONG..");
                            printf("\tx:%d y:%d z:%d\n", x,y,z);
                            exit(1);
                            
                        }
                    }
                }
            }
            int c=0;
            while(getline(lineStream, element, ',')){
                c++;
            }
            if (c>0) {
                printf("ERROR: Is this really RGBD file? Seems file is corrupted..\n");
                exit(1);
            }
            
            storeRGBDImage(data);
        }
    }    
        
    file.close();
}

int main(int argc, char* argv[]) {
    if (argc == 2) {
        dataFile = argv[1];
    } else {
       cout << "USAGE ERROR!" << endl;
       exit(1);        
    }
    
    printf("\nRaw RGBD File: \t%s\n", dataFile);
    
    // extract filename
    dataFile_name = (string) dataFile;
    int temp1 = dataFile_name.rfind('/') + 1;
    int temp2 = dataFile_name.rfind(".txt");
    dataFile_name = dataFile_name.substr(temp1, temp2-temp1);
    printf("File name: \t%s\n", (char*)dataFile_name.c_str());
    
    // create output directory
    string mkdir = "mkdir -p ./out/" + dataFile_name;
    if (system((char*)mkdir.c_str()) != 0) {
        exit(1);
    }
    printf("PNG Files: \tout/%s/\n\n", (char*)dataFile_name.c_str());

    
    // read and convert to PNG and depth only log
    readDataFile(dataFile);
    
    printf("DONE.\n");
    return 0;
}


