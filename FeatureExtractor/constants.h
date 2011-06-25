const int SLEEP_TIME = 0;

const int JOINT_NUM = 11;
const int JOINT_DATA_ORI_NUM = 9;
const int JOINT_DATA_POS_NUM = 3;
const int JOINT_DATA_NUM = (JOINT_DATA_ORI_NUM+JOINT_DATA_POS_NUM);
const int JOINT_DATA_TYPE_NUM = 2; // two types : orientation and xyz position

const int TORSO_JOINT_NUM = 2;
const int HEAD_JOINT_NUM = 0;

const int POS_JOINT_NUM = 4;
const int POS_JOINT_DATA_NUM = 3;

const int POS_LEFT_HAND_NUM = 0;
const int POS_RIGHT_HAND_NUM = 1;
const int POS_LEFT_FOOT_NUM = 2;
const int POS_RIGHT_FOOT_NUM = 3;

const int X_RES = 320;
const int Y_RES = 240;
const int RGBD_data = 4;

// 30 fps
const int frameStoreNum = 66;
const int compareFrame[] = {0, -5, -9, -14, -20, -27, -35, -44, -54, -65};
const int compareFrameNum = sizeof(compareFrame)/sizeof(compareFrame[0]);

