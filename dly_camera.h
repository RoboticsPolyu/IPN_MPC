#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include "DxImageProc.h"
#include "GxIAPI.h"

using namespace std;

namespace sensor
{
    namespace camera
    {
        class dly_camera
        {
        private:
            GX_DEV_HANDLE m_hDevice;

            unsigned int iLastError;
            bool m_bOpen;

        public:
            dly_camera(/* args */);
            ~dly_camera();

            bool InitCameraLib();
            bool CloseLib();
            bool OpenCamera();
            bool CloseCamera();
            cv::Mat GetImg();
            bool GetImg(cv::Mat pic);
            unsigned int GetLastError();
        };
    }
}