#include "dly_camera.h"

using namespace sensor::camera;

dly_camera::dly_camera(/* args */)
{
}

dly_camera::~dly_camera()
{
}

//初始化相机的SDK库
bool dly_camera::InitCameraLib()
{
    GX_STATUS emStatus = GX_STATUS_SUCCESS;
    emStatus = GXInitLib();
    if (emStatus != GX_STATUS_SUCCESS)
    {
        return false;
        iLastError = 10001;
    }
    else
    {
        return true;
    }
}

//关闭相机的SDK库
bool dly_camera::CloseLib()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXCloseDevice(m_hDevice);
    if (status == GX_STATUS_SUCCESS)
    {
        return true;
    }
    else
    {
        return false;
        iLastError = 00000;
    }
}

//开启相机
bool dly_camera::OpenCamera()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t nDeviceNum = 0;
    GX_OPEN_PARAM stOpenParam;

    status = GXUpdateDeviceList(&nDeviceNum, 1000);
    if (status == GX_STATUS_SUCCESS && nDeviceNum > 0)
    {
        GX_DEVICE_IP_INFO stIPInfo;
        //获取第一台设备的网络信息
        status = GXGetDeviceIPInfo(1, &stIPInfo);
        if (status != GX_STATUS_SUCCESS)
        {
            return false;
            iLastError = 10003;
        }

        stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE; //访问设备的方式: 只读、控制、独占等
        stOpenParam.openMode = GX_OPEN_INDEX;         //通过枚举序号打开
        stOpenParam.pszContent = "1";                 //枚举序号

        status = GXOpenDevice(&stOpenParam, &m_hDevice);
        if (status == GX_STATUS_SUCCESS)
        {
            return true;
            m_bOpen = true;
        }
        else
        {
            return false;
            iLastError = 10004;
        }
    }
    else
    {
        return false;
        iLastError = 10002;
    }
}

//获取图像
cv::Mat dly_camera::GetImg()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    PGX_FRAME_BUFFER pFrameBuffer;
    if (m_bOpen == true)
    {
        status = GXStreamOn(m_hDevice);
        if (status == GX_STATUS_SUCCESS)
        {
            status = GXDQBuf(m_hDevice, &pFrameBuffer, 1000);
            if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS)
            {
                // QImage img = ShowPic(pFrameBuffer);
                int width = pFrameBuffer->nWidth;
                int height = pFrameBuffer->nHeight;

                cv::Mat img(cv::Size(width, height), CV_8UC1, (void *)pFrameBuffer->pImgBuf, cv::Mat::AUTO_STEP);

                status = GXQBuf(m_hDevice, pFrameBuffer);
                if (status != GX_STATUS_SUCCESS)
                {
                    iLastError = 10008;
                }
                return img;
            }
            else
            {
                iLastError = 10007;
                return;
            }
        }
        else
        {
            iLastError = 10006;
            return;
        }
    }
    else
    {
        iLastError = 10005;
        return;
    }
}
bool dly_camera::GetImg(cv::Mat pic)
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    PGX_FRAME_BUFFER pFrameBuffer;
    if (m_bOpen == true)
    {
        status = GXStreamOn(m_hDevice);
        if (status == GX_STATUS_SUCCESS)
        {
            status = GXDQBuf(m_hDevice, &pFrameBuffer, 1000);
            if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS)
            {
                int width = pFrameBuffer->nWidth;
                int height = pFrameBuffer->nHeight;

                cv::Mat img(cv::Size(width, height), CV_8UC1, (void *)pFrameBuffer->pImgBuf, cv::Mat::AUTO_STEP);
                pic = img;

                status = GXQBuf(m_hDevice, pFrameBuffer);
                if (status != GX_STATUS_SUCCESS)
                {
                    iLastError = 10008;
                }

                return true;
            }
            else
            {
                iLastError = 10007;
                return false;
            }
        }
        else
        {
            iLastError = 10006;
            return false;
        }
    }
    else
    {
        iLastError = 10005;
        return false;
    }
}

//获取最后一个错误
unsigned int dly_camera::GetLastError()
{
    return iLastError;
}