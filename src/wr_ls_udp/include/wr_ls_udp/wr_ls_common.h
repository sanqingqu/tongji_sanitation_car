#ifndef WR_LS_COMMON_H_
#define WR_LS_COMMON_H_

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <dynamic_reconfigure/server.h>
#include <wr_ls_udp/WrLsConfig.h>

#include "wr_ls_constants.h"
#include "parser_base.h"

/*Fixed received buffer size*/
#define RECV_BUFFER_SIZE                65536

#define CMD_FRAME_HEADER_START          0

#define CMD_FRAME_HEADER_LENGTH_H       4
#define CMD_FRAME_HEADER_LENGTH_L       5
#define CMD_FRAME_HEADER_CHECK_SUM      6
#define CMD_FRAME_HEADER_TYPE           7
#define CMD_FRAME_HEADER_TOTAL_INDEX_H  8
#define CMD_FRAME_HEADER_TOTAL_INDEX_L  9
#define CMD_FRAME_HEADER_SUB_PKG_NUM    10  /* 包含的子包数目 */
#define CMD_FRAME_HEADER_SUB_INDEX      11
#define CMD_FRAME_DATA_START            12

#define INDEX_RANGE_MAX                 65415       //65535-(20*60)
#define INDEX_RANGE_MIN                 (20*60)     //60 sencod

#define CMD_FRAME_MAX_LEN               1500
#define CMD_FRAME_MAX_SUB_PKG_NUM       4
#define CMD_FRAME_MIN_SUB_PKG_NUM       2

namespace wr_ls_udp
{
class CWrLsCommon
{
    struct dataSaveSt
    {
        unsigned int   totaIndexlCount;
        unsigned char  subPkgNum;
        unsigned char  subPkgIndex;
        unsigned int   rawDataLen;
        unsigned char  sens_data[CMD_FRAME_MAX_LEN];
    };

public:
    CWrLsCommon(CParserBase *parser, ros::NodeHandle &nh, ros::NodeHandle &pnh);
    virtual ~CWrLsCommon();
    virtual int  Init();
    int          LoopOnce();
    void         CheckAngleRange(wr_ls_udp::WrLsConfig &config);
    void         UpdateConfig(wr_ls_udp::WrLsConfig &newConfig, uint32_t level = 0);
    virtual bool RebootDevice();

    double       GetExpectedFreq() const
    {
        return dExpectedFreq;
    }

protected:
    virtual int InitDevice() = 0;
    virtual int InitScanner();
    virtual int StopScanner();
    virtual int CloseDevice() = 0;

    /*Send command/message to the device and print out the response to the console*/
    /**
     * \param [in]  req the command to send
     * \param [out] resp if not NULL, will be filled with the response package to the command.
     */
    virtual int SendDeviceReq(const char *req, std::vector<unsigned char> *resp) = 0;

    /*Read a datagram from the device*/
    /*
     * \param [in]  receiveBuffer data buffer to fill.
     * \param [in]  bufferSize max data size to buffer (0 terminated).
     * \param [out] length the actual amount of data written.
     * */
    virtual int GetDataGram(unsigned char *receiveBuffer, int bufferSize, int *length) = 0;

    /*Helper function. Conver response of SendDeviceReq to string*/
    /*
     * \param [in] resp the response from SendDeviceReq
     * \returns response as string with special characters stripped out
     * */
    static std::string StringResp(const std::vector<unsigned char> &resp);

    /*Check the given device identify is supported by this driver*/
    /*
     * \param [in] strIdentify the identifier of the dvice.
     * \return indicate wether it's supported by this driver
     * */
    bool IsCompatibleDevice(const std::string strIdentify) const;

    void DumpLaserMessage(sensor_msgs::LaserScan &msg);

    void ClearConnectFlag(void);

protected:
    diagnostic_updater::Updater mDiagUpdater;

private:
    ros::NodeHandle mNodeHandler;
    ros::NodeHandle mPrivNodeHandler;
    ros::Publisher  mScanPublisher;
    ros::Publisher  mDataPublisher;

    // Parser
    CParserBase    *mParser;

    bool            mPublishData;
    double          dExpectedFreq;

    struct dataSaveSt mDataSaveSt[CMD_FRAME_MAX_SUB_PKG_NUM];

    unsigned char   mStoreBuffer[RECV_BUFFER_SIZE];
    unsigned char   mRecvBuffer[RECV_BUFFER_SIZE];
    int             mDataLength;

    bool            mConnectFlag;

    // Diagnostics
    diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan> *mDiagPublisher;

    // Dynamic Reconfigure
    WrLsConfig  mConfig;
    dynamic_reconfigure::Server<wr_ls_udp::WrLsConfig> mDynaReconfigServer;
};
} // wr_ls_udp

#endif // WR_LS_COMMON_H_

