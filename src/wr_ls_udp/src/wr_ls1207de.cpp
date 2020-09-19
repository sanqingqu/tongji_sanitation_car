#include <wr_ls_udp/wr_ls_common_udp.h>
#include <wr_ls_udp/wr_ls1207de_parser.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wr_ls1207de");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    /*Check whether hostname is provided*/
    bool isTcpConnection = false;
    std::string strHostName;
    std::string strPort;
    if(pnh.getParam("hostname", strHostName))
    {
        isTcpConnection = true;
        pnh.param<std::string>("port", strPort, "2112");
    }

    /*Get configured time limit*/
    int iTimeLimit = 5;
    pnh.param("timelimit", iTimeLimit, 5);

    bool isDataSubscribed = false;
    pnh.param("subscribe_datagram", isDataSubscribed, false);

    int iDeviceNumber = 0;
    pnh.param("device_number", iDeviceNumber, 0);

    /*Create and initialize parser*/
    wr_ls_udp::CWrLs1207DEParser *pParser = new wr_ls_udp::CWrLs1207DEParser();
    double param;
    std::string frame_id;

    if(pnh.getParam("range_min", param))
    {
        ROS_INFO("range_min: %f", param);
        pParser->SetRangeMin(param);
    }
    if(pnh.getParam("range_max", param))
    {
        ROS_INFO("range_max: %f", param);
        pParser->SetRangeMax(param);
    }
    if(pnh.getParam("time_increment", param))
    {
        ROS_INFO("time_increment: %f", param);
        pParser->SetTimeIncrement(param);
    }

    if(pnh.getParam("frame_id", frame_id))
    {
        ROS_INFO("frame_id: %s", frame_id.c_str());
        pParser->SetFrameId(frame_id);
    }


    /*Setup UDP connection and attempt to connect/reconnect*/
    wr_ls_udp::CWrLsCommon *pWrLs = NULL;
    int result = wr_ls_udp::ExitError;
    while(ros::ok())
    {
        if(pWrLs != NULL)
        {
            delete pWrLs;
        }

        pWrLs = new wr_ls_udp::CWrLsCommonUdp(strHostName, strPort, iTimeLimit, pParser, nh, pnh);
        result = pWrLs->Init();

        /*Device has been initliazed successfully*/
        while(ros::ok() && (result == wr_ls_udp::ExitSuccess))
        {
            ros::spinOnce();
            result = pWrLs->LoopOnce();
        }

        if(result == wr_ls_udp::ExitFatal)
        {
            return result;
        }
    }

    if(pWrLs != NULL)
    {
        delete pWrLs;
    }

    if(pParser != NULL)
    {
        delete pParser;
    }

    return result;
}
