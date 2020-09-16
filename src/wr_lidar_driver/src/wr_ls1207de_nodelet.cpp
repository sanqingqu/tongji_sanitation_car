#include <wr_ls_udp/wr_ls_common_udp.h>
#include <wr_ls_udp/wr_ls1207de_parser.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

namespace wr_ls_udp
{
class WrLs1207DENodelet: public nodelet::Nodelet
{
private:
    boost::shared_ptr<boost::thread> publish_scan_thread_;
    bool enabled_;
public:
    virtual void onInit()
    {
        ROS_INFO("Bringup WrLs1207DE nodelet");
        enabled_ = true;
        publish_scan_thread_ = boost::make_shared<boost::thread>(boost::bind(&WrLs1207DENodelet::publishScan, this));
    }

    void publishScan()
    {
        ros::NodeHandle nh  = getNodeHandle();
        ros::NodeHandle pnh = getPrivateNodeHandle();

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

        /*Setup TCP connection and attempt to connect/reconnect*/
        wr_ls_udp::CWrLsCommon *pWrLs = NULL;
        int result = wr_ls_udp::ExitError;
        while(ros::ok() && enabled_ == true)
        {
            if(pWrLs != NULL)
            {
                delete pWrLs;
            }

            pWrLs = new wr_ls_udp::CWrLsCommonUdp(strHostName, strPort, iTimeLimit, pParser, nh, pnh);
            result = pWrLs->Init();

            /*Device has been initliazed successfully*/
            while(ros::ok() && (result == wr_ls_udp::ExitSuccess) && enabled_ == true)
            {
                ros::spinOnce();
                result = pWrLs->LoopOnce();
            }

            if(result == wr_ls_udp::ExitFatal)
            {
                ROS_ERROR("Grab laser data error:ExitFatal");
                return ;
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
    }

    ~WrLs1207DENodelet()
    {
        enabled_ = false;
        publish_scan_thread_->join();
    }
};
}/*wr_ls_udp*/

// watch the capitalization carefully
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(wr_ls_udp::WrLs1207DENodelet, nodelet::Nodelet)
