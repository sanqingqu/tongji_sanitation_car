#ifndef PARSER_BASE__
#define PARSER_BASE__

#include "wr_ls_udp/WrLsConfig.h"
#include "sensor_msgs/LaserScan.h"

namespace wr_ls_udp
{
enum ExitCode
{
    ExitSuccess = 0,
    ExitError   = 1,
    ExitFatal   = 2
};

class CParserBase
{
public:
    CParserBase();
    virtual ~CParserBase();

    virtual int Parse(char *data,
                      size_t data_length,
                      WrLsConfig &config,
                      sensor_msgs::LaserScan &msg) = 0;
};
} /*namespace wr_ls_udp*/

#endif /*PARSER_BASE__*/
