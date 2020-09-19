#ifndef WR_LS1207DE_PARSER__
#define WR_LS1207DE_PARSER__

#include <wr_ls_udp/parser_base.h>

namespace wr_ls_udp
{
class CWrLs1207DEParser : public CParserBase
{
public:
    CWrLs1207DEParser();
    virtual ~CWrLs1207DEParser();

    virtual int Parse(char *data, size_t data_length, WrLsConfig &config, sensor_msgs::LaserScan &msg);

    void SetRangeMin(float minRange);
    void SetRangeMax(float maxRange);
    void SetTimeIncrement(float time);
    void SetFrameId(std::string frame_id);

private:
    float fRangeMin;
    float fRangeMax;
    float fTimeIncrement;
    std::string fFrame_id;
};
} /*namespace wr_ls_udp*/

#endif /*WR_LS1207DE_PARSER__*/
