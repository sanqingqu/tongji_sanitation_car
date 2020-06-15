#include <wr_ls_udp/wr_ls_sensor_frame.h>
#include <wr_ls_udp/wr_ls_constants.h>

namespace wr_ls_udp
{
CWrLsSensFrame::CWrLsSensFrame()
{
    mSensDataLength = 0;
    m_pSensData  = NULL;
}

CWrLsSensFrame::~CWrLsSensFrame()
{
    if(m_pSensData != NULL)
    {
        delete m_pSensData;
    }
}

uint8_t CWrLsSensFrame::GetFrameHeader()
{
    return m_pSensData->header;
}

uint8_t CWrLsSensFrame::GetCommandId()
{
    return m_pSensData->cmd_id;
}

uint16_t CWrLsSensFrame::GetRangeStart()
{
    return m_pSensData->range_start;
}

uint16_t CWrLsSensFrame::GetRangeEnd()
{
    return m_pSensData->range_end;
}

int  CWrLsSensFrame::GetSensDataCount()
{
    return m_pSensData->range_end - m_pSensData->range_start + 1;
}

uint16_t CWrLsSensFrame::GetSensDataOfIndex(int index)
{
    if(index < 0 || index > (m_pSensData->range_end - m_pSensData->range_start))
    {
        ROS_ERROR("Fail to get of index %d.", index);
        return 0;
    }

    return m_pSensData->sens_data[index];
}

uint16_t CWrLsSensFrame::GetSensIntensityOfIndex(int index)
{
    uint16_t offsetAddr = m_pSensData->range_end - m_pSensData->range_start + 1 + index;

    if(index < 0 || index > (m_pSensData->range_end - m_pSensData->range_start))
    {
        ROS_ERROR("Fail to get of index %d.", index);
        return 0;
    }

    return m_pSensData->sens_data[offsetAddr];
}

bool CWrLsSensFrame::CheckFrame(char *buff, int length, uint8_t value)
{
    int i = 0;
    uint8_t result = 0;
    bool checkframe;

    /* Get configure form launch script */
    ros::param::get("~checkframe", checkframe);
    if (checkframe == false)
    {
        /* Disable check frame function, default check true*/
        return true;
    }

    if (buff == NULL || length <= 0)
    {
        printf("CheckFrame: parameter failed\n");
        return false;
    }

    for (i = 0; i < length; i++)
    {
        result += (*buff++);
    }

    if (result == value)
    {
        return true;
    }

    printf("CheckFrame: check failed, length = %d, result = 0x%X, value = 0x%X\n",
           length, result, value);

    return false;
}

bool CWrLsSensFrame::InitFromSensBuff(char *buff, int length)
{
    if(buff == NULL)
    {
        ROS_ERROR("Invalide input buffer!");
        return false;
    }

    char *pData = new char[length + 100];
    if(pData == NULL)
    {
        ROS_ERROR("Insufficiant memory!");
        return NULL;
    }

    //memcpy(pData, buff, length);
    m_pSensData = new(pData) CWrLsSensFrame::SensData;
    mSensDataLength = length;

    /*System is using LOW END ENCODING, swtich the words*/
    m_pSensData->range_start = 0; //SWITCH_UINT16(m_pSensData->range_start);
    m_pSensData->range_end   = 810; //SWITCH_UINT16(m_pSensData->range_end);

    /*Switch sensor data*/
    int dataCount = this->GetSensDataCount();
    /*       if (true != CheckFrame((char*)m_pSensData->sens_data, dataCount * 2, m_pSensData->check_value))
           {
               ROS_ERROR("CheckFrame failed");
               return false;
           }*/

    if(length == (m_pSensData->range_end - m_pSensData->range_start + 1) * 4)
    {
        dataCount *= 2; /* with intensity data */
    }

    int index = 0;
    uint16_t *tmp_buf;
    tmp_buf = (uint16_t *)buff;
    while(index < dataCount)
    {
        m_pSensData->sens_data[index] = SWITCH_UINT16(tmp_buf[index]);
        index ++;
    }

    return true;
}

void CWrLsSensFrame::DumpFrameHeader()
{
    if(m_pSensData == NULL || mSensDataLength == 0)
    {
        return;
    }

    ROS_DEBUG("Frame Header: 0x%02X", this->GetFrameHeader());
    ROS_DEBUG("Command   ID: 0x%02X", this->GetCommandId());
    ROS_DEBUG("Angle  START: 0x%04X", this->GetRangeStart());
    ROS_DEBUG("Angle    END: 0x%04X", this->GetRangeEnd());
}

void CWrLsSensFrame::DumpFrameData()
{
    if(m_pSensData == NULL || mSensDataLength == 0)
    {
        return;
    }

    int dataCount = this->GetSensDataCount();
    ROS_DEBUG("Data   Count: %d", dataCount);

    int idx = 1;
    while(idx <= dataCount)
    {
        printf("%u ", static_cast<unsigned int>(this->GetSensDataOfIndex(idx - 1)));

        idx++;
        if(idx % 48 == 0)
        {
            printf("\n");
        }
    }
    printf("\n");
}

} /*namespace wr_ls_udp*/


