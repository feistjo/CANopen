#pragma once

#include "can_interface.h"
#include <array>

class CANopenTXMessage
{
public:
    CANopenTXMessage(ICAN &can_interface) : can_interface_{can_interface} {}

    enum class FunctionCode : uint16_t
    {
        kNMTNodeControl = 0,
        kGlobalFailsafeCommand = 0x1,
        kSync = 0x80,
        kEmergency = 0x80,
        kTimestamp = 0x100,
        kTransmitPDO1 = 0x180,
        kReceivePDO1 = 0x200,
        kTransmitPDO2 = 0x180,
        kReceivePDO2 = 0x200,
        kTransmitPDO3 = 0x180,
        kReceivePDO3 = 0x200,
        kTransmitPDO4 = 0x180,
        kReceivePDO4 = 0x200,
        kTransmitSDO = 0x580, // transmitted by client
        kReceiveSDO = 0x600,  // transmitted by server/master
        kNMTNodeMonitoring = 0x700,
        kTransmitLSS = 0x7E4,
        kReceiveLSS = 0x7E5
    };

    enum class CCS : uint8_t
    {
        kSegmentDownload = 0,
        kInitiateDownload = 1, // download from client/master to server
        kInitiateUpload = 2,   // upload request from server to client/master
        kSegmentUpload = 3,
        kAbortTransfer = 4,
        kBlockUpload = 5,
        kBlockDownload = 6
    };

    void
    SendExpeditedSDO(uint8_t node_id, CCS request_type, uint16_t od_index, uint8_t od_subindex, uint8_t *data, uint8_t length)
    {
        message.id_ = (uint16_t)FunctionCode::kReceiveSDO + node_id;
        message.data_[0] = (uint8_t)request_type << 5 | (4 - length) << 2 | 0b11;
        message.data_[1] = ((uint8_t *)&od_index)[0];
        message.data_[2] = ((uint8_t *)&od_index)[1];
        message.data_[3] = od_subindex;
        for (int i = 0; i < 4; i++)
        {
            message.data_[4 + i] = i < length ? data[i] : 0;
        }

        can_interface_.SendMessage(message);
    }

private:
    const uint16_t kBaseID = 0x180;
    ICAN &can_interface_;
    CANMessage message{(uint16_t)FunctionCode::kReceiveSDO, 8, std::array<uint8_t, 8>()};
};