#pragma once

#include <array>
#include <map>

#include "can_interface.h"

class CANopenSDO
{
public:
    CANopenSDO(ICAN &can_interface, uint8_t node_id, std::map<uint32_t, uint8_t *> data_map)
        : can_interface_{can_interface},
          sdo_message_{static_cast<uint16_t>(FunctionCode::kReceiveSDO) + node_id, 8, std::array<uint8_t, 8>()},
          sdo_response_message_{can_interface_,
                                static_cast<uint16_t>(FunctionCode::kTransmitSDO) + node_id,
                                [this]() { SDORXCallback(); },
                                sdo_response_signal_},
          data_map_{data_map}
    {
    }

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
        kTransmitSDO = 0x580,  // transmitted by client
        kReceiveSDO = 0x600,   // transmitted by server/master
        kNMTNodeMonitoring = 0x700,
        kTransmitLSS = 0x7E4,
        kReceiveLSS = 0x7E5
    };

    enum class CCS : uint8_t
    {
        kSegmentDownload = 0,
        kInitiateDownload = 1,  // download from client/master to server
        kInitiateUpload = 2,    // upload request from server to client/master
        kSegmentUpload = 3,
        kAbortTransfer = 4,
        kBlockUpload = 5,
        kBlockDownload = 6
    };

    void SendExpeditedSDO(CCS request_type, uint16_t od_index, uint8_t od_subindex, uint8_t *data, uint8_t length)
    {
        // sdo_message_.id_ = (uint16_t)FunctionCode::kReceiveSDO + node_id;
        sdo_message_.data_[0] = (uint8_t)request_type << 5 | (4 - length) << 2 | 0b11;
        sdo_message_.data_[1] = ((uint8_t *)&od_index)[0];
        sdo_message_.data_[2] = ((uint8_t *)&od_index)[1];
        sdo_message_.data_[3] = od_subindex;
        for (int i = 0; i < 4; i++)
        {
            sdo_message_.data_[4 + i] = i < length ? data[i] : 0;
        }

        can_interface_.SendMessage(sdo_message_);
    }

    void SDORXCallback()
    {
        uint8_t *message_data_pointer = reinterpret_cast<uint8_t *>(&(sdo_response_signal_.value_ref()));
        CCS ccs = static_cast<CCS>(message_data_pointer[0] >> 5);
        uint8_t data_len = 4 - (message_data_pointer[0] >> 2 & 0b11);
        bool expedited = message_data_pointer[0] >> 1 & 0b1;
        uint16_t od_index = message_data_pointer[1] | message_data_pointer[2] << 8;
        uint8_t od_subindex = message_data_pointer[3];

        auto data_map_value = data_map_.find(IndexesToUint32_t(od_index, od_subindex));
        if (data_map_value == data_map_.end())
        {
            // Data not found, ignore
            return;
        }

        uint8_t *storage_data_pointer = data_map_value->second;
        if (expedited)
        {
            for (int i = 0; i < data_len; i++)
            {
                storage_data_pointer[i] = message_data_pointer[i];
            }
        }
        else
        {
            // unexpedited not yet implemented
        }
    }

    static inline uint32_t IndexesToUint32_t(uint16_t od_index, uint8_t od_subindex)
    {
        return od_index << 8 & od_subindex;
    }

private:
    ICAN &can_interface_;
    CANMessage sdo_message_;
    MakeUnsignedCANSignal(uint64_t, 0, 8, 1, 0) sdo_response_signal_;
    CANRXMessage<1> sdo_response_message_;
    std::map<uint32_t, uint8_t *> data_map_;
};