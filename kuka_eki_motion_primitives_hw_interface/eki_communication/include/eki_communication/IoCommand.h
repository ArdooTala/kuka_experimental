#pragma once

#include <eki_communication/xml/XmlWriter.h>

namespace rbt
{
class IoCommand
{
public:
    IoCommand() {}
    IoCommand(int sig1_value, int sig1_mask, int sig2_value, int sig2_mask)
        : sig1_value(sig1_value), sig1_mask(sig1_mask), sig2_value(sig2_value), sig2_mask(sig2_mask) {}
    ~IoCommand() {}

    int sig1_value = 0;
    int sig1_mask = 0;
    int sig2_value = 0;
    int sig2_mask = 0;

    void to_xml(XmlWriter &writer) const;
};
} // namespace rbt
