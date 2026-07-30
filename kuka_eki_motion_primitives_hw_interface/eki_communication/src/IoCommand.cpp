#include <eki_communication/IoCommand.h>

void rbt::IoCommand::to_xml(XmlWriter &writer) const
{
    writer.open_element("IO");
    writer.open_element("DO");

    writer.add_element("Sig1", {{"Value", std::to_string(sig1_value)},
                                {"Mask", std::to_string(sig1_mask)}});

    writer.add_element("Sig2", {{"Value", std::to_string(sig2_value)},
                                {"Mask", std::to_string(sig2_mask)}});

    writer.close_element();
    writer.close_element();
}
