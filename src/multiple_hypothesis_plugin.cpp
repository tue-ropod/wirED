#include "multiple_hypothesis_plugin.h"

#include <iostream>

// ----------------------------------------------------------------------------------------------------

WireED::WireED()
{
}

// ----------------------------------------------------------------------------------------------------

WireED::~WireED()
{
}

// ----------------------------------------------------------------------------------------------------

void WireED::initialize(ed::InitData& init)
{
    init.config.value("text", text_);
}

// ----------------------------------------------------------------------------------------------------

void WireED::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    std::cout << text_ << std::endl;
}

ED_REGISTER_PLUGIN(WireED)
