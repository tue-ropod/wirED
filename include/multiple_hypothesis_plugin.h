#ifndef ED_WIRED_PLUGIN_H_
#define ED_WIRED_PLUGIN_H_

#include <ed/plugin.h>

class WireED : public ed::Plugin
{

public:

    WireED();

    virtual ~WireED();

    void initialize(ed::InitData& init);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    std::string text_;

};

#endif

