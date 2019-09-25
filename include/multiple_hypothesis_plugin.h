#ifndef ED_WIRED_PLUGIN_H_
#define ED_WIRED_PLUGIN_H_

#include <ed/plugin.h>

//#include <memory>

class WireED : public ed::Plugin
{

public:

    WireED();

    virtual ~WireED();

    void initialize(ed::InitData& init);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:
        
     // frame in which objects are tracked and stored in world model
    std::string world_model_frame_id_;

    // frame in which objects are published
    std::string output_frame_id_;

    int max_num_hyps_;

    double min_prob_ratio_;


};

#endif

