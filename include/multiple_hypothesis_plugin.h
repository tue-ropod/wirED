#ifndef ED_WIRED_PLUGIN_H_
#define ED_WIRED_PLUGIN_H_

#include <ed/plugin.h>

#include "wire/logic/HypothesesTree.h"

//#include <memory>

class WireED : public ed::Plugin
{   
  /*  class HypothesisTree;
    class Hypothesis;
    class KnowledgeProbModel;
    class PropertySet;
    class ClassModel;
    class SemanticObject;
    */  
public:

    WireED();

    virtual ~WireED();

    void initialize(ed::InitData& init);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);
    
    void showStatistics() const;
    
    void processEvidence(const double max_duration);

private:
        
     // frame in which objects are tracked and stored in world model
    std::string world_model_frame_id_;

    // frame in which objects are published
    std::string output_frame_id_;

    int max_num_hyps_;

    double min_prob_ratio_;
    
    // multiple hypothesis filter_
    mhf::HypothesisTree* world_model_;
    
    int printCounter_;
    
    // computation time needed for last tree update
    double last_update_duration;
    double max_update_duration;
    
    double maxLoopDuration_;
    
};

#endif

