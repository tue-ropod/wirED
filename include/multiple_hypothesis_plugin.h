#ifndef ED_WIRED_PLUGIN_H_
#define ED_WIRED_PLUGIN_H_

#include <ed/plugin.h>

#include "wire/core/datatypes.h"

#include "wire/logic/Hypothesis.h"
#include "wire/logic/HypothesesTree.h"

#include "wire_msgs/WorldEvidence.h"
#include "wire_msgs/ObjectEvidence.h"
#include "wire_msgs/ObjectState.h" // TEMP TODO via ED
#include "wire_msgs/WorldState.h"  // TEMP TODO via ED
#include "problib/conversions.h"

#include "wire/storage/KnowledgeDatabase.h"
#include "wire/util/ObjectModelParser.h"
#include "wiredData.h"

#include "wire/core/ClassModel.h"
#include "wire/core/Property.h"
#include "wire/core/Evidence.h"
#include "wire/core/EvidenceSet.h"
#include "wire/storage/SemanticObject.h"
#include "wire/util/ObjectModelParser.h"

#include "wire_state_estimators/featureProperties.h"
#include "featureProperties_info.h"

//ed_multiple_hypothesis_plugin

// transform listener
#include "tf/transform_listener.h"

class Wired : public ed::Plugin
{   

public:
    Wired(tf::TransformListener* tf_listener = 0);

    virtual ~Wired();

    void initialize(ed::InitData& init);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);
    
//    void publish() const; // TODO TEMP
    
    void showStatistics() const;
    
    void processEvidence(const double max_duration, ed::UpdateRequest& req);
    
    void processEvidence(const wire_msgs::WorldEvidence& world_evidence_msg);
    
     const std::list<std::shared_ptr<mhf::SemanticObject>>* getMAPObjects() const;// TODO TEMP?
     
     void swapPointers(std::vector<mhf::ObjectID> **r, std::vector<mhf::ObjectID> **s);
    
protected:
        
     // frame in which objects are tracked and stored in world model
    std::string hypothesisTree_frame_id_;

    // frame in which objects are published
    std::string output_frame_id_;

    int max_num_hyps_;

    double min_prob_ratio_;
    
    // multiple hypothesis filter_
    mhf::HypothesisTree* hypothesisTree_;
    
    int printCounter_;
    
    // transform listener
    tf::TransformListener* tf_listener_;
    bool is_tf_owner_;
    
    // computation time needed for last tree update
    double last_update_duration;
    double max_update_duration;
    
    double maxLoopDuration_;
    
    std::stringstream warnings_;
    
    ros::Time last_update_;
    
    // vector containing all object id's of the previous hypothesis. Used to check if there are objects in the previous  hypothesis which are not in the current one anymore.
    std::vector<mhf::ObjectID>* objectIDs2entitiesPrev_; 
    
    std::vector<mhf::ObjectID>* objectIDs2entities_;
    
    ed::PropertyKey<tracking::FeatureProperties> featureProperties_; 
    
    double object_timeout_;
    
   // world model publishers
 //   ros::Publisher pub_wm_; // TODO TEMP
    
    bool transformPosition(std::shared_ptr<const pbl::PDF> pdf_in, const std::string& frame_in, std::shared_ptr<pbl::Gaussian> pdf_out) const;

    bool transformOrientation(std::shared_ptr<const pbl::PDF> pdf_in, const std::string& frame_in, std::shared_ptr<pbl::Gaussian> pdf_out) const;  
    
  //  bool objectToMsg(const mhf::SemanticObject& obj, wire_msgs::ObjectState& msg) const; // TODO TEMP
    
    bool objectToEntity(const mhf::SemanticObject& obj,ed::UpdateRequest& req) const;

   // bool hypothesis2Msg(const mhf::Hypothesis& hyp, wire_msgs::WorldState& msg) const; // TODO TEMP

    bool hypothesis2Entity(const mhf::Hypothesis& hyp, ed::UpdateRequest& req);
    
    bool object2Entity(const mhf::SemanticObject& obj, ed::UpdateRequest& req) const;
    
    ed::UUID getEntityIDForMHTObject(mhf::ObjectID objectID) const;
    

};

#endif

