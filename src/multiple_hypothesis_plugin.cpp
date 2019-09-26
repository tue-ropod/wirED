#include "multiple_hypothesis_plugin.h"

//#include "wire/core/ClassModel.h"
//#include "wire/core/Property.h"
//#include "wire/core/Evidence.h"
//#include "wire/core/EvidenceSet.h"
//#include "wire/storage/SemanticObject.h"
//#include "wire/core/ClassModel.h"
#include "wire/storage/KnowledgeDatabase.h"
#include "wire/util/ObjectModelParser.h"

#include <iostream>

// ----------------------------------------------------------------------------------------------------

WireED::WireED()
{
}

// ----------------------------------------------------------------------------------------------------

WireED::~WireED()
{
    // delete multiple hypothesis filter_
    delete world_model_;
}

// ----------------------------------------------------------------------------------------------------

void WireED::initialize(ed::InitData& init)
{
    std::cout << "WirED init" << std::endl;
        
    // Try to read the config parameters from the ed config file. Otherwise load default ones which are set below
    tue::Configuration& config = init.config;
    if (!config.value("world_model_frame", world_model_frame_id_, tue::OPTIONAL))
            world_model_frame_id_ = "/map";
        
    if (!config.value("output_frame", output_frame_id_, tue::OPTIONAL))
            output_frame_id_ = "/map";
        
    if (!config.value("max_num_hypotheses", max_num_hyps_, tue::OPTIONAL))
            max_num_hyps_ = 100;
        
    if (!config.value("min_probability_ratio", min_prob_ratio_, tue::OPTIONAL))
            min_prob_ratio_ = 1e-10;
    
    // print init values
    std::cout << "WirED is initialized with the following values:" << std::endl;
    std::cout << "world_model_frame = " << world_model_frame_id_ << std::endl;
    std::cout << "output_frame = " << output_frame_id_ << std::endl;
    std::cout << "max_num_hypotheses = " << max_num_hyps_ << std::endl;
    std::cout << "min_probability_ratio = " << min_prob_ratio_ << std::endl;
    
    std::cout << "\n\nThe following knowledge is loaded: " << std::endl;
    mhf::ObjectModelParser parser(config);
    if (!parser.parse(mhf::KnowledgeDatabase::getInstance())) 
    {
        // parsing failed
        ROS_ERROR_STREAM("While parsing knowledgeDatabase" << std::endl << std::endl << parser.getErrorMessage());
    }
    
    if (config.hasError())
        return;        
    
    
    world_model_ = new mhf::HypothesisTree(max_num_hyps_, min_prob_ratio_);
    
    printCounter_ = 0;
    
    maxLoopDuration_ = 1/this->getLoopFrequency();
}

// ----------------------------------------------------------------------------------------------------

void WireED::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{   
    processEvidence(maxLoopDuration_);
         
    ++printCounter_;
    if (printCounter_ >= 15) {
       showStatistics();
       printCounter_ = 0;
    } 
}

void WireED::processEvidence(const double max_duration) {

    ros::Time start_time = ros::Time::now();
    std::cout << "Start time = " << start_time.toSec()  << std::endl;

    /*while(!evidence_buffer_.empty() && ros::Time::now() - start_time < max_duration) {*/
    ros::Duration d(max_duration);
    
    int testsCounter = 0;
    int maxCounter = 100;
    while( testsCounter < maxCounter && ros::Time::now() - start_time < d) {

        ros::Time time_before_update = ros::Time::now();
 //std::cout << "processEvidence: going to process message with timestamp = " << evidence_buffer_.back().header.stamp << std::endl;
       // processEvidence(evidence_buffer_.back());
          usleep(10000);         //make the programme waiting for 10ms
          std::cout << testsCounter++ << std::endl;

        last_update_duration = (ros::Time::now().toSec() - time_before_update.toSec());
        max_update_duration = std::max(max_update_duration, last_update_duration);

        //evidence_buffer_.pop_back(); // TODO define this evidence buffer
    }
    
    ros::Duration duration = ros::Time::now() - start_time;
    bool timeCheck = ros::Time::now() - start_time < d;
   
    std::cout << "Duration = " << duration.toSec() << " time check = "  << timeCheck << " max_duration = " << max_duration << std::endl;//" evidence buffer size  = " << evidence_buffer_.size() << std::endl;
}

void WireED::showStatistics() const {
    std::printf("***** %f *****\n", ros::Time::now().toSec());
    world_model_->showStatistics();
    std::cout << "Num MAP objects:      " << world_model_->getMAPObjects().size() << std::endl;
    std::cout << "Last update:          " << last_update_duration << " seconds" << std::endl;
    std::cout << "Max update:           " << max_update_duration << " seconds" << std:: endl;
//    cout << "Evidence buffer size: " << evidence_buffer_.size() << endl; // TODO create own datatype for evidence

    /*
    const list<Hypothesis*>& hyp_list = world_model_->getHypotheses();
    for(list<Hypothesis* >::const_iterator it_hyp = hyp_list.begin(); it_hyp != hyp_list.end(); ++it_hyp) {

        const list<SemanticObject*>& objs = (*it_hyp)->getObjects();

        double hyp_prob = (*it_hyp)->getProbability();

        cout << "Hyp P = " << hyp_prob << ": " << objs.size() << " object(s)" << endl;

    }
    */
}

ED_REGISTER_PLUGIN(WireED)
