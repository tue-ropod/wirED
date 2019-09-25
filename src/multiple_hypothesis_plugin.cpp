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
    
   mhf::ObjectModelParser parser(config);
   
    if (!parser.parse(mhf::KnowledgeDatabase::getInstance())) 
    {
        // parsing failed
 //       ROS_ERROR_STREAM("While parsing '" << object_models_filename << "': " << endl << endl << parser.getErrorMessage());
            ROS_ERROR_STREAM("While parsing knowledgeDatabase" << std::endl << std::endl << parser.getErrorMessage());
 //       return false;
    }
std::cout << "Parsing completed" << std::endl;
    /*
     if (config.readArray("plugins"))
    {
        while(config.nextArrayItem())
        {
            std::string name;
            if (!config.value("name", name))
                return;

    
    object_class: 
            behavior_model: 
              attribute: "position"
              model: "wire_state_estimators/PositionEstimator"
              pnew: 
                type: "uniform"
                dimensions: 3
                density: "0.0001"
              pclutter: 
                type: "uniform"
                dimensions: 3
                density: "0.0001"
              parameters: 
                max_acceleration: 8
                kalman_timeout: 1
                fixed_pdf_cov: 0.008
      
    */
    if (config.hasError())
        return;

    // print init values
    std::cout << "WirED is initialized with the following values:" << std::endl;
    std::cout << "world_model_frame = " << world_model_frame_id_ << std::endl;
    std::cout << "output_frame = " << output_frame_id_ << std::endl;
    std::cout << "max_num_hypotheses = " << max_num_hyps_ << std::endl;
    std::cout << "min_probability_ratio = " << min_prob_ratio_ << std::endl;
        
}

// ----------------------------------------------------------------------------------------------------

void WireED::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
/*
         processEvidence(r.expectedCycleTime());
        publish();
        ++count;
        if (count == 15) {
            //showStatistics();
            count = 0;
        }
  */      
}

ED_REGISTER_PLUGIN(WireED)
