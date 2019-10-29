#include "multiple_hypothesis_plugin.h"

#include "wire_msgs/WorldEvidence.h"
#include "wire/storage/KnowledgeDatabase.h"
#include "wire/util/ObjectModelParser.h"
#include "wiredData.h"

#include <iostream>
#include <typeinfo>

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
    // Try to read the config parameters from the ed config file. Otherwise load default ones which are set below
    tue::Configuration& config = init.config;
    std::string bufferName;
    int bufferSize;
      
    if (!config.value("world_model_frame", world_model_frame_id_, tue::OPTIONAL)) { world_model_frame_id_ = "/map";}
    if (!config.value("output_frame", output_frame_id_, tue::OPTIONAL))           { output_frame_id_ = "/map";}
    if (!config.value("max_num_hypotheses", max_num_hyps_, tue::OPTIONAL))        { max_num_hyps_ = 100;}
    if (!config.value("min_probability_ratio", min_prob_ratio_, tue::OPTIONAL))   { min_prob_ratio_ = 1e-10;}
    if (!config.value("bufferName", bufferName, tue::OPTIONAL))                   { bufferName = "MHT-buffer";}
    if (!config.value("bufferSize", bufferSize, tue::OPTIONAL))                   { bufferSize = 100;}
        
    // print init values
    std::cout << "WirED is initialized with the following values:" << std::endl;
    std::cout << "world_model_frame = " << world_model_frame_id_ << std::endl;
    std::cout << "output_frame = " << output_frame_id_ << std::endl;
    std::cout << "max_num_hypotheses = " << max_num_hyps_ << std::endl;
    std::cout << "min_probability_ratio = " << min_prob_ratio_ << std::endl;
    
    std::cout << "\nThe following knowledge is loaded: " << std::endl;
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
    
    if(newBufferDesired( bufferName) )
    {
            createDatabuffer(boost::make_shared< wiredDataBuffer>(bufferSize) , bufferName );
    } 

    boost::shared_ptr<wiredDataBuffer> buf = boost::static_pointer_cast<wiredDataBuffer>( getDataBuffer() );
    
    std::cout << "Wired init: getDataBuffer() = " << getDataBuffer() << std::endl;
    
    // TEMP, some testdata  
/*        wire_msgs::WorldEvidence world_evidence;
    for (unsigned int i = 0; i < 100; i++) // to test
    {
            world_evidence.header.seq = 2*i;
            buf->getDataDerived().push_front(world_evidence);
            std::cout << "data_buf has remaining space = " << buf->getDataDerived().is_not_full() << " i = " << i << std::endl;
    }
  */  
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
        
          usleep(10000);         //make the programme waiting for a bit. Here the real stuff is supposed to happen.
          
         boost::shared_ptr<wiredDataBuffer> p_test =boost::static_pointer_cast<wiredDataBuffer>( getDataBuffer() );
          wire_msgs::WorldEvidence world_evidence;
          std::cout << "wired: going to pop_back" << std::endl;
          std::cout << "p_test = " << p_test << std::endl;          
          
          if( p_test->getDataDerived().is_not_empty() ) // This ensures that we do not wait for until new data are published
          {
                  p_test->getDataDerived().pop_back(&world_evidence);
                  std::cout << "stamp test " << world_evidence.header.stamp  << ", seq test = " <<  world_evidence.header.seq << std::endl; // First in First Out policy
          }
          
          
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
