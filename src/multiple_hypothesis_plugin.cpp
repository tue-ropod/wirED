#include "multiple_hypothesis_plugin.h"
//#include "wire_core/include/wire/WorldModelROS.h"



#include <iostream>
#include <typeinfo>

// ----------------------------------------------------------------------------------------------------

WireED::WireED(tf::TransformListener* tf_listener) :   tf_listener_(tf_listener), is_tf_owner_(false)
{
}

// ----------------------------------------------------------------------------------------------------

WireED::~WireED()
{
    // delete multiple hypothesis filter_
        if(hypothesisTree_)
        {
                delete hypothesisTree_;
        }
}

// ----------------------------------------------------------------------------------------------------

void WireED::initialize(ed::InitData& init)
{        
    // Try to read the config parameters from the ed config file. Otherwise load default ones which are set below
    tue::Configuration& config = init.config;
    std::string bufferName;
    int bufferSize;
      
    if (!config.value("hypothesisTree_frame", hypothesisTree_frame_id_, tue::OPTIONAL)) { hypothesisTree_frame_id_ = "/map";}
    if (!config.value("output_frame", output_frame_id_, tue::OPTIONAL))           { output_frame_id_ = "/map";}
    if (!config.value("max_num_hypotheses", max_num_hyps_, tue::OPTIONAL))        { max_num_hyps_ = 100;}
    if (!config.value("min_probability_ratio", min_prob_ratio_, tue::OPTIONAL))   { min_prob_ratio_ = 1e-10;}
    if (!config.value("bufferName", bufferName, tue::OPTIONAL))                   { bufferName = "MHT-buffer";}
    if (!config.value("bufferSize", bufferSize, tue::OPTIONAL))                   { bufferSize = 100;}
        
    // print init values
    std::cout << "WirED is initialized with the following values:" << std::endl;
    std::cout << "hypothesisTree_frame = " << hypothesisTree_frame_id_ << std::endl;
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
    
    // Publishers
    ros::NodeHandle n("~");
    pub_wm_ = n.advertise<wire_msgs::WorldState>("/world_state", 10); // TEMP TODO via ED
    
    if (config.hasError())
        return;      
    
    // create tf listener
    if (!tf_listener_) {
        tf_listener_ = new tf::TransformListener();
        is_tf_owner_ = true;
    }
    
    hypothesisTree_ = new mhf::HypothesisTree(max_num_hyps_, min_prob_ratio_);
    printCounter_ = 0;
    maxLoopDuration_ = 1/this->getLoopFrequency();
    
    if(newBufferDesired( bufferName) )
    {
            createDatabuffer(boost::make_shared< wiredDataBuffer>(bufferSize) , bufferName );
    } 

    boost::shared_ptr<wiredDataBuffer> buf = boost::static_pointer_cast<wiredDataBuffer>( getDataBuffer() ); 
}

// ----------------------------------------------------------------------------------------------------

void WireED::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{   
        std::cout << "WIRED: process" << std::endl;
    processEvidence(maxLoopDuration_);
         
    ++printCounter_;
    if (printCounter_ >= 15) {
       showStatistics();
       printCounter_ = 0;
    } 
}

void WireED::processEvidence(const double max_duration) {

    ros::Time start_time = ros::Time::now();
//    std::cout << "Start time = " << start_time.toSec()  << std::endl;

    ros::Duration d(max_duration);
    
    boost::shared_ptr<wiredDataBuffer> wiredBuffer =boost::static_pointer_cast<wiredDataBuffer>( getDataBuffer() );
    wire_msgs::WorldEvidence world_evidence;
          
    while( wiredBuffer->getBuffer().is_not_empty() && ros::Time::now() - start_time < d)  // This ensures that we do not wait for until new data are published
    { 
        ros::Time time_before_update = ros::Time::now();     
        
        std::cout << "wirebuffer: has data = " <<wiredBuffer->getBuffer().is_not_empty() << std::endl;
        wiredBuffer->getBuffer().pop_back(&world_evidence);        
        processEvidence(world_evidence);
          
        last_update_duration = (ros::Time::now().toSec() - time_before_update.toSec());
        max_update_duration = std::max(max_update_duration, last_update_duration);

        //evidence_buffer_.pop_back(); // TODO define this evidence buffer
    }
    
    ros::Duration duration = ros::Time::now() - start_time;
    bool timeCheck = ros::Time::now() - start_time < d;
    
    publish();
   
    //std::cout << "Duration = " << duration.toSec() << " time check = "  << timeCheck << " max_duration = " << max_duration << std::endl;//" evidence buffer size  = " << evidence_buffer_.size() << std::endl;
}

void WireED::processEvidence(const wire_msgs::WorldEvidence& world_evidence_msg) {
    ros::Time current_time = ros::Time::now();

    if (current_time < last_update_) {
        ROS_WARN("Saw a negative time change of %f seconds; resetting the world model.", (current_time - last_update_).toSec());
        delete hypothesisTree_;
        hypothesisTree_ = new mhf::HypothesisTree(max_num_hyps_, min_prob_ratio_);
    }
    last_update_ = current_time;

    // reset the warnings stringstream
    warnings_.str("");

    mhf::EvidenceSet evidence_set;
    std::list<mhf::Evidence*> measurements_mem;

    const std::vector<wire_msgs::ObjectEvidence>& object_evidence = world_evidence_msg.object_evidence;
    
    for(std::vector<wire_msgs::ObjectEvidence>::const_iterator it_ev = object_evidence.begin(); it_ev != object_evidence.end(); ++it_ev) {
        const wire_msgs::ObjectEvidence& evidence = (*it_ev);

        mhf::Evidence* meas = new mhf::Evidence(current_time.toSec()); // measurement/evidence set to timestamp at the start of processEvidence!
         
       // std::cout << "Meas: " << meas->toString() << std::endl;
        
        measurements_mem.push_back(meas);

        bool position_ok = true;

        for(std::vector<wire_msgs::Property>::const_iterator it_prop = evidence.properties.begin(); it_prop != evidence.properties.end(); ++it_prop ) {
            const wire_msgs::Property& prop = *it_prop;

            std::shared_ptr<pbl::PDF> pdf = pbl::msgToPDF(prop.pdf);
            
      //       std::cout << "wired: prop.attribute = " << prop.attribute << " pdf = " << prop.pdf << std::endl;

            if (pdf) {
                if (prop.attribute == "position") {
                        std::shared_ptr<pbl::Gaussian> pos_pdf = std::make_shared<pbl::Gaussian>(3);
                    if (!transformPosition(pdf, world_evidence_msg.header.frame_id, pos_pdf)) {
                        // position is a necessary property. If the transform was not successful, abort and don't use the evidence
                        position_ok = false;
                        break;
                    } else {
                        meas->addProperty(mhf::AttributeConv::attribute(prop.attribute), *pos_pdf);
                    }
                } else if (prop.attribute == "orientation") {
                    std::shared_ptr<pbl::Gaussian> ori_pdf = std::make_shared<pbl::Gaussian>(4);
                    if (!transformOrientation(pdf, world_evidence_msg.header.frame_id, ori_pdf)) {
                        meas->addProperty(mhf::AttributeConv::attribute(prop.attribute), *ori_pdf);
                    }
                } else {
                    meas->addProperty(mhf::AttributeConv::attribute(prop.attribute), *pdf);
                }
                //delete pdf;
            } else {
                ROS_ERROR_STREAM("For attribute '" << prop.attribute << "': malformed pdf: " << prop.pdf);
            }
        }

        if (position_ok) {                
            evidence_set.add(meas);
        } else {
            ROS_ERROR("Unable to transform position.");
        }

    } // end iteration over object evidence list

    
    hypothesisTree_->addEvidence(evidence_set); // Here, we process the evidence in the HypothesisTree having current_time (the time at which we started the process evidence)
    // as variable

    for(std::list<mhf::Evidence*>::iterator it = measurements_mem.begin(); it != measurements_mem.end(); ++it) {
        delete (*it);
    }
}

bool WireED::transformPosition(std::shared_ptr<const pbl::PDF> pdf_in, const std::string& frame_in, std::shared_ptr<pbl::Gaussian> pdf_out) const {
    std::shared_ptr<const pbl::Gaussian> gauss = pbl::PDFtoGaussian(pdf_in);

    if (!gauss) {
        ROS_ERROR("Position evidence is not a gaussian!");
        return false;
    }

     const arma::vec& pos = gauss->getMean();
    tf::Stamped<tf::Point> pos_stamped(tf::Point(pos(0), pos(1), pos(2)), ros::Time(), frame_in);

    try{
        tf::Stamped<tf::Point> pos_stamped_world;
        tf_listener_->transformPoint(hypothesisTree_frame_id_, pos_stamped, pos_stamped_world);

        pbl::Vector3 pos_world(pos_stamped_world.getX(), pos_stamped_world.getY(), pos_stamped_world.getZ());
        pdf_out->setMean(pos_world);

        // todo: also transform covariance
        pdf_out->setCovariance(gauss->getCovariance());

    } catch (tf::TransformException& ex){
        ROS_ERROR("[WORLD_MODEL] %s",ex.what());
        return false;
    }
    return true;
}

bool WireED::transformOrientation(std::shared_ptr<const pbl::PDF> pdf_in, const std::string& frame_in, std::shared_ptr<pbl::Gaussian> pdf_out) const {
    std::shared_ptr<const pbl::Gaussian> gauss = pbl::PDFtoGaussian(pdf_in);

    if (!gauss) {
        ROS_ERROR("Orientation evidence is not a gaussian!");
        return false;
    }

    //const Eigen::VectorXd& ori = gauss->getMean();
    const arma::vec& ori = gauss->getMean();
    tf::Stamped<tf::Quaternion> ori_stamped(tf::Quaternion(ori(0), ori(1), ori(2), ori(3)), ros::Time(), frame_in);

    try{
        tf::Stamped<tf::Quaternion> ori_stamped_world;
        tf_listener_->transformQuaternion(hypothesisTree_frame_id_, ori_stamped, ori_stamped_world);

        pbl::Vector4 ori_world(ori_stamped_world.getX(), ori_stamped_world.getY(), ori_stamped_world.getZ(), ori_stamped_world.getW());
        pdf_out->setMean(ori_world);

        // todo: also transform covariance
        pdf_out->setCovariance(gauss->getCovariance());

    } catch (tf::TransformException& ex){
        ROS_ERROR("[WORLD MODEL] %s",ex.what());
        return false;
    }
    return true;
}

bool WireED::objectToMsg(const mhf::SemanticObject& obj, wire_msgs::ObjectState& msg) const { // TODO TEMP
    msg.ID = obj.getID();

    const std::map<mhf::Attribute, mhf::Property*>& properties = obj.getPropertyMap();

    for(std::map<mhf::Attribute, mhf::Property*>::const_iterator it_prop = properties.begin(); it_prop != properties.end(); ++it_prop) {
        mhf::Property* prop = it_prop->second;

        wire_msgs::Property prop_msg;
        prop_msg.attribute = mhf::AttributeConv::attribute_str(it_prop->first);
        pbl::PDFtoMsg(*prop->getValue(), prop_msg.pdf);
        msg.properties.push_back(prop_msg);
    }

    return true;
}

bool WireED::hypothesisToMsg(const mhf::Hypothesis& hyp, wire_msgs::WorldState& msg) const { // TODO TEMP
    ros::Time time = ros::Time::now();

    msg.header.frame_id = hypothesisTree_frame_id_;
    msg.header.stamp = time;
    
    //std::cout << "properties_.size() = " << hyp.getObjects().size() << std::endl;
    int counter = 0;
    std::cout << "For MAP-hypotheis: n Objects = " << hyp.getObjects().size() << std::endl;

    for(std::list<mhf::SemanticObject*>::const_iterator it = hyp.getObjects().begin(); it != hyp.getObjects().end(); ++it) {
        //counter++;
         std::cout << "Count = " << counter++ << "time = " << time  << std::endl;
        mhf::SemanticObject* obj_clone = (*it)->clone();
        
        std::cout << "Prop object" << std::endl;
        obj_clone->propagate(time.toSec());
std::cout << "get pos value" << std::endl;
        
        std::shared_ptr<const pbl::PDF> pdf = obj_clone->getProperty("position")->getValue();
        
        std::cout << "object-pdf = " << pdf->toString() << std::endl;
        
        wire_msgs::ObjectState obj_msg;
        if (objectToMsg(*obj_clone, obj_msg)) {
                
            msg.objects.push_back(obj_msg);
        }

        delete obj_clone;
        std::cout << "obj deleted" << std::endl;
    }
    return true;
}

void WireED::publish() const { // TODO TEMP: convert to entities and remove old entities.
    wire_msgs::WorldState map_world_msg;
    hypothesisToMsg(hypothesisTree_->getMAPHypothesis(), map_world_msg); // TODO temp, convert to ED-entities

    // Publish results
    pub_wm_.publish(map_world_msg);

}

const std::list<mhf::SemanticObject*>& WireED::getMAPObjects() const {
    return hypothesisTree_->getMAPObjects();
}


void WireED::showStatistics() const {
    std::printf("***** %f *****\n", ros::Time::now().toSec());
    hypothesisTree_->showStatistics();
    std::cout << "Num MAP objects:      " << hypothesisTree_->getMAPObjects().size() << std::endl;
    std::cout << "Last update:          " << last_update_duration << " seconds" << std::endl;
    std::cout << "Max update:           " << max_update_duration << " seconds" << std:: endl;
//    cout << "Evidence buffer size: " << evidence_buffer_.size() << endl; // TODO create own datatype for evidence

    /*
    const list<Hypothesis*>& hyp_list = hypothesisTree_->getHypotheses();
    for(list<Hypothesis* >::const_iterator it_hyp = hyp_list.begin(); it_hyp != hyp_list.end(); ++it_hyp) {

        const list<SemanticObject*>& objs = (*it_hyp)->getObjects();

        double hyp_prob = (*it_hyp)->getProbability();

        cout << "Hyp P = " << hyp_prob << ": " << objs.size() << " object(s)" << endl;

    }
    */
}

ED_REGISTER_PLUGIN(WireED)
