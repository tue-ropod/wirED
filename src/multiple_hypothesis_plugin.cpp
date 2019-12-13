#include "multiple_hypothesis_plugin.h"
#include <ed/update_request.h>
//#include "wire_core/include/wire/WorldModelROS.h"



#include <iostream>
#include <typeinfo>

// ----------------------------------------------------------------------------------------------------

Wired::Wired(tf::TransformListener* tf_listener) :   tf_listener_(tf_listener), is_tf_owner_(false)
{
        objectIDs2entitiesPrev_ = new std::vector<mhf::ObjectID>;
        objectIDs2entities_ = new std::vector<mhf::ObjectID>;
}

// ----------------------------------------------------------------------------------------------------

Wired::~Wired()
{
    // delete multiple hypothesis filter_
        if(hypothesisTree_)
        {
                delete hypothesisTree_;
        }
        
        if( objectIDs2entitiesPrev_)
        {
                 delete objectIDs2entitiesPrev_;
        }
                      
        if( objectIDs2entities_)
        {
                 delete objectIDs2entities_;
        }
}

// ----------------------------------------------------------------------------------------------------

void Wired::initialize(ed::InitData& init)
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
//     ros::NodeHandle n("~"); // TEMP TODO
//     pub_wm_ = n.advertise<wire_msgs::WorldState>("/world_state", 10); // TEMP TODO via ED
    
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
    
        
    init.properties.registerProperty ( "Feature", featureProperties_, new FeaturPropertiesInfo );
}

// ----------------------------------------------------------------------------------------------------

void Wired::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{   
//     std::cout << "WIRED: process" << std::endl;
    processEvidence(maxLoopDuration_, req);
         
    ++printCounter_;
    if (printCounter_ >= 1){//15) {
       showStatistics();
       printCounter_ = 0;
    } 
}

void Wired::processEvidence(const double max_duration, ed::UpdateRequest& req) {

    ros::Time start_time = ros::Time::now();
//    std::cout << "Start time = " << start_time.toSec()  << std::endl;

    ros::Duration d(max_duration);
    
    boost::shared_ptr<wiredDataBuffer> wiredBuffer =boost::static_pointer_cast<wiredDataBuffer>( getDataBuffer() );
    wire_msgs::WorldEvidence world_evidence;
          
    while( wiredBuffer->getBuffer().is_not_empty() && ros::Time::now() - start_time < d)  // This ensures that we do not wait for until new data are published
    { 
        ros::Time time_before_update = ros::Time::now();     
        
//         std::cout << "wirebuffer: has data = " <<wiredBuffer->getBuffer().is_not_empty() << std::endl;
        wiredBuffer->getBuffer().pop_back(&world_evidence);        
        processEvidence(world_evidence);
          
        last_update_duration = (ros::Time::now().toSec() - time_before_update.toSec());
        max_update_duration = std::max(max_update_duration, last_update_duration);

        //evidence_buffer_.pop_back(); // TODO define this evidence buffer
    }
    
    ros::Duration duration = ros::Time::now() - start_time;
    bool timeCheck = ros::Time::now() - start_time < d;
    
//     publish(); // TEMP TODO

    hypothesis2Entity(hypothesisTree_->getMAPHypothesis(), req);
   
    //std::cout << "Duration = " << duration.toSec() << " time check = "  << timeCheck << " max_duration = " << max_duration << std::endl;//" evidence buffer size  = " << evidence_buffer_.size() << std::endl;
}

void Wired::processEvidence(const wire_msgs::WorldEvidence& world_evidence_msg) {
    ros::Time current_time = ros::Time::now();

    if (current_time < last_update_) {
        ROS_WARN("Saw a negative time change of %f seconds; resetting the world model.", (current_time - last_update_).toSec());
        std::cout << "Saw a negative time change of " << (current_time - last_update_).toSec() << "seconds; resetting the world model." << std::endl;
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

//             std::cout << " Wired::processEvidence: properties = " << prop.attribute << ", " << prop.pdf << std::endl;
            
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

bool Wired::transformPosition(std::shared_ptr<const pbl::PDF> pdf_in, const std::string& frame_in, std::shared_ptr<pbl::Gaussian> pdf_out) const {
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

bool Wired::transformOrientation(std::shared_ptr<const pbl::PDF> pdf_in, const std::string& frame_in, std::shared_ptr<pbl::Gaussian> pdf_out) const {
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
/*
bool Wired::objectToMsg(const mhf::SemanticObject& obj, wire_msgs::ObjectState& msg) const { // TODO TEMP
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
*/
/*
bool Wired::hypothesis2Msg(const mhf::Hypothesis& hyp, wire_msgs::WorldState& msg) const { // TODO TEMP
    ros::Time time = ros::Time::now();

    msg.header.frame_id = hypothesisTree_frame_id_;
    msg.header.stamp = time;

    //std::cout << "hypothesis2Msg: hyp = " << hyp << std::endl;
    
     //const std::list<mhf::SemanticObject*>& objs = hyp.getObjects();
    
//     std::cout << "hypothesis2MSg 1: hyp = " << hyp.getProbability() << std::endl;
//    hyp.getObjects();
//      std::cout << "Wired::hypothesis2Msg, For MAP-hypotheis: n Objects = " << hyp.getObjects()->size() << std::endl;
// std::cout << "hypothesis2MSg 2: hyp = " << hyp.getProbability() << std::endl;
   //  std::cout << "hyp.getObjects().begin() = " << hyp.getObjects().begin() << std::endl;
   //  std::cout << "hyp.getObjects().end() = " << hyp.getObjects().end() << std::endl;
  //   bool check =hyp.getObjects()->begin() == hyp.getObjects()->end();
//      std::cout << "hypothesis2MSg 3: hyp = " << hyp.getProbability() << std::endl;
//      std::cout << "Wired::hypothesis2Msg, hyp.getObjects().begin() == hyp.getObjects().end(): " << check << std::endl;
     std::list<mhf::SemanticObject*>::const_iterator first = hyp.getObjects()->begin();
     //std::list<mhf::SemanticObject*>::const_iterator last = hyp.getObjects().end();
//      std::cout << "hypothesis2MSg 4: hyp = " << hyp.getProbability() << std::endl;
     
     if(hyp.getObjects()->size() > 0 )
     {
//              std::cout << "Wired::hypothesis2Msg, first = " << (*first)->toString() << std::endl;
     }
     
     //std::cout << "Wired::hypothesis2Msg, distance = " << std::distance(hyp.getObjects().begin(), hyp.getObjects().end() ) << std::endl;
//     std::cout << "hypothesis2MSg 5: hyp = " << hyp.getProbability() << std::endl;
    for(std::list<mhf::SemanticObject*>::const_iterator it = hyp.getObjects()->begin(); it != hyp.getObjects()->end(); ++it) {
//             std::cout << "Wired::hypothesis2Msg, Going to clone" << std::endl;
        mhf::SemanticObject* obj_clone = (*it)->clone();
        
        obj_clone->propagate(time.toSec());
        
        //std::shared_ptr<const pbl::PDF> pdf = obj_clone->getProperty("position")->getValue();
        wire_msgs::ObjectState obj_msg;
        if (objectToMsg(*obj_clone, obj_msg)) {
            msg.objects.push_back(obj_msg);
        }

        delete obj_clone;
    }
    return true;
}

*/
// What are the relevant properties which should be converted in a entity? -> featureProperties & pose for this case, . Is this information 
// present in the entity descriptions?
// Possible to add PDF's to ED as a list of properties?!

// Check which object-ids are not present anymore from latest hypothesis and request to remove them.
// 

// Visualisation: ED_GUI_SERVER. Is the information available there sufficient? -> yes, if we convert it to a featureproperties-type

bool Wired::hypothesis2Entity(const mhf::Hypothesis& hyp, ed::UpdateRequest& req) {
    ros::Time time = ros::Time::now();
         
    const std::list<mhf::SemanticObject*>* objs = hyp.getObjects();
    
    
//     std::cout << "hypothesis2Entity start " << std::endl;
    
    
    for(std::list<mhf::SemanticObject*>::const_iterator it = objs->begin(); it != objs->end(); ++it) 
    {
        mhf::SemanticObject* semObj = *it;
            
        mhf::SemanticObject* obj_clone = (*it)->clone();
        obj_clone->propagate(time.toSec());

        if (object2Entity(*obj_clone, req)) {
           // msg.objects.push_back(obj_msg);
        }

        delete obj_clone;
    } 
  
//   std::cout << "objectIDs2entitiesPrev_ = " << objectIDs2entitiesPrev_ << std::endl;

//   std::cout << "objectIDs2entities_ = " << objectIDs2entities_ << " objectIDs2entitiesPrev_ = " << objectIDs2entitiesPrev_  << std::endl;

//   std::cout << "objectIDs2entities_: " << std::endl;
//   for(std::vector<mhf::ObjectID>::const_iterator it = objectIDs2entities_->begin(); it != objectIDs2entities_->end(); ++it) 
//   {
//            mhf::ObjectID objectID = *it;
//            std::cout << "\tid = " << objectID ;
//   }
//   std::cout << "\n";
          
//   std::cout << "objectIDs2entitiesPrev_, going to remove entities: " << std::endl;
  for(std::vector<mhf::ObjectID>::const_iterator it = objectIDs2entitiesPrev_->begin(); it != objectIDs2entitiesPrev_->end(); ++it) 
  {
        // remove all objects which are not in the current hypothesis anymore
        mhf::ObjectID objectID = *it;
        ed::UUID id = getEntityIDForMHTObject(objectID);
//         std::cout << "\tid = " << objectID;
        
        req.removeEntity (id );
   }
          std::cout << "\n";
          
        objectIDs2entitiesPrev_->clear();
        
//         std::cout << "objectIDs2entitiesPrev_ cleared, size = " << objectIDs2entitiesPrev_->size() << std::endl;
     
        swapPointers(&objectIDs2entities_, &objectIDs2entitiesPrev_);

//        std::cout << "objectIDs2entities_ = " << objectIDs2entities_ << " objectIDs2entitiesPrev_ = " << objectIDs2entitiesPrev_  << std::endl;
     
//      std::cout << "hypothesis2Entity end" << std::endl;
     return true;
}

/*
    void setWorld(const WorldModelConstPtr& world)
    {
        boost::lock_guard<boost::mutex> lg(mutex_world_);
        world_new_ = world;
    }
*/

void Wired::swapPointers(std::vector<mhf::ObjectID> **r, std::vector<mhf::ObjectID> **s)
{// THIS FUNCTION DOES NOT WORK!!!!!!
    std::vector<mhf::ObjectID>* pSwap = *r;
    *r = *s;
    *s = pSwap;
    return;
}

bool Wired::object2Entity(const mhf::SemanticObject& obj, ed::UpdateRequest& req) const
{
         ros::Time time = ros::Time::now();
         std::shared_ptr<const pbl::PDF> pdf = obj.getProperty("positionAndDimension")->getValue();
       
//         std::cout << "object2Entity: going to convert object to entity, obj id = " << obj.getID() << std::endl;
         
      //   std::vector<mhf::ObjectID> objectIDs2entities;
        
         if( pdf )
         {
//                  std::cout << "PDF-check passed" << std::endl;
                 tracking::FeatureProperties featureProperties;
                 featureProperties.setFeatureProperties(pdf);
                 double existenceProbability = 1.0; // TODO magic number
                        
                 geo::Pose3D pose;
                 
                if ( featureProperties.getFeatureProbabilities().get_pCircle() < featureProperties.getFeatureProbabilities().get_pRectangle() )
                {
                        // determine corners
                        tracking::Rectangle rectangle = featureProperties.getRectangle();
                        pose = rectangle.getPose();
                }
                else
                {
                        // determine cilinder-properties
                        tracking::Circle circle = featureProperties.getCircle();
                        pose = circle.getPose();
                }
                
                     //    std::cout << "multiple hypothesis plugin: featureProperties_.idx = " << featureProperties_.idx << std::endl;
                     //    std::cout << "multiple hypothesis plugin 2 entity: featureProperties of MAP hypothesis = " ; featureProperties.printProperties(); std::cout << " having pose " << pose << std::endl;
                         
                ed::UUID id = getEntityIDForMHTObject(obj.getID());
//                 std::cout << "wired updaterequest = " << id << std::endl;
                // TESTED: nan of circle is within its properties, it is not due to the conversion to the circle pose.
                
//                 std::cout << "featureproperties update request: " << std::endl;
                featureProperties.printProperties();
                
                req.setProperty ( id, featureProperties_, featureProperties );
                req.setLastUpdateTimestamp ( id, time.toSec() ); // TODO desired time?
                req.setPose ( id, pose );
                req.setExistenceProbability ( id, existenceProbability );
                //         int nMeasurements = entityProperties.getNMeasurements() + 1;
                        // entityProperties.setNMeasurements(  nMeasurements ); TODO?

                 objectIDs2entities_->push_back(obj.getID());
//                  std::cout << "objectIDs2entitiesPrev_: object added with id = " << obj.getID() << std::endl;
                 
                 if(objectIDs2entitiesPrev_->size() > 0)
                 {       
                         std::vector<mhf::ObjectID>::const_iterator it = std::find(objectIDs2entitiesPrev_->begin(), objectIDs2entitiesPrev_->end(),  obj.getID());
                         
                         if (it != objectIDs2entitiesPrev_->end())
                         {
//                                  std::cout << "objectIDs2entitiesPrev_: object removed with id = " << (*it) << std::endl;
                                 objectIDs2entitiesPrev_->erase(it);
                         }
                 }
         }
           
           
           //
           
        
        // TODO general properties and PDF's to ED. 
        /*
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
    */
}

/*
void Wired::publish() const { // TODO TEMP: convert to entities and remove old entities.

        std::cout << "*hypothesisTree_ = " << hypothesisTree_ << std::endl;
        
        wire_msgs::WorldState map_world_msg;
        
        const mhf::Hypothesis& test = hypothesisTree_->getMAPHypothesis();
        
        std::cout <<"MAP hypothesis = " << &test << std::endl;
        
//     std::cout << "wire, publish, get MAP objects" << std::endl;
      
        
    hypothesisTree_->getMAPObjects(); // TODO can be removed, just for debugging now.
//     std::cout << "wire, publish, going to get MAP hypothesis and convert it to msg" << std::endl;
    hypothesis2Msg(hypothesisTree_->getMAPHypothesis(), map_world_msg); // TODO temp, convert to ED-entities

    // Publish results
    pub_wm_.publish(map_world_msg);

}
*/
const std::list<mhf::SemanticObject*>* Wired::getMAPObjects() const {
    return hypothesisTree_->getMAPObjects();
}


ed::UUID Wired::getEntityIDForMHTObject(mhf::ObjectID objectID) const { return "MHT-" + std::to_string(objectID) + "-laserTracking"; }

void Wired::showStatistics() const {
    std::printf("***** %f *****\n", ros::Time::now().toSec());
    hypothesisTree_->showStatistics();
    std::cout << "Num MAP objects:      " << hypothesisTree_->getMAPObjects()->size() << std::endl;
    std::cout << "Last update:          " << last_update_duration << " seconds" << std::endl;
    std::cout << "Max update:           " << max_update_duration << " seconds" << std:: endl;
//    cout << "Evidence buffer size: " << evidence_buffer_.size() << endl; // TODO create own datatype for evidence

//    double sumProb = 0.0;
//     const std::list<mhf::Hypothesis*>& hyp_list = hypothesisTree_->getHypotheses();
//     for(std::list<mhf::Hypothesis* >::const_iterator it_hyp = hyp_list.begin(); it_hyp != hyp_list.end(); ++it_hyp) {
// 
//         const std::list<mhf::SemanticObject*>* objs = (*it_hyp)->getObjects();
// 
//         double hyp_prob = (*it_hyp)->getProbability();
// sumProb += hyp_prob;
//         std::cout << "Hyp P = " << hyp_prob << ": " << objs->size() << " object(s)" << std::endl;
// 
//     }
//     
//     std::cout << "totalProb = " << sumProb << std::endl;
   
}

ED_REGISTER_PLUGIN(Wired)
