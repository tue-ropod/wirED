#include "multiple_hypothesis_plugin.h"
#include <ed/update_request.h>

#include <iostream>
#include <typeinfo>


 // TODO Problem: objects of MAP-hypothesis are propagated in an object clone. Checking for an old update age is not done on a fixed rate base. Where an how to do?
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
    if (!config.value("object_timeout", object_timeout_, tue::OPTIONAL))          { object_timeout_ = 100;}
    if (!config.value("single_object_assignment", 
                                single_object_assignment_, tue::OPTIONAL))        { single_object_assignment_ = true;}
        
    // print init values
    std::cout << "WirED is initialized with the following values:" << std::endl;
    std::cout << "hypothesisTree_frame = " << hypothesisTree_frame_id_ << std::endl;
    std::cout << "output_frame = " << output_frame_id_ << std::endl;
    std::cout << "max_num_hypotheses = " << max_num_hyps_ << std::endl;
    std::cout << "min_probability_ratio = " << min_prob_ratio_ << std::endl;
    std::cout << "object_timeout = " << object_timeout_ << std::endl;
    std::cout << "single_object_assignment = " << single_object_assignment_ << std::endl;
    
    std::cout << "\nThe following knowledge is loaded: " << std::endl;
    mhf::ObjectModelParser parser(config);
    if (!parser.parse(mhf::KnowledgeDatabase::getInstance())) 
    {
        // parsing failed
        ROS_ERROR_STREAM("While parsing knowledgeDatabase" << std::endl << std::endl << parser.getErrorMessage());
    }
    
    if (config.hasError())
        return;      
    
    // create tf listener
    if (!tf_listener_) {
        tf_listener_ = new tf::TransformListener();
        is_tf_owner_ = true;
    }
    
    //hypothesisTree_ = new mhf::HypothesisTree(max_num_hyps_, min_prob_ratio_);
    hypothesisTree_ = new mhf::HypothesisTree(max_num_hyps_, min_prob_ratio_, single_object_assignment_);
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
    processEvidence(maxLoopDuration_, req);
         
    ++printCounter_;
    if (printCounter_ >= 1){//15) {
       showStatistics();
       printCounter_ = 0;
    } 
}

void Wired::processEvidence(const double max_duration, ed::UpdateRequest& req) {

    ros::Time start_time = ros::Time::now();
    ros::Duration d(max_duration);
    
    boost::shared_ptr<wiredDataBuffer> wiredBuffer =boost::static_pointer_cast<wiredDataBuffer>( getDataBuffer() );
    wire_msgs::WorldEvidence world_evidence;
    
    while( wiredBuffer->getBuffer().is_not_empty() && ros::Time::now() - start_time < d)  // This ensures that we do not wait for until new data are published
    { 
        ros::Time time_before_update = ros::Time::now();     
        
        wiredBuffer->getBuffer().pop_back(&world_evidence);        
        processEvidence(world_evidence);
          
        last_update_duration = (ros::Time::now().toSec() - time_before_update.toSec());
        max_update_duration = std::max(max_update_duration, last_update_duration);
    }
    
    ros::Duration duration = ros::Time::now() - start_time;
    bool timeCheck = ros::Time::now() - start_time < d;
    
    hypothesisTree_->removeOldObjects( start_time.toSec() - object_timeout_ );

    hypothesis2Entities(hypothesisTree_->getMAPHypothesis(), req);
    
//     printObjectsInHypotheses();
}

void Wired::processEvidence(const wire_msgs::WorldEvidence& world_evidence_msg) {
    ros::Time current_time = ros::Time::now();

    if (current_time < last_update_) {
        ROS_WARN("Saw a negative time change of %f seconds; resetting the world model.", (current_time - last_update_).toSec());
        
        delete hypothesisTree_;
        hypothesisTree_ = new mhf::HypothesisTree(max_num_hyps_, min_prob_ratio_, single_object_assignment_);
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
        
        measurements_mem.push_back(meas);

        bool position_ok = true;

        for(std::vector<wire_msgs::Property>::const_iterator it_prop = evidence.properties.begin(); it_prop != evidence.properties.end(); ++it_prop ) {
            const wire_msgs::Property& prop = *it_prop;
            
            std::shared_ptr<pbl::PDF> pdf = pbl::msgToPDF(prop.pdf);

            if (pdf) {
                if (prop.attribute == "position") {
                        std::shared_ptr<pbl::Gaussian> pos_pdf = std::make_shared<pbl::Gaussian>(3);
                    if (!transformPosition(pdf, world_evidence_msg.header.frame_id, pos_pdf)) {
                        // position is a necessary property. If the transform was not successful, abort and don't use the evidence
                        position_ok = false;
                        
                        ROS_WARN("wired, processEvidence: position_ok == false");
                        
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
            } else {
                ROS_ERROR_STREAM("For attribute '" << prop.attribute << "': malformed pdf: " << prop.pdf);
                 ROS_WARN("MHT-plugin: malformed pdf");
            }
        }

        if (position_ok) {                
            evidence_set.add(meas);
        } else {
            ROS_ERROR("Unable to transform position.");
        }

    } // end iteration over object evidence list

    std::cout << "The following evidence is added:\n" << evidence_set.toString() << std::endl;
    
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

bool Wired::hypothesis2Entities(const mhf::Hypothesis& hyp, ed::UpdateRequest& req) {
    ros::Time time = ros::Time::now();
         
    const std::list<std::shared_ptr<mhf::SemanticObject>>* objs = hyp.getObjects();    
    
    for(std::list<std::shared_ptr<mhf::SemanticObject>>::const_iterator it = objs->begin(); it != objs->end(); ++it) 
    {
        std::shared_ptr<mhf::SemanticObject> semObj = *it;
            
        std::shared_ptr<mhf::SemanticObject> obj_clone = (*it)->cloneThis();
       
        obj_clone->propagate(time.toSec());

        if (object2Entity(*obj_clone, req)) {
        }

//         delete obj_clone;
    } 
    
    for(std::vector<mhf::ObjectID>::const_iterator it = objectIDs2entitiesPrev_->begin(); it != objectIDs2entitiesPrev_->end(); ++it) 
    {
        // remove all objects which are not in the current hypothesis anymore
        mhf::ObjectID objectID = *it;
        ed::UUID id = getEntityIDForMHTObject(objectID);
        req.removeEntity (id );
        
         std::cout << "Wired::hypothesis2Entities: request to remove entity with id = " << id << std::endl;
    }
          
    objectIDs2entitiesPrev_->clear();
     
    swapPointers(&objectIDs2entities_, &objectIDs2entitiesPrev_);
    
    return true;
}

void Wired::swapPointers(std::vector<mhf::ObjectID> **r, std::vector<mhf::ObjectID> **s)
{
    std::vector<mhf::ObjectID>* pSwap = *r;
    *r = *s;
    *s = pSwap;
    return;
}

bool Wired::object2Entity(const mhf::SemanticObject& obj, ed::UpdateRequest& req) const
{
         ros::Time time = ros::Time::now();
         std::shared_ptr<const pbl::PDF> pdf = obj.getProperty("positionAndDimension")->getFullValue();

         if( pdf )
         {
                 tracking::FeatureProperties featureProperties;
                 
                 featureProperties.setObservedFeatureProperties(pdf);
                 
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
                         
                ed::UUID id = getEntityIDForMHTObject(obj.getID());
                
//                 std::cout << "Wired::object " << id << " to entity: " << std::endl;
//                 featureProperties.printProperties();
                
                req.setProperty ( id, featureProperties_, featureProperties );
                req.setLastUpdateTimestamp ( id, time.toSec() ); // TODO desired time? Communicate the meaurement time. Do not use the propagated time.
                req.setPose ( id, pose );
                req.setExistenceProbability ( id, existenceProbability );
                //         int nMeasurements = entityProperties.getNMeasurements() + 1;
                // entityProperties.setNMeasurements(  nMeasurements ); TODO?

                 objectIDs2entities_->push_back(obj.getID());
                 
                 if(objectIDs2entitiesPrev_->size() > 0)
                 {       
                         std::vector<mhf::ObjectID>::const_iterator it = std::find(objectIDs2entitiesPrev_->begin(), objectIDs2entitiesPrev_->end(),  obj.getID());
                         
                         if (it != objectIDs2entitiesPrev_->end())
                         {
                                 objectIDs2entitiesPrev_->erase(it);
                         }
                 }
         }
        
        // TODO general properties and PDF's to ED. 
        return true;

}

void Wired::printObjectsInHypotheses() const
{
    const std::list<mhf::Hypothesis*>& allHyp = hypothesisTree_->getHypotheses();
    
    for(std::list<mhf::Hypothesis*>::const_iterator itHyp = allHyp.begin(); itHyp != allHyp.end(); itHyp++)
    {
            mhf::Hypothesis* hyp = *itHyp;
            hyp->showStatictics();
            std::cout << " objects are" << std::endl; 
            printObjectsInHypothesis(*hyp);
            
            std::cout << "\n\n" << std::endl;
    }
}

void Wired::printObjectsInHypothesis(const mhf::Hypothesis& hyp) const
{
    ros::Time time = ros::Time::now();
    const std::list<std::shared_ptr<mhf::SemanticObject>>* objs = hyp.getObjects(); 
    
    for(std::list<std::shared_ptr<mhf::SemanticObject>>::const_iterator it = objs->begin(); it != objs->end(); ++it) 
    {
        std::shared_ptr<mhf::SemanticObject> semObj = *it;
            
        std::shared_ptr<mhf::SemanticObject> obj_clone = (*it)->cloneThis();
       
        obj_clone->propagate(time.toSec());

        printFeaturePropertiesOfObject(*obj_clone);
    } 
}

void Wired::printFeaturePropertiesOfObject(const mhf::SemanticObject& obj) const
{
        ros::Time time = ros::Time::now();
        std::shared_ptr<const pbl::PDF> pdf = obj.getProperty("positionAndDimension")->getFullValue();

        if( pdf )
        {
                std::cout << "FeatureProperties of object " << obj.getID() << " having pointer " << pdf << " are" << std::endl;
                tracking::FeatureProperties featureProperties;
                featureProperties.setObservedFeatureProperties(pdf);
                featureProperties.printProperties();                
        }
}

const std::list<std::shared_ptr<mhf::SemanticObject>>* Wired::getMAPObjects() const {
    return hypothesisTree_->getMAPObjects();
}


ed::UUID Wired::getEntityIDForMHTObject(mhf::ObjectID objectID) const { return "MHT-" + std::to_string(objectID) + "-laserTracking"; }

void Wired::showStatistics() const {
    std::printf("***** %f *****\n", ros::Time::now().toSec());
    hypothesisTree_->showStatistics();
    std::cout << "Num MAP objects:      " << hypothesisTree_->getMAPObjects()->size() << std::endl; 
    std::cout << "Last update:          " << last_update_duration << " seconds" << std::endl;
    std::cout << "Max update:           " << max_update_duration << " seconds" << std:: endl;   
}

ED_REGISTER_PLUGIN(Wired)
