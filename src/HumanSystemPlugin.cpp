#include "arena_human_plugin/HumanSystemPlugin.h"



// Fix for chrono duration traits error
namespace gz::sim::traits {
  template<>
  struct HasEqualityOperator<std::chrono::steady_clock::duration> {
    static constexpr bool value = true;
  };
}
using namespace arena_human_plugin;

//////////////////////////////////////////////////
HumanSystemPlugin::HumanSystemPlugin()
  : update_rate_secs_(0.1),
    namespace_(""),
    global_frame_("map"),
    pedestrians_topic_("arena_peds"),
    agents_initialized_(false),
    first_update_(true),
    counter_(0)
{
}

//////////////////////////////////////////////////
HumanSystemPlugin::~HumanSystemPlugin()
{
}

//////////////////////////////////////////////////
void HumanSystemPlugin::Configure(const gz::sim::Entity& _entity,
                                  const std::shared_ptr<const sdf::Element>& _sdf,
                                  gz::sim::EntityComponentManager& _ecm,
                                  gz::sim::EventManager& _eventMgr)
{
  (void)_eventMgr;
  counter_ = 0;
  worldEntity_ = _entity;

  // Initialize ROS 2 
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  
  std::string nodename = "human_system_plugin_" + std::to_string(_entity);
  rosnode_ = std::make_shared<rclcpp::Node>(nodename.c_str());
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(rosnode_);

  // Get plugin parameters 
  sdf_ = _sdf->Clone();

  if (sdf_->HasElement("namespace"))
    namespace_ = sdf_->Get<std::string>("namespace");
  else
    namespace_ = "/";

  if(namespace_.empty() || namespace_.back() != '/')
    namespace_.push_back('/');

  if (sdf_->HasElement("global_frame_to_publish"))
    global_frame_ = sdf_->Get<std::string>("global_frame_to_publish");
  else
    global_frame_ = "map";

  if (sdf_->HasElement("pedestrians_topic"))
    pedestrians_topic_ = sdf_->Get<std::string>("pedestrians_topic");

  if (sdf_->HasElement("update_rate"))
  {
    double update_rate = sdf_->Get<double>("update_rate");
    update_rate_secs_ = 1.0 / update_rate;
  }

  // Subscribe to arena_peds topic 
  std::string full_topic = namespace_ + pedestrians_topic_;
  
  pedestrians_sub_ = rosnode_->create_subscription<arena_people_msgs::msg::Pedestrians>(
    full_topic,
    rclcpp::QoS(10),
    std::bind(&HumanSystemPlugin::pedestriansCallback, this, std::placeholders::_1));

  // Initialize timing
  last_update_time_ = std::chrono::steady_clock::now();


  // Create Delete Actor Service 
  RCLCPP_INFO(this->rosnode_->get_logger(), "Creating delete_actors service...");
  RCLCPP_INFO(this->rosnode_->get_logger(), "Namespace: '%s'", namespace_.c_str());
  RCLCPP_INFO(this->rosnode_->get_logger(), "Full service name will be: '%s'", (namespace_ + "delete_actors").c_str());
  RCLCPP_INFO(this->rosnode_->get_logger(), "ROS node name: %s", rosnode_->get_name());
  delete_actors_service_ = this->rosnode_->create_service<arena_people_msgs::srv::DeleteActors>(
      namespace_ + "delete_actors",
      std::bind(&HumanSystemPlugin::deleteActorsCallback, this, std::placeholders::_1, std::placeholders::_2)
  );
  RCLCPP_ERROR(this->rosnode_->get_logger(), "Delete actors service created successfully!");

  gzmsg << "HumanSystemPlugin configured - Namespace: " << namespace_ 
        << ", Topic: " << full_topic << ", Frame: " << global_frame_ << std::endl;
}

//////////////////////////////////////////////////
void HumanSystemPlugin::pedestriansCallback(const arena_people_msgs::msg::Pedestrians::SharedPtr msg)
{
  current_pedestrians_ = msg;
  
  RCLCPP_DEBUG(rosnode_->get_logger(), 
    "Received %zu pedestrians update", msg->pedestrians.size());
}

//////////////////////////////////////////////////
void HumanSystemPlugin::PreUpdate(const gz::sim::UpdateInfo& _info,
                                  gz::sim::EntityComponentManager& _ecm)
{
    // Spin ROS node first
    rclcpp::spin_some(rosnode_);

    // DELETE FIRST ECM ENTITIES! 
    if (delete_requested_) {
        RCLCPP_INFO(rosnode_->get_logger(), "Processing entity deletion for %zu actors", 
                   entities_to_delete_.size());
        
        for (auto entity : entities_to_delete_) {
            // ENTITY STILL EXISTS?
            if (!_ecm.EntityHasComponentType(entity, gz::sim::components::Actor::typeId)) {
                RCLCPP_WARN(rosnode_->get_logger(), "Entity %lu already deleted, skipping", entity);
                continue;
            }
            
            // important step: delete all animation components (segfault sends greetings)
            if (_ecm.Component<gz::sim::components::AnimationName>(entity)) {
                _ecm.RemoveComponent<gz::sim::components::AnimationName>(entity);
            }
            if (_ecm.Component<gz::sim::components::AnimationTime>(entity)) {
                _ecm.RemoveComponent<gz::sim::components::AnimationTime>(entity);
            }
            
            // delete trajectory components 
            if (_ecm.Component<gz::sim::components::TrajectoryPose>(entity)) {
                _ecm.RemoveComponent<gz::sim::components::TrajectoryPose>(entity);
            }
            
            // delete pose component 
            if (_ecm.Component<gz::sim::components::WorldPose>(entity)) {
                _ecm.RemoveComponent<gz::sim::components::WorldPose>(entity);
            }
            
            // finally entity delete request 
            _ecm.RequestRemoveEntity(entity);
            
            RCLCPP_INFO(rosnode_->get_logger(), "Requested removal of entity %lu", entity);
        }
        
        // Cleanup
        delete_requested_ = false;
        entities_to_delete_.clear();
        
        // important  to force reinitialization
        agents_initialized_ = false;
        first_update_ = false;
        
        RCLCPP_INFO(rosnode_->get_logger(), "Entity deletion process completed - ready for reinitialization");
        
        // return after reset 
        return;
    }

    // Forced re-initialization check 
    if (!agents_initialized_ || pedestrians_.empty()) {
        RCLCPP_INFO(rosnode_->get_logger(), "Checking for new actors to initialize...");
        
        
        size_t actor_count = 0;
        _ecm.Each<gz::sim::components::Actor, gz::sim::components::Name>(
            [&](const gz::sim::Entity&, const gz::sim::components::Actor*, const gz::sim::components::Name*) -> bool {
                actor_count++;
                return true;
            });
        
        if (actor_count > 0) {
            RCLCPP_INFO(rosnode_->get_logger(), "Found %zu actors, reinitializing...", actor_count);
            initializeAgents(_ecm);
            agents_initialized_ = true;
        }
    }

    
    if (agents_initialized_ && current_pedestrians_ && !current_pedestrians_->pedestrians.empty()) {
        updateGazeboPedestrians(_ecm, _info, *current_pedestrians_);
    }

    first_update_ = false;
}

//////////////////////////////////////////////////
void HumanSystemPlugin::initializeAgents(gz::sim::EntityComponentManager& _ecm)
{
    
    pedestrians_.clear();
    
    RCLCPP_INFO(rosnode_->get_logger(), "Initializing agents...");

    // Find all actor entities 
    _ecm.Each<gz::sim::components::Actor, gz::sim::components::Name>(
        [&](const gz::sim::Entity& agentEntity,
            const gz::sim::components::Actor* actorComp,
            const gz::sim::components::Name* name) -> bool
        {
            std::string actor_name = name->Data();
            
            RCLCPP_INFO(rosnode_->get_logger(), "Processing actor: %s (Entity: %lu)", 
                       actor_name.c_str(), agentEntity);
            
            // Create a pedestrian entry for this actor
            arena_people_msgs::msg::Pedestrian pedestrian;
            pedestrian.name = actor_name;
            pedestrian.id = static_cast<int>(agentEntity);
            
            // Store entity mapping
            pedestrians_[agentEntity] = pedestrian;
            
            
            if (actorComp && actorComp->Data().AnimationCount() > 0) {
                auto ani = actorComp->Data().AnimationByIndex(0);
                
               
                _ecm.SetComponentData<gz::sim::components::AnimationName>(agentEntity, ani->Name().c_str());
                _ecm.SetChanged(agentEntity, gz::sim::components::AnimationName::typeId, 
                               gz::sim::ComponentState::OneTimeChange);

              
                std::chrono::steady_clock::duration zeroTime = std::chrono::seconds(0);
                _ecm.SetComponentData<gz::sim::components::AnimationTime>(agentEntity, zeroTime);
                _ecm.SetChanged(agentEntity, gz::sim::components::AnimationTime::typeId, 
                               gz::sim::ComponentState::OneTimeChange);
                
                RCLCPP_INFO(rosnode_->get_logger(), "Set animation '%s' for actor %s", 
                           ani->Name().c_str(), actor_name.c_str());
            }
            
           
            
            RCLCPP_INFO(rosnode_->get_logger(), "Successfully initialized actor: %s (Entity: %lu)", 
                       actor_name.c_str(), agentEntity);
            return true;
        });

    RCLCPP_INFO(rosnode_->get_logger(), "Completed initialization of %zu pedestrian actors", 
               pedestrians_.size());
    
    // Force agents_initialized flag
    agents_initialized_ = true;
}

//////////////////////////////////////////////////
void HumanSystemPlugin::updateGazeboPedestrians(gz::sim::EntityComponentManager& _ecm,
                                                const gz::sim::UpdateInfo& _info,
                                                const arena_people_msgs::msg::Pedestrians& _pedestrians)
{
  for (const auto& pedestrian : _pedestrians.pedestrians)
  {
    // Find the corresponding actor entity by name
    gz::sim::Entity agentEntity = gz::sim::kNullEntity;
    
    for (const auto& [entity, stored_ped] : pedestrians_)
    {
      if (stored_ped.name == pedestrian.name)
      {
        
        if (!_ecm.EntityHasComponentType(entity, gz::sim::components::Actor::typeId)) {
          RCLCPP_WARN_THROTTLE(rosnode_->get_logger(), *rosnode_->get_clock(), 5000,
                              "Entity %lu for actor %s no longer exists, triggering reinit", 
                              entity, pedestrian.name.c_str());
          
          // Force reinitialization
          agents_initialized_ = false;
          return;
        }

        agentEntity = entity;
        break;
      }
    }
    
    if (agentEntity == gz::sim::kNullEntity)
    {
      RCLCPP_WARN_THROTTLE(rosnode_->get_logger(), *rosnode_->get_clock(), 5000,
                          "Actor not found for pedestrian: %s - triggering reinit", 
                          pedestrian.name.c_str());
      
      // Force reinitialization when actor not found
      agents_initialized_ = false;
      return;
    }

    // CRITICAL: Zero the local pose like HuNavSystemPlugin does
    // This prevents the SDF initial pose from acting as an offset
    auto lposeComp = _ecm.Component<gz::sim::components::Pose>(agentEntity);
    if (nullptr == lposeComp)
    {
      _ecm.CreateComponent(agentEntity, gz::sim::components::Pose(gz::math::Pose3d::Zero));
    }
    else
    {
      auto p = lposeComp->Data();
      auto newPose = p;
      newPose.Pos().X(0);  // Zero X position
      newPose.Pos().Y(0);  // Zero Y position
      *lposeComp = gz::sim::components::Pose(newPose);
    }
    _ecm.SetChanged(agentEntity, gz::sim::components::Pose::typeId, gz::sim::ComponentState::OneTimeChange);

    // Create actor pose from pedestrian data 
    gz::math::Pose3d actorPose;
    actorPose.Pos().X(pedestrian.pose.position.x);
    actorPose.Pos().Y(pedestrian.pose.position.y);
    actorPose.Pos().Z(0.80); // Fixed height 
    
    // Convert quaternion to yaw
    tf2::Quaternion quat(
      pedestrian.pose.orientation.x,
      pedestrian.pose.orientation.y,
      pedestrian.pose.orientation.z,
      pedestrian.pose.orientation.w
    );
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    actorPose.Rot() = gz::math::Quaterniond(0, 0.0, yaw);

    // UPDATE TRAJECTORY POSE 
    auto trajPoseComp = _ecm.Component<gz::sim::components::TrajectoryPose>(agentEntity);
    if (trajPoseComp) {
      *trajPoseComp = gz::sim::components::TrajectoryPose(actorPose);
      _ecm.SetChanged(agentEntity, gz::sim::components::TrajectoryPose::typeId, gz::sim::ComponentState::OneTimeChange);
    } else {
      _ecm.CreateComponent(agentEntity, gz::sim::components::TrajectoryPose(actorPose));
    }

    // UPDATE WORLD POSE 
    auto wpose = _ecm.Component<gz::sim::components::WorldPose>(agentEntity);
    if (wpose) {
      wpose->Data() = actorPose;
      _ecm.SetChanged(agentEntity, gz::sim::components::WorldPose::typeId, gz::sim::ComponentState::OneTimeChange);
    } else {
      _ecm.CreateComponent(agentEntity, gz::sim::components::WorldPose(actorPose));
    }

    // ANIMATION MANAGEMENT 
    // Get previous pose for distance calculation
    gz::math::Pose3d prevPose = gz::math::Pose3d::Zero;
    if (previous_poses_.find(agentEntity) != previous_poses_.end()) {
      prevPose = previous_poses_[agentEntity];
    }

    // Calculate distance traveled for animation synchronization
    double distanceTraveled = (actorPose.Pos() - prevPose.Pos()).Length();

    // Animation factor 
    double animationFactor = 5.0;

    // Update AnimationTime based on distance traveled
    auto animTimeComp = _ecm.Component<gz::sim::components::AnimationTime>(agentEntity);
    if (!animTimeComp) {
      std::chrono::steady_clock::duration initialTime = std::chrono::seconds(0);
      _ecm.CreateComponent(agentEntity, gz::sim::components::AnimationTime(initialTime));
      _ecm.SetChanged(agentEntity, gz::sim::components::AnimationTime::typeId, gz::sim::ComponentState::OneTimeChange);
    } else {
      auto animTime = animTimeComp->Data() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(distanceTraveled * animationFactor));
      *animTimeComp = gz::sim::components::AnimationTime(animTime);
      _ecm.SetChanged(agentEntity, gz::sim::components::AnimationTime::typeId, gz::sim::ComponentState::OneTimeChange);
    }

    // Store current pose for next iteration
    previous_poses_[agentEntity] = actorPose;

    // TF Broadcasting 
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = rosnode_->get_clock()->now();
    tf_msg.header.frame_id = global_frame_;
    tf_msg.child_frame_id = "agent_" + pedestrian.name;
    
    tf_msg.transform.translation.x = actorPose.Pos().X();
    tf_msg.transform.translation.y = actorPose.Pos().Y();
    tf_msg.transform.translation.z = actorPose.Pos().Z();
    
    tf_msg.transform.rotation.x = actorPose.Rot().X();
    tf_msg.transform.rotation.y = actorPose.Rot().Y();
    tf_msg.transform.rotation.z = actorPose.Rot().Z();
    tf_msg.transform.rotation.w = actorPose.Rot().W();
    
    // Publish the transform
    tf_broadcaster_->sendTransform(tf_msg);

    // Update stored pedestrian data
    pedestrians_[agentEntity] = pedestrian;

    RCLCPP_DEBUG(rosnode_->get_logger(),
      "Updated actor %s: pos[%.2f, %.2f, %.2f]",
      pedestrian.name.c_str(), 
      actorPose.Pos().X(), actorPose.Pos().Y(), actorPose.Pos().Z());
  }
}

//////////////////////////////////////////////////
gz::math::Pose3d HumanSystemPlugin::worldPose(gz::sim::Entity entity,
                                              const gz::sim::EntityComponentManager& _ecm) const
{
  
  auto poseComp = _ecm.Component<gz::sim::components::WorldPose>(entity);
  if (poseComp)
  {
    return poseComp->Data();
  }
  
  // Fallback to local pose
  auto localPoseComp = _ecm.Component<gz::sim::components::Pose>(entity);
  if (localPoseComp)
  {
    return localPoseComp->Data();
  }
  
  return gz::math::Pose3d::Zero;
}


// HumanSystemPlugin.cpp - deleteActorsCallback
void HumanSystemPlugin::deleteActorsCallback(
    const std::shared_ptr<arena_people_msgs::srv::DeleteActors::Request> request,
    std::shared_ptr<arena_people_msgs::srv::DeleteActors::Response> response)
{
    (void)request; // Unused parameter
    
    RCLCPP_INFO(this->rosnode_->get_logger(), "=== DELETE ACTORS CALLBACK CALLED ===");
    
    std::vector<gz::sim::Entity> actorsToDelete;
    
    // Collect all pedestrian actors
    for (const auto& [entity, agent] : pedestrians_) {
        actorsToDelete.push_back(entity);
        RCLCPP_INFO(this->rosnode_->get_logger(), "Marking actor %s (entity: %lu) for deletion", 
                   agent.name.c_str(), entity);
    }
    
    
    pedestrians_.clear();
    agents_initialized_ = false;
    current_pedestrians_.reset();
    first_update_ = false;  // Reset first_update flag
    
    
    entities_to_delete_ = actorsToDelete;
    delete_requested_ = true;
    
    response->success = true;
    response->deleted_count = static_cast<int32_t>(actorsToDelete.size());
    
    RCLCPP_INFO(this->rosnode_->get_logger(), "Marked %zu actors for deletion in next PreUpdate cycle", 
               actorsToDelete.size());
}




// Register the plugin 
GZ_ADD_PLUGIN(HumanSystemPlugin,gz::sim::System,HumanSystemPlugin::ISystemConfigure,HumanSystemPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(HumanSystemPlugin, "HumanSystemPlugin")