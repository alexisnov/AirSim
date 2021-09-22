#include "MultirotorPawnSimApi.h"
#include "AirBlueprintLib.h"
#include "vehicles/multirotor/MultiRotorParamsFactory.hpp"
#include "UnrealSensors/UnrealSensorFactory.h"
#include <exception>
#include <Physics/JSBSimPhysicsBody.hpp>

using namespace msr::airlib;

MultirotorPawnSimApi::MultirotorPawnSimApi(const Params& params)
    : PawnSimApi(params), pawn_events_(static_cast<MultirotorPawnEvents*>(params.pawn_events))
{
}

void MultirotorPawnSimApi::initialize()
{
    PawnSimApi::initialize();

    //create vehicle API
    std::shared_ptr<UnrealSensorFactory> sensor_factory = std::make_shared<UnrealSensorFactory>(getPawn(), &getNedTransform());
    vehicle_params_ = MultiRotorParamsFactory::createConfig(getVehicleSetting(), sensor_factory);
    vehicle_api_ = vehicle_params_->createMultirotorApi();
    //setup physics vehicle
    if (getVehicleSetting()->vehicle_type == AirSimSettings::kVehicleTypeJSBSim) {
        jsbsim_body = new JSBSimPhysicsBody(vehicle_params_.get(), vehicle_api_.get(), getKinematics(), getEnvironment(), getVehicleSetting());
        rotor_count_ = jsbsim_body->wrenchVertexCount();
        rotor_actuator_info_.assign(rotor_count_, RotorActuatorInfo());
    }
    else {
        multirotor_physics_body_ = std::unique_ptr<MultiRotor>(new MultiRotorPhysicsBody(vehicle_params_.get(), vehicle_api_.get(), getKinematics(), getEnvironment()));
        rotor_count_ = multirotor_physics_body_->wrenchVertexCount();
        rotor_actuator_info_.assign(rotor_count_, RotorActuatorInfo());
    }

    vehicle_api_->setSimulatedGroundTruth(getGroundTruthKinematics(), getGroundTruthEnvironment());

    //initialize private vars
    last_phys_pose_ = Pose::nanPose();
    pending_pose_status_ = PendingPoseStatus::NonePending;
    reset_pending_ = false;
    did_reset_ = false;
    rotor_states_.rotors.assign(rotor_count_, RotorParameters());

    //reset roll & pitch of vehicle as multirotors required to be on plain surface at start
    Pose pose = getPose();
    float pitch, roll, yaw;
    VectorMath::toEulerianAngle(pose.orientation, pitch, roll, yaw);
    //pose.orientation = VectorMath::toQuaternion(0, 0, yaw);
    AirSimSettings::Rotation rotation = getVehicleSetting()->rotation;
    pose.orientation = VectorMath::toQuaternion(rotation.pitch * (M_PI / 180.0), rotation.roll * (M_PI / 180.0), rotation.yaw * (M_PI / 180.0));
    setPose(pose, false);
}

void MultirotorPawnSimApi::pawnTick(float dt)
{
    unused(dt);
    //calls to update* are handled by physics engine and in SimModeWorldBase
}

void MultirotorPawnSimApi::updateRenderedState(float dt)
{
    //Utils::log("------Render tick-------");

    //if reset is pending then do it first, no need to do other things until next tick
    if (reset_pending_) {
        reset_task_();
        did_reset_ = true;
        return;
    }

    //move collision info from rendering engine to vehicle
    const CollisionInfo& collision_info = getCollisionInfo();

    if (getVehicleSetting()->vehicle_type == AirSimSettings::kVehicleTypeJSBSim) {
        jsbsim_body->setCollisionInfo(collision_info);

        last_phys_pose_ = jsbsim_body->getPose();

        collision_response = jsbsim_body->getCollisionResponseInfo();

        //update rotor poses
        for (unsigned int i = 0; i < rotor_count_; ++i) {
            const auto& rotor_output = jsbsim_body->getRotorOutput(i);
            // update private rotor variable
            rotor_states_.rotors[i].update(rotor_output.thrust, rotor_output.torque_scaler, rotor_output.speed);
            RotorActuatorInfo* info = &rotor_actuator_info_[i];
            info->rotor_speed = rotor_output.speed;
            info->rotor_direction = static_cast<int>(rotor_output.turning_direction);
            info->rotor_thrust = rotor_output.thrust;
            info->rotor_control_filtered = rotor_output.control_signal_filtered;
        }
    }
    else {
        multirotor_physics_body_->setCollisionInfo(collision_info);

        last_phys_pose_ = multirotor_physics_body_->getPose();

        collision_response = multirotor_physics_body_->getCollisionResponseInfo();

        //update rotor poses
        for (unsigned int i = 0; i < rotor_count_; ++i) {
            const auto& rotor_output = multirotor_physics_body_->getRotorOutput(i);
            // update private rotor variable
            rotor_states_.rotors[i].update(rotor_output.thrust, rotor_output.torque_scaler, rotor_output.speed);
            RotorActuatorInfo* info = &rotor_actuator_info_[i];
            info->rotor_speed = rotor_output.speed;
            info->rotor_direction = static_cast<int>(rotor_output.turning_direction);
            info->rotor_thrust = rotor_output.thrust;
            info->rotor_control_filtered = rotor_output.control_signal_filtered;
        }
    }

    vehicle_api_->getStatusMessages(vehicle_api_messages_);

    if (getRemoteControlID() >= 0)
        vehicle_api_->setRCData(getRCData());
    rotor_states_.timestamp = clock()->nowNanos();
    vehicle_api_->setRotorStates(rotor_states_);
}

void MultirotorPawnSimApi::updateRendering(float dt)
{
    //if we did reset then don't worry about synchronizing states for this tick
    if (reset_pending_) {
        // Continue to wait for reset
        if (!did_reset_) {
            return;
        }
        else {
            reset_pending_ = false;
            did_reset_ = false;
            return;
        }
    }

    if (!VectorMath::hasNan(last_phys_pose_)) {
        if (pending_pose_status_ == PendingPoseStatus::RenderPending) {
            PawnSimApi::setPose(last_phys_pose_, pending_pose_collisions_);
            pending_pose_status_ = PendingPoseStatus::NonePending;
        }
        else
            PawnSimApi::setPose(last_phys_pose_, false);
    }

    //UAirBlueprintLib::LogMessage(TEXT("Collision (raw) Count:"), FString::FromInt(collision_response.collision_count_raw), LogDebugLevel::Unimportant);
    UAirBlueprintLib::LogMessage(TEXT("Collision Count:"),
                                 FString::FromInt(collision_response.collision_count_non_resting),
                                 LogDebugLevel::Informational);

    for (auto i = 0; i < vehicle_api_messages_.size(); ++i) {
        UAirBlueprintLib::LogMessage(FString(vehicle_api_messages_[i].c_str()), TEXT(""), LogDebugLevel::Success, 30);
    }

    try {
        vehicle_api_->sendTelemetry(dt);
    }
    catch (std::exception& e) {
        UAirBlueprintLib::LogMessage(FString(e.what()), TEXT(""), LogDebugLevel::Failure, 30);
    }

    pawn_events_->getActuatorSignal().emit(rotor_actuator_info_);
}

void MultirotorPawnSimApi::setPose(const Pose& pose, bool ignore_collision)
{
    if (getVehicleSetting()->vehicle_type == AirSimSettings::kVehicleTypeJSBSim) {
        jsbsim_body->lock();
        jsbsim_body->setPose(pose);
        jsbsim_body->setGrounded(false);
        jsbsim_body->unlock();
    }
    else {
        multirotor_physics_body_->lock();
        multirotor_physics_body_->setPose(pose);
        multirotor_physics_body_->setGrounded(false);
        multirotor_physics_body_->unlock();
    }
    pending_pose_collisions_ = ignore_collision;
    pending_pose_status_ = PendingPoseStatus::RenderPending;
}

//*** Start: UpdatableState implementation ***//
void MultirotorPawnSimApi::resetImplementation()
{
    PawnSimApi::resetImplementation();

    vehicle_api_->reset();
    if (getVehicleSetting()->vehicle_type == AirSimSettings::kVehicleTypeJSBSim) {
        jsbsim_body->reset();
    }
    else {
        multirotor_physics_body_->reset();
    }
    vehicle_api_messages_.clear();
}

//this is high frequency physics tick, flier gets ticked at rendering frame rate
void MultirotorPawnSimApi::update()
{
    //environment update for current position
    PawnSimApi::update();

    if (getVehicleSetting()->vehicle_type == AirSimSettings::kVehicleTypeJSBSim) {
        jsbsim_body->update();
    }
    else {
        //update forces on vertices
        multirotor_physics_body_->update();
    }

    //update to controller must be done after kinematics have been updated by physics engine
}

void MultirotorPawnSimApi::reportState(StateReporter& reporter)
{
    PawnSimApi::reportState(reporter);

    if (getVehicleSetting()->vehicle_type == AirSimSettings::kVehicleTypeJSBSim) {
        jsbsim_body->reportState(reporter);
    }
    else {
        multirotor_physics_body_->reportState(reporter);
    }
}

MultirotorPawnSimApi::UpdatableObject* MultirotorPawnSimApi::getPhysicsBody()
{
    if (getVehicleSetting()->vehicle_type == AirSimSettings::kVehicleTypeJSBSim) {
        return jsbsim_body->getPhysicsBody();
    }
    else {
        return multirotor_physics_body_->getPhysicsBody();
    }
}

double MultirotorPawnSimApi::getJSBSimProperty(const std::string& property_name)
{
    if (getVehicleSetting()->vehicle_type == AirSimSettings::kVehicleTypeJSBSim) {
        return jsbsim_body->getJSBSimProperty(property_name);
    }

    return NAN;
}
void MultirotorPawnSimApi::setJSBSimProperty(const std::string& property_name, double property_value)
{
    if (getVehicleSetting()->vehicle_type == AirSimSettings::kVehicleTypeJSBSim) {
        jsbsim_body->setJSBSimProperty(property_name, property_value);
    }
}
//*** End: UpdatableState implementation ***//
