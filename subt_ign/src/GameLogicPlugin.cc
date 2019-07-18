/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <tinyxml2.h>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/float.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <chrono>
#include <map>
#include <mutex>
#include <utility>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/common/Time.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "subt_ign/CommonTypes.hh"
#include "subt_ign/GameLogicPlugin.hh"
#include "subt_ign/protobuf/artifact.pb.h"

using namespace ignition;
using namespace subt;

class subt::GameLogicPluginPrivate
{
  /// \brief Mapping between enum types and strings.
  public: const std::array<
      const std::pair<subt::ArtifactType, std::string>, 10> kArtifactTypes
      {
        {
          {subt::ArtifactType::TYPE_BACKPACK      , "TYPE_BACKPACK"},
          {subt::ArtifactType::TYPE_DRILL         , "TYPE_DRILL"},
          {subt::ArtifactType::TYPE_DUCT          , "TYPE_DUCT"},
          {subt::ArtifactType::TYPE_ELECTRICAL_BOX, "TYPE_ELECTRICAL_BOX"},
          {subt::ArtifactType::TYPE_EXTINGUISHER  , "TYPE_EXTINGUISHER"},
          {subt::ArtifactType::TYPE_PHONE         , "TYPE_PHONE"},
          {subt::ArtifactType::TYPE_RADIO         , "TYPE_RADIO"},
          {subt::ArtifactType::TYPE_RESCUE_RANDY  , "TYPE_RESCUE_RANDY"},
          {subt::ArtifactType::TYPE_TOOLBOX       , "TYPE_TOOLBOX"},
          {subt::ArtifactType::TYPE_VALVE         , "TYPE_VALVE"}
        }
      };

  /// \brief Write a simulation timestamp to a logfile.
  /// \return A file stream that can be used to write additional
  /// information to the logfile.
  public: std::ofstream &Log();

  /// \brief Handle gazebo pose messages
  /// \param[in] _msg New set of poses.
  public: void OnPose(const ignition::msgs::Pose_V &_msg);

  /// \brief Create an ArtifactType from a string.
  /// \param[in] _name The artifact in string format.
  /// \param[out] _type The artifact type.
  /// \return True when the conversion succeed or false otherwise.
  public: bool ArtifactFromString(const std::string &_name,
                                  subt::ArtifactType &_type);

  /// \brief Calculate the score of a new artifact request.
  /// \param[in] _type The object type. See ArtifactType.
  /// \param[in] _pose The object pose.
  /// \return The score obtained for this object.
  public: double ScoreArtifact(const subt::ArtifactType &_type,
                               const ignition::msgs::Pose &_pose);

  /// \brief Create an ArtifactType from an integer.
  //
  /// \param[in] _typeInt The artifact in int format.
  /// \param[out] _type The artifact type.
  /// \return True when the conversion succeed or false otherwise.
  public: bool ArtifactFromInt(const uint32_t &_typeInt,
                               subt::ArtifactType &_type);

  /// \brief Callback executed to process a new artifact request
  /// sent by a team.
  /// \param[in] _req The service request.
  public: bool OnNewArtifact(const subt::msgs::Artifact &_req,
                             subt::msgs::ArtifactScore &_resp);

  /// \brief Parse all the artifacts.
  /// \param[in] _sdf The SDF element containing the artifacts.
  public: void ParseArtifacts(const tinyxml2::XMLElement *_elem);

  /// \brief Publish the score.
  /// \param[in] _event Unused.
  public: void PublishScore();

  public: void Finish();

  /// \brief Ignition service callback triggered when the service is called.
  /// \param[in]  _req The message containing the robot name.
  /// \param[out] _res The response message.
  /// \return true on success.
  public: bool OnPoseFromArtifact(
               const ignition::msgs::StringMsg &_req,
               ignition::msgs::Pose &_res);


  private: bool PoseFromArtifactHelper(const std::string &_robot,
    ignition::math::Pose3d &_result);

  /// \brief Ignition service callback triggered when the service is called.
  /// \param[in] _req The message containing a flag telling if the game
  /// is to start.
  /// \param[out] _res The response message.
  public: bool OnStartCall(const ignition::msgs::Boolean &_req,
                            ignition::msgs::Boolean &_res);

  /// \brief Ignition service callback triggered when the service is called.
  /// \param[in] _req The message containing a flag telling if the game is to
  /// be finished.
  /// \param[out] _res The response message.
  public: bool OnFinishCall(const ignition::msgs::Boolean &_req,
               ignition::msgs::Boolean &_res);

  /// \brief Ignition Transport node.
  public: transport::Node node;

  /// \brief Current simulation time.
  public: ignition::msgs::Time simTime;

  /// \brief Thread on which scores are published
  public: std::unique_ptr<std::thread> publishThread = nullptr;

  /// \brief Whether the task has started.
  public: bool started = false;

  /// \brief Whether the task has finished.
  public: bool finished = false;

  /// \brief Start time used for scoring.
  public: std::chrono::steady_clock::time_point startTime;

  /// \brief Finish time used for scoring.
  public: std::chrono::steady_clock::time_point finishTime;

  /// \brief Store all artifacts.
  /// The key is the object type. See ArtifactType for all supported values.
  /// The value is another map, where the key is the model name and the value
  /// is a Model pointer.
  public: std::map<subt::ArtifactType,
                    std::map<std::string, ignition::math::Pose3d>> artifacts;

  public: std::map<std::string, ignition::math::Pose3d> poses;

  /// \brief Counter to track unique identifiers.
  public: uint32_t reportCount = 1u;

  /// \brief Total score.
  public: double totalScore = 0.0;

  /// \brief A mutex.
  public: std::mutex mutex;

  /// \brief Log file output stream.
  public: std::ofstream logStream;

  /// \brief The pose of the object marking the origin of the artifacts.
  public: ignition::math::Pose3d artifactOriginPose;

  /// \brief Ignition transport start publisher. This is needed by cloudsim
  /// to know when a run has been started.
  public: transport::Node::Publisher startPub;
};

//////////////////////////////////////////////////
GameLogicPlugin::GameLogicPlugin()
  : dataPtr(new GameLogicPluginPrivate)
{
}

//////////////////////////////////////////////////
GameLogicPlugin::~GameLogicPlugin()
{
  this->dataPtr->finished = true;
  if (this->dataPtr->publishThread)
    this->dataPtr->publishThread->join();
}

//////////////////////////////////////////////////
bool GameLogicPlugin::Load(const tinyxml2::XMLElement *_elem)
{
  // Default log path is /dev/null.
  std::string logPath = "/dev/null";

  // Check if the game logic plugin has a <logging> element.
  // The <logging> element can contain a <filename_prefix> child element.
  // The <filename_prefix> is used to specify the log filename prefix. For
  // example:
  // <logging>
  //   <path>/tmp</path>
  //   <filename_prefix>subt_tunnel_qual</filename_prefix>
  // </logging>
  const tinyxml2::XMLElement *loggingElem = _elem->FirstChildElement("logging");
  const tinyxml2::XMLElement *fileElem = nullptr;
  if (loggingElem &&
      (fileElem = loggingElem->FirstChildElement("filename_prefix")))
  {
    // Get the log filename prefix.
    std::string filenamePrefix = fileElem->GetText();
    const tinyxml2::XMLElement *pathElem =
      loggingElem->FirstChildElement("path");

    // Get the logpath from the <path> element, if it exists.
    if (pathElem)
    {
      logPath = pathElem->GetText();
    }
    else
    {
      // Make sure that we can access the HOME environment variable.
      char *homePath = getenv("HOME");
      if (!homePath)
      {
        ignerr << "Unable to get HOME environment variable. Report this error "
          << "to https://bitbucket.org/osrf/subt/issues/new. "
          << "SubT logging will be disabled.\n";
      }
      else
      {
        logPath = homePath;
      }
    }

    // Construct the final log filename.
    logPath += "/" + filenamePrefix + "_" +
      ignition::common::systemTimeISO() + ".log";
  }

  // Open the log file.
  this->dataPtr->logStream.open(logPath.c_str(), std::ios::out);

  // Advertise the service to receive artifact reports.
  // Note that we're setting the scope to this service to SCOPE_T, so only
  // nodes within the same process will be able to reach this plugin.
  // The reason for this is to avoid the teams to use this service directly.
  ignition::transport::AdvertiseServiceOptions opts;
  opts.SetScope(ignition::transport::Scope_t::PROCESS);
  this->dataPtr->node.Advertise(kNewArtifactSrv,
    &GameLogicPluginPrivate::OnNewArtifact, this->dataPtr.get(), opts);

  this->dataPtr->ParseArtifacts(_elem);

  const tinyxml2::XMLElement *worldNameElem =
    _elem->FirstChildElement("world_name");
  std::string worldName = "default";
  if (worldNameElem)
  {
    worldName = worldNameElem->GetText();
  }
  else
  {
    ignerr << "Missing <world_name>, the GameLogicPlugin will assume a "
      << " world name of 'default'. This could lead to incorrect scoring\n";
  }

  // Subscribe to pose messages. We will pull out model and artifact
  // information from the published message.
  this->dataPtr->node.Subscribe("/world/" + worldName + "/pose/info",
      &GameLogicPluginPrivate::OnPose, this->dataPtr.get());

  this->dataPtr->node.Advertise("/subt/pose_from_artifact_origin",
      &GameLogicPluginPrivate::OnPoseFromArtifact, this->dataPtr.get());

  this->dataPtr->node.Advertise("/subt/start",
      &GameLogicPluginPrivate::OnStartCall, this->dataPtr.get());

  this->dataPtr->node.Advertise("/subt/finish",
      &GameLogicPluginPrivate::OnFinishCall, this->dataPtr.get());

  this->dataPtr->startPub =
    this->dataPtr->node.Advertise<ignition::msgs::StringMsg>("/subt/start");

  this->dataPtr->publishThread.reset(new std::thread(
        &GameLogicPluginPrivate::PublishScore, this->dataPtr.get()));

  ignmsg << "Starting SubT" << std::endl;

  return true;
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::OnPose(const ignition::msgs::Pose_V &_msg)
{
  // Store sim time if present
  if (_msg.has_header() && _msg.header().has_stamp())
    this->simTime = _msg.header().stamp();

  // Check the all the published poses
  for (int i = 0; i < _msg.pose_size(); ++i)
  {
    const ignition::msgs::Pose &pose = _msg.pose(i);
    this->poses[pose.name()] = ignition::msgs::Convert(pose);

    // Update artifact positions.
    for (std::pair<const subt::ArtifactType,
         std::map<std::string, ignition::math::Pose3d>> &artifactPair :
         this->artifacts)
    {
      for (std::pair<const std::string, ignition::math::Pose3d> &artifact :
          artifactPair.second)
      {
        if (artifact.first == pose.name())
        {
          artifact.second = ignition::msgs::Convert(pose);
          break;
        }
      }
    }
  }
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnNewArtifact(const subt::msgs::Artifact &_req,
                                           subt::msgs::ArtifactScore &_resp)
{
  this->Log() << "new_artifact_reported" << std::endl;
  auto realTime = std::chrono::steady_clock::now().time_since_epoch();
  auto s = std::chrono::duration_cast<std::chrono::seconds>(realTime);
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(realTime-s);

  _resp.set_report_id(reportCount++);
  *_resp.mutable_artifact() = _req;

  _resp.mutable_submitted_datetime()->set_sec(s.count());
  _resp.mutable_submitted_datetime()->set_nsec(ns.count());
  *_resp.mutable_sim_time() = this->simTime;

  // TODO(anyone) Where does run information come from?
  _resp.set_run(1);

  ArtifactType artifactType;

  if (this->started && this->finished)
  {
    _resp.set_report_status("scoring finished");
  }
  else if (!this->started && !this->finished)
  {
    _resp.set_report_status("run not started");
  }
  else if (this->artifacts.size() == 0)
  {
    _resp.set_report_status("report limit exceeded");
  }
  else if (!this->ArtifactFromInt(_req.type(), artifactType))
  {
    ignerr << "Unknown artifact code. The number should be between 0 and "
          << this->kArtifactTypes.size() - 1 << " but we received "
          << _req.type() << std::endl;

    this->Log() << "error Unknown artifact code. The number should be between "
                << "0 and " << this->kArtifactTypes.size() - 1
                << " but we received " << _req.type() << std::endl;
    _resp.set_report_status("scored");
  }
  else
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    auto scoreDiff = this->ScoreArtifact(artifactType, _req.pose());
    _resp.set_score_change(scoreDiff);
    _resp.set_report_status("scored");
    this->totalScore += scoreDiff;

    ignmsg << "Total score: " << this->totalScore << std::endl;
    this->Log() << "new_total_score " << this->totalScore << std::endl;
  }

  this->Log() << _resp.DebugString() << std::endl;
  return true;
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::ArtifactFromInt(const uint32_t &_typeInt,
    ArtifactType &_type)
{
  if (_typeInt > this->kArtifactTypes.size())
    return false;

  _type = static_cast<ArtifactType>(_typeInt);
  return true;
}

/////////////////////////////////////////////////
double GameLogicPluginPrivate::ScoreArtifact(const ArtifactType &_type,
  const ignition::msgs::Pose &_pose)
{
  // Sanity check: Make sure that we have crossed the starting gate.
  if (!this->started)
  {
    ignmsg << "  The task hasn't started yet" << std::endl;
    this->Log() << "task_not_started" << std::endl;
    return 0.0;
  }

  // Sanity check: Make sure that we still have artifacts.
  if (this->artifacts.find(_type) == this->artifacts.end())
  {
    ignmsg << "  No artifacts remaining" << std::endl;
    this->Log() << "no_remaining_artifacts_of_specified_type" << std::endl;
    return 0.0;
  }

  // The teams are reporting the artifact poses relative to the fiducial located
  // in the staging area. Now, we convert the reported pose to world coordinates
  ignition::math::Pose3d artifactPose = ignition::msgs::Convert(_pose);
  ignition::math::Pose3d pose = artifactPose +
    this->artifactOriginPose;

  double score = 0.0;
  std::map<std::string, ignition::math::Pose3d> &potentialArtifacts =
    this->artifacts[_type];

  // From the list of potential artifacts, find out which one is
  // closer (Euclidean distance) to the located by this request.
  ignition::math::Vector3d observedObjectPose = pose.Pos();
  std::tuple<std::string, ignition::math::Vector3d, double> minDistance =
    {"", ignition::math::Vector3d(), std::numeric_limits<double>::infinity()};
  for (const std::pair<std::string, ignition::math::Pose3d> &object :
       potentialArtifacts)
  {
    double distance = observedObjectPose.Distance(object.second.Pos());

    if (distance < std::get<2>(minDistance))
    {
      std::get<0>(minDistance) = object.first;
      std::get<1>(minDistance) = object.second.Pos();
      std::get<2>(minDistance) = distance;
    }
  }

  // Calculate the score based on accuracy in the location.
  if (std::get<2>(minDistance) < 5)
  {
    score = 1.0;
    // Remove this artifact to avoid getting score from the same artifact
    // multiple times.
    potentialArtifacts.erase(std::get<0>(minDistance));
    if (potentialArtifacts.empty())
      this->artifacts.erase(_type);
  }

  ignmsg << "  [Total]: " << score << std::endl;
  this->Log() << "modified_score " << score << std::endl;

  return score;
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::ArtifactFromString(const std::string &_name,
    ArtifactType &_type)
{
  auto pos = std::find_if(
    std::begin(this->kArtifactTypes),
    std::end(this->kArtifactTypes),
    [&_name](const typename std::pair<ArtifactType, std::string> &_pair)
    {
      return (std::get<1>(_pair) == _name);
    });

  if (pos == std::end(this->kArtifactTypes))
    return false;

  _type = std::get<0>(*pos);
  return true;
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::ParseArtifacts(const tinyxml2::XMLElement *_elem)
{
  const tinyxml2::XMLElement *artifactElem =
    _elem->FirstChildElement("artifact");
  while (artifactElem)
  {
    const tinyxml2::XMLElement *nameElem =
      artifactElem->FirstChildElement("name");

    // Sanity check: "Name" is required.
    if (!nameElem)
    {
      ignerr << "[GameLogicPlugin]: Parameter <name> not found. Ignoring this "
            << "artifact" << std::endl;
      this->Log() << "error Parameter <name> not found. Ignoring this artifact"
                  << std::endl;
      artifactElem = artifactElem->NextSiblingElement("artifact");
      continue;
    }
    std::string modelName = nameElem->GetText();

    const tinyxml2::XMLElement *typeElem =
      artifactElem->FirstChildElement("type");
    // Sanity check: "Type" is required.
    if (!typeElem)
    {
      ignerr << "[GameLogicPlugin]: Parameter <type> not found. Ignoring this "
        << "artifact" << std::endl;
      this->Log() << "error Parameter <type> not found. Ignoring this artifact"
                  << std::endl;

      artifactElem = artifactElem->NextSiblingElement("artifact");
      continue;
    }

    // Sanity check: Make sure that the artifact type is supported.
    std::string modelTypeStr = typeElem->GetText();
    ArtifactType modelType;
    if (!this->ArtifactFromString(modelTypeStr, modelType))
    {
      ignerr << "[GameLogicPlugin]: Unknown artifact type ["
        << modelTypeStr << "]. Ignoring artifact" << std::endl;
      this->Log() << "error Unknown artifact type ["
                  << modelTypeStr << "]. Ignoring artifact" << std::endl;
      artifactElem = artifactElem->NextSiblingElement("artifact");
      continue;
    }

    // Sanity check: The artifact shouldn't be repeated.
    if (this->artifacts.find(modelType) != this->artifacts.end())
    {
      const std::map<std::string, ignition::math::Pose3d> &modelNames =
        this->artifacts[modelType];

      if (modelNames.find(modelName) != modelNames.end())
      {
        ignerr << "[GameLogicPlugin]: Repeated model with name ["
          << modelName << "]. Ignoring artifact" << std::endl;
        this->Log() << "error Repeated model with name ["
                    << modelName << "]. Ignoring artifact" << std::endl;
        artifactElem = artifactElem->NextSiblingElement("artifact");
        continue;
      }
    }

    ignmsg << "Adding artifact name[" << modelName << "] type["
      << modelTypeStr << "]\n";
    this->artifacts[modelType][modelName] = ignition::math::Pose3d::Zero;
        artifactElem = artifactElem->NextSiblingElement("artifact");
  }
}

/////////////////////////////////////////////////
void GameLogicPluginPrivate::PublishScore()
{
  transport::Node::Publisher scorePub =
    this->node.Advertise<ignition::msgs::Float>("/subt/score");
  ignition::msgs::Float msg;

  while (!this->finished)
  {
    msg.set_data(this->totalScore);

    scorePub.Publish(msg);
    IGN_SLEEP_S(1);
  }
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnFinishCall(const ignition::msgs::Boolean &_req,
  ignition::msgs::Boolean &_res)
{
  if (this->started && _req.data() && !this->finished)
  {
    this->Finish();
    _res.set_data(true);
  }
  else
    _res.set_data(false);

  return true;
}


/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnStartCall(const ignition::msgs::Boolean &_req,
  ignition::msgs::Boolean &_res)
{
  if (_req.data() && !this->started && !this->finished)
  {
    _res.set_data(true);
    this->started = true;
    this->startTime = std::chrono::steady_clock::now();
    ignmsg << "Scoring has Started" << std::endl;
    this->Log() << "scoring_started" << std::endl;

    ignition::msgs::StringMsg msg;
    msg.mutable_header()->mutable_stamp()->CopyFrom(this->simTime);
    msg.set_data("started");
    this->startPub.Publish(msg);
  }
  else
    _res.set_data(false);

  return true;
}


/////////////////////////////////////////////////
void GameLogicPluginPrivate::Finish()
{
  if (this->started && !this->finished)
  {
    this->finished = true;
    this->finishTime = std::chrono::steady_clock::now();

    auto elapsed = this->finishTime - this->startTime;
    ignmsg << "Scoring has finished. Elapsed time: "
          << std::chrono::duration_cast<std::chrono::seconds>(elapsed).count()
          << " seconds" << std::endl;
    this->Log() << "finished_elapsed_time "
      << std::chrono::duration_cast<std::chrono::seconds>(elapsed).count()
      << " s." << std::endl;
    this->Log() << "finished_score " << this->totalScore << std::endl;
    this->logStream.flush();
  }
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::PoseFromArtifactHelper(const std::string &_robot,
    ignition::math::Pose3d &_result)
{
  // Get an iterator to the robot's pose.
  std::map<std::string, ignition::math::Pose3d>::iterator robotIter =
    this->poses.find(_robot);

  if (robotIter == this->poses.end())
  {
    ignerr << "[GameLogicPlugin]: Unable to find robot with name ["
           << _robot << "]. Ignoring PoseFromArtifact request" << std::endl;
    return false;
  }

  // Get an iterator to the base station's pose.
  std::map<std::string, ignition::math::Pose3d>::iterator baseIter =
    this->poses.find(subt::kBaseStationName);

  // Sanity check: Make sure that the robot is in the stagging area, as this
  // service is only available in that zone.
  if (baseIter == this->poses.end())
  {
    ignerr << "[GameLogicPlugin]: Unable to find the staging area  ["
      << subt::kBaseStationName
      << "]. Ignoring PoseFromArtifact request" << std::endl;
    return false;
  }

  if (baseIter->second.Pos().Distance(robotIter->second.Pos()) > 15)
  {
    ignerr << "[GameLogicPlugin]: Robot [" << _robot << "] is too far from the "
      << "staging area. Ignoring PoseFromArtifact request" << std::endl;
    return false;
  }

  // Get the artifact origin's pose.
  std::map<std::string, ignition::math::Pose3d>::iterator originIter =
    this->poses.find(subt::kArtifactOriginName);
  if (originIter == this->poses.end())
  {
    ignerr << "[GameLogicPlugin]: Unable to find the artifact origin ["
      << subt::kArtifactOriginName
      << "]. Ignoring PoseFromArtifact request" << std::endl;
    return false;
  }
  this->artifactOriginPose = originIter->second;
  std::cout << "\n\n\nAOP[" << this->artifactOriginPose << "]\n\n";

  // Pose.
  _result = robotIter->second - this->artifactOriginPose;
  return true;
}

/////////////////////////////////////////////////
bool GameLogicPluginPrivate::OnPoseFromArtifact(
    const ignition::msgs::StringMsg &_req,
    ignition::msgs::Pose &_res)
{
  ignition::math::Pose3d pose;
  bool result = this->PoseFromArtifactHelper(_req.data(), pose);
  ignition::msgs::Set(&_res, pose);

  _res.mutable_header()->mutable_stamp()->CopyFrom(this->simTime);
  ignition::msgs::Header::Map *frame = _res.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(subt::kArtifactOriginName);

  return result;
}

/////////////////////////////////////////////////
std::ofstream &GameLogicPluginPrivate::Log()
{
  this->logStream << this->simTime.sec()
                  << " " << this->simTime.nsec() << " ";
  return this->logStream;
}
