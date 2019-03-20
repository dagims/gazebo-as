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
#ifndef _GAZEBO_UTIL_STEAMAUDIO_HH_
#define _GAZEBO_UTIL_STEAMAUDIO_HH_

#include <algorithm>
#include <iterator>
#include <vector>

#include <phonon.h>

#include <sdf/sdf.hh>

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/common/SingletonT.hh"
#include "gazebo/common/common.hh"
#include "gazebo/util/UtilTypes.hh"

#include "gazebo/gazebo_config.h"
#include "gazebo/util/system.hh"

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"



namespace gazebo
{
  namespace util
  {
    /// \addtogroup gazebo_util Utility
    /// \{

    /// \class SteamAudio SteamAudio.hh util/util.hh
    /// \brief Physics Based Audio Simulation
    class GZ_UTIL_VISIBLE SteamAudio : public SingletonT<SteamAudio>
    {
      /// \brief Constructor
      private: SteamAudio();

      /// \brief Destructor
      private: ~SteamAudio();

      /// \brief Load the SteamAudio server.
      /// \return True on success.
      public: bool Load(sdf::ElementPtr _sdf = sdf::ElementPtr());

      /// \brief Finalize.
      public: void Fini();

      /// \brief Get a list of available audio devices
      /// \return A list of audio device names
      public: std::set<std::string> DeviceList() const;

      /// \brief Initialize SteamAudio
      private: void Init();
      
      /// \brief Set the SOFA file
      /// \param[in] The URI of the SOFA file
      public: void SetSOFA(const std::string &_filename);

      /// \brief Get url of the SOFA file
      /// \return The URI of the SOFA file, empty if none loaded
      public: std::string GetSOFAuri() const;

      /// \brief Set Listener Pose
      /// \param[in] World Pose of the Audio Listener link
      public: void SetListenerPose(const ignition::math::Pose3d &_pose);

      /// \brief Set Generator Pose
      /// \param[in] World Pose of the Audio Generator Link
      public: void SetGeneratorPose(const ignition::math::Pose3d &_pose);

      /// \brief Apply binaural effect
      /// \param[in] _buf input audio
      /// \param[in] _bufSize number of samples in buffer
      /// \return std::vector<float> of the output audio
      public: std::vector<float> SteamBinauralEffect(float *_buf, long _bufSize);

      /// \internal
      /// \brief Convert Gazebo style mesh to Steam style
      /// \param[in] std::string _meshURI mesh uri from sdf
      /// \return std::vector<IPLTriangle> Steam style mesh triangles
      private: void ConvertMesh(std::string _meshURI);

      
      /// \brief Steam Error Check
      private: bool SteamErrorCheck(IPLerror errCode);

      /// \internal
      /// \brief WorldUpdate Event Callback
      private: void Update();

      /// \internal
      /// \brief WorldCreated Event Callback
      private: void WorldCreated();

      /// \internal
      /// \brief Audio Generator Pose
      private: ignition::math::Pose3d generatorPose;

      /// \internal
      /// \brief Audio Listener Pose
      private: ignition::math::Pose3d listenerPose;

      /// \internal
      /// \brief Audio Listener Relative Location
      private: ignition::math::Vector3d listenerLocation;

      /// \internal
      /// \brief SteamAudio Direction Vector
      private: IPLVector3 listenerDirection;

      /// \internal
      /// \brief Audio Input object
      private: event::ConnectionPtr worldCreatedEventCon;

      /// \internal
      /// \brief Audio Input object
      private: event::ConnectionPtr worldUpdateEventCon;

      /// \internal
      /// \brief Audio Input object
      private: common::Audio *iAudio;

      /// \internal
      /// \brief Audio Output object
      private: common::Audio *oAudio;

      /// \internal
      /// \brief Audio Buffer
      private: float *audioBuffer;

      /// \internal
      /// \brief Audio Buffer Size
      private: long bufferSize;

      /// \internal
      /// \brief SteamAudio Context
      private: IPLhandle steamContext{nullptr};

      /// \internal
      /// \brief SteamAudio Rendering settings
      private: IPLRenderingSettings steamRendSettings;

      /// \internal
      /// \brief SteamAudio Mono Audio Format
      private: IPLAudioFormat monoAudio;

      /// \internal
      /// \brief SteamAudio Stereo Audio Format
      private: IPLAudioFormat stereoAudio;

      /// \internal
      /// \brief SteamAudio Binaural Renderer
      private: IPLhandle steamBinauralRenderer{nullptr};

      /// \internal
      /// \brief SteamAudio Binaural Effect
      private: IPLhandle steamBinauralEffect{nullptr};

      /// \internal
      /// \brief SteamAudio Scene
      private: IPLhandle steamScene{nullptr};

      /// \internal
      /// \brief SteamAudio Environment Object
      private: IPLhandle steamEnvObj{nullptr};

      /// \internal
      /// \brief SteamAudio Environmental Renderer
      private: IPLhandle steamEnvRenderer{nullptr};

      /// \internal
      /// \brief SteamAudio Direct Sound Path
      private: IPLDirectSoundPath steamDirectSoundPath;

      /// \internal
      /// \brief SteamAudio Direct Sound Effect Options
      private: IPLDirectSoundEffectOptions directSoundMode;

      /// \internal
      /// \brief SteamAudio Direct Sound Effect
      private: IPLhandle steamDirectSoundEffect{nullptr};

      /// \internal
      /// \brief Input Audio Buffer
      private: std::vector<float> inputAudio;

      /// \internal
      /// \brief SteamAudio Input Audio Buffer - mono
      private: IPLAudioBuffer inputAudioBuffer;

      /// \internal
      /// \brief Output Audio Buffer
      private: std::vector<float> outputAudio;

      /// \internal
      /// \brief SteamAudio Output Audio Buffer - stereo
      private: IPLAudioBuffer outputAudioBuffer;

      /// \internal
      /// \brief SOFA file
      private: std::string SOFAfile;

      /// \internal
      /// \brief Indices of Triangles of Meshes
      private: std::vector<std::vector<IPLTriangle> > triangles;

      /// \internal
      /// \brief Vertices of Triangles of Meshes
      private: std::vector<std::vector<IPLVector3> > vertices;

      /// \internal
      /// \brief Material name
      private: std::vector<std::string> materialName;

      /// \internal
      /// \brief Preset Material Acoustic Properties
      private: std::map<std::string, IPLMaterial> materialAcousticProp;

      /// \internal
      /// \brief Material Acoustic Properties
      private: std::vector<IPLMaterial> materialProperties;

      /// \internal
      /// \brief Steam Audio Static Mesh Objects
      private: std::vector<IPLhandle> staticMeshes;

      /// \brief This is a singleton
      private: friend class SingletonT<SteamAudio>;
    };
    /// \}
  }
}
#endif
