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
#include "gazebo/util/UtilTypes.hh"

#include "gazebo/gazebo_config.h"
#include "gazebo/util/system.hh"

#ifdef HAVE_STEAMAUDIO

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

      /// \brief Create an SteamAudioGenerator object.
      /// \param[in] _sdf SDF element parameters for an audio_generator
      /// \return A pointer to an SteamAudioGenerator object.
      public: SteamAudioGeneratorPtr CreateSource(sdf::ElementPtr _sdf);

      /// \brief Create an audio listener.
		/// Only one listener may be created.
      /// \param[in] _sdf SDF element parameters for an audio_listener
      /// \return A pointer to an SteamAudioListener object.
      public: SteamAudioListenerPtr CreateListener(sdf::ElementPtr _sdf);

      /// \brief Get a list of available audio devices
      /// \return A list of audio device names
      public: std::set<std::string> DeviceList() const;

      /// \brief Set the SOFA file
      /// \param[in] The URI of the SOFA file
      /// \return True if the file was successfully loaded
      public: bool SetSOFA(const std::string &_filename);

      /// \brief Get url of the SOFA file
      /// \return The URI of the SOFA file, empty if none loaded
      public: std::string GetSOFAuri() const;

      /// \internal
      /// \brief Callback for log messages from SteamAudio
      private: IPLvoid steamLogCallback(char *msg);

      /// \internal
      /// \brief AudioDecoder object
      private: common::AudioDecoder *audiodecoder;

      /// \internal
      /// \brief SteamAudio Context
      private: IPLhandle context{nullptr};

      /// \internal
      /// \brief SteamAudio Binaural Renderer
      private: IPLhandle binauralRenderer{nullptr};

      /// \internal
      /// \brief SteamAudio Direction Vector
      private: IPLVector3 listenerDirection;

      /// \internal
      /// \brief SteamAudio Binaural Effect
      private: IPLhandle binauralEffect{nullptr};

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

      /// \brief This is a singleton
      private: friend class SingletonT<SteamAudio>;
    };

    /// \class SteamAudioListener SteamAudio.hh util/util.hh
    /// \brief SteamAudio Listener. This can be thought of as a microphone.
    class GZ_UTIL_VISIBLE SteamAudioListener
    {
      /// \brief Constructor
      public: SteamAudioListener();

      /// \brief Destructor
      public: virtual ~SteamAudioListener();

      /// \brief Set the position of the listener
      /// \param[in] _pose New pose of the listener
      /// \return True on success.
      public: bool SetPose(const ignition::math::Pose3d &_pose);
    };

    /// \class SteamAudioGenerator SteamAudioGenerator.hh util/util.hh
    /// \brief SteamAudio Source. This can be thought of as a speaker.
    class GZ_UTIL_VISIBLE SteamAudioGenerator
    {
      /// \brief Constructor.
      public: SteamAudioGenerator();

      /// \brief Destructor.
      public: ~SteamAudioGenerator();

      /// \brief Load the source from sdf.
      /// \param[in] _sdf SDF element parameters for an audio_source.
      /// \return True on success.
      public: bool Load(sdf::ElementPtr _sdf);

      /// \brief Set the position of the source.
      /// \param[in] _pose New pose of the source.
      /// \return True on success.
      public: bool SetPose(const ignition::math::Pose3d &_pose);

      /// \brief Play a sound
      public: void Play();

      /// \brief Pause a sound
      public: void Pause();

      /// \brief Is the audio playing
      public: bool IsPlaying();

      /// \brief Fill the SteamAudio audio buffer from PCM data
      /// \param[in] _pcmData Pointer to the PCM audio data.
      /// \param[in] _dataCount Size of the PCM data.
      /// \param[in] _sampleRate Sample rate for the PCM data.
      /// \return True on success.
      public: bool FillBufferFromPCM(float *_pcmData, unsigned long _dataCount,
                                     unsigned int _sampleRate);

      /// \brief Fill the SteamAudio audio buffer with data from a sound file.
      /// \param[in] _audioFile Name and an audio file.
      public: void FillBufferFromFile(const std::string &_audioFile);

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<SteamAudioGeneratorPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
#endif
