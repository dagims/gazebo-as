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

#ifndef _GAZEBO_AUDIO_HH_
#define _GAZEBO_AUDIO_HH_ 

#define ALSA_PCM_NEW_HW_PARAMS_API
#include <alsa/asoundlib.h>
#include <stdint.h>
#include <stdint.h>
#include <string>
#include <thread>
#include <mutex>

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \brief Classes and functions used by SteamAudio module
    /// \{

    /// \class Audio Audio.hh common/common.hh
    /// \brief Audio device manager for input and output
    class GZ_COMMON_VISIBLE Audio
    {
      /// \brief Constructor
      public: Audio(bool _captureMode = true,
                    std::string _deviceName = "default",
                    unsigned int _sampleRate = 44100,
                    unsigned long _frames = 64);

      /// \brief Destructor
      public: ~Audio();

      /// \brief Get Status
      /// \return True if running
      public: bool IsRunning() const;

      /// \brief Start Capture/Playback
      /// \return True if successfully started
      public: bool Start();

      /// \brief Stop Capture/Playback
      /// \return True if successfully paused
      public: bool Stop();

      /// \brief ShutDown
      public: void Shutdown();

      /// \brief Get Sample Rate
      /// \return Sample Rate
      public: unsigned int GetSampleRate() const;

      /// \brief Audio Write Status Codes
      public: typedef enum {
                       AUDIO_READ_OK,
                       AUDIO_READ_OVERRUN,
                       AUDIO_READ_ERROR,
                       AUDIO_SHORT_READ,
                       AUDIO_WRITE_OK,
                       AUDIO_WRITE_UNDERRUN,
                       AUDIO_WRITE_ERROR,
                       AUDIO_SHORT_WRITE,
                       AUDIO_INPUT_ERROR,
                       AUDIO_OBJ_ERROR}
                       AudioIOStatusCode;

      /// \brief Fill buffer with data from capture device
      /// \param[out] _buf a buffer for holding captured data
      /// \param[out] _numFrames the number of frames in buffer *_buf
      /// \return AudioReadStatusCode
      public: AudioIOStatusCode ReadFrames(float **_buf, long &_numFrames);

      /// \brief Write data to playback device
      /// \param[in] _buf buffer holding data to be written to device
      /// \param[in] _numFrames number of of frames in buffer
      /// \return AudioWriteStatusCode
      public: AudioIOStatusCode WriteFrames(float *_buf, long _numFrames);

      /// \brief Sample Rate
      private: unsigned int sampleRate;

      /// \brief Number of Frames
      private: unsigned long audioFrames;

      /// \brief Alsa Audio Device Handle
      private: snd_pcm_t *deviceHandle;

      /// \brief Audio Data Buffer
      private: float *audioBuffer;

      /// \brief Audio Data Buffer Size
      private: uint32_t bufferSize;

      /// \brief Audio Obj Mode -> Capture/Playback
      private: bool captureMode;

      /// \brief Audio Capture/Playback Running Status
      private: bool runningStatus;

      /// \internal
      private: int ret;
    };
    /// \}
  }
}

#endif //_GAZEBO_AUDIO_HH_
