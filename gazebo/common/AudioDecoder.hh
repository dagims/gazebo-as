/*
* Copyright (C) 2012 Open Source Robotics Foundation
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

#ifndef _GAZEBO_AUDIO_DECODER_HH_
#define _GAZEBO_AUDIO_DECODER_HH_

#include <stdint.h>
#include <string>
#include "gazebo/util/system.hh"

//newer alsa api
#define ALSA_PCM_NEW_HW_PARAMS_API
#include <alsa/asoundlib.h>

struct AVFormatContext;
struct AVCodecContext;
struct AVCodec;

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \brief Classes and functions used by multiple modules.
    /// \{

    /// \class AudioDecoder AudioDecoder.hh common/common.hh
    /// \brief An audio decoder based on FFMPEG.
    class GZ_COMMON_VISIBLE AudioDecoder
    {
      /// \brief Constructor.
      public: AudioDecoder();

      /// \brief Constructor for recording.
      public: AudioDecoder(const std::string &_dev);

      /// \brief Destructor.
      public: virtual ~AudioDecoder();

      /// \brief Set the file to decode.
      /// \param[in] _filename Path to an audio file.
      /// \return True if the file was successfull opened.
      public: bool SetFile(const std::string &_filename);

      /// \brief Get the audio filename that was set.
      /// \return The name of the set audio file.
      /// \sa AudioDecoder::SetFile
      public: std::string GetFile() const;

      /// \brief Decode the loaded audio file.
      /// \sa AudioDecoder::SetFile
      /// \param[out] _outBuffer Buffer that holds the decoded audio data.
      /// \param[out] _outBufferSize Size of the _outBuffer.
      /// \return True if decoding was succesful.
      public: bool Decode(uint8_t **_outBuffer, unsigned int *_outBufferSize);

      /// \brief Get the sample rate from the latest decoded file.
      /// \return Integer sample rate, such as 44100.
      public: int GetSampleRate();

      /// \brief Free audio object, close files, streams.
      private: void Cleanup();

      /// \brief Get Frames from Capture
      /// \param[out] _buf a buffer holding captured data
      /// \param[out] _numFrames in buffer holding captured data, -1 if error.
      private: void GetFrames(float **_buf, unsigned long *_numFrames);

      /// \brief libav Format I/O context.
      private: AVFormatContext *formatCtx;

      /// \brief libav main external API structure.
      private: AVCodecContext *codecCtx;

      /// \brief libavcodec audio codec.
      private: AVCodec *codec;

      /// \brief Index of the audio stream.
      private: int audioStream;

      /// \brief True when initialized. We just want to initialize once..
      private: static bool initialized;

      /// \brief Audio file to decode.
      private: std::string filename;

      /// \brief Audio capture device
      private: std::string capDevice;

      /// \brief Audio Device Alsa Handle
      private: snd_pcm_t *deviceHandle;

      /// \brief Audio capture sample rate
      private: unsigned int sampleRate;

      /// \brief Audio capture frames
      private: unsigned long audioFrames;

      /// \brief Audio capture data buffer
      private: float *audioBuffer;

      /// \brief Audio capture data buffer size
      private: uint32_t bufferSize;

      /// \brief Audio capture indicator
      private: bool Capture;
    };
    /// \}
  }
}
#endif
