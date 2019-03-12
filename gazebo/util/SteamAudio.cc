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
#ifndef _WIN32
  #include <stdio.h>
  #include <unistd.h>
  #include <iostream>
#endif

#include <gazebo/gazebo_config.h>

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/AudioDecoder.hh"
#include "gazebo/util/SteamAudio.hh"

using namespace gazebo;
using namespace util;


/////////////////////////////////////////////////
extern "C" {
  IPLvoid steamLogCallback(char *msg);
}
  
IPLvoid steamLogCallback(char *msg)
{
  gzlog << "STEAMAUDIO LOG: " << msg << "\n";
}

/////////////////////////////////////////////////
SteamAudio::SteamAudio()
{
  this->SOFAfile = "";
  iplCreateContext(steamLogCallback, nullptr, nullptr, &this->context);
  this->Init();
}

/////////////////////////////////////////////////
SteamAudio::~SteamAudio()
{
  this->Fini();
}

/////////////////////////////////////////////////
void SteamAudio::Init()
{
  // TODO these params are static for now
  //      can only support 44100 and 64
  IPLint32 sample_rate = 44100;
  IPLint32 frames = 64;
  IPLHrtfParams hrtfParams;
  if(this->SOFAfile.size() != 0)
  {
    hrtfParams.type = IPL_HRTFDATABASETYPE_SOFA;
    hrtfParams.hrtfData = nullptr;
    hrtfParams.sofaFileName = strdup(this->SOFAfile.c_str());
  }
  else
  {
    hrtfParams.type = IPL_HRTFDATABASETYPE_DEFAULT;
    hrtfParams.hrtfData = nullptr;
    hrtfParams.sofaFileName = nullptr;
  }

  IPLRenderingSettings settings { sample_rate, frames,
                                  IPL_CONVOLUTIONTYPE_PHONON};
  IPLAudioFormat mono;
  mono.channelLayoutType = IPL_CHANNELLAYOUTTYPE_SPEAKERS;
  mono.channelLayout = IPL_CHANNELLAYOUT_MONO;
  mono.numSpeakers = 1;
  mono.channelOrder = IPL_CHANNELORDER_INTERLEAVED;

  IPLAudioFormat stereo;
  stereo.channelLayoutType = IPL_CHANNELLAYOUTTYPE_SPEAKERS;
  stereo.channelLayout = IPL_CHANNELLAYOUT_STEREO;
  stereo.numSpeakers = 2;
  stereo.channelOrder = IPL_CHANNELORDER_INTERLEAVED;

  iplCreateBinauralRenderer(this->context, settings, hrtfParams, &this->binauralRenderer);
  iplCreateBinauralEffect(this->binauralRenderer, mono, stereo, &this->binauralEffect);

  this->inputAudio.clear();
  this->inputAudioBuffer.format = mono;
  this->inputAudioBuffer.numSamples = frames;
  this->inputAudioBuffer.interleavedBuffer = this->inputAudio.data();
  this->inputAudioBuffer.deinterleavedBuffer = nullptr;

  this->outputAudio.clear();
  this->outputAudio.insert(this->outputAudio.begin(), frames*2, 0); 
  this->outputAudioBuffer.format = stereo;
  this->outputAudioBuffer.numSamples = frames;
  this->outputAudioBuffer.interleavedBuffer = this->outputAudio.data();
  this->outputAudioBuffer.deinterleavedBuffer = nullptr;
}

/////////////////////////////////////////////////
void SteamAudio::Fini()
{
  iplDestroyBinauralEffect(&this->binauralEffect);
  iplDestroyBinauralRenderer(&this->binauralRenderer);
  iplDestroyContext(&this->context);
  iplCleanup();
}

/////////////////////////////////////////////////
void SteamAudio::SetSOFA(const std::string &_filename)
{
  // not checking for sofa file existence or validity
  // because if the file is invalid, phonon will revert
  // back to default hrtf itself - no harm.
  this->SOFAfile = common::find_file(_filename);
  this->binauralEffect = nullptr;
  this->binauralRenderer = nullptr;
  iplDestroyBinauralEffect(&this->binauralEffect);
  iplDestroyBinauralRenderer(&this->binauralRenderer);
  this->Init();
}

/////////////////////////////////////////////////
void SteamAudio::SetGeneratorPose(const ignition::math::Pose3d &_pose)
{
  this->generatorPose = _pose;
}

/////////////////////////////////////////////////
void SteamAudio::SetListenerPose(const ignition::math::Pose3d &_pose)
{
  ignition::math::Vector3d listenerAheadVec = 
             _pose.Rot().RotateVector(ignition::math::Vector3d(1.0, 0.0, 0.0));
  ignition::math::Vector3d generatorUpVec = 
             _pose.Rot().RotateVector(ignition::math::Vector3d(0.0, 0.0, 1.0));
  IPLVector3 listenerp = iplCalculateRelativeDirection(
                                IPLVector3{(float)this->generatorPose.Pos().X(),
                                           (float)this->generatorPose.Pos().Y(),
                                           (float)this->generatorPose.Pos().Z()},
                                IPLVector3{(float)_pose.Pos().X(),
                                           (float)_pose.Pos().Y(),
                                           (float)_pose.Pos().Z()},
                                IPLVector3{(float)listenerAheadVec.X(),
                                           (float)listenerAheadVec.Y(),
                                           (float)listenerAheadVec.Z()},
                                IPLVector3{(float)generatorUpVec.X(),
                                           (float)generatorUpVec.Y(),
                                           (float)generatorUpVec.Z()});
                                
  this->listenerLocation = ignition::math::Vector3d(listenerp.x, listenerp.y, listenerp.z);
}

/////////////////////////////////////////////////
std::vector<float> SteamAudio::SteamBinauralEffect(float *_buf, long _bufSize)
{
  this->inputAudio = std::vector<float>(_buf, _buf+_bufSize);
  this->inputAudioBuffer.interleavedBuffer = this->inputAudio.data();
  iplApplyBinauralEffect(this->binauralEffect,
                         this->binauralRenderer,
                         this->inputAudioBuffer,
                         IPLVector3{(float)this->listenerLocation.X(),
                                    (float)this->listenerLocation.Y(),
                                    (float)this->listenerLocation.Z()},
                         IPL_HRTFINTERPOLATION_NEAREST,
                         this->outputAudioBuffer);
  return this->outputAudio;
}

