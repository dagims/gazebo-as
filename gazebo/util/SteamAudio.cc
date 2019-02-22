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

#ifdef HAVE_STEAMAUDIO

/////////////////////////////////////////////////
SteamAudio::SteamAudio()
{
  this->SOFAFILE = "";
  iplCreateContext(steamLogCallback, nullptr, nullptr, &this->context);
}

/////////////////////////////////////////////////
SteamAudio::~SteamAudio()
{
  this->Fini();
}

/////////////////////////////////////////////////
SteamAudio::initSteamAudio()
{
  // TODO these params are static for now
  //      can only support 44100 and 64
  unsigned int sample_rate = 44100;
  unsigned int frames = 32;
  IPLHrtfParams hrtfParams{ IPL_HRTFDATABASETYPE_DEFAULT, nullptr, nullptr};
  if (this->SOFAfile.size() != 0)
  {
    hrtfParams.type = IPL_HRTFDATABASETYPE_SOFA;
    hrtfParams.sofaFileName = this->SOFAfile.c_str();
  }
  IPLRenderingSettings settings { sample_rate, frames };
  IPLAudioFormat mono{IPL_CHANNELLAYOUTTYPE_SPEAKERS,
                      IPL_CHANNELLAYOUT_MONO,
                      1,
                      nullptr,
                      nullptr,
                      nullptr,
                      nullptr,
                      IPL_CHANNELORDER_INTERLEAVED};
  IPLAudioFormat stereo{IPL_CHANNELLAYOUTTYPE_SPEAKERS,
                      IPL_CHANNELLAYOUT_STEREO,
                      2,
                      nullptr,
                      nullptr,
                      nullptr,
                      nullptr,
                      IPL_CHANNELORDER_INTERLEAVED};

  iplCreateBinauralRenderer(this->context, settings, hrtfParams, &this->binauralRenderer);
  iplCreateBinauralEffect(this->binauralRenderer, mono, stereo, &this->binauralEffect);

  this->inputAudio.clear();
  this->inputAudioBuffer.format = mono;
  this->inputAudioBuffer.numSamples = sample_rate;
  this->inputAudioBuffer.interleavedBuffer = this->inputAudio.data();
  this->inputAudioBuffer.deinterleavedBuffer = nullptr;

  this->outputAudio.clear();
  this->outputAudio.insert(this->outputAudio.begin(), frames*2, 0); 
  this->outputAudioBuffer.format = stereo;
  this->outputAudioBuffer.numSamples = sample_rate;
  this->outputAudioBuffer.interleavedBuffer = this->outputAudio.data();
  this->outputAudioBuffer.deinterleavedBuffer = nullptr;
}

/////////////////////////////////////////////////
bool SteamAudio::Load(sdf::ElementPtr _sdf)
{
  std::string deviceName = "default";

  return true;
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
bool SteamAudio::SetSOFA(const std::string &_filename)
{
  // not checking for sofa file existence or validity
  // because if the file is invalid, phonon will revert
  // back to default hrtf itself - no harm.
  this->SOFAfile = _filename;
  // XXX might need to recreate the binaural renderer when 
  // hrtf params changed - look into it!
  // perhaps this will do but test!
  delete this->binauralEffect;
  delete this->binauralRenderer;
  iplDestroyBinauralEffect(&this->binauralEffect);
  iplDestroyBinauralRenderer(&this->binauralRenderer);
  this->Init();
}

/////////////////////////////////////////////////
IPLvoid SteamAudio::steamLogCallback(char *msg)
{
  gzlog << "STEAMAUDIO LOG: " << msg << "\n";
}

/////////////////////////////////////////////////
IPLVector3 SteamAudio::SetListenerPose(const ignition::math::Pose3d &_pose)
{
  //TODO think about the coordinate to be transformed
  //     sourcePosition, listenerPosition,
  //     listenerAhead, listenerUp
  IPLVector3 listenerp = CalculateRelativeDirection(
                                IPLVector3{this->generatorPose.Pos().X(),
                                           this->generatorPose.Pos().Y(),
                                           this->generatorPose.Pos().Z()},
                                IPLVector3{_pose.Pos().X(),
                                           _pose.Pos().Y(),
                                           _pose.Pos().Z()},
                                IPLVector3{ 1.0f, 0.0f, 0.0f}, // the x direction as the front of the listener
                                IPLVector3{ 0.0f, 0.0f, 1.0f}); // the 'up' simply taken as Z
  this->listenerLocation = ignition::math::Vector3d(listenerp.x, listenerp.y, listenerp.z);
  return true;
}

/////////////////////////////////////////////////
std::vector<float> SteamAudio::SteamBinauralEffect(float *_buf, long _bufSize)
{
  this->inputAudioBuffer.interleavedBuffer = _buf;
  // XXX ^ this is just terrible!
  this->inputAudioBuffer.numSamples = _bufSize;
  iplApplyBinauralEffect(this->binauralEffect,
                         this->binauralRenderer,
                         this->inputAudioBuffer,
                         this->listenerLocation,
                         IPL_HRTFINTERPOLATION_NEAREST,
                         this->outputAudioBuffer);
  return this->outputAudio;
}

/////////////////////////////////////////////////
bool SteamAudioGenerator::IsPlaying()
{
  // TODO
  return true;
}

#endif
