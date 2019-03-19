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
  void closestHitCallback(const IPLfloat32* origin, const IPLfloat32* direction,
       const IPLfloat32 minDistance, const IPLfloat32 maxDistance,
       IPLfloat32* hitDistance, IPLfloat32* hitNormal,
       IPLMaterial** hitMaterial, IPLvoid* userData);
  void anyHitCallback(const IPLfloat32* origin, const IPLfloat32 *direction,
       const IPLfloat32 minDistance, const IPLfloat32 maxDistance,
       IPLint32 *hitExists, IPLvoid *userData);
}
  
IPLvoid steamLogCallback(char *msg)
{
  gzlog << "STEAMAUDIO LOG: " << msg << "\n";
}

void closestHitCallback(const IPLfloat32* origin, const IPLfloat32* direction,
       const IPLfloat32 minDistance, const IPLfloat32 maxDistance,
       IPLfloat32* hitDistance, IPLfloat32* hitNormal,
       IPLMaterial** hitMaterial, IPLvoid* userData)
{
  *hitDistance = 0.0;
  *hitNormal = 0.0;
}


void anyHitCallback(const IPLfloat32* origin, const IPLfloat32 *direction,
       const IPLfloat32 minDistance, const IPLfloat32 maxDistance,
       IPLint32 *hitExists, IPLvoid *userData)
{
  *hitExists = 0;
}

/////////////////////////////////////////////////
SteamAudio::SteamAudio()
{
  this->SOFAfile = "";
  this->materialAcousticProp["generic"] = IPLMaterial{0.10f,0.20f,0.30f,0.05f,0.100f,0.050f,0.030f};
  this->materialAcousticProp["brick"] = IPLMaterial{0.03f,0.04f,0.07f,0.05f,0.015f,0.015f,0.015f};
  this->materialAcousticProp["concrete"] = IPLMaterial{0.05f,0.07f,0.08f,0.05f,0.015f,0.002f,0.001f};
  this->materialAcousticProp["ceramic"] = IPLMaterial{0.01f,0.02f,0.02f,0.05f,0.060f,0.044f,0.011f};
  this->materialAcousticProp["gravel"] = IPLMaterial{0.60f,0.70f,0.80f,0.05f,0.031f,0.012f,0.008f};
  this->materialAcousticProp["carpet"] = IPLMaterial{0.24f,0.69f,0.73f,0.05f,0.020f,0.005f,0.003f};
  this->materialAcousticProp["glass"] = IPLMaterial{0.06f,0.03f,0.02f,0.05f,0.060f,0.044f,0.011f};
  this->materialAcousticProp["plaster"] = IPLMaterial{0.12f,0.06f,0.04f,0.05f,0.056f,0.056f,0.004f};
  this->materialAcousticProp["wood"] = IPLMaterial{0.11f,0.07f,0.06f,0.05f,0.070f,0.014f,0.005f};
  this->materialAcousticProp["metal"] = IPLMaterial{0.20f,0.07f,0.06f,0.05f,0.200f,0.025f,0.010f};
  this->materialAcousticProp["rock"] = IPLMaterial{0.13f,0.20f,0.24f,0.05f,0.015f,0.002f,0.001f};

  iplCreateContext(steamLogCallback, nullptr, nullptr, &this->steamContext);
  this->Init();
}

/////////////////////////////////////////////////
SteamAudio::~SteamAudio()
{
  this->Fini();
}

/////////////////////////////////////////////////
bool SteamAudio::Load(sdf::ElementPtr _sdf)
{
  if(!_sdf->HasElement("acoustic"))
    return false;

  sdf::ElementPtr acousticElem = _sdf->GetElement("acoustic");
  printf("Acoustic!!!!!!!!!!!!!!!!!!!\n");
  if(acousticElem->HasElement("material"))
    this->materialName.push_back(acousticElem->Get<std::string>("material"));
  if(acousticElem->HasElement("properties"))
  {
    sdf::ElementPtr propElem = acousticElem->GetElement("properties");
    if(propElem->HasElement("lowFrequencyAbsorption"))
    {
      this->materialProperties.push_back(IPLMaterial{propElem->Get<double>("lowFrequencyAbsorption"),
                                                     propElem->Get<double>("midFrequencyAbsorption"),
                                                     propElem->Get<double>("highFrequencyAbsorption"),
                                                     propElem->Get<double>("scattering"),
                                                     propElem->Get<double>("lowFrequencyTransmission"),
                                                     propElem->Get<double>("midFrequencyTransmission"),
                                                     propElem->Get<double>("highFrequencyTransmission")});
      this->ConvertMesh(_sdf->Get<std::string>("uri"));
    }
  }
  this->worldCreatedEventCon =
        event::Events::ConnectWorldCreated(
              std::bind(&SteamAudio::WorldCreated, this));
  this->worldUpdateEventCon =
        event::Events::ConnectWorldUpdateBegin(
              std::bind(&SteamAudio::Update, this));
  return true;
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

  iplCreateBinauralRenderer(this->steamContext, settings, hrtfParams, &this->binauralRenderer);
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
  iplDestroyContext(&this->steamContext);
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

/////////////////////////////////////////////////
void SteamAudio::ConvertMesh(std::string _meshURI)
{
  const common::Mesh *mesh = common::MeshManager::Instance()->Load(_meshURI);
  const common::SubMesh *subMesh;
  ignition::math::Vector3d vertex;
  std::vector<IPLTriangle> tris;
  std::vector<IPLVector3> verts;
  for(size_t i = 0; i < mesh->GetSubMeshCount(); i++)
  {
    subMesh = mesh->GetSubMesh(i);
    for(size_t j = 0; j < subMesh->GetVertexCount(); j++)
    {
      vertex = subMesh->Vertex(j);
      verts.push_back(IPLVector3{(float)vertex.X(),
                                 (float)vertex.Y(),
                                 (float)vertex.Z()});
    }
    for(size_t j = 0; j < subMesh->GetIndexCount(); j += 3)
    {
      tris.push_back(IPLTriangle{subMesh->GetIndex(j),
                                 subMesh->GetIndex(j+1),
                                 subMesh->GetIndex(j+2)});
    }
  }
  this->vertices.push_back(verts);
  this->triangles.push_back(tris);
}

/////////////////////////////////////////////////
void SteamAudio::WorldCreated()
{
  IPLSimulationSettings simSettings;
  simSettings.sceneType = IPL_SCENETYPE_EMBREE;
  simSettings.numOcclusionSamples = 10; //XXX don't know what this is
  simSettings.numRays = 2048;
  simSettings.numDiffuseSamples = 2048;
  simSettings.numBounces = 8;
  simSettings.numThreads = 4;
  simSettings.irDuration = 1;
  simSettings.ambisonicsOrder = 0;
  simSettings.maxConvolutionSources = 2;
  simSettings.bakingBatchSize = 0;

  this->userD = malloc(4000);

  IPLerror ret = iplCreateScene(this->steamContext, nullptr, simSettings,
                                this->materialProperties.size(),
                                this->materialProperties.data(),
                                closestHitCallback, anyHitCallback, nullptr, nullptr, nullptr,
                                &this->steamScene);
  if(ret == IPL_STATUS_FAILURE)
  {
    fprintf(stderr, "Unspecified error during scene creation\n");
    return;
  }
  if(ret == IPL_STATUS_OUTOFMEMORY)
  {
    fprintf(stderr, "out of memory during scene creation\n");
    return;
  }
  if(ret == IPL_STATUS_INITIALIZATION)
  {
    fprintf(stderr, "init error during scene creation\n");
    return;
  }

  fprintf(stderr, "Successfully created the fucking scene!!!!\n");
}

/////////////////////////////////////////////////
void SteamAudio::Update()
{
  
}
