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

#include <gazebo/common/Console.hh>
#include <gazebo/common/Audio.hh>

using namespace gazebo;
using namespace common;


////////////////////////////////////////////////////////
Audio::Audio(bool _captureMode, std::string _deviceName,
             unsigned int _sampleRate, unsigned long _frames)
{
  this->captureMode = _captureMode;
  this->sampleRate = _sampleRate;
  this->audioFrames = _frames;
  this->bufferSize = 0;
  this->audioBuffer = nullptr;
  this->runningStatus = false;

  snd_pcm_hw_params_t *hw_params;
  int dir;

  snd_pcm_stream_t direction = this->captureMode ? SND_PCM_STREAM_CAPTURE : SND_PCM_STREAM_PLAYBACK;

  ret = snd_pcm_open(&this->deviceHandle, _deviceName.c_str(), direction, 0);
  if(ret < 0)
  {
    gzerr << "Error Opening Capture Device: " << snd_strerror(ret) << "\n";
    return;
  }

  snd_pcm_hw_params_alloca(&hw_params);
  snd_pcm_hw_params_any(this->deviceHandle, hw_params);

  if (this->captureMode)
  {
    snd_pcm_hw_params_set_channels(this->deviceHandle, hw_params, 1);
    this->bufferSize = this->audioFrames * 4; // 4 bytes/s , 1 channel
  }
  else
  {
    snd_pcm_hw_params_set_channels(this->deviceHandle, hw_params, 2);
    snd_pcm_hw_params_set_access(this->deviceHandle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    this->bufferSize = this->audioFrames * 4 * 2; // 4 bytes/s , 2 channels
  }

  snd_pcm_hw_params_set_format(this->deviceHandle, hw_params, SND_PCM_FORMAT_FLOAT_LE);
  snd_pcm_hw_params_set_rate_near(this->deviceHandle, hw_params, &this->sampleRate, &dir);
  snd_pcm_hw_params_set_period_size_near(this->deviceHandle, hw_params, &this->audioFrames, &dir);
  ret = snd_pcm_hw_params(this->deviceHandle, hw_params);
  
  if(ret < 0)
  {
    gzerr << "Error Writing Capture HW Params: " << snd_strerror(ret) << "\n";
    return;
  }

  this->audioBuffer = (float *)malloc(this->bufferSize);
  this->runningStatus = true;

  gzerr << "yeaaaaaaaaaaaaaaaaaaaaaaaa maaaaaaaaaaaaaaaan\n";
}

////////////////////////////////////////////////////////
Audio::~Audio()
{
  this->Shutdown();
}

////////////////////////////////////////////////////////
bool Audio::IsRunning() const
{
  return this->runningStatus;
}

////////////////////////////////////////////////////////
bool Audio::Start()
{
  //TODO threaded callback start
  this->runningStatus = true;
  return true;
}

////////////////////////////////////////////////////////
bool Audio::Stop()
{
  //TODO threaded callback stop
  this->runningStatus = false;
  return true;
}

////////////////////////////////////////////////////////
void Audio::Shutdown()
{
  this->runningStatus = false;
  snd_pcm_drain(this->deviceHandle);
  snd_pcm_close(this->deviceHandle);
  free(this->audioBuffer);
}

////////////////////////////////////////////////////////
unsigned int Audio::GetSampleRate() const
{
  return this->sampleRate;
}

////////////////////////////////////////////////////////
Audio::AudioIOStatusCode Audio::ReadFrames(float **_buf, long &_numFrames)
{
  if (!this->runningStatus)
  {
    gzerr << "Audio Object Not Alive! See if there's a problem during creation.\n";
    return AUDIO_OBJ_ERROR;
  }

  if (!this->captureMode)
  {
    gzerr << "Audio Object is Not in Capture Mode.\n";
    return AUDIO_OBJ_ERROR;
  }

  //XXX fix this! this shouldn't be like this.
  this->ret = snd_pcm_readi(this->deviceHandle, *_buf, this->audioFrames);
  if (this->ret == -EPIPE)
  {
    gzwarn << "Audio Read Overrun!\n";
    snd_pcm_prepare(this->deviceHandle);
    return AUDIO_READ_OVERRUN;
  }
  else if (this->ret < 0)
  {
    gzerr << "Error! Problem Reading: " << snd_strerror(this->ret) << "\n";
    _numFrames = 0;
    return AUDIO_READ_ERROR;
  }
  else if (this->ret != this->audioFrames)
  {
    gzwarn << "Audio Short Read. Read: " << this->ret << " For Frames: " << this->audioFrames << "\n";
    _numFrames = this->ret;
    return AUDIO_SHORT_READ;
  }
  _numFrames = this->audioFrames;
  return AUDIO_READ_OK;
}

////////////////////////////////////////////////////////
Audio::AudioIOStatusCode Audio::WriteFrames(float *_buf, long _numFrames)
{
  if (!this->runningStatus)
  {
    gzerr << "Audio Object Not Alive! See if there's a problem during creation.\n";
    return AUDIO_OBJ_ERROR;
  }

  if (this->captureMode)
  {
    gzerr << "Audio Object is Not in Playback Mode.\n";
    return AUDIO_OBJ_ERROR;
  }

  if (_numFrames < 0)
  {
    gzerr << "Can Have Negative Number of Frames\n";
    return AUDIO_INPUT_ERROR;
  }

  if (_buf == nullptr)
  {
    gzerr << "Buffer Must Contain Appropriate Data\n";
    return AUDIO_INPUT_ERROR;
  }

  this->ret = snd_pcm_writei(this->deviceHandle, _buf, _numFrames);

  if(this->ret == -EPIPE)
  {
    gzwarn << "Audio Write UnderRun\n";
    return AUDIO_WRITE_UNDERRUN;
  }
  else if (this->ret < 0)
  {
    gzerr << "Error! Problem Writing to Playback Device: " << snd_strerror(this->ret) << "\n";
    return AUDIO_WRITE_ERROR;
  }
  else if (this->ret != _numFrames)
  {
    gzwarn << "Audi Short Write. Wrote: " << this->ret << "From Frames: " << _numFrames << "\n";
    return AUDIO_SHORT_WRITE;
  }
  return AUDIO_WRITE_OK;
}
