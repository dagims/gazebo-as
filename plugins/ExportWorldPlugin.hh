#ifndef GAZEBO_PLUGINS_EXPORTWORLDPLUGIN_HH_
#define GAZEBO_PLUGINS_EXPORTWORLDPLUGIN_HH_

#include <ignition/math/Pose3.hh>
#include <ignition/math.hh>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "gazebo/util/SteamAudio.hh"


namespace gazebo
{
class GAZEBO_VISIBLE ExportWorldPlugin : public WorldPlugin
{
  private: physics::WorldPtr world_p;
  private: physics::ModelPtr model_p;
  private: physics::Model_V models_v;
  private: physics::Link_V links_v;
  private: physics::Collision_V collisions_v;
  private: event::ConnectionPtr conn_p;
  private: common::SubMesh *submesh_p;
  private: common::MeshManager *mesh_m;
  private: msgs::Geometry geom;
  private: unsigned int models_n, n;
  private: common::Mesh *mesh_p;
  private: std::map<uint32_t, std::string> shape_types;

  private: util::SteamAudio *steamAudio;
  private: common::Audio *iaudio;
  private: common::Audio *oaudio;
  private: float *audioBuffer;
  private: long bufferSize;
  private: std::vector<float> resAudio;
  private: std::vector<float> thee_audio;

  typedef std::map<uint32_t, msgs::Visual> Visuals_M;

  public: ExportWorldPlugin() {}
  public: ~ExportWorldPlugin();
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
  private: void OnUpdate();
  private: void ExportMesh();
  private: std::string ShapeType(unsigned int shp_n);
};
}

#endif
