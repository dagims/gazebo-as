#include "ExportWorldPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(ExportWorldPlugin)



//////////////////////////////////////////////////
void ExportWorldPlugin::ExportMesh()
{
  mesh_p = new common::Mesh();
  models_n = this->world_p->ModelCount();
  models_v = this->world_p->Models();

  for(int i = 0; i < models_v.size(); i++)
  {
    links_v = models_v[i]->GetLinks();
    if(mesh_m->HasMesh(models_v[i]->GetName()))
      mesh_m->RemoveMesh(models_v[i]->GetName());
    for(int j = 0; j < links_v.size(); j++)
    {
      collisions_v = links_v[j]->GetCollisions();
      for(int k = 0; k < collisions_v.size(); k++)
      {
        if(strcmp(ShapeType(collisions_v[k]->GetShapeType()).c_str(), "mesh") == 0)
        {
          physics::MeshShapePtr meshshape_p =
                     boost::dynamic_pointer_cast<physics::MeshShape>(collisions_v[k]->GetShape());
          common::Mesh *mesh_tmp = new common::Mesh(meshshape_p->GetMesh());
          n = mesh_tmp->GetSubMeshCount();
          for(int l = 0; l < n; l++)
          {
            submesh_p = new common::SubMesh(mesh_tmp->GetSubMesh(l));
            submesh_p->SetScale(meshshape_p->Size());
            submesh_p->Center(ignition::math::Vector3d(links_v[j]->WorldPose().Pos().X(),
                                                       links_v[j]->WorldPose().Pos().Y(),
                                                       links_v[j]->WorldPose().Pos().Z() +
                                                                       (submesh_p->Max().Z()/2)));
            mesh_p->AddSubMesh(submesh_p);
          }
        }
        else if(strcmp(ShapeType(collisions_v[k]->GetShapeType()).c_str(), "box") == 0)
        {
          physics::BoxShapePtr box_p =
                       boost::dynamic_pointer_cast<physics::BoxShape>(collisions_v[k]->GetShape());
          submesh_p = new common::SubMesh();
          mesh_m->CreateBox(models_v[i]->GetName(), box_p->Size(),
                            ignition::math::Vector2d::One, submesh_p);
          submesh_p->Center(links_v[j]->WorldPose().Pos());
          mesh_p->AddSubMesh(submesh_p);
        }
        else if(strcmp(ShapeType(collisions_v[k]->GetShapeType()).c_str(), "cylinder") == 0)
        {
          physics::CylinderShapePtr cylinder_p =
                 boost::dynamic_pointer_cast<physics::CylinderShape>(collisions_v[k]->GetShape());
          submesh_p = new common::SubMesh();
          mesh_m->CreateCylinder(models_v[i]->GetName(), cylinder_p->GetRadius(),
                                 cylinder_p->GetLength(), 1, 32, submesh_p);
          submesh_p->Center(links_v[j]->WorldPose().Pos());
          mesh_p->AddSubMesh(submesh_p);
        }
        else if(strcmp(ShapeType(collisions_v[k]->GetShapeType()).c_str(), "plane") == 0)
        {
          physics::PlaneShapePtr plane_p =
                    boost::dynamic_pointer_cast<physics::PlaneShape>(collisions_v[k]->GetShape());
          submesh_p = new common::SubMesh();
          mesh_m->CreatePlane(models_v[i]->GetName(), 
                              ignition::math::Planed(plane_p->Normal(), plane_p->Size(), 0),
                              ignition::math::Vector2d(1, 1),
                              ignition::math::Vector2d(1, 1), submesh_p);
          submesh_p->Center(links_v[j]->WorldPose().Pos());
          mesh_p->AddSubMesh(submesh_p);
         }
        else if(strcmp(ShapeType(collisions_v[k]->GetShapeType()).c_str(), "sphere") == 0)
        {
          physics::SphereShapePtr sphere_p =
                   boost::dynamic_pointer_cast<physics::SphereShape>(collisions_v[k]->GetShape());
          submesh_p = new common::SubMesh();
          mesh_m->CreateSphere(models_v[i]->GetName(), 
                               (float)sphere_p->GetRadius(), 32, 32, submesh_p);
          submesh_p->Center(links_v[j]->WorldPose().Pos());
          mesh_p->AddSubMesh(submesh_p);
        }
      }
    }
  }
  printf("Exported Mesh!!!!!!!!!!!!!\n");
  common::MeshManager::Instance()->Export(mesh_p, "MeshExport", "dae", false);
  }

//////////////////////////////////////////////////////////////////////
void ExportWorldPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  shape_types[0x20000] = "box";       shape_types[0x40000] = "cylinder";
  shape_types[0x800000] = "plane";    shape_types[0x1000000] = "sphere";
  shape_types[0x2000000] = "mesh";    shape_types[0x4000000] = "polyline";
  shape_types[0x80000] = "heightmap"; shape_types[0x100000] = "map";
  shape_types[0x200000] = "multiray"; shape_types[0x400000] = "ray";
  this->mesh_m = common::MeshManager::Instance();
  this->conn_p = event::Events::ConnectWorldUpdateBegin(std::bind(&ExportWorldPlugin::OnUpdate, this));
  this->world_p = _parent;
  this->models_n = this->world_p->ModelCount();
  
  this->iaudio = new common::Audio();
  this->oaudio = new common::Audio(false);
  this->audioBuffer = (float *)malloc(64*4);

}

//////////////////////////////////////////////////////////////////////
void ExportWorldPlugin::OnUpdate()
{
  this->models_v = this->world_p->Models();
  if(this->models_n != this->models_v.size()) 
  {
    printf("****  Models' status changed  ****\n");
    for(int i = 0; i < this->models_v.size(); ++i)
      printf("Model %d: %s\n", i, this->models_v[i]->GetName().c_str());
    this->models_n = models_v.size();
    this->ExportMesh();
  }
  this->iaudio->ReadFrames(&this->audioBuffer, this->bufferSize);
  printf("The Buffer Size: %d\n", this->bufferSize);
  write(1, this->audioBuffer, 64);
  this->oaudio->WriteFrames(this->audioBuffer, 32);
}

///////////////////////////////////////////////////
std::string ExportWorldPlugin::ShapeType(unsigned int shp_n)
{
  return std::string(shape_types[shp_n - 0x10000]);
}

