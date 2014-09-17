/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/plugins/gripper/dynamics3d_gripper_plugin.h>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#ifndef DYNAMICS3D_GRIPPER_PLUGIN_H
#define DYNAMICS3D_GRIPPER_PLUGIN_H

namespace argos {
   class CGripperEquippedEntity;
}

#include <vector>
#include <string>

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_plugin.h>
#include <argos3/core/utility/datatypes/datatypes.h>

namespace argos {

   /****************************************/
   /****************************************/

   class CDynamics3DGripperPlugin : public CDynamics3DPlugin {

   public:

      CDynamics3DGripperPlugin();

      ~CDynamics3DGripperPlugin() {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void Reset() {}
      virtual void Destroy() {}

      virtual void RegisterModel(CDynamics3DModel& c_model);
      virtual void UnregisterModel(CDynamics3DModel& c_model);

      virtual void Update();

   private:
      Real m_fGraspRange;

      /****************************************/
      /****************************************/
      
      struct SGripper {
         SGripper(CGripperEquippedEntity* pc_gripper_equipped_entity,
                  CDynamics3DBody* pc_gripper_body) :
            GripperEquippedEntity(pc_gripper_equipped_entity),
            GripperBody(pc_gripper_body),
            GraspedBody(NULL),
            StateOnLastCheck(UNLOCKED) {}

         CGripperEquippedEntity* GripperEquippedEntity;
         CDynamics3DBody* GripperBody;
         CDynamics3DBody* GraspedBody;
         
         enum EState {
            UNLOCKED,
            LOCKED
         } StateOnLastCheck;
      };

      /****************************************/
      /****************************************/
      
      std::vector<SGripper> m_vecGrippers;

   };

   /****************************************/
   /****************************************/
   
}

#endif
