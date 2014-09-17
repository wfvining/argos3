/**
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/plugin/gripper/dynamics3d_plugin.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_gripper_plugin.h"

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_body.h>

#include <argos3/plugins/simulator/entities/gripper_equipped_entity.h>

#include <argos3/core/simulator/entity/composable_entity.h>

namespace argos {

   CDynamics3DGripperPlugin::CDynamics3DGripperPlugin() :
      CDynamics3DPlugin(),
      m_fGraspRange(0.05) {}

   /****************************************/
   /****************************************/

   void CDynamics3DGripperPlugin::Init(TConfigurationNode& t_tree) {
      GetNodeAttributeOrDefault(t_tree, "range", m_fGraspRange, m_fGraspRange);
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DGripperPlugin::RegisterModel(CDynamics3DModel& c_model) {
      CComposableEntity& cComposable = c_model.GetEmbodiedEntity().GetParent();
      if(cComposable.HasComponent("gripper")) {
         CGripperEquippedEntity& cGripper = cComposable.GetComponent<CGripperEquippedEntity>("gripper");
         
         /* create struct with gripper entity, last checked state, 
            pointer to model, relevant body (for forming joint) */
         m_vecGrippers.push_back(SGripper(&cGripper, NULL));
      }
   }

   /****************************************/
   /****************************************/

   void CDynamics3DGripperPlugin::UnregisterModel(CDynamics3DModel& c_model) {
      
   }

   /****************************************/
   /****************************************/   

   void CDynamics3DGripperPlugin::Update() {
      for(std::vector<SGripper>::iterator itGripper = m_vecGrippers.begin();
          itGripper != m_vecGrippers.end();
          itGripper++) {

         if(itGripper->GripperEquippedEntity->IsLocked()) {
            if(itGripper->StateOnLastCheck == SGripper::UNLOCKED) {
               /* get nearby objects */
               /* check if any are less than threshold */
               

               /* create joint */
               
               /* update the gripped entity attribute in CGripperEquippedEntity */
               itGripper->StateOnLastCheck = SGripper::LOCKED;
            }
         }
         else {
            if(itGripper->StateOnLastCheck == SGripper::LOCKED) {
               /* remove joint */
            }
            itGripper->StateOnLastCheck = SGripper::UNLOCKED;
         }
      }
   }

   /****************************************/
   /****************************************/

   REGISTER_DYN3D_PHYSICS_PLUGIN(CDynamics3DGripperPlugin,
                                 "gripper",
                                 "Michael Allwright [allsey87@gmail.com]",
                                 "1.0",
                                 "A plugin developed for simulating the gripper of various robots",
                                 "This plugin creates a joint between the gripper and a nearby\n"
                                 "object",
                                 "Under development");
}
