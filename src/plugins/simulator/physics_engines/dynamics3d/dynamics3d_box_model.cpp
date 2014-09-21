/*sudo*
 * @file <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_box_model.cpp>
 *
 * @author Michael Allwright - <allsey87@gmail.com>
 */

#include "dynamics3d_box_model.h"

#include <argos3/plugins/simulator/physics_engines/dynamics3d/dynamics3d_engine.h>

namespace argos {

   /****************************************/
   /****************************************/

   CDynamics3DBoxModel::CDynamics3DBoxModel(CDynamics3DEngine& c_engine,
                                            CBoxEntity& c_box) :
      CDynamics3DModel(c_engine, c_box.GetEmbodiedEntity(), c_box.GetId()),
      m_cBoxEntity(c_box) {
      /* Get origin anchor and register its update method */
      const SAnchor& sOrigin = GetEmbodiedEntity().GetOriginAnchor();
      RegisterAnchorMethod<CDynamics3DBoxModel>(sOrigin,
                                                &CDynamics3DBoxModel::UpdateOriginAnchor);
      /* When defining size of objects we must manually swap the Z and Y components */
      m_pcCollisionShape = m_cEngine.GetBoxShapeManager().RequestShape(
         btVector3(c_box.GetSize().GetX() * 0.5f,
                   c_box.GetSize().GetZ() * 0.5f, 
                   c_box.GetSize().GetY() * 0.5f));
      btTransform cBoxGeometricOffset(
         btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
         btVector3(0.0f, -c_box.GetSize().GetZ() * 0.5f, 0.0f));
      Real fMass = c_box.GetEmbodiedEntity().IsMovable() ? c_box.GetMass() : 0.0f;
      m_vecLocalBodies.push_back(new CDynamics3DBody(this,
                                                     "box",
                                                     m_pcCollisionShape,
                                                     btTransform::getIdentity(),
                                                     cBoxGeometricOffset,
                                                     fMass));
      /* move the model to the specified coordinates */
      SetModelCoordinates(btTransform(ARGoSToBullet(GetEmbodiedEntity().GetOriginAnchor().Orientation),
                                      ARGoSToBullet(GetEmbodiedEntity().GetOriginAnchor().Position)));
   }
   
   /****************************************/
   /****************************************/
   
   CDynamics3DBoxModel::~CDynamics3DBoxModel() {
      m_cEngine.GetBoxShapeManager().ReleaseShape(m_pcCollisionShape);
   }
   
   /****************************************/
   /****************************************/

   void CDynamics3DBoxModel::UpdateEntityStatus() {
      if(m_cBoxEntity.GetEmbodiedEntity().IsMovable()) {      
         CPhysicsModel::UpdateEntityStatus();
     }
   }

   /****************************************/
   /****************************************/

   btTransform CDynamics3DBoxModel::GetModelCoordinates() const {
      return m_vecLocalBodies[0]->GetMotionStateTransform();
   }

   /****************************************/
   /****************************************/

   void CDynamics3DBoxModel::UpdateOriginAnchor(SAnchor& s_anchor) {
      s_anchor.Position = BulletToARGoS(GetModelCoordinates().getOrigin());
      s_anchor.Orientation = BulletToARGoS(GetModelCoordinates().getRotation());
   }

   /****************************************/
   /****************************************/

   REGISTER_STANDARD_DYNAMICS3D_OPERATIONS_ON_ENTITY(CBoxEntity, CDynamics3DBoxModel);

}
