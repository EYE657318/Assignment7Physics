#pragma once

#include "GLView.h"
#include "../include/physx/PxPhysicsAPI.h"

namespace Aftr
{
   class Camera;

/**
   \class GLViewAAATestingPhysics
   \author Scott Nykl 
   \brief A child of an abstract GLView. This class is the top-most manager of the module.

   Read \see GLView for important constructor and init information.

   \see GLView

    \{
*/

class GLViewAAATestingPhysics : public GLView
{
public:
   static GLViewAAATestingPhysics* New( const std::vector< std::string >& outArgs );
   virtual ~GLViewAAATestingPhysics();
   virtual void updateWorld(); ///< Called once per frame
   virtual void loadMap(); ///< Called once at startup to build this module's scene
   virtual void createAAATestingPhysicsWayPoints();
   virtual void onResizeWindow( GLsizei width, GLsizei height );
   virtual void onMouseDown( const SDL_MouseButtonEvent& e );
   virtual void onMouseUp( const SDL_MouseButtonEvent& e );
   virtual void onMouseMove( const SDL_MouseMotionEvent& e );
   virtual void onKeyDown( const SDL_KeyboardEvent& key );
   virtual void onKeyUp( const SDL_KeyboardEvent& key );

   //Physics stuff
   physx::PxDefaultAllocator a;
   physx::PxDefaultErrorCallback e;
   physx::PxScene* scene;
   physx::PxFoundation* f;
   physx::PxPhysics* p;
   physx::PxRigidDynamic* actor;
   physx::PxRigidDynamic* actor2;

protected:
   GLViewAAATestingPhysics( const std::vector< std::string >& args );
   virtual void onCreate();   
};

/** \} */

} //namespace Aftr
