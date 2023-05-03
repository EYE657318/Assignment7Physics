#include "GLViewAAATestingPhysics.h"

#include "WorldList.h" //This is where we place all of our WOs
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "Axes.h" //We can set Axes to on/off with this
#include "PhysicsEngineODE.h"

//Different WO used by this module
#include "WO.h"
#include "WOStatic.h"
#include "WOStaticPlane.h"
#include "WOStaticTrimesh.h"
#include "WOTrimesh.h"
#include "WOHumanCyborg.h"
#include "WOHumanCal3DPaladin.h"
#include "WOWayPointSpherical.h"
#include "WOLight.h"
#include "WOSkyBox.h"
#include "WOCar1970sBeater.h"
#include "Camera.h"
#include "CameraStandard.h"
#include "CameraChaseActorSmooth.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WONVStaticPlane.h"
#include "WONVPhysX.h"
#include "WONVDynSphere.h"
#include "WOImGui.h" //GUI Demos also need to #include "AftrImGuiIncludes.h"
#include "AftrImGuiIncludes.h"
#include "AftrGLRendererBase.h"

#include "../include/physx/PxPhysicsAPI.h"

using namespace Aftr;

GLViewAAATestingPhysics* GLViewAAATestingPhysics::New( const std::vector< std::string >& args )
{
   GLViewAAATestingPhysics* glv = new GLViewAAATestingPhysics( args );
   glv->init( Aftr::GRAVITY, Vector( 0, 0, -1.0f ), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE );
   glv->onCreate();
   return glv;
}


GLViewAAATestingPhysics::GLViewAAATestingPhysics( const std::vector< std::string >& args ) : GLView( args )
{
   //Initialize any member variables that need to be used inside of LoadMap() here.
   //Note: At this point, the Managers are not yet initialized. The Engine initialization
   //occurs immediately after this method returns (see GLViewAAATestingPhysics::New() for
   //reference). Then the engine invoke's GLView::loadMap() for this module.
   //After loadMap() returns, GLView::onCreate is finally invoked.

   //The order of execution of a module startup:
   //GLView::New() is invoked:
   //    calls GLView::init()
   //       calls GLView::loadMap() (as well as initializing the engine's Managers)
   //    calls GLView::onCreate()

   //GLViewAAATestingPhysics::onCreate() is invoked after this module's LoadMap() is completed.
}


void GLViewAAATestingPhysics::onCreate()
{
   //GLViewAAATestingPhysics::onCreate() is invoked after this module's LoadMap() is completed.
   //At this point, all the managers are initialized. That is, the engine is fully initialized.

   if( this->pe != NULL )
   {
      //optionally, change gravity direction and magnitude here
      //The user could load these values from the module's aftr.conf
      this->pe->setGravityNormalizedVector( Vector( 0,0,-1.0f ) );
      this->pe->setGravityScalar( Aftr::GRAVITY );
   }
   this->setActorChaseType( STANDARDEZNAV ); //Default is STANDARDEZNAV mode
   //this->setNumPhysicsStepsPerRender( 0 ); //pause physics engine on start up; will remain paused till set to 1

   physx::PxVec3 gravity(0.0f, 0.0f, -0.03f);
   scene->setGravity(gravity);
}


GLViewAAATestingPhysics::~GLViewAAATestingPhysics()
{
   //Implicitly calls GLView::~GLView()
}


void GLViewAAATestingPhysics::updateWorld()
{
   GLView::updateWorld(); //Just call the parent's update world first.
                          //If you want to add additional functionality, do it after
                          //this call.
   //
   //TODO:
   //
   //Seems like there's a fair amount of inheritence involved to get this to work.
   //Specifically, I think we need our own WO class
    
   //TODO: not sure how we're supposed to handle time
   // It compiles with just an int, but that crashes
   //int test = 100;
   //float test2 = 100.5f;
   //std::vector<bool> test3 = { true, false, true };

   auto dt = ManagerSDLTime::getTimeSinceLastMainLoopIteration();
   dt /= 10;                        //Change 19 to something higher, and it lags. change it to something lower, and the cube turns into the flash
   scene->simulate(dt);

   //WO* wo = static_cast<WO*>(actor2->userData);
   //std::cout << "the secret object is at " << wo->getPosition() << "\n";
   
       /////Get old rotations
       auto thePose = actor->getGlobalPose();
       auto rotation = thePose.q;
       float oldrotx = rotation.x;
       float oldroty = rotation.y;
       float oldrotz = rotation.z;
       float oldrotw = rotation.w;
       /////Get old rotations
       auto thePose2 = actor2->getGlobalPose();
       auto rotation2 = thePose2.q;
       float oldrotx2 = rotation2.x;
       float oldroty2 = rotation2.y;
       float oldrotz2 = rotation2.z;
       float oldrotw2 = rotation2.w;


       //scene->fetchResults(true);
       scene->fetchResults(true);
       physx::PxU32 errorState = 0;

   
       
       thePose = actor->getGlobalPose();
       WO* wo = static_cast<WO*>(actor->userData);
       auto position = thePose.p;
       rotation = thePose.q;
       
       float xpos = position.x;
       float ypos = position.y;
       float zpos = position.z;

       float rotx = rotation.x;
       float roty = rotation.y;
       float rotz = rotation.z;
       float rotw = rotation.w;

       //auto oldrot = wo->getPose().m;
       //float oldrotx = oldrot[0];
       //float oldroty = oldrot[1];
       //float oldrotz = oldrot[2];
       //float oldrotw = oldrot[3];

       //Position
       wo->setPosition(xpos, ypos, zpos);

       //Rotation
       //I sure hope 'w' isn't important!
       wo->rotateAboutGlobalX(rotx - oldrotx);
       wo->rotateAboutGlobalY(roty - oldroty);
       wo->rotateAboutGlobalZ(rotz - oldrotz);

       //std::cout << "new z = " << zpos << "\n";

       ////seems only RigidDynamic has getGlobalTransform, and I don't think I have enough actors to justify this anyway
       //physx::PxU32 numActors = 0;
       //I am assuming 's' is supposed to refer to 'scene' since this works
       //physx::PxActor** actors = scene->getActiveActors(numActors);
       //for (physx::PxU32 i = 0; i < numActors; ++i) {
       //    physx::PxActor* thisactor = actors[i];
       //    std::cout << "Update found actor " << i << ", wow!\n";
       //    WO* wo = static_cast<WO*>(thisactor->userData);
       //    thisactor->PxRigidActor::getGlobalTransform();

       //}




       physx::PxU32 errorState2 = 0;

       thePose2 = actor2->getGlobalPose();
       WO* wo2 = static_cast<WO*>(actor2->userData);
       auto position2 = thePose2.p;
       rotation2 = thePose2.q;

       float xpos2 = position2.x;
       float ypos2 = position2.y;
       float zpos2 = position2.z;

       float rotx2 = rotation2.x;
       float roty2 = rotation2.y;
       float rotz2 = rotation2.z;
       float rotw2 = rotation2.w;

       //Position
       wo2->setPosition(xpos2, ypos2, zpos2);

       //Rotation
       //I sure hope 'w' isn't important!
       wo2->rotateAboutGlobalX(rotx2 - oldrotx2);
       wo2->rotateAboutGlobalY(roty2 - oldroty2);
       wo2->rotateAboutGlobalZ(rotz2 - oldrotz2);

       


   

}


void GLViewAAATestingPhysics::onResizeWindow( GLsizei width, GLsizei height )
{
   GLView::onResizeWindow( width, height ); //call parent's resize method.
}


void GLViewAAATestingPhysics::onMouseDown( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseDown( e );
}


void GLViewAAATestingPhysics::onMouseUp( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseUp( e );
}


void GLViewAAATestingPhysics::onMouseMove( const SDL_MouseMotionEvent& e )
{
   GLView::onMouseMove( e );
}


void GLViewAAATestingPhysics::onKeyDown( const SDL_KeyboardEvent& key )
{
   GLView::onKeyDown( key );
   if( key.keysym.sym == SDLK_0 )
      this->setNumPhysicsStepsPerRender( 1 );

   if( key.keysym.sym == SDLK_1 )
   {
       physx::PxVec3 gravity(0.1f, 0.0f, 0.0f);
       scene->setGravity(gravity);
   }

   if (key.keysym.sym == SDLK_2)
   {
       physx::PxVec3 gravity(-0.1f, 0.0f, 0.0f);
       scene->setGravity(gravity);
   }

   if (key.keysym.sym == SDLK_3)
   {
       physx::PxVec3 gravity(0.0f, 0.1f, 0.0f);
       scene->setGravity(gravity);
   }

   if (key.keysym.sym == SDLK_4)
   {
       physx::PxVec3 gravity(0.1f, -0.1f, 0.0f);
       scene->setGravity(gravity);
   }

   if (key.keysym.sym == SDLK_5)
   {
       physx::PxVec3 gravity(0.0f, 0.0f, 0.1f);
       scene->setGravity(gravity);
   }

   if (key.keysym.sym == SDLK_6)
   {
       physx::PxVec3 gravity(0.0f, 0.0f, -0.1f);
       scene->setGravity(gravity);
   }

   if (key.keysym.sym == SDLK_7)
   {
       physx::PxVec3 gravity(0.0f, 0.0f, 0.0f);
       scene->setGravity(gravity);
   }
}


void GLViewAAATestingPhysics::onKeyUp( const SDL_KeyboardEvent& key )
{
   GLView::onKeyUp( key );
}


void Aftr::GLViewAAATestingPhysics::loadMap()
{
   this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
   this->actorLst = new WorldList();
   this->netLst = new WorldList();

   ManagerOpenGLState::GL_CLIPPING_PLANE = 1000.0;
   ManagerOpenGLState::GL_NEAR_PLANE = 0.1f;
   ManagerOpenGLState::enableFrustumCulling = false;
   Axes::isVisible = true;
   this->glRenderer->isUsingShadowMapping( false ); //set to TRUE to enable shadow mapping, must be using GL 3.2+

   this->cam->setPosition( 15,15,10 );

   std::string shinyRedPlasticCube( ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl" );
   std::string wheeledCar( ManagerEnvironmentConfiguration::getSMM() + "/models/rcx_treads.wrl" );
   std::string grass( ManagerEnvironmentConfiguration::getSMM() + "/models/grassFloor400x400_pp.wrl" );
   std::string human( ManagerEnvironmentConfiguration::getSMM() + "/models/human_chest.wrl" );
   
   //SkyBox Textures readily available
   std::vector< std::string > skyBoxImageNames; //vector to store texture paths
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_water+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_dust+6.jpg" );
   skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_mountains+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_winter+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/early_morning+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_afternoon+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_cloudy+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_cloudy3+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_day+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_day2+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_deepsun+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_evening+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_morning+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_morning2+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_noon+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_warp+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_Hubble_Nebula+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_gray_matter+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_easter+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_hot_nebula+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_ice_field+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_lemon_lime+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_milk_chocolate+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_solar_bloom+6.jpg" );
   //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_thick_rb+6.jpg" );

   {
      //Create a light
      float ga = 0.1f; //Global Ambient Light level for this module
      ManagerLight::setGlobalAmbientLight( aftrColor4f( ga, ga, ga, 1.0f ) );
      WOLight* light = WOLight::New();
      light->isDirectionalLight( true );
      light->setPosition( Vector( 0, 0, 100 ) );
      //Set the light's display matrix such that it casts light in a direction parallel to the -z axis (ie, downwards as though it was "high noon")
      //for shadow mapping to work, this->glRenderer->isUsingShadowMapping( true ), must be invoked.
      light->getModel()->setDisplayMatrix( Mat4::rotateIdentityMat( { 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD ) );
      light->setLabel( "Light" );
      worldLst->push_back( light );
   }

   {
      //Create the SkyBox
      WO* wo = WOSkyBox::New( skyBoxImageNames.at( 0 ), this->getCameraPtrPtr() );
      wo->setPosition( Vector( 0, 0, 0 ) );
      wo->setLabel( "Sky Box" );
      wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
      worldLst->push_back( wo );
   }

   { 
      ////Create the infinite grass plane (the floor)
      WO* wo = WO::New( grass, Vector( 1, 1, 1 ), MESH_SHADING_TYPE::mstFLAT );
      wo->setPosition( Vector( 0, 0, 0 ) );
      wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
      wo->upon_async_model_loaded( [wo]()
         {
            ModelMeshSkin& grassSkin = wo->getModel()->getModelDataShared()->getModelMeshes().at( 0 )->getSkins().at( 0 );
            grassSkin.getMultiTextureSet().at( 0 ).setTexRepeats( 5.0f );
            grassSkin.setAmbient( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Color of object when it is not in any light
            grassSkin.setDiffuse( aftrColor4f( 1.0f, 1.0f, 1.0f, 1.0f ) ); //Diffuse color components (ie, matte shading color of this object)
            grassSkin.setSpecular( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Specular color component (ie, how "shiney" it is)
            grassSkin.setSpecularCoefficient( 10 ); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
         } );
      wo->setLabel( "Grass" );
      worldLst->push_back( wo );
   }

   //{
   //   //Create the infinite grass plane that uses the Open Dynamics Engine (ODE)
   //   WO* wo = WOStatic::New( grass, Vector(1,1,1), MESH_SHADING_TYPE::mstFLAT );
   //   ((WOStatic*)wo)->setODEPrimType( ODE_PRIM_TYPE::PLANE );
   //   wo->setPosition( Vector(0,0,0) );
   //   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   //   wo->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0).getMultiTextureSet().at(0)->setTextureRepeats( 5.0f );
   //   wo->setLabel( "Grass" );
   //   worldLst->push_back( wo );
   //}

   //{
   //   //Create the infinite grass plane that uses NVIDIAPhysX(the floor)
   //   WO* wo = WONVStaticPlane::New( grass, Vector( 1, 1, 1 ), MESH_SHADING_TYPE::mstFLAT );
   //   wo->setPosition( Vector( 0, 0, 0 ) );
   //   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   //   wo->getModel()->getModelDataShared()->getModelMeshes().at( 0 )->getSkins().at( 0 ).getMultiTextureSet().at( 0 )->setTextureRepeats( 5.0f );
   //   wo->setLabel( "Grass" );
   //   worldLst->push_back( wo );
   //}

   //{
   //   //Create the infinite grass plane (the floor)
   //   WO* wo = WONVPhysX::New( shinyRedPlasticCube, Vector( 1, 1, 1 ), MESH_SHADING_TYPE::mstFLAT );
   //   wo->setPosition( Vector( 0, 0, 50.0f ) );
   //   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   //   wo->setLabel( "Grass" );
   //   worldLst->push_back( wo );
   //}

   //{
   //   WO* wo = WONVPhysX::New( shinyRedPlasticCube, Vector( 1, 1, 1 ), MESH_SHADING_TYPE::mstFLAT );
   //   wo->setPosition( Vector( 0, 0.5f, 75.0f ) );
   //   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   //   wo->setLabel( "Grass" );
   //   worldLst->push_back( wo );
   //}

   //{
   //   WO* wo = WONVDynSphere::New( ManagerEnvironmentConfiguration::getVariableValue( "sharedmultimediapath" ) + "/models/sphereRp5.wrl", Vector( 1.0f, 1.0f, 1.0f ), mstSMOOTH );
   //   wo->setPosition( 0, 0, 100.0f );
   //   wo->setLabel( "Sphere" );
   //   this->worldLst->push_back( wo );
   //}

   //{
   //   WO* wo = WOHumanCal3DPaladin::New( Vector( .5, 1, 1 ), 100 );
   //   ((WOHumanCal3DPaladin*)wo)->rayIsDrawn = false; //hide the "leg ray"
   //   ((WOHumanCal3DPaladin*)wo)->isVisible = false; //hide the Bounding Shell
   //   wo->setPosition( Vector( 20, 20, 20 ) );
   //   wo->setLabel( "Paladin" );
   //   worldLst->push_back( wo );
   //   actorLst->push_back( wo );
   //   netLst->push_back( wo );
   //   this->setActor( wo );
   //}
   //
   //{
   //   WO* wo = WOHumanCyborg::New( Vector( .5, 1.25, 1 ), 100 );
   //   wo->setPosition( Vector( 20, 10, 20 ) );
   //   wo->isVisible = false; //hide the WOHuman's bounding box
   //   ((WOHuman*)wo)->rayIsDrawn = false; //show the 'leg' ray
   //   wo->setLabel( "Human Cyborg" );
   //   worldLst->push_back( wo );
   //   actorLst->push_back( wo ); //Push the WOHuman as an actor
   //   netLst->push_back( wo );
   //   this->setActor( wo ); //Start module where human is the actor
   //}

   //{
   //   //Create and insert the WOWheeledVehicle
   //   std::vector< std::string > wheels;
   //   std::string wheelStr( "../../../shared/mm/models/WOCar1970sBeaterTire.wrl" );
   //   wheels.push_back( wheelStr );
   //   wheels.push_back( wheelStr );
   //   wheels.push_back( wheelStr );
   //   wheels.push_back( wheelStr );
   //   WO* wo = WOCar1970sBeater::New( "../../../shared/mm/models/WOCar1970sBeater.wrl", wheels );
   //   wo->setPosition( Vector( 5, -15, 20 ) );
   //   wo->setLabel( "Car 1970s Beater" );
   //   ((WOODE*)wo)->mass = 200;
   //   worldLst->push_back( wo );
   //   actorLst->push_back( wo );
   //   this->setActor( wo );
   //   netLst->push_back( wo );
   //}
   
   //Make a Dear Im Gui instance via the WOImGui in the engine... This calls
   //the default Dear ImGui demo that shows all the features... To create your own,
   //inherit from WOImGui and override WOImGui::drawImGui_for_this_frame(...) (among any others you need).
   WOImGui* gui = WOImGui::New( nullptr );
   gui->setLabel( "My Gui" );
   gui->subscribe_drawImGuiWidget(
      [this, gui]() //this is a lambda, the capture clause is in [], the input argument list is in (), and the body is in {}
      {
         ImGui::ShowDemoWindow(); //Displays the default ImGui demo from C:/repos/aburn/engine/src/imgui_implot/implot_demo.cpp
         WOImGui::draw_AftrImGui_Demo( gui ); //Displays a small Aftr Demo from C:/repos/aburn/engine/src/aftr/WOImGui.cpp
         ImPlot::ShowDemoWindow(); //Displays the ImPlot demo using ImGui from C:/repos/aburn/engine/src/imgui_implot/implot_demo.cpp
      } );
   this->worldLst->push_back( gui );

   createAAATestingPhysicsWayPoints();

   /////PHYSX

   f = PxCreateFoundation(PX_PHYSICS_VERSION, a, e);
   p = PxCreatePhysics(PX_PHYSICS_VERSION, *f, physx::PxTolerancesScale(), (bool) true, (physx::PxPvd*) 0);

   //Used different syntax cuz the other one was weird
   //physx::PxPhysics* p = PxCreatePhysics(PX_PHYSICS_VERSION, *f, physx::PxTolerancesScale(), (bool)true);
   physx::PxSceneDesc sc(p->getTolerancesScale());
   sc.filterShader = physx::PxDefaultSimulationFilterShader;
   sc.cpuDispatcher = physx::PxDefaultCpuDispatcherCreate(2);
    scene = p->createScene(sc);
    scene->setFlag(physx::PxSceneFlag::eENABLE_ACTIVE_ACTORS, true);

    physx::PxMaterial* gMaterial = p->createMaterial(0.5f, 0.5f, 0.6f);
    physx::PxRigidStatic* groundPlane = PxCreatePlane(*p, physx::PxPlane(0, 0, 1, 0), *gMaterial);//good for the grass
    scene->addActor(*groundPlane);

    physx::PxShape* shape = p->createShape(physx::PxBoxGeometry(2, 2, 2), *gMaterial);
    physx::PxTransform t({ 30, 20, 300 });
    actor = p->createRigidDynamic(t);
    actor->attachShape(*shape);
    scene->addActor(*actor);

    WO* wo = WO::New(shinyRedPlasticCube, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
    actor->userData = wo;
    
    this->worldLst->push_back(wo);

    physx::PxShape* shape2 = p->createShape(physx::PxBoxGeometry(2, 2, 2), *gMaterial);
    physx::PxTransform t2({ 30, 20, 150 });
    actor2 = p->createRigidDynamic(t2);
    actor2->attachShape(*shape2);
    scene->addActor(*actor2);

    WO* wo2 = WO::New(shinyRedPlasticCube, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
    actor2->userData = wo2;

    this->worldLst->push_back(wo2);




}


void GLViewAAATestingPhysics::createAAATestingPhysicsWayPoints()
{
   // Create a waypoint with a radius of 3, a frequency of 5 seconds, activated by GLView's camera, and is visible.
   WayPointParametersBase params(this);
   params.frequency = 5000;
   params.useCamera = true;
   params.visible = true;
   WOWayPointSpherical* wayPt = WOWayPointSpherical::New( params, 3 );
   wayPt->setPosition( Vector( 50, 0, 3 ) );
   worldLst->push_back( wayPt );
}
