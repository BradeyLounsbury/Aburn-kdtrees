#include "GLViewKD_Trees.h"

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
#include "MGLPointCloud.h"
#include "MGLIndexedGeometry.h"
#include "IndexedGeometryLines.h"
#include "GLSLShaderDefaultIndexedGeometryLinesGL32.h"
#include "IndexedGeometryPointCloud.h"
#include "GLSLShaderPointTesselatorBillboard.h"
#include "WOPointCloud.h"
#include "quicksort.h"
#include "KD_tree.h"
#include "helpers.h"
#include "WORay.h"
#include "AftrUtilities.h"
#include "WOQuad.h"

using namespace Aftr;

Vector head, tail;
unsigned int cube_id;
unsigned int plane_id;
unsigned int griff_id;
WORay* ray;
WOPointCloud* pt_cloud;
std::map<WO*, KD_Node*> MapPlanetoTree;
std::map<KD_Node*, std::vector<Vector>> MapNodetoVerts;

bool gen_tree_for_pt, gen_tree_for_griff, clear_tree, test_griff, test_pt, gen_ray_for_pt, gen_ray_for_griff;
int iterations = 0;

GLViewKD_Trees* GLViewKD_Trees::New( const std::vector< std::string >& args )
{
   GLViewKD_Trees* glv = new GLViewKD_Trees( args );
   glv->init( Aftr::GRAVITY, Vector( 0, 0, -1.0f ), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE );
   glv->onCreate();
   return glv;
}


GLViewKD_Trees::GLViewKD_Trees( const std::vector< std::string >& args ) : GLView( args )
{
   //Initialize any member variables that need to be used inside of LoadMap() here.
   //Note: At this point, the Managers are not yet initialized. The Engine initialization
   //occurs immediately after this method returns (see GLViewKD_Trees::New() for
   //reference). Then the engine invoke's GLView::loadMap() for this module.
   //After loadMap() returns, GLView::onCreate is finally invoked.

   //The order of execution of a module startup:
   //GLView::New() is invoked:
   //    calls GLView::init()
   //       calls GLView::loadMap() (as well as initializing the engine's Managers)
   //    calls GLView::onCreate()

   //GLViewKD_Trees::onCreate() is invoked after this module's LoadMap() is completed.
}


void GLViewKD_Trees::onCreate()
{
   //GLViewKD_Trees::onCreate() is invoked after this module's LoadMap() is completed.
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
}


GLViewKD_Trees::~GLViewKD_Trees()
{
   //Implicitly calls GLView::~GLView()
}

static bool move_forward, move_left, move_right, move_backward = false;
void GLViewKD_Trees::updateWorld()
{
   GLView::updateWorld(); //Just call the parent's update world first.
                          //If you want to add additional functionality, do it after
                          //this call.

   if (move_forward) {
       this->cam->moveInLookDirection();
   }
   if (move_left) {
       this->cam->moveLeft();
   }
   if (move_right) {
       this->cam->moveRight();
   }
   if (move_backward) {
       this->cam->moveOppositeLookDirection();
   }

   ray->setRayHeadAndTail(head, tail);

   if (gen_tree_for_pt) {
       gen_tree_for_pt = false;

       generate_KD_Tree(this, pt_cloud->getPosition(), pt_cloud->getPoints(), pt_cloud->getModel()->getBoundingBox().getMin(), pt_cloud->getModel()->getBoundingBox().getMax(), MapPlanetoTree, MapNodetoVerts, iterations, true);
   }

   if (gen_tree_for_griff) {
       gen_tree_for_griff = false;

       WO* wo = this->worldLst->getWOByID(griff_id);
       auto bb = wo->getModel()->getBoundingBox();
       auto verts = wo->getModel()->getCompositeVertexList();
       generate_KD_Tree(this, wo->getPosition(), wo->getModel()->getCompositeVertexList(), bb.getMin(), bb.getMax(), MapPlanetoTree, MapNodetoVerts, iterations, true);
   }

   if (clear_tree) {
       clear_tree = false;

       for (auto p : MapPlanetoTree) {
           this->worldLst->eraseViaWOptr(p.first);
       }
       MapPlanetoTree.clear();
       MapNodetoVerts.clear();
   }

   if (test_griff) {
       test_griff = false;

       Vector output{ 0,0,0 };
       std::set<Vector> possible_verts, og_verts;
       for (auto v : this->worldLst->getWOByID(griff_id)->getModel()->getCompositeVertexList())
           og_verts.insert(v);
       for (auto p : MapPlanetoTree) {
           //std::cout << MapNodetoVerts.find(p.second)->second.size() << std::endl;
           if (line_intersects_plane(p.first, ray, output))
           {
               for (auto v : MapNodetoVerts.find(p.second)->second) {
                   possible_verts.insert(v);
               }
               std::cout << "Intersects!" << std::endl;
               std::cout << output << std::endl;
           }
       }

       std::cout << "Griff total vert count = " << og_verts.size() << std::endl;
       std::cout << "KD_Tree total possible vert count = " << possible_verts.size() << std::endl;
   }

   if (test_pt) {
       test_pt = false;

       Vector output{ 0,0,0 };
       std::set<Vector> possible_verts, og_verts;
       for (auto v : pt_cloud->getPoints())
           og_verts.insert(v);
       for (auto p : MapPlanetoTree) {
           //std::cout << MapNodetoVerts.find(p.second)->second.size() << std::endl;
           if (line_intersects_plane(p.first, ray, output))
           {
               for (auto v : MapNodetoVerts.find(p.second)->second) {
                   possible_verts.insert(v);
               }
               std::cout << "Intersects!" << std::endl;
               std::cout << output << std::endl;
           }
       }

       std::cout << "Pt Cloud total vert count = " << og_verts.size() << std::endl;
       std::cout << "KD_Tree total possible vert count = " << possible_verts.size() << std::endl;
   }

   if (gen_ray_for_griff) {
       gen_ray_for_griff = false;

       WO* wo = this->worldLst->getWOByID(griff_id);
       auto bb = wo->getModel()->getBoundingBox();

       for (int i = 0; i < 3; i++) {
           head[i] = ManagerRandomNumber::getRandomFloat(bb.getMin()[i] - 1 + wo->getPosition()[i], bb.getMax()[i] + 1 + wo->getPosition()[i]);
           tail[i] = ManagerRandomNumber::getRandomFloat(bb.getMin()[i] - 1 + wo->getPosition()[i], bb.getMax()[i] + 1 + wo->getPosition()[i]);
       }
   }

   if (gen_ray_for_pt) {
       gen_ray_for_pt = false;

       auto bb = pt_cloud->getModel()->getBoundingBox();

       for (int i = 0; i < 3; i++) {
           head[i] = ManagerRandomNumber::getRandomFloat(bb.getMin()[i] - 1 + pt_cloud->getPosition()[i], bb.getMax()[i] + 1 + pt_cloud->getPosition()[i]);
           tail[i] = ManagerRandomNumber::getRandomFloat(bb.getMin()[i] - 1 + pt_cloud->getPosition()[i], bb.getMax()[i] + 1 + pt_cloud->getPosition()[i]);
       }
   }
}


void GLViewKD_Trees::onResizeWindow( GLsizei width, GLsizei height )
{
   GLView::onResizeWindow( width, height ); //call parent's resize method.
}


void GLViewKD_Trees::onMouseDown( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseDown( e );
}


void GLViewKD_Trees::onMouseUp( const SDL_MouseButtonEvent& e )
{
   GLView::onMouseUp( e );
}


void GLViewKD_Trees::onMouseMove( const SDL_MouseMotionEvent& e )
{
   GLView::onMouseMove( e );
}


void GLViewKD_Trees::onKeyDown( const SDL_KeyboardEvent& key )
{
   GLView::onKeyDown( key );
   if( key.keysym.sym == SDLK_0 )
      this->setNumPhysicsStepsPerRender( 1 );

   if (key.keysym.sym == SDLK_w)
   {
       move_forward = true;
   }
   if (key.keysym.sym == SDLK_a)
   {
       move_left = true;
   }
   if (key.keysym.sym == SDLK_s)
   {
       move_backward = true;
   }
   if (key.keysym.sym == SDLK_d)
   {
       move_right = true;
   }

   if( key.keysym.sym == SDLK_1 )
   {
       this->worldLst->getWOByID(cube_id)->isVisible = true;
       this->worldLst->getWOByID(cube_id)->getModel()->renderBBox = true;
   }

   if (key.keysym.sym == SDLK_2)
   {
       this->worldLst->getWOByID(cube_id)->isVisible = true;
       this->worldLst->getWOByID(cube_id)->getModel()->renderBBox = false;
   }

   if (key.keysym.sym == SDLK_3)
   {       
       std::vector<aftrColor4ub> c;
       for (int i = 0; i < pt_cloud->getNumPoints(); i++) {
           float r[4];
           for (int j = 0; j < 3; j++)
               r[j] = ManagerRandomNumber::getRandomFloat(0, 255);
           r[3] = 255.0f;

           c.push_back(aftrColor4ub(r));
       }

       pt_cloud->setColors(c);
   }

   if (key.keysym.sym == SDLK_4)
   {
       std::vector< Vector > v;
       for (int i = 0; i < pt_cloud->getNumPoints(); i++)
       {
           Vector r;
           for (int j = 0; j < 3; j++)
               r[j] = ManagerRandomNumber::getRandomFloat(-5, 5);
           v.push_back(r);
       }

       std::vector<aftrColor4ub> c;
       for (int i = 0; i < pt_cloud->getNumPoints(); i++) {
           float r[4];
           for (int j = 0; j < 3; j++)
               r[j] = 236.0f;
           r[3] = 0.0f;

           c.push_back(aftrColor4ub(r));
       }

       pt_cloud->setPoints(v);
       pt_cloud->setColors(c);
   }
}


void GLViewKD_Trees::onKeyUp( const SDL_KeyboardEvent& key )
{
   GLView::onKeyUp( key );

   if (key.keysym.sym == SDLK_w)
   {
       move_forward = false;
   }
   if (key.keysym.sym == SDLK_a)
   {
       move_left = false;
   }
   if (key.keysym.sym == SDLK_s)
   {
       move_backward = false;
   }
   if (key.keysym.sym == SDLK_d)
   {
       move_right = false;
   }
}


void Aftr::GLViewKD_Trees::loadMap()
{
   this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
   this->actorLst = new WorldList();
   this->netLst = new WorldList();

   ManagerOpenGLState::GL_CLIPPING_PLANE = 1000.0;
   ManagerOpenGLState::GL_NEAR_PLANE = 0.1f;
   ManagerOpenGLState::enableFrustumCulling = false;
   Axes::isVisible = false;
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
   //    WO* wo = WO::New(shinyRedPlasticCube, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
   //    wo->setPosition(Vector(10, 0, 5));
   //    wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   //    wo->upon_async_model_loaded([wo]()
   //        {
   //            ModelMeshSkin& cubeSkin = wo->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);
   //            cubeSkin.setAmbient(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Color of object when it is not in any light
   //            cubeSkin.setDiffuse(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f)); //Diffuse color components (ie, matte shading color of this object)
   //            cubeSkin.setSpecular(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Specular color component (ie, how "shiney" it is)
   //            cubeSkin.setSpecularCoefficient(10); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
   //        });
   //    wo->setLabel("cube rendered");
   //    worldLst->push_back(wo);
   //}
   
   //{
   //    WO* wo = WO::New(shinyRedPlasticCube, Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
   //    wo->setPosition(Vector(0, 0, 5));
   //    wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   //    wo->upon_async_model_loaded([wo]()
   //        {
   //            ModelMeshSkin& cubeSkin = wo->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0);
   //            cubeSkin.setAmbient(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Color of object when it is not in any light
   //            cubeSkin.setDiffuse(aftrColor4f(1.0f, 1.0f, 1.0f, 1.0f)); //Diffuse color components (ie, matte shading color of this object)
   //            cubeSkin.setSpecular(aftrColor4f(0.4f, 0.4f, 0.4f, 1.0f)); //Specular color component (ie, how "shiney" it is)
   //            cubeSkin.setSpecularCoefficient(10); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
   //        });
   //    wo->setLabel("cube bbox");
   //    cube_id = wo->getID();
   //    worldLst->push_back(wo);
   //}

   //cube wireframe
   {       
       WO* wo = WO::New();
       MGLIndexedGeometry* mgl = MGLIndexedGeometry::New(wo);
       std::vector<Vector> lines;
       lines.push_back(Vector(0, 0, 0)); lines.push_back(Vector(1, 0, 0));
       lines.push_back(Vector(0, 0, 0)); lines.push_back(Vector(0, 1, 0));
       lines.push_back(Vector(0, 0, 0)); lines.push_back(Vector(0, 0, 1));
       lines.push_back(Vector(0, 0, 1)); lines.push_back(Vector(1, 0, 1));
       lines.push_back(Vector(0, 0, 1)); lines.push_back(Vector(0, 1, 1));
       lines.push_back(Vector(0, 1, 0)); lines.push_back(Vector(0, 1, 1));
       lines.push_back(Vector(0, 1, 0)); lines.push_back(Vector(1, 1, 0));
       lines.push_back(Vector(1, 0, 0)); lines.push_back(Vector(1, 0, 1));
       lines.push_back(Vector(1, 0, 0)); lines.push_back(Vector(1, 1, 0));
       lines.push_back(Vector(1, 1, 1)); lines.push_back(Vector(1, 0, 1));
       lines.push_back(Vector(1, 1, 1)); lines.push_back(Vector(0, 1, 1));
       lines.push_back(Vector(1, 1, 1)); lines.push_back(Vector(1, 1, 0));
       aftrColor4ub r = aftrColor4ub{ 255,0,0,255 };
       aftrColor4ub g = aftrColor4ub{ 0,255,0,255 };
       aftrColor4ub b = aftrColor4ub{ 0,0,255,255 };
       std::vector< aftrColor4ub > colors = { r,r,r,r,r,r,r,r,r,r,r,r,r,r,r,r,r,r,r,r,r,r,r,r };
       IndexedGeometryLines* geom = IndexedGeometryLines::New(lines, colors);
       geom->setLineWidthInPixels(1.5);
       mgl->setIndexedGeometry(geom);
       GLSLShaderDefaultIndexedGeometryLinesGL32* shdr = GLSLShaderDefaultIndexedGeometryLinesGL32::New();
       mgl->getSkin().setShader(shdr);
       wo->setModel(mgl);
       wo->setLabel("IndexedGeometryCube");
       wo->setPosition(Vector(0,10,10));
       //this->worldLst->push_back(wo);
   }

   //plane wireframe
   {
       WO* wo = WO::New();
       MGLIndexedGeometry* mgl = MGLIndexedGeometry::New(wo);
       auto &bb = mgl->getBoundingBox();
       std::vector<Vector> lines;
       lines.push_back(Vector(0, 0, 0)); lines.push_back(Vector(1, 0, 0));
       lines.push_back(Vector(0, 0, 0)); lines.push_back(Vector(0, 1, 0));
       lines.push_back(Vector(1, 1, 0)); lines.push_back(Vector(1, 0, 0));
       lines.push_back(Vector(1, 1, 0)); lines.push_back(Vector(0, 1, 0));
       aftrColor4ub r = aftrColor4ub{ 255,0,0,255 };
       aftrColor4ub g = aftrColor4ub{ 0,255,0,255 };
       aftrColor4ub b = aftrColor4ub{ 0,0,255,255 };
       std::vector< aftrColor4ub > colors = { b,b,b,b,b,b,b,b };
       IndexedGeometryLines* geom = IndexedGeometryLines::New(lines, colors);
       geom->setLineWidthInPixels(1.5);
       mgl->setIndexedGeometry(geom);
       bb = mgl->getBoundingBox();
       //mgl->setBoundingBox(Vector());
       GLSLShaderDefaultIndexedGeometryLinesGL32* shdr = GLSLShaderDefaultIndexedGeometryLinesGL32::New();
       mgl->getSkin().setShader(shdr);
       wo->setModel(mgl);
       bb = wo->getModel()->getBoundingBox();
       plane_id = wo->getID();
       wo->setPosition(Vector(0, 15, 10));
       bb = wo->getModel()->getBoundingBox();
       //this->worldLst->push_back(wo);
   }

   // wo plane
   {
       WOQuad* wo = WOQuad::New(QuadOrientation::qoXY, 2, 2, Vector(0.0f,0.0f,1.0f), false, true, false);
       auto &skins = wo->getModel()->getSkins();
       wo->setPosition(Vector(0, 15, 10));
       //this->worldLst->push_back(wo);
   }

   // wo point cloud
   {
       std::vector< Vector > v;

       for (int i = 0; i < 500; i++)
       {
           Vector r;
           for (int j = 0; j < 3; j++)
               r[j] = ManagerRandomNumber::getRandomFloat(-5, 5);
           v.push_back(r);
       }

       std::vector<aftrColor4ub> c;
       for (int i = 0; i < v.size(); i++) {
           float r[4];
           for (int j = 0; j < 3; j++)
               r[j] = ManagerRandomNumber::getRandomFloat(0, 255);
           r[3] = 255.0f;

           c.push_back(aftrColor4ub(r));
       }

       pt_cloud = WOPointCloud::New(this->getCameraPtrPtr(), true, false, false);
       pt_cloud->setPoints(v);
       pt_cloud->setColors(c);
       pt_cloud->setPosition(0, 0, 5);
       pt_cloud->setLabel("PointCloud");
       pt_cloud->getModel()->renderBBox = false;
       pt_cloud->setSizeOfEachPoint(0.3, 0.3);
       this->worldLst->push_back(pt_cloud);
   }

   // griff
   {
       std::string griff(ManagerEnvironmentConfiguration::getLMM() + "/models/griff/griff.obj");
       WO* griffWO = WO::New(griff, Vector(0.07, 0.07, 0.07), MESH_SHADING_TYPE::mstFLAT);
       griffWO->setPosition(10, 0, 6.5);
       griffWO->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
       griff_id = griffWO->getID();
       //griffWO->isVisible = false;

       this->worldLst->push_back(griffWO);
   }

   // wo ray
   {
       ray = WORay::New(Vector(7, 1.5, 0), Vector(7, 1.5, 5));
       ray->setColor_Head_and_Tail(aftrColor4ub{ 255,255,0,255 }, aftrColor4ub{ 255,255,0,255 });
       this->worldLst->push_back(ray);
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
           ImGui::Begin("KD_Trees");

           ImGui::Text("KD-Tree Properties");
           ImGui::SliderInt("Iterations", &iterations, 1, 20);
           if (ImGui::Button("Tree for Griff")) {
               gen_tree_for_griff = true;
           }
           ImGui::SameLine();
           if (ImGui::Button("Tree for Pt Cloud")) {
               gen_tree_for_pt = true;
           }
           ImGui::SameLine();
           if (ImGui::Button("Clear Trees")) {
               clear_tree = true;
           }
           ImGui::Separator();

           ImGui::Text("Ray Properties");
           ImGui::SliderFloat("Head.x", &head.x, -5, 20);
           ImGui::SliderFloat("Head.y", &head.y, -5, 20);
           ImGui::SliderFloat("Head.z", &head.z, 0, 20);
           ImGui::Separator();

           ImGui::SliderFloat("Tail.x", &tail.x, -5, 20);
           ImGui::SliderFloat("Tail.y", &tail.y, -5, 20);
           ImGui::SliderFloat("Tail.z", &tail.z, 0, 20);
           ImGui::Separator();

           if (ImGui::Button("Gen Ray for Griff")) {
               gen_ray_for_griff = true;
           }
           ImGui::SameLine();
           if (ImGui::Button("Gen Ray for Pt Cloud")) {
               gen_ray_for_pt = true;
           }
           if (ImGui::Button("Test Ray for Griff")) {
               test_griff = true;
           }
           ImGui::SameLine();
           if (ImGui::Button("Test Ray for Pt Cloud")) {
               test_pt = true;
           }

           ImGui::End();
      } );
   this->worldLst->push_back( gui );

   //createKD_TreesWayPoints();
}


void GLViewKD_Trees::createKD_TreesWayPoints()
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
