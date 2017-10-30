/**************************************************************************************/
/*                                                                                    */
/*  Visualization Library                                                             */
/*  http://www.visualizationlibrary.org                                               */
/*                                                                                    */
/*  Copyright (c) 2005-2010, Michele Bosi                                             */
/*  All rights reserved.                                                              */
/*                                                                                    */
/*  Redistribution and use in source and binary forms, with or without modification,  */
/*  are permitted provided that the following conditions are met:                     */
/*                                                                                    */
/*  - Redistributions of source code must retain the above copyright notice, this     */
/*  list of conditions and the following disclaimer.                                  */
/*                                                                                    */
/*  - Redistributions in binary form must reproduce the above copyright notice, this  */
/*  list of conditions and the following disclaimer in the documentation and/or       */
/*  other materials provided with the distribution.                                   */
/*                                                                                    */
/*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND   */
/*  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED     */
/*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE            */
/*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR  */
/*  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    */
/*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      */
/*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON    */
/*  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT           */
/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS     */
/*  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                      */
/*                                                                                    */
/**************************************************************************************/

#include "BaseDemo.hpp"
#include <vlGraphics/Geometry.hpp>
#include <vlGraphics/GLSL.hpp>
#include <vlGraphics/GeometryPrimitives.hpp>
#include <vlGraphics/DrawTransformFeedback.hpp>

#include <vlGraphics/Extrusion.hpp>

#include <vlGraphics/OpenGLContext.hpp>
//#include <3rdparty/Khronos/GL/khronos_glext.h>

#include <array>
#include <random>

using namespace vl;

constexpr unsigned int const MAX_PARTICLES = 100;

#define PARTICLE_TYPE_LAUNCHER 0.0f
#define PARTICLE_TYPE_SHELL 1.0f
#define PARTICLE_TYPE_SECONDARY_SHELL 2.0f

class App_GLSL_TransformFeedback: public BaseDemo
{
	struct Particle
	{
		float Type; 
		vl::vec3 Pos;
		vl::vec3 Vel; 
		float LifetimeMillis; 
	};

	enum {
	   VA_Type = VA_MaxAttribCount + 1,
	   VA_Velocity = VA_MaxAttribCount + 2,
	   VA_Lifetime = VA_MaxAttribCount + 3
	};

	class SimpleDrawArraysCallback : public ActorEventCallback 
	{
        public:
          SimpleDrawArraysCallback(OpenGLContext* glContext) : 
            ActorEventCallback(),
	    mGLContext(glContext) {
		VL_CHECK(mGLContext.get() != nullptr);
	  }

          virtual void onActorRenderStarted(Actor* actor, real, const Camera*, Renderable*, const Shader* shader, int) {
             const GLSLProgram* program = shader->glslProgram();
             const Geometry* geometry = actor->lod(0)->as< Geometry >();
         if(geometry && geometry->vertexAttribArray(VA_Position) && geometry->vertexAttribArray(VA_Position) ->bufferObject() &&
                 program && program->handle() && program->linked()) {
               mGLContext->useGLSLProgram(program);
               glBindBuffer(GL_ARRAY_BUFFER, geometry->vertexAttribArray(VA_Position)->bufferObject()->handle());
               glDrawArrays(PT_POINTS, 0, 1);
               glBindBuffer(GL_ARRAY_BUFFER, 0);
	       mGLContext->useGLSLProgram(nullptr);
	       setEnabled(false);
         }
         // Remettre états originaux ( car hors pipeline)
	  }
 
          virtual void onActorDelete(Actor*) {}

	    
	private:
	  ref< OpenGLContext > mGLContext;
	};
	
public:
  App_GLSL_TransformFeedback()
  {		
  }

  void initEvent()
  {
    if (!Has_GLSL)
    {
      Log::error("OpenGL Shading Language not supported.\n");
      Time::sleep(2000);
      exit(1);
    }
	
	if(!Has_Transform_Feedback)
	{
	  Log::error("Transform Feedback not supported.\n");
	  Time::sleep(2000);
	  exit(1);
	}

    // Generates the actual extrusion
    vl::Extrusion extrusion;

    // Define the silhouette to be extruded: 24 sided circle
    const int sides = 24;
    const float radius = 1.0f;
    for(int i=0; i<sides; ++i)
    {
      float t = (float)i/sides*vl::fPi*2.0f;
      extrusion.silhouette().push_back(vl::fvec2(::cos(t)*radius, ::sin(t)*radius));
    }

    // Define the path along which the silhouette is extruded.
    // Note: the first and last control points are not part of the actual extrusion
    // but are used to define the orientation of the first and start segment.
    extrusion.positionPath().push_back(vl::fvec3(-5, 0, 0) + vl::fvec3(-1,0,0));
    extrusion.positionPath().push_back(vl::fvec3(-5, 0, 0));
    extrusion.positionPath().push_back(vl::fvec3(-5, 5, 0));
    extrusion.positionPath().push_back(vl::fvec3(+5, 5, 0));
    extrusion.positionPath().push_back(vl::fvec3(+5, 0, 0));
    extrusion.positionPath().push_back(vl::fvec3(+5, 0, 0) + vl::fvec3(+1,0,0));

    // Setup the extrusion options.
    extrusion.setSilhouetteMode(vl::SilhouetteClosed);
    extrusion.setSmooth(false);
    extrusion.setFillBottom(true); // start of the extrusion is closed
    extrusion.setFillTop(true);    // end of the extrusion is closed

    // Setup a simple white effect.
    vl::ref<vl::Effect> effect = new vl::Effect;
    effect->shader()->setRenderState( new vl::Light, 0 );
    effect->shader()->enable(vl::EN_LIGHTING);
    effect->shader()->enable(vl::EN_DEPTH_TEST);

    // Generates the extrusion.
    vl::ref<vl::Geometry> geom = extrusion.extrude();
    sceneManager()->tree()->addActor( geom.get(), effect.get(), NULL );

    // Utility function that visualizes the extrusion path.
    showPath(extrusion.positionPath(),vl::red);


    trackball()->setPivot(vl::vec3(0.0f, 0.0f, 0.0f));

    Log::notify(appletInfo());

    createRandomTexture(MAX_PARTICLES);

    initShaders();
		
		
/*    particles.fill({0});

    particles[0].Type = PARTICLE_TYPE_LAUNCHER;
    particles[0].Pos = vl::vec3(0.0f, 0.0f, 0.0f);
    particles[0].Vel = vl::vec3(0.0f, 0.0001f, 0.0f);
    particles[0].LifetimeMillis = 0.0f;

    particlesBuffer1 = new BufferObject();
    particlesBuffer1->setBufferData(particles.size() * sizeof(Particle), &particles[0], BU_DYNAMIC_DRAW);

    particlesBuffer2 = new BufferObject();
    particlesBuffer2->setBufferData(particles.size() * sizeof(Particle), &particles[0], BU_DYNAMIC_DRAW);*/


    // Initialization
    particlesType.fill(0.0f);
    particlesPosition.fill(vec3(0.0f, 0.0f, 0.0f));
    particlesVelocity.fill(vec3(0.0f, 0.0f, 0.0f));
    particlesLifetime.fill(0.0f);


    // First particule
    particlesType[0] = PARTICLE_TYPE_LAUNCHER;
    particlesPosition[0] = vec3(1.0f, 1.0f, 1.0f);  //!< Careful -> for test
    particlesVelocity[0] = vec3(0.0f, 0.0001f, 0.0f);
    particlesLifetime[0] = 0.0f;


    // Creation and filling of the GPU buffer
    for(unsigned short int i = 0; i < 2; ++i) {
        particlesBufferType[i] = new ArrayFloat1();
        particlesBufferPosition[i] = new ArrayFloat3();
        particlesBufferVelocity[i] = new ArrayFloat3();
        particlesBufferLifetime[i] = new ArrayFloat1();

        particlesBufferType[i]->setUsage(BU_DYNAMIC_DRAW);
        particlesBufferPosition[i]->setUsage(BU_DYNAMIC_DRAW);
        particlesBufferVelocity[i]->setUsage(BU_DYNAMIC_DRAW);
        particlesBufferLifetime[i]->setUsage(BU_DYNAMIC_DRAW);

        particlesBufferType[i]->resize(MAX_PARTICLES);
        particlesBufferPosition[i]->resize(MAX_PARTICLES);
        particlesBufferVelocity[i]->resize(MAX_PARTICLES);
        particlesBufferLifetime[i]->resize(MAX_PARTICLES);

        std::memcpy(particlesBufferType[i]->ptr(), &particlesType[0], MAX_PARTICLES * sizeof(float));
        std::memcpy(particlesBufferPosition[i]->ptr(), &particlesPosition[0], MAX_PARTICLES * sizeof(vec3));
        std::memcpy(particlesBufferVelocity[i]->ptr(), &particlesVelocity[0], MAX_PARTICLES * sizeof(vec3));
        std::memcpy(particlesBufferLifetime[i]->ptr(), &particlesLifetime[0], MAX_PARTICLES * sizeof(float));
    }

    // Setup a simple white effect.

    // Setup debug 1st particle
    vl::ref<vl::Effect> effect1stParticle = new vl::Effect;
    //effect1stParticle->shader()->enable(vl::EN_LIGHTING);
    effect1stParticle->shader()->gocColor()->setValue(vl::fvec4(1.0f, 0.0f, 0.0f, 1.0f));
    effect1stParticle->shader()->enable(vl::EN_DEPTH_TEST);
    effect1stParticle->shader()->gocPolygonMode()->set(vl::PM_POINT, vl::PM_POINT);
    effect1stParticle->shader()->gocPointSize()->set(3.0f);

    vl::ref< vl::Geometry > geom1stParticle = new vl::Geometry;
    vl::ref< vl::DrawArrays > drawArrays = new vl::DrawArrays(vl::PT_POINTS, 0, 1);
    geom1stParticle->setVertexArray(particlesBufferPosition[0].get());

    geom1stParticle->drawCalls().push_back(drawArrays.get());

    sceneManager()->tree()->addActor(geom1stParticle.get(), effect1stParticle.get(), nullptr);

    transformFeedback1->setTransformFeedbackBufferMode(BM_SeparateAttribs);
    transformFeedback2->setTransformFeedbackBufferMode(BM_SeparateAttribs);
    
    transformFeedback1->addArray(particlesBufferType[0].get());
    transformFeedback1->addArray(particlesBufferPosition[0].get());
    transformFeedback1->addArray(particlesBufferVelocity[0].get());
    transformFeedback1->addArray(particlesBufferLifetime[0].get());

    transformFeedback2->addArray(particlesBufferType[1].get());
    transformFeedback2->addArray(particlesBufferPosition[1].get());
    transformFeedback2->addArray(particlesBufferVelocity[1].get());
    transformFeedback2->addArray(particlesBufferLifetime[1].get());

    // First Rendering geometry creation (to initialize and fill transform feedback)
    /*ref< Geometry > geom = new Geometry();

    geom->setVertexAttribArray(VA_Position, particlesBufferPosition[0].get());
    geom->setVertexAttribArray(VA_Type, particlesBufferType[0].get());
    geom->setVertexAttribArray(VA_Velocity, particlesBufferVelocity[0].get());
    geom->setVertexAttribArray(VA_Lifetime, particlesBufferLifetime[0].get());

    ref< DrawArrays > drawArrays = new DrawArrays(PT_POINTS, 0, 1);

    geom->drawCalls().push_back(drawArrays.get());

    ref< Actor > actor = sceneManager()->tree()->addActor(geom.get(), fx1.get(), nullptr);*/

    ref< SimpleDrawArraysCallback > drawArraysCallback = new SimpleDrawArraysCallback(openglContext());


    /* On créé 3 géometries. Les deux premieres servent à faire un rendu hors ecran pour mettre à jour les particules.
     * La dernière sert à faire le rendu réels des particules, en ne prenant que le buffer courant des positions
     * des particules
     *
     *
     */

    ref< Geometry > geomToRender1 = new Geometry();
    ref< Geometry > geomToRender2 = new Geometry();

    ref< DrawTransformFeedback > dtf1 = new DrawTransformFeedback(PT_POINTS, transformFeedback1.get());
    ref< DrawTransformFeedback > dtf2 = new DrawTransformFeedback(PT_POINTS, transformFeedback2.get());

    drawTransformFeedbacks.push_back(dtf1.get());
    drawTransformFeedbacks.push_back(dtf2.get());
   
    //geomToRender1->drawCalls() = drawTransformFeedbacks;

    geomToRender1->drawCalls().push_back(dtf1.get());
    geomToRender2->drawCalls().push_back(dtf2.get());


    ref< Actor > actor = sceneManager()->tree()->addActor(geomToRender1.get(), fx1.get(), nullptr);
    sceneManager()->tree()->addActor(geomToRender2.get(), fx2.get(), nullptr);

    actor->actorEventCallbacks()->push_back(drawArraysCallback.get());
    
    mTime.start();

  }

  // Utility function to display a path
  void showPath(const std::vector<vl::fvec3>& ctrl_points, const vl::fvec4& color)
  {
    // generate line geometry with lines and points
    vl::ref<vl::Geometry>   geom       = new vl::Geometry;
    vl::ref<vl::ArrayFloat3> vert_array = new vl::ArrayFloat3;
    geom->setVertexArray( vert_array.get() );
    vert_array->initFrom(ctrl_points);
    geom->drawCalls().push_back(new vl::DrawArrays(vl::PT_LINE_STRIP, 0, (int)vert_array->size())); // lines
    geom->drawCalls().push_back(new vl::DrawArrays(vl::PT_POINTS,     0, (int)vert_array->size())); // points

    // setup simple effect
    vl::ref<vl::Effect> effect = new vl::Effect;
    effect->shader()->gocColor()->setValue(color);
    effect->shader()->enable(vl::EN_LINE_STIPPLE);
    effect->shader()->gocPointSize()->set(3);
    effect->shader()->gocLineStipple()->setPattern(0x3333);
    effect->setRenderRank(1); // always draw over the pipe

    sceneManager()->tree()->addActor( geom.get(), effect.get(), NULL );
  }

  void createRandomTexture(unsigned int size) 
  {
	  std::random_device rd;
	  std::mt19937 gen(rd());
	  std::uniform_real_distribution< float > dis(0, 1);

	  vec3* randomData = new vec3[size];

	  for(unsigned int i = 0; i < size; ++i) 
	  {
		  randomData[i].x() = dis(gen);
		  randomData[i].y() = dis(gen);
		  randomData[i].z() = dis(gen);
	  }

	  //ref< Image > randomImage = new Image(randomData, size * sizeof(vec3));

	  //mRandomTexture = new Texture(randomImage.get());

	  mRandomTexture = new Texture(size);

      glBindTexture(GL_TEXTURE_1D, mRandomTexture->handle());
      VL_CHECK_OGL();
      glTexSubImage1D(GL_TEXTURE_1D, 0, 0, mRandomTexture->width(), GL_RGB, GL_FLOAT, &randomData[0]);
      VL_CHECK_OGL();
      glBindTexture(GL_TEXTURE_1D, 0);
      VL_CHECK_OGL();

	  TexParameter *texParameter = mRandomTexture->getTexParameter();
	  texParameter->setMinFilter(TPF_LINEAR);
	  texParameter->setMagFilter(TPF_LINEAR);
	  texParameter->setWrapS(TPW_REPEAT);

      VL_CHECK_OGL();
  }

  void initShaders()
  {
      ref<GLSLProgram> glsl = new GLSLProgram;

      ref<GLSLVertexShader> transformFeedback_vs = new GLSLVertexShader("/glsl/ps_update.vs");
      ref<GLSLGeometryShader> transformFeedback_gs = new GLSLGeometryShader("/glsl/ps_update.gs");
      glsl->attachShader(transformFeedback_vs.get());
      glsl->attachShader(transformFeedback_gs.get());
      glsl->bindAttribLocation(0, "Type");
      glsl->bindAttribLocation(1, "Position");
      glsl->bindAttribLocation(2, "Velocity");
      glsl->bindAttribLocation(3, "Age");
      mUniformDeltaTimeMillis = glsl->gocUniform("gDeltaTimeMillis");
      mUniformTime = glsl->gocUniform("gTime");
      glsl->gocUniform("gRandomTexture")->setUniform(0);
      glsl->gocUniform("glauncherLifeTime")->setUniform(3.0f);
      glsl->gocUniform("gShellLifeTime")->setUniform(3.0f);
      glsl->gocUniform("gSecondaryShellLifetime")->setUniform(3.0f);

      ref< GLSLProgram > renderingGlsl = new GLSLProgram;
      ref<GLSLVertexShader> rendering_vs = new GLSLVertexShader("/glsl/simple.vs");
      ref<GLSLGeometryShader> rendering_fs = new GLSLGeometryShader("/glsl/simple.fs");
      renderingGlsl->attachShader(rendering_vs.get());
      renderingGlsl->attachShader(rendering_fs.get());
      renderingGlsl->gocUniform("color")->setUniform(vec3(1.0f, 0.0f, 0.0f));

      ref< Effect > renderingEffect = new Effect;
      renderingEffect->shader()->enable(EN_DEPTH_TEST);
      renderingEffect->shader()->setRenderState(renderingGlsl.get());


      fx1 = new Effect;
      fx1->shader()->enable(EN_DEPTH_TEST);
      fx1->shader()->gocPointSize()->set(4.0f);
      //fx1->shader()->enable(EN_RASTERIZER_DISCARD);
      fx1->shader()->gocTextureSampler(0)->setTexture(mRandomTexture.get());
      fx1->shader()->setRenderState(glsl.get());

      transformFeedback1 = fx1->shader()->gocTransformFeedback();
      fx1->shader()->setRenderState(transformFeedback1.get());
      glsl->setTransformFeedback(transformFeedback1.get());

      fx2 = new Effect;
      fx2->shader()->enable(EN_DEPTH_TEST);
      //fx2->shader()->enable(EN_RASTERIZER_DISCARD);
      fx2->shader()->setRenderState(glsl.get());
      fx2->shader()->gocTextureSampler(0)->setTexture(mRandomTexture.get());
      fx2->shader()->setRenderState(glsl.get());

      transformFeedback2 = fx2->shader()->gocTransformFeedback();
      fx2->shader()->setRenderState(transformFeedback2.get());
      glsl->setTransformFeedback(transformFeedback2.get());

      transformFeedback1->set(PT_POINTS);
      transformFeedback2->set(PT_POINTS);

      std::list< String > varyings;
      varyings.push_back("Type1");
      varyings.push_back("Position1");
      varyings.push_back("Velocity1");
      varyings.push_back("Age1");

      transformFeedback1->setTransformFeedbackVaryings(varyings);
      transformFeedback2->setTransformFeedbackVaryings(varyings);
  }


  void updateScene()
  {
    mUniformDeltaTimeMillis->setUniform(mTime.elapsed());
    int time = vl::Time::currentTime();

    mUniformTime->setUniform(time);	// Maybe not the best idea

    drawTransformFeedbacks[currentTransformFeedback]->setEnabled(false);

	currentBufferObject = currentTransformFeedback;
	currentTransformFeedback = (currentTransformFeedback + 1) & 0x1;

    drawTransformFeedbacks[currentTransformFeedback]->setEnabled(true);
  }
  
  void keyReleaseEvent(unsigned short, EKey key)
  {
	if(key == Key_T)
	{
		
		/*std::list< String > varyings;
		varyings.push_back("normale");
		tf->setTransformFeedbackVaryings(varyings);
		
		normales = new ArrayInt1;
		normales->resize(6);
		
		
		
		tf->addArray(normales.get());*/
				
		
		ref<Geometry> geom = new Geometry;
		geom->setObjectName("Pyramid");
		
		//particle1
		
		/*ref<ArrayFloat3> vert3 = new ArrayFloat3;
		geom->setVertexAttribArray(VA_Position, vert3.get());
		
		real x = 5.0f   / 2.0f;
		real y = 10.0f;
		real z = 5.0f   / 2.0f;
		
		fvec3 a0( (fvec3)(vec3(+0,+y,+0) + vec3(0.0f, 0.0f, 0.0f)) );
		fvec3 a1( (fvec3)(vec3(-x,+0,-z) + vec3(0.0f, 0.0f, 0.0f)) );
		fvec3 a2( (fvec3)(vec3(-x,-0,+z) + vec3(0.0f, 0.0f, 0.0f)) );
		fvec3 a3( (fvec3)(vec3(+x,-0,+z) + vec3(0.0f, 0.0f, 0.0f)) );
		fvec3 a4( (fvec3)(vec3(+x,+0,-z) + vec3(0.0f, 0.0f, 0.0f)) );
		
		ref<DrawArrays> polys = new DrawArrays(PT_TRIANGLES, 0, 6*3);
		geom->drawCalls().push_back( polys.get() );
		
		vert3->resize(6*3);
		
		vert3->at(0)  = a4; vert3->at(1)  = a2; vert3->at(2)  = a1; 
		vert3->at(3)  = a2; vert3->at(4)  = a4; vert3->at(5)  = a3; 
		vert3->at(6)  = a4; vert3->at(7)  = a1; vert3->at(8)  = a0; 
		vert3->at(9)  = a1; vert3->at(10) = a2; vert3->at(11) = a0;
		vert3->at(12) = a2; vert3->at(13) = a3; vert3->at(14) = a0;
		vert3->at(15) = a3; vert3->at(16) = a4; vert3->at(17) = a0;
		
		ref< Actor > actor = sceneManager()->tree()->addActor( geom.get(), fx.get(), NULL);
				
		rendering()->render();
		
		sceneManager()->tree()->eraseActor(actor.get());
		
		tf->updateArray(normales.get());
		
		for(unsigned int i = 0; i < normales->size(); ++i)
		{
			defLogger()->error("primitive " + String::fromUInt(i) + " : " + String::fromDouble(normales->at(i)) + "\n");
		}*/
	}
  }

protected:
	ref< ArrayInt1 > normales;
	ref< TransformFeedback > transformFeedback1;
	ref< TransformFeedback > transformFeedback2;
	Collection< DrawCall > drawTransformFeedbacks;
	ref< ArrayFloat1 > particlesBufferType[2];
	ref< ArrayFloat3 > particlesBufferPosition[2];
	ref< ArrayFloat3 > particlesBufferVelocity[2];
	ref< ArrayFloat1 > particlesBufferLifetime[2];
		
	int currentBufferObject = 0;
	int currentTransformFeedback = 0; 
                       
	std::array< float, MAX_PARTICLES > particlesType;
	std::array< vec3, MAX_PARTICLES > particlesPosition;
	std::array< vec3, MAX_PARTICLES > particlesVelocity;
	std::array< float, MAX_PARTICLES > particlesLifetime;

	ref< Texture > mRandomTexture;
	ref< Uniform > mUniformTime;
	ref< Uniform > mUniformDeltaTimeMillis;

    ref< Effect > fx1;
    ref< Effect > fx2;

	Time mTime;

	bool mIsFirst = true;
	

};

// Have fun!

BaseDemo* Create_App_GLSL_TransformFeedback() { return new App_GLSL_TransformFeedback; }
