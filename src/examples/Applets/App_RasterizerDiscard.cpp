/**************************************************************************************/
/*                                                                                    */
/*  Visualization Library                                                             */
/*  http://visualizationlibrary.org                                                   */
/*                                                                                    */
/*  Copyright (c) 2005-2020, Michele Bosi                                             */
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
#include <vlGraphics/GeometryPrimitives.hpp>
#include <vlGraphics/Light.hpp>
#include <vlCore/Time.hpp>

using namespace vl;

/*
 * Shows how to use EN_RASTERIZER_DISCARD (GL_RASTERIZER_DISCARD, OpenGL 3.0).
 *
 * When enabled, primitives are processed by the vertex/geometry stages (and by transform
 * feedback if active) but are discarded right before rasterization, so nothing is drawn.
 *
 * Press SPACE to toggle EN_RASTERIZER_DISCARD on the right object's Shader.
 */
class App_RasterizerDiscard: public BaseDemo
{
public:
  virtual String appletInfo()
  {
    return BaseDemo::appletInfo() +
    "- SPACE: toggle EN_RASTERIZER_DISCARD on the right object\n" +
    "\n";
  }

  virtual void initEvent()
  {
    Log::notify(appletInfo());

    if ( !Is_Enable_Supported[EN_RASTERIZER_DISCARD] )
    {
      Log::error("EN_RASTERIZER_DISCARD requires OpenGL 3.0.\n");
      Time::sleep(2000);
      exit(1);
    }

    // left object: always rendered
    mLeftTransform = new Transform;
    rendering()->as<Rendering>()->transform()->addChild( mLeftTransform.get() );
    ref<Geometry> teapot = makeTeapot( vec3(0,0,0), 4 );
    teapot->computeNormals();
    sceneManager()->tree()->addActor( teapot.get(), makeEffect( royalblue ).get(), mLeftTransform.get() );

    // right object: its Shader toggles EN_RASTERIZER_DISCARD
    mRightTransform = new Transform;
    rendering()->as<Rendering>()->transform()->addChild( mRightTransform.get() );
    ref<Geometry> box = makeBox( vec3(0,0,0), 3, 3, 3 );
    box->computeNormals();
    mDiscardEffect = makeEffect( crimson );
    sceneManager()->tree()->addActor( box.get(), mDiscardEffect.get(), mRightTransform.get() );
  }

  ref<Effect> makeEffect( const fvec4& color )
  {
    ref<Effect> fx = new Effect;
    fx->shader()->enable( EN_DEPTH_TEST );
    fx->shader()->enable( EN_LIGHTING );
    fx->shader()->setRenderState( new Light, 0 );
    fx->shader()->gocMaterial()->setDiffuse( color );
    return fx;
  }

  virtual void updateScene()
  {
    real degrees = Time::currentTime() * 45.0f;
    mLeftTransform->setLocalMatrix( mat4::getTranslation( -4, 0, 0 ) * mat4::getRotation( degrees, 0, 1, 0 ) );
    mRightTransform->setLocalMatrix( mat4::getTranslation( 4, 0, 0 ) * mat4::getRotation( degrees, 1, 1, 0 ) );
  }

  virtual void keyPressEvent(unsigned short ch, EKey key)
  {
    BaseDemo::keyPressEvent(ch, key);

    if ( key == Key_Space )
    {
      Shader* shader = mDiscardEffect->shader();
      if ( shader->gocEnableSet()->isEnabled( EN_RASTERIZER_DISCARD ) )
        shader->disable( EN_RASTERIZER_DISCARD );
      else
        shader->enable( EN_RASTERIZER_DISCARD );

      Log::print( Say("EN_RASTERIZER_DISCARD %s\n") << ( shader->gocEnableSet()->isEnabled( EN_RASTERIZER_DISCARD ) ? "enabled" : "disabled" ) );
    }
  }

protected:
  ref<Transform> mLeftTransform;
  ref<Transform> mRightTransform;
  ref<Effect> mDiscardEffect;
};

// Have fun!

BaseDemo* Create_App_RasterizerDiscard() { return new App_RasterizerDiscard; }
