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
#include <vlGraphics/CoreText.hpp>
#include <vlGraphics/FontManager.hpp>
#include <vlGraphics/GLSL.hpp>
#include <vlGraphics/Geometry.hpp>
#include <vlGraphics/DrawArrays.hpp>

using namespace vl;

/*
 * Shows how to render text with vl::CoreText and vl::CoreFont.
 *
 * Unlike vl::Text, CoreText does not use the fixed function pipeline: the glyph quads are
 * stored in a buffer object when the text is set and are rendered by a user GLSLProgram
 * (see data/glsl/coretext.vs/fs) sampling the CoreFont's font atlas, a single texture
 * containing all the glyphs of the font.
 *
 * The bottom of the window shows the font atlas of the 24px VeraMono font.
 */
class App_CoreText: public BaseDemo
{
public:
  virtual void initEvent()
  {
    Log::notify(appletInfo());

    // 2D application: pixel coordinates, no camera manipulation
    trackball()->setEnabled(false);

    mProgram = new GLSLProgram;
    mProgram->attachShader( new GLSLVertexShader("/glsl/coretext.vs") );
    mProgram->attachShader( new GLSLFragmentShader("/glsl/coretext.fs") );
    mProgram->gocUniform("font_atlas")->setUniformI(0);

    CoreFont* mono_24 = defFontManager()->acquireCoreFont("/font/bitstream-vera/VeraMono.ttf", 24, true);
    CoreFont* vera_40 = defFontManager()->acquireCoreFont("/font/bitstream-vera/Vera.ttf", 40, true);
    CoreFont* bold_16 = defFontManager()->acquireCoreFont("/font/bitstream-vera/VeraBd.ttf", 16, true);

    // texts anchored to the top-left corner of the window
    mTopLeft = new Transform;
    addText( mono_24, "CoreText: text rendered with shaders", AlignTop | AlignLeft, fvec4(1, 1, 1, 1), mTopLeft.get() );

    mTopLeft2 = new Transform;
    addText( bold_16, "Each CoreFont packs its glyphs in a single font atlas texture", AlignTop | AlignLeft, fvec4(1, 0.8f, 0.2f, 1), mTopLeft2.get() );

    // text centered in the window
    mCenter = new Transform;
    addText( vera_40, "Centered text origin", AlignHCenter | AlignVCenter, fvec4(0.3f, 0.7f, 1, 1), mCenter.get() );

    // a quad showing the font atlas used by the 24px VeraMono font
    mAtlasTransform = new Transform;
    rendering()->as<Rendering>()->transform()->addChild( mAtlasTransform.get() );
    mAtlasTexture = mono_24->getAtlas();
    sceneManager()->tree()->addActor( makeQuad().get(), makeEffect( mono_24 ).get(), mAtlasTransform.get() )->gocUniform("text_color")->setUniform( fvec4(1, 1, 1, 1) );

    resizeEvent( openglContext()->width(), openglContext()->height() );
  }

  void addText( CoreFont* font, const String& str, int origin, const fvec4& color, Transform* tr )
  {
    rendering()->as<Rendering>()->transform()->addChild( tr );

    ref<CoreText> text = new CoreText;
    // font and origin must be set before the text: setText() precomputes the glyph quads
    text->setFont( font );
    text->setTextOrigin( origin );
    text->setText( str );

    Actor* act = sceneManager()->tree()->addActor( text.get(), makeEffect( font ).get(), tr );
    act->gocUniform("text_color")->setUniform( color );
  }

  ref<Effect> makeEffect( CoreFont* font )
  {
    ref<Effect> fx = new Effect;
    fx->shader()->setRenderState( mProgram.get() );
    fx->shader()->gocTextureImageUnit(0)->setTexture( font->getAtlas() );
    fx->shader()->enable( EN_BLEND );
    return fx;
  }

  // a unit quad, scaled to the atlas size in resizeEvent()
  ref<Geometry> makeQuad()
  {
    ref<Geometry> quad = new Geometry;
    ref<ArrayFloat3> pos = new ArrayFloat3;
    pos->resize(4);
    pos->at(0) = fvec3(0, 0, 0);
    pos->at(1) = fvec3(1, 0, 0);
    pos->at(2) = fvec3(1, 1, 0);
    pos->at(3) = fvec3(0, 1, 0);
    ref<ArrayFloat2> tex = new ArrayFloat2;
    tex->resize(4);
    tex->at(0) = fvec2(0, 0);
    tex->at(1) = fvec2(1, 0);
    tex->at(2) = fvec2(1, 1);
    tex->at(3) = fvec2(0, 1);
    quad->setVertexArray( pos.get() );
    quad->setTexCoordArray( 0, tex.get() );
    quad->drawCalls().push_back( new DrawArrays( PT_TRIANGLE_FAN, 0, 4 ) );
    return quad;
  }

  virtual void resizeEvent(int w, int h)
  {
    Camera* camera = rendering()->as<Rendering>()->camera();
    camera->viewport()->setWidth( w );
    camera->viewport()->setHeight( h );
    camera->setProjectionOrtho( -0.5f );
    camera->setViewMatrix( mat4() );

    const real margin = 10;
    mTopLeft->setLocalMatrix( mat4::getTranslation( margin, h - margin, 0 ) );
    mTopLeft2->setLocalMatrix( mat4::getTranslation( margin, h - margin - 50, 0 ) );
    mCenter->setLocalMatrix( mat4::getTranslation( w / 2.0f, h / 2.0f, 0 ) );

    // fit the atlas to the window width keeping its aspect ratio
    if ( mAtlasTexture )
    {
      real atlas_w = mAtlasTexture->width();
      real atlas_h = mAtlasTexture->height();
      real scale = atlas_w > w - 2 * margin ? ( w - 2 * margin ) / atlas_w : 1;
      mAtlasTransform->setLocalMatrix( mat4::getTranslation( margin, margin, 0 ) * mat4::getScaling( atlas_w * scale, atlas_h * scale, 1 ) );
    }
  }

protected:
  ref<GLSLProgram> mProgram;
  ref<Transform> mTopLeft;
  ref<Transform> mTopLeft2;
  ref<Transform> mCenter;
  ref<Transform> mAtlasTransform;
  ref<Texture> mAtlasTexture;
};

// Have fun!

BaseDemo* Create_App_CoreText() { return new App_CoreText; }
