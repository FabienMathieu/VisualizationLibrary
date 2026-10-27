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
#include <vlGraphics/Geometry.hpp>
#include <vlGraphics/DrawArrays.hpp>
#include <vlGraphics/DrawElements.hpp>
#include <vlGraphics/LineIterator.hpp>
#include <vlCore/Time.hpp>

using namespace vl;

/*
 * Shows how to use DrawCall::lineIterator() to iterate over the line segments of a DrawCall,
 * in the same way DrawCall::triangleIterator() iterates over triangles.
 *
 * The top row uses DrawArrays, the bottom row DrawElementsUInt, with from left to right
 * PT_LINES, PT_LINE_STRIP and PT_LINE_LOOP. Every segment found by the LineIterator is logged
 * and highlighted in red, one after the other.
 */
class App_LineIterator: public BaseDemo
{
public:
  virtual void initEvent()
  {
    Log::notify(appletInfo());

    EPrimitiveType types[] = { PT_LINES, PT_LINE_STRIP, PT_LINE_LOOP };
    const char* type_names[] = { "PT_LINES", "PT_LINE_STRIP", "PT_LINE_LOOP" };

    for(int row = 0; row < 2; ++row)
    {
      for(int col = 0; col < 3; ++col)
      {
        bool indexed = row == 1;
        ref<Geometry> geom = makeShape( vec3( (col - 1) * 4.0f, indexed ? -2.0f : 2.0f, 0 ), types[col], indexed );
        sceneManager()->tree()->addActor( geom.get(), makeEffect( 1 ).get(), NULL );

        // iterate over the segments of the draw call
        const ArrayAbstract* pos = geom->vertexArray();
        DrawCall* dc = geom->drawCalls().at(0);
        String segments;
        int count = 0;
        for( LineIterator it = dc->lineIterator(); it.hasNext(); it.next(), ++count )
        {
          segments += Say(" (%n,%n)") << it.a() << it.b();
          mSegments.push_back( pos->getAsVec3( it.a() ) );
          mSegments.push_back( pos->getAsVec3( it.b() ) );
        }
        Log::print( Say("%s %s: %n segments:%s\n") << ( indexed ? "DrawElementsUInt" : "DrawArrays" ) << type_names[col] << count << segments );
      }
    }

    // the highlighted segment
    mHighlight = new Geometry;
    mHighlight->setBufferObjectEnabled( false );
    mHighlightPos = new ArrayFloat3;
    mHighlightPos->resize( 2 );
    mHighlight->setVertexArray( mHighlightPos.get() );
    mHighlight->setColorArray( red );
    mHighlight->drawCalls().push_back( new DrawArrays( PT_LINES, 0, 2 ) );
    sceneManager()->tree()->addActor( mHighlight.get(), makeEffect( 5 ).get(), NULL );
  }

  // 6 points on a circle, rendered with the given primitive type
  ref<Geometry> makeShape( const vec3& center, EPrimitiveType type, bool indexed )
  {
    const int count = 6;
    ref<ArrayFloat3> pos = new ArrayFloat3;
    pos->resize( count );
    for(int i = 0; i < count; ++i)
    {
      float t = (float)i / count * fPi * 2.0f;
      pos->at(i) = (fvec3)center + fvec3( ::cos(t) * 1.5f, ::sin(t) * 1.5f, 0 );
    }

    ref<Geometry> geom = new Geometry;
    geom->setVertexArray( pos.get() );
    geom->setColorArray( white );
    if ( indexed )
    {
      ref<DrawElementsUInt> de = new DrawElementsUInt( type );
      de->indexBuffer()->resize( count );
      for(int i = 0; i < count; ++i)
        de->indexBuffer()->at(i) = i;
      geom->drawCalls().push_back( de.get() );
    }
    else
      geom->drawCalls().push_back( new DrawArrays( type, 0, count ) );
    return geom;
  }

  ref<Effect> makeEffect( float line_width )
  {
    ref<Effect> fx = new Effect;
    fx->shader()->gocLineWidth()->set( line_width );
    return fx;
  }

  virtual void updateScene()
  {
    if ( mSegments.empty() )
      return;

    // highlight a different segment twice per second
    int segment = (int)( Time::currentTime() * 2.0 ) % ( mSegments.size() / 2 );
    mHighlightPos->at(0) = (fvec3)mSegments[segment * 2 + 0];
    mHighlightPos->at(1) = (fvec3)mSegments[segment * 2 + 1];
  }

protected:
  std::vector<vec3> mSegments;
  ref<Geometry> mHighlight;
  ref<ArrayFloat3> mHighlightPos;
};

// Have fun!

BaseDemo* Create_App_LineIterator() { return new App_LineIterator; }
