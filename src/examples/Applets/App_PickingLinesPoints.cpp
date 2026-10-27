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
#include <vlGraphics/DrawArrays.hpp>
#include <vlGraphics/DrawElements.hpp>
#include <vlGraphics/RayIntersector.hpp>
#include <vlGraphics/Light.hpp>

using namespace vl;

/*
 * Shows how RayIntersector picks line segments (PT_LINES, PT_LINE_STRIP, PT_LINE_LOOP) and
 * points (PT_POINTS) in addition to triangles.
 *
 * A line or a point is intersected when its distance from the picking ray is lower than
 * RayIntersector::distance(). The picked segment or point is highlighted in red. For triangles
 * the edge and the vertex nearest to the intersection point are highlighted.
 */
class App_PickingLinesPoints: public BaseDemo
{
public:
  virtual String appletInfo()
  {
    return BaseDemo::appletInfo() +
    "- Left click: pick a line segment, a point or a triangle\n" +
    "\n";
  }

  virtual void initEvent()
  {
    Log::notify(appletInfo());

    // a spiral drawn as a line strip
    ref<ArrayFloat3> spiral = new ArrayFloat3;
    spiral->resize( 60 );
    for(int i = 0; i < 60; ++i)
    {
      float t = i / 60.0f * fPi * 6.0f;
      spiral->at(i) = fvec3( -5.0f + ::cos(t) * t * 0.1f, 3.0f + ::sin(t) * t * 0.1f, 0 );
    }
    addLines( "line strip", spiral.get(), new DrawArrays( PT_LINE_STRIP, 0, 60 ) );

    // a circle drawn as an indexed line loop
    ref<ArrayFloat3> circle = new ArrayFloat3;
    ref<DrawElementsUInt> loop = new DrawElementsUInt( PT_LINE_LOOP );
    circle->resize( 24 );
    loop->indexBuffer()->resize( 24 );
    for(int i = 0; i < 24; ++i)
    {
      float t = i / 24.0f * fPi * 2.0f;
      circle->at(i) = fvec3( 5.0f + ::cos(t) * 1.8f, 3.0f + ::sin(t) * 1.8f, 0 );
      loop->indexBuffer()->at(i) = i;
    }
    addLines( "line loop", circle.get(), loop.get() );

    // a grid made of separate segments
    ref<ArrayFloat3> grid = new ArrayFloat3;
    for(int i = 0; i <= 4; ++i)
    {
      grid->resize( grid->size() + 4 );
      grid->at( grid->size() - 4 ) = fvec3( -7.0f + i, -5.0f, 0 );
      grid->at( grid->size() - 3 ) = fvec3( -7.0f + i, -1.0f, 0 );
      grid->at( grid->size() - 2 ) = fvec3( -7.0f, -5.0f + i, 0 );
      grid->at( grid->size() - 1 ) = fvec3( -3.0f, -5.0f + i, 0 );
    }
    addLines( "lines", grid.get(), new DrawArrays( PT_LINES, 0, (int)grid->size() ) );

    // a point cloud
    ref<ArrayFloat3> cloud = new ArrayFloat3;
    cloud->resize( 7 * 7 );
    for(int i = 0; i < 7; ++i)
      for(int j = 0; j < 7; ++j)
        cloud->at( i * 7 + j ) = fvec3( 3.0f + i * 0.6f, -5.0f + j * 0.6f, 0 );
    ref<Effect> points_fx = makeUnlitEffect( white );
    points_fx->shader()->gocPointSize()->set( 6 );
    addGeometry( "points", cloud.get(), new DrawArrays( PT_POINTS, 0, (int)cloud->size() ), points_fx.get() );

    // a box made of triangles
    ref<Geometry> box = makeBox( vec3( 0, 0, 0 ), 2, 2, 2, false );
    box->computeNormals();
    ref<Effect> box_fx = new Effect;
    box_fx->shader()->enable( EN_DEPTH_TEST );
    box_fx->shader()->enable( EN_LIGHTING );
    box_fx->shader()->setRenderState( new Light, 0 );
    box_fx->shader()->gocMaterial()->setDiffuse( royalblue );
    sceneManager()->tree()->addActor( box.get(), box_fx.get(), NULL )->setObjectName( "box" );

    // highlights
    mHighlightLine = makeHighlight( PT_LINES, 2, 4 );
    mHighlightPoint = makeHighlight( PT_POINTS, 1, 12 );
  }

  void addLines( const char* name, ArrayFloat3* pos, DrawCall* dc )
  {
    ref<Effect> fx = makeUnlitEffect( white );
    fx->shader()->gocLineWidth()->set( 2 );
    addGeometry( name, pos, dc, fx.get() );
  }

  void addGeometry( const char* name, ArrayFloat3* pos, DrawCall* dc, Effect* fx )
  {
    ref<Geometry> geom = new Geometry;
    geom->setVertexArray( pos );
    geom->drawCalls().push_back( dc );
    sceneManager()->tree()->addActor( geom.get(), fx, NULL )->setObjectName( name );
  }

  ref<Effect> makeUnlitEffect( const fvec4& color )
  {
    ref<Effect> fx = new Effect;
    fx->shader()->enable( EN_DEPTH_TEST );
    fx->shader()->gocColor()->setValue( color );
    return fx;
  }

  ref<Actor> makeHighlight( EPrimitiveType type, int count, float size )
  {
    ref<Geometry> geom = new Geometry;
    geom->setBufferObjectEnabled( false );
    ref<ArrayFloat3> pos = new ArrayFloat3;
    pos->resize( count );
    geom->setVertexArray( pos.get() );
    geom->drawCalls().push_back( new DrawArrays( type, 0, count ) );
    ref<Effect> fx = makeUnlitEffect( red );
    fx->shader()->gocLineWidth()->set( size );
    fx->shader()->gocPointSize()->set( size );
    // always visible
    fx->shader()->disable( EN_DEPTH_TEST );
    fx->setRenderRank( 1 );
    ref<Actor> act = sceneManager()->tree()->addActor( geom.get(), fx.get(), NULL );
    act->setEnabled( false );
    return act;
  }

  void setHighlight( Actor* act, const ArrayAbstract* pos, int ia, int ib = -1 )
  {
    ArrayFloat3* hpos = act->lod(0)->as<Geometry>()->vertexArray()->as<ArrayFloat3>();
    hpos->at(0) = (fvec3)pos->getAsVec3( ia );
    if ( ib != -1 )
      hpos->at(1) = (fvec3)pos->getAsVec3( ib );
    act->lod(0)->setBoundsDirty( true );
    act->setEnabled( true );
  }

  virtual void mouseDownEvent(EMouseButton, int x, int y)
  {
    Camera* camera = rendering()->as<Rendering>()->camera();
    // convert Y coordinates to the OpenGL conventions
    y = openglContext()->height() - y;
    pick( camera->computeRay( x, y ) );
  }

  void pick( const Ray& ray )
  {
    mHighlightLine->setEnabled( false );
    mHighlightPoint->setEnabled( false );

    RayIntersector intersector;
    // lines and points closer than 0.1 to the ray are intersected
    intersector.setDistance( 0.1f );
    intersector.intersect( ray, sceneManager() );

    // the highlights are not pickable
    const RayIntersectionGeometry* hit = NULL;
    for(size_t i = 0; i < intersector.intersections().size() && !hit; ++i)
    {
      const RayIntersection* inter = intersector.intersections()[i].get();
      if ( inter->actor() != mHighlightLine.get() && inter->actor() != mHighlightPoint.get() )
        hit = inter->as<RayIntersectionGeometry>();
    }

    if ( !hit )
    {
      Log::print( "Nothing picked.\n" );
      return;
    }

    const ArrayAbstract* pos = hit->geometry()->vertexArray();
    String name = hit->actor()->objectName().c_str();
    if ( hit->triangleIndex() != -1 )
    {
      Log::print( Say("%s: triangle #%n (%n,%n,%n), nearest edge (%n,%n), nearest vertex %n\n") << name
        << hit->triangleIndex() << hit->triangle()[0] << hit->triangle()[1] << hit->triangle()[2]
        << hit->line()[0] << hit->line()[1] << hit->nearestPoint() );
      setHighlight( mHighlightLine.get(), pos, hit->line()[0], hit->line()[1] );
      setHighlight( mHighlightPoint.get(), pos, hit->nearestPoint() );
    }
    else
    if ( hit->lineIndex() != -1 )
    {
      Log::print( Say("%s: segment #%n (%n,%n)\n") << name << hit->lineIndex() << hit->line()[0] << hit->line()[1] );
      setHighlight( mHighlightLine.get(), pos, hit->line()[0], hit->line()[1] );
    }
    else
    {
      Log::print( Say("%s: point %n\n") << name << hit->nearestPoint() );
      setHighlight( mHighlightPoint.get(), pos, hit->nearestPoint() );
    }
  }

protected:
  ref<Actor> mHighlightLine;
  ref<Actor> mHighlightPoint;
};

// Have fun!

BaseDemo* Create_App_PickingLinesPoints() { return new App_PickingLinesPoints; }
