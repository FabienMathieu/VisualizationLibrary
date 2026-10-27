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

#include <vlGraphics/RayIntersector.hpp>
#include <vlGraphics/SceneManager.hpp>
#include <vlGraphics/LineIterator.hpp>

using namespace vl;

//-----------------------------------------------------------------------------
void RayIntersector::intersect(const Ray& ray, SceneManager* scene_manager)
{
  actors()->clear();
  scene_manager->extractVisibleActors( *actors(), NULL );
  setRay(ray);
  intersect();
}
//-----------------------------------------------------------------------------
void RayIntersector::intersect()
{
  mIntersections.clear();
  for(size_t i=0; i<actors()->size(); ++i)
  {
    if (!frustum().cull(actors()->at(i)->boundingBox()))
    {
      intersect(actors()->at(i));
    }
  }

  std::sort( mIntersections.begin(), mIntersections.end(), sorter );
}
//-----------------------------------------------------------------------------
void RayIntersector::intersect(Actor* act)
{
  Geometry* geom = cast<Geometry>(act->lod(0));
  if (geom)
    intersectGeometry(act, geom);
}
//-----------------------------------------------------------------------------
void RayIntersector::intersectGeometry(Actor* act, Geometry* geom)
{
  ArrayAbstract* posarr = geom->vertexArray();
  if (posarr)
  {
    mat4 matrix = act->transform() ? act->transform()->worldMatrix() : mat4();
    for(size_t i=0; i<geom->drawCalls().size(); ++i)
    {
      DrawCall* prim = geom->drawCalls().at(i);
      int itri = 0;
      if(prim->primitiveType() == vl::PT_LINES ||
              prim->primitiveType() == vl::PT_LINE_STRIP ||
              prim->primitiveType() == vl::PT_LINE_LOOP)
      {
        for(LineIterator liit = prim->lineIterator(); liit.hasNext(); liit.next(), ++itri)
        {
            int ia = liit.a();
            int ib = liit.b();
            vec3 a = posarr->getAsVec3(ia);
            vec3 b = posarr->getAsVec3(ib);
            if(act->transform())
            {
                a = matrix * a;
                b = matrix * b;
            }
            // Compute distance to the line. Compute all and keep the minimum( attribute for an epsilon?)
            intersectLine(a, b, ia, ib, act, geom, prim, itri);
        }
      }
      else if(prim->primitiveType() == vl::PT_POINTS)
      {
        for(IndexIterator it = prim->indexIterator(); it.hasNext(); it.next(), ++itri)
        {
          int ia = it.index();
          vec3 a = posarr->getAsVec3(ia);
          if(act->transform())
          {
            a = matrix * a;
          }
          intersectPoint(a, ia, act, geom, prim, itri);
        }
      }
      else
      {
        for(TriangleIterator trit = prim->triangleIterator(); trit.hasNext(); trit.next(), ++itri)
        {
            int ia = trit.a();
            int ib = trit.b();
            int ic = trit.c();
            vec3 a = posarr->getAsVec3(ia);
            vec3 b = posarr->getAsVec3(ib);
            vec3 c = posarr->getAsVec3(ic);
            if (act->transform())
            {
                a = matrix * a;
                b = matrix * b;
                c = matrix * c;
            }
            intersectTriangle(a, b, c, ia, ib, ic, act, geom, prim, itri);
        }
      }
    }
  }
}
//-----------------------------------------------------------------------------
template<class T>
void RayIntersector::intersectTriangle(const T& a, const T& b, const T& c, int ia, int ib, int ic, Actor* act, Geometry* geom, DrawCall* prim, int tri_idx)
{
  T v1 = b-a;
  T v2 = c-a;
  T n = cross(v1,v2).normalize();
  real det = (real)dot(n,(T)ray().direction());
  if(det == 0)
    return;
  real t = (real)dot(n, a-(T)ray().origin()) / det;
  if (t<0)
    return;
  vec3  rp = ray().origin() + ray().direction() * t;
  T fp = (T)rp;
  T pts[] = { a, b, c, a };
  for(int i=0; i<3; ++i)
  {
    T bi_norm = -cross(pts[i+1]-pts[i],n).normalize();
    if (dot(fp-pts[i],bi_norm) < 0)
      return;
  }

  // compute the nearest point
  T aRp = rp - a;
  T bRp = rp - b;
  T cRp = rp - c;

  T v3 = b - c;
  float d1 = cross(aRp, v1).length() / v1.length();
  float d2 = cross(aRp, v2).length() / v2.length();
  float d3 = cross(cRp, v3).length() / v3.length();

  int ila, ilb;
  int l_idx;
  if(d1 < d2)
  {
      if(d1 < d3)
      {
          ila = ia;
          ilb = ib;
          l_idx = 1;
      }
      else
      {
          ila = ic;
          ilb = ib;
          l_idx = 2;
      }
  }
  else
  {
      if(d2 < d3)
      {
          ila = ia;
          ilb = ic;
          l_idx = 3;
      }
      else
      {
          ila = ic;
          ilb = ib;
          l_idx = 2;
      }
  }

  ref<RayIntersectionGeometry> record = new vl::RayIntersectionGeometry;
  record->setIntersectionPoint( rp );
  record->setTriangleIndex(tri_idx);
  record->setTriangle(ia, ib, ic);
  record->setLineIndex(l_idx);
  record->setLine(ila, ilb);
  record->setNearestPoint(aRp.lengthSquared() < bRp.lengthSquared() ? (aRp.lengthSquared() < cRp.lengthSquared() ? ia : ic) : (bRp.lengthSquared() < cRp.lengthSquared() ? ib : ic));
  record->setActor(act);
  record->setGeometry(geom);
  record->setPrimitives(prim);
  record->setDistance( t );
  mIntersections.push_back(record);
}
//-----------------------------------------------------------------------------
template<class T>
void RayIntersector::intersectLine(const T& a, const T& b, int ia, int ib, Actor* act, Geometry* geom, DrawCall* prim, int line_idx)
{
  // closest points between the segment a + s*u (s in [0,1]) and the ray o + t*v (t >= 0),
  // see http://geomalgorithms.com/a07-_distance.html
  T u = b - a;
  T v = (T)ray().direction();
  T w = a - (T)ray().origin();
  real uu = (real)dot(u,u);
  real uv = (real)dot(u,v);
  real vv = (real)dot(v,v);
  real uw = (real)dot(u,w);
  real vw = (real)dot(v,w);
  real den = uu*vv - uv*uv; // always >= 0
  if (uu == 0 || vv == 0)
    return;

  // segment parameter, clamped to the segment
  real s = den < 1e-12 ? 0 : (uv*vw - vv*uw) / den; // 0 if parallel
  s = clamp(s, (real)0, (real)1);
  // ray parameter of the point of the ray closest to the segment point
  real t = (real)dot(v, w + u * s) / vv;
  if (t < 0)
    return;

  T on_segment = a + u * s;
  T on_ray = (T)ray().origin() + v * t;
  if ((real)(on_segment - on_ray).length() > distance())
    return;

  ref<RayIntersectionGeometry> record = new vl::RayIntersectionGeometry;
  record->setIntersectionPoint( (vec3)on_segment );
  record->setLineIndex(line_idx);
  record->setLine(ia, ib);
  record->setActor(act);
  record->setGeometry(geom);
  record->setPrimitives(prim);
  record->setDistance( t );
  mIntersections.push_back(record);
}
//-----------------------------------------------------------------------------
template<class T>
void RayIntersector::intersectPoint(const T& a, int ia, Actor* act, Geometry* geom, DrawCall* prim, int /*point_idx*/)
{
  // the point of the ray closest to a
  T v = (T)ray().direction();
  real vv = (real)dot(v,v);
  if (vv == 0)
    return;
  real t = (real)dot(v, a - (T)ray().origin()) / vv;
  if (t < 0)
    return;

  T on_ray = (T)ray().origin() + v * t;
  if ((real)(a - on_ray).length() > distance())
    return;

  ref<RayIntersectionGeometry> record = new vl::RayIntersectionGeometry;
  record->setIntersectionPoint( (vec3)a );
  record->setNearestPoint(ia);
  record->setActor(act);
  record->setGeometry(geom);
  record->setPrimitives(prim);
  record->setDistance( t );
  mIntersections.push_back(record);
}
//-----------------------------------------------------------------------------
