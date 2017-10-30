/**************************************************************************************/
/*                                                                                    */
/*  Visualization Library                                                             */
/*  http://visualizationlibrary.org                                                   */
/*                                                                                    */
/*  Copyright (c) 2005-2011, Michele Bosi , Fabien Mathieu                            */
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

#ifndef DrawTransformFeedback_INCLUDE_DEFINE
#define DrawTransformFeedback_INCLUDE_DEFINE

#include <vlGraphics/DrawCall.hpp>
#include <vlGraphics/TriangleIterator.hpp>
#include <vlGraphics/TransformFeedback.hpp>

namespace vl
{
  //------------------------------------------------------------------------------
  // DrawArrays
  //------------------------------------------------------------------------------
  /**
   * Wraps the OpenGL function glDrawTransformFeedback(). See vl::DrawCall for an overview of the different draw call methods.
   *
   * This class wraps the following OpenGL functions:
   * - glDrawArrays (https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glDrawTransformFeedback.xhtml)
   * - glDrawArraysInstanced (https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glDrawTransformFeedbackInstanced.xhtml)
   *
   * Supports:
   * - <b>Multi instancing</b>: YES
   * - <b>Base vertex</b>: N/A
   * - <b>Primitive restart</b>: N/A
   *
   * DrawArrays, DrawElements, MultiDrawElements and DrawRangeElements are used by Geometry to define a set of primitives to be rendered.
   * @sa Geometry::drawCalls(), DrawCall, DrawElements, MultiDrawElements, DrawRangeElements, Geometry, Actor */
  class DrawTransformFeedback: public DrawCall
  {
    VL_INSTRUMENT_CLASS(vl::DrawArrays, DrawCall)

  public:
    DrawTransformFeedback()
    {
      VL_DEBUG_SET_OBJECT_NAME()
      mType      = PT_TRIANGLES;
    }

    DrawTransformFeedback(EPrimitiveType primitive, TransformFeedback* transformFeedback = nullptr)
      : mTransformFeedback(transformFeedback)
    {
      VL_DEBUG_SET_OBJECT_NAME()
      mType = primitive;
    }

    DrawTransformFeedback& operator=(const DrawTransformFeedback& other)
    {
      super::operator=(other);
      return *this;
    }

    virtual ref<DrawCall> clone() const
    {
      return new DrawTransformFeedback( primitiveType(), const_cast< TransformFeedback* >(mTransformFeedback.get()) );
    }

    virtual void deleteBufferObject() {}
    virtual void updateDirtyBufferObject(EBufferObjectUpdateMode) {}

    virtual void render(bool) const
    {
      VL_CHECK_OGL()

      // apply patch parameters if any and if using PT_PATCHES
      applyPatchParameters();

      glDrawTransformFeedback( primitiveType(), transformFeedback()->handle() );

      #ifndef NDEBUG
        unsigned int glerr = glGetError();
        if (glerr != GL_NO_ERROR)
        {
          String msg( getGLErrorString(glerr) );
          Log::error( Say("glGetError() [%s:%n]: %s\n") << __FILE__ << __LINE__ << msg );
          Log::warning( "- If you are using geometry instancing in conjunction with display lists you will have to disable one of them.\n" );
          Log::warning( "- If you are using OpenGL ES you must NOT use GL_QUADS, GL_QUAD_STRIP and GL_POLYGON primitive types.\n" );
          VL_TRAP()
        }
      #endif
    }

    //! Sets the transform feedback for this set of primitives.
    void setTransformFeedback(TransformFeedback* transformFeedback) { mTransformFeedback = transformFeedback; }

    //! Returns the transform feedback id for this set of primitives.
    TransformFeedback* transformFeedback() { return mTransformFeedback.get(); }

    //! Returns the transform feedback id for this set of primitives.
    const TransformFeedback* transformFeedback() const { return mTransformFeedback.get(); }

    TriangleIterator triangleIterator() const
    {
      ref<TriangleIteratorDirect> tid = new TriangleIteratorDirect( primitiveType() );
      //tid->initialize(mStart, mStart+mCount);
      return TriangleIterator(tid.get());
    }

    IndexIterator indexIterator() const 
    {
      IndexIterator iit;
      return iit;
    }


    protected:
      ref< TransformFeedback > mTransformFeedback;
  };

}

#endif

