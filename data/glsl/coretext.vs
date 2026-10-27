/**************************************************************************************/
/*                                                                                    */
/*  Copyright (c) 2005-2020, Michele Bosi.                                            */
/*  All rights reserved.                                                              */
/*                                                                                    */
/*  This file is part of Visualization Library                                        */
/*  http://visualizationlibrary.org                                                   */
/*                                                                                    */
/*  Released under the OSI approved Simplified BSD License                            */
/*  http://www.opensource.org/licenses/bsd-license.php                                */
/*                                                                                    */
/**************************************************************************************/


// Renders a vl::CoreText: one textured quad per glyph, sampled from the CoreFont atlas.

#version 150 compatibility

#pragma VL include /glsl/std/uniforms.glsl
#pragma VL include /glsl/std/vertex_attribs.glsl

out vec2 TexCoord;

void main(void)
{
  gl_Position = vl_ModelViewProjectionMatrix * vl_VertexPosition;
  TexCoord = vl_VertexTexCoord0.xy;
}
