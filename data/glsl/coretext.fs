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


// The CoreFont atlas is a single channel (GL_RED) texture holding the glyph coverage.

#version 150 compatibility

uniform sampler2D font_atlas;
uniform vec4 text_color;

in vec2 TexCoord;
out vec4 FragColor;

void main(void)
{
  float coverage = texture(font_atlas, TexCoord).r;
  FragColor = vec4(text_color.rgb, text_color.a * coverage);
}
