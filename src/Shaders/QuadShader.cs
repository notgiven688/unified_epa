/* Copyright <2021> <Thorben Linneweber>
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
* 
*/
using System;

namespace GJKEPADemo
{
    public class QuadShader : BasicShader
    {
        public UniformVector2 Offset { private set; get; }
        public UniformMatrix4 Projection { private set; get; }
        public UniformTexture FontTexture { private set; get; }

        public QuadShader() : base(vshader, fshader)
        {
             Offset = this.Uniforms["offset"] as UniformVector2;
             Projection = this.Uniforms["projection"] as UniformMatrix4;
             FontTexture = this.Uniforms["fontTexture"] as UniformTexture;
        }

        private static string vshader = @"
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec2 aTc;

        uniform vec2 offset;
        uniform mat4 projection;

        out vec2 TexCoord;

        void main()
        {
            gl_Position = projection * vec4(aPos + vec3(offset, 0.0), 1.0);
            TexCoord=vec2(aTc.x, aTc.y);
        }
        ";

        private static string fshader = @"
        #version 330 core

        uniform sampler2D fontTexture;

        in vec2 TexCoord;
        out vec4 FragColor;

        void main()
        {
            vec4 col = texture(fontTexture, TexCoord);
            FragColor = vec4(1.0f,1.0f,1.0f,min(1, col.w*1.2));
        }
        ";
    }
}