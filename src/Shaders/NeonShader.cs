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
    public class NeonShader : BasicShader, IModelViewProjection
    {
        public UniformMatrix4 Model { private set; get; }
        public UniformMatrix4 View { private set; get; }
        public UniformMatrix4 Projection { private set; get; }

        public NeonShader() : base(vshader, fshader)
        {
            Model = this.Uniforms["model"] as UniformMatrix4;
            View = this.Uniforms["view"] as UniformMatrix4;
            Projection = this.Uniforms["projection"] as UniformMatrix4;
        }

        private static string vshader = @"
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec3 aNorm;

        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;

        void main()
        {
            gl_Position = projection * view * model * vec4(aPos, 1.0);
        }
        ";

        private static string fshader = @"
        #version 330 core
        out vec4 FragColor;

        void main()
        {
            FragColor = vec4(0,1,0,1);
        }
        ";
    }
}