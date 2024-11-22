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
using System.Diagnostics;

using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;

namespace UEPADemo
{
    public abstract class Uniform
    {
        protected Shader shader;
        protected int location = 0;

        public Uniform(Shader shader, int location)
        {
            this.shader = shader;
            this.location = location;
        }
    }

    public class UniformFloat : Uniform
    {
        public UniformFloat(Shader shader, int location) : base(shader, location) { }

        public void Set(float value)
        {
            Debug.Assert(this.shader.IsActive);
            GL.Uniform1f(this.location, value);
        }
    }

    public class UniformTexture : Uniform
    {
        public UniformTexture(Shader shader, int location) : base(shader, location) { }

        public void Set(TextureUnit value)
        {
            Debug.Assert(this.shader.IsActive);
            GL.Uniform1i(this.location, (int)value - (int)TextureUnit.Texture0); // ToDo
        }
    }

    public class UniformMatrix4 : Uniform
    {
        public UniformMatrix4(Shader shader, int location) : base(shader, location) { }

        public void Set(in Matrix4 value)
        {
            Debug.Assert(this.shader.IsActive);
            GL.UniformMatrix4f(this.location, false, value);
        }

        public void SetTranposed(in Matrix4 value)
        {
            Debug.Assert(this.shader.IsActive);
            GL.UniformMatrix4f(this.location, true, value);
        }
    }

    public class UniformVector4 : Uniform
    {
        public UniformVector4(Shader shader, int location) : base(shader, location) { }

        public void Set(in Vector4 value)
        {
            Debug.Assert(this.shader.IsActive);
            GL.Uniform4f(this.location, value);
        }

        public void Set(float x, float y, float z, float w)
        {
            Debug.Assert(this.shader.IsActive);
            GL.Uniform4f(this.location, x, y, z, w);
        }
    }

    public class UniformVector3 : Uniform
    {
        public UniformVector3(Shader shader, int location) : base(shader, location) { }

        public void Set(in Vector3 value)
        {
            Debug.Assert(this.shader.IsActive);
            GL.Uniform3f(this.location, value);
        }

        public void Set(float x, float y, float z)
        {
            Debug.Assert(this.shader.IsActive);
            GL.Uniform3f(this.location, x, y, z);
        }
    }

    public class UniformVector2 : Uniform
    {
        public UniformVector2(Shader shader, int location) : base(shader, location) { }

        public void Set(in Vector2 value)
        {
            Debug.Assert(this.shader.IsActive);
            GL.Uniform2f(this.location, value);
        }

        public void Set(float x, float y)
        {
            Debug.Assert(this.shader.IsActive);
            GL.Uniform2f(this.location, x, y);
        }
    }

}