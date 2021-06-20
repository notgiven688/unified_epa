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

namespace GJKEPADemo
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
            GL.Uniform1(this.location, 1, value);
        }
    }

    public class UniformTexture : Uniform
    {
        public UniformTexture(Shader shader, int location) : base(shader, location) { }

        public void Set(TextureUnit value)
        {
            Debug.Assert(this.shader.IsActive);
            GL.Uniform1(this.location, 1, (int)value - (int)TextureUnit.Texture0); // TODO
        }
    }

    public class UniformMatrix4 : Uniform
    {
        public UniformMatrix4(Shader shader, int location) : base(shader, location) { }

        public void Set(Matrix4 value)
        {
            this.Set(ref value);
        }

        public void Set(ref Matrix4 value)
        {
            Debug.Assert(this.shader.IsActive);
            GL.UniformMatrix4(this.location, 1, false, value.Row0.X);
        }

        public void SetTranposed(Matrix4 value)
        {
            this.SetTranposed(ref value);
        }

        public void SetTranposed(ref Matrix4 value)
        {
            Debug.Assert(this.shader.IsActive);
            GL.UniformMatrix4(this.location, 1, true, value.Row0.X);
        }
    }

    public class UniformVector4 : Uniform
    {
        public UniformVector4(Shader shader, int location) : base(shader, location) { }

        public void Set(Vector4 value)
        {
            Debug.Assert(this.shader.IsActive);
            GL.Uniform4(this.location, 1, value.X);
        }

        public void Set(ref Vector4 value)
        {
            Debug.Assert(this.shader.IsActive);
            GL.Uniform4(this.location, 1, value.X);
        }

        public void Set(float x, float y, float z, float w)
        {
            Vector4 xyzw = new Vector4(x, y, z, w);
            this.Set(ref xyzw);
        }
    }

    public class UniformVector3 : Uniform
    {
        public UniformVector3(Shader shader, int location) : base(shader, location) { }

        public void Set(Vector3 value)
        {
            Debug.Assert(this.shader.IsActive);
            GL.Uniform3(this.location, 1, value.X);
        }

        public void Set(ref Vector3 value)
        {
            Debug.Assert(this.shader.IsActive);
            GL.Uniform3(this.location, 1, value.X);
        }

        public void Set(float x, float y, float z)
        {
            Vector3 xyz = new Vector3(x, y, z);
            this.Set(ref xyz);
        }
    }

    public class UniformVector2 : Uniform
    {
        public UniformVector2(Shader shader, int location) : base(shader, location) { }

        public void Set(Vector2 value)
        {
            Debug.Assert(this.shader.IsActive);
            GL.Uniform2(this.location, 1, value.X);
        }

        public void Set(ref Vector2 value)
        {
            Debug.Assert(this.shader.IsActive);
            GL.Uniform2(this.location, 1, value.X);
        }

        public void Set(float x, float y)
        {
            Vector2 xy = new Vector2(x, y);
            this.Set(ref xy);
        }
    }

}