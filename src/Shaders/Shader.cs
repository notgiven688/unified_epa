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
using System.Collections.Generic;
using System.Collections.ObjectModel;

using OpenTK.Graphics.OpenGL;
using OpenTK.Graphics;

namespace GJKEPADemo
{
    interface IModelViewProjection
    {
        public UniformMatrix4 Model { get; }
        public UniformMatrix4 View { get; }
        public UniformMatrix4 Projection { get; }
    }

    interface IViewPosition
    {
        public UniformVector3 ViewPosition { get; }
    }

    public class BasicShader : Shader
    {
        public string VertexShader { get; private set; }
        public string FragmentShader { get; private set; }

        public BasicShader(string vertex, string fragment)
        {
            this.VertexShader = vertex;
            this.FragmentShader = fragment;

            this.Build();
        }

        private void Build()
        {
            ShaderProgram = GL.CreateProgram();

            ShaderHandle fragmentShader = this.CompileShader(this.FragmentShader, ShaderType.FragmentShader);
            ShaderHandle vertexShader = this.CompileShader(this.VertexShader, ShaderType.VertexShader);

            GL.AttachShader(ShaderProgram, vertexShader);
            GL.AttachShader(ShaderProgram, fragmentShader);
            GL.LinkProgram(ShaderProgram);

            int success = -1;
            GL.GetProgrami(ShaderProgram, ProgramPropertyARB.LinkStatus, ref success);

            if (success == 0)
            {
                //string info = GL.GetProgramInfoLog(ShaderProgram);
                //throw new Exception(info);
                // TODO: !!
                throw new Exception();
            }

            GL.DeleteShader(fragmentShader);
            GL.DeleteShader(vertexShader);

            this.PrepareUniforms();
        }
    }

    public class Shader : IDisposable
    {
        public ReadOnlyDictionary<string, Uniform> Uniforms { get; private set; }

        internal ProgramHandle ShaderProgram;

        public void Use()
        {
            GL.UseProgram(this.ShaderProgram);
        }

        public bool IsActive
        {
            get
            {
                int active = -1;
                GL.GetInteger(GetPName.CurrentProgram, ref active);
                return (active == this.ShaderProgram.Handle);
            }
        }

        protected void PrepareUniforms()
        {
            var uniforms = new Dictionary<string, Uniform>();
            this.Uniforms = new ReadOnlyDictionary<string, Uniform>(uniforms);

            int size, length, count, location;
            string name;
            UniformType type = UniformType.Bool;

            count = length = size = location = -1;

            GL.GetProgrami(this.ShaderProgram, ProgramPropertyARB.ActiveUniforms, ref count);

            for (uint i = 0; i < count; i++)
            {
                GL.GetActiveUniform(this.ShaderProgram, i, 1024, ref length, ref size, ref type, out name);
                location = GL.GetUniformLocation(this.ShaderProgram, name.ToString());

                switch (type)
                {
                    case UniformType.Float:
                        uniforms.Add(name.ToString(), new UniformFloat(this, location));
                        break;
                    case UniformType.FloatVec2:
                        uniforms.Add(name.ToString(), new UniformVector2(this, location));
                        break;
                    case UniformType.FloatVec3:
                        uniforms.Add(name.ToString(), new UniformVector3(this, location));
                        break;
                    case UniformType.FloatVec4:
                        uniforms.Add(name.ToString(), new UniformVector4(this, location));
                        break;
                    case UniformType.FloatMat4:
                        uniforms.Add(name.ToString(), new UniformMatrix4(this, location));
                        break;
                    case UniformType.Sampler2d:
                        uniforms.Add(name.ToString(), new UniformTexture(this, location));
                        break;
                    default:
                        throw new NotImplementedException($"Type '{type.ToString()}' not supported!");
                }
            }
        }

        protected ShaderHandle CompileShader(string code, ShaderType type)
        {
            ShaderHandle shader;

            shader = GL.CreateShader(type);
            GL.ShaderSource(shader, code);
            GL.CompileShader(shader);

            int success = -1;
            GL.GetShaderi(shader, ShaderParameterName.CompileStatus, ref success);

            if (success == 0)
            {
                GL.GetShaderInfoLog(shader, out string info);
                throw new Exception(info);
            }

            return shader;
        }

        public virtual void Dispose()
        {
            GL.DeleteProgram(ShaderProgram);
        }
    }
}
