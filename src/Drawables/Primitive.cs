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

using OpenTK.Graphics.OpenGL;
using OpenTK.Graphics;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;

namespace GJKEPADemo
{
    public class Primitive : IDrawableComponent
    {
        private BufferHandle VBO, EBO;
        private VertexArrayHandle VAO;

        private List<float> vertices = new List<float>();
        private List<uint> indices = new List<uint>();

        public int NumVertices { get { return this.vertices.Count / 6; } }
        public int NumIndices { get { return this.indices.Count; } }

        public Matrix4 WorldMatrix { set; get; } = Matrix4.Identity;
        public Vector3 Position { get; set; } = Vector3.Zero;

        public JVector GetJPosition()
        {
            return new JVector(Position.X, Position.Y, Position.Z);
        }

        public virtual JMatrix GetJMatrix()
        {
            JMatrix result;
            result.M11 = WorldMatrix.M11; result.M12 = WorldMatrix.M12; result.M13 = WorldMatrix.M13;
            result.M21 = WorldMatrix.M21; result.M22 = WorldMatrix.M22; result.M23 = WorldMatrix.M23;
            result.M31 = WorldMatrix.M31; result.M32 = WorldMatrix.M32; result.M33 = WorldMatrix.M33;
            return result;
        }

        protected void AddVertex(Vector3 position, Vector3 normal)
        {
            vertices.Add(position.X); vertices.Add(position.Y); vertices.Add(position.Z);
            vertices.Add(normal.X); vertices.Add(normal.Y); vertices.Add(normal.Z);
        }

        protected void AddIndex(int index)
        {
            if (index > ushort.MaxValue)
                throw new ArgumentOutOfRangeException("index");

            indices.Add((ushort)index);
        }

        public virtual void Build() { }

        public void Bind()
        {
            GL.BindVertexArray(VAO);
        }

        public virtual void Draw(FrameEventArgs e)
        {
            GL.BindVertexArray(VAO);
            GL.DrawElements(PrimitiveType.Triangles, this.NumIndices, DrawElementsType.UnsignedInt, 0);
        }

        public virtual void Load()
        {
            this.Build();

            VAO = GL.GenVertexArray();
            GL.BindVertexArray(VAO);

            VBO = GL.GenBuffer();
            EBO = GL.GenBuffer();

            GL.BindBuffer(BufferTargetARB.ArrayBuffer, VBO);
            GL.BufferData(BufferTargetARB.ArrayBuffer, vertices.ToArray(), BufferUsageARB.StaticDraw);

            GL.BindBuffer(BufferTargetARB.ElementArrayBuffer, EBO);
            GL.BufferData(BufferTargetARB.ElementArrayBuffer, indices.ToArray(), BufferUsageARB.StaticDraw);

            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);

            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(1);
        }

        public virtual void Update(FrameEventArgs e)
        {
            // nothing to do
        }

        public virtual void Dispose()
        {
            GL.DeleteVertexArray(VAO);
            GL.DeleteBuffer(EBO);
            GL.DeleteBuffer(VBO);
        }
    }
}