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

using OpenTK.Mathematics;
using OpenTK.Windowing.Common;

namespace UEPADemo
{
    public class Connection : IDrawableComponent
    {
        Tube tube = new Tube();
        Sphere sphereA = new Sphere();
        Sphere sphereB = new Sphere();

        public void Draw(FrameEventArgs e)
        {
            tube.Draw(e);
            sphereA.Draw(e);
            sphereB.Draw(e);
        }

        public void Load()
        {
            tube.Load();
            sphereA.Load();
            sphereB.Load();
        }

        public void Update(FrameEventArgs e)
        {
            tube.Update(e);
            sphereA.Update(e);
            sphereB.Update(e);
        }

        public void SetProperties(Vector3 pointA, Vector3 pointB)
        {
            // stretch and rotate the tube in such a way that it reaches
            // from pointA to pointB

            Vector3 connection = pointA - pointB;
            float distance = connection.Length;
            connection.Normalize();

            Vector3 center = (pointA + pointB) * 0.5f;

            float angle = (float)Math.Acos((double)Vector3.Dot(connection, Vector3.UnitY));
            Vector3 axis = Vector3.Cross(Vector3.UnitY, connection);

            Matrix4 scale = Matrix4.CreateScale(0.01f, distance, 0.01f);
            Matrix4 rotation = Matrix4.CreateFromAxisAngle(axis, angle);
            Matrix4 translation = Matrix4.CreateTranslation(center);
            tube.WorldMatrix = scale * rotation * translation;

            // place the spheres on the ends

            scale = Matrix4.CreateScale(0.05f);
            translation = Matrix4.CreateTranslation(pointA);
            sphereA.WorldMatrix = scale * translation;

            translation = Matrix4.CreateTranslation(pointB);
            sphereB.WorldMatrix = scale * translation;
        }

        public void Dispose()
        {
            tube.Dispose();
            sphereA.Dispose();
            sphereB.Dispose();
        }
    }

    public class Tube : Primitive
    {
        public Vector3 PointA = Vector3.Zero;
        public Vector3 PointB = Vector3.One;

        private const float height = 1.0f;
        private const float radius = 0.5f;
        private const int tessellation = 32;

        private NeonShader shader;

        public override void Draw(FrameEventArgs e)
        {
            shader.Use();
            shader.Projection.Set(Showcase.Current.Camera.ProjectionMatrix);
            shader.View.Set(Showcase.Current.Camera.ViewMatrix);
            shader.Model.Set(WorldMatrix);

            base.Draw(e);
        }

        public override void Load()
        {
            shader = new NeonShader();
            base.Load();
        }

        static Vector3 GetCircleVector(int i, int tessellation)
        {
            float angle = i * MathHelper.TwoPi / tessellation;
            float dx = (float)Math.Cos(angle);
            float dz = (float)Math.Sin(angle);
            return new Vector3(dx, 0, dz);
        }

        public override void Build()
        {
            Vector3 up = Vector3.UnitY;
            Vector3 down = -up;

            if (tessellation < 3)
                throw new ArgumentOutOfRangeException("tessellation");

            for (int i = 0; i < tessellation; i++)
            {
                Vector3 normal = GetCircleVector(i, tessellation);

                AddVertex(normal * radius + up * height / 2, normal);
                AddVertex(normal * radius + down * height / 2, normal);

                AddIndex(i * 2);
                AddIndex((i * 2 + 2) % (tessellation * 2));
                AddIndex(i * 2 + 1);

                AddIndex(i * 2 + 1);
                AddIndex((i * 2 + 2) % (tessellation * 2));
                AddIndex((i * 2 + 3) % (tessellation * 2));
            }
        }

    }

    public class Sphere : Primitive
    {
        private const int tessellation = 32;
        private const float radius = 0.5f;

        private NeonShader shader;

        public override void Draw(FrameEventArgs e)
        {
            shader.Use();
            shader.Projection.Set(Showcase.Current.Camera.ProjectionMatrix);
            shader.View.Set(Showcase.Current.Camera.ViewMatrix);
            shader.Model.Set(WorldMatrix);

            base.Draw(e);
        }

        public override void Load()
        {
            shader = new NeonShader();
            base.Load();
        }

        public override void Build()
        {
            if (tessellation < 3)
                throw new ArgumentOutOfRangeException("tessellation");

            Vector3 up = Vector3.UnitY;
            Vector3 down = -up;

            int verticalSegments = tessellation;
            int horizontalSegments = tessellation * 2;

            AddVertex(down * radius, down);

            for (int i = 0; i < verticalSegments - 1; i++)
            {
                float latitude = ((i + 1) * MathHelper.Pi /
                                            verticalSegments) - MathHelper.PiOver2;

                float dy = (float)Math.Sin(latitude);
                float dxz = (float)Math.Cos(latitude);

                for (int j = 0; j < horizontalSegments; j++)
                {
                    float longitude = j * MathHelper.TwoPi / horizontalSegments;
                    float dx = (float)Math.Cos(longitude) * dxz;
                    float dz = (float)Math.Sin(longitude) * dxz;
                    Vector3 normal = new Vector3(dx, dy, dz);
                    AddVertex(normal * radius, normal);
                }
            }

            AddVertex(up * radius, up);

            for (int i = 0; i < horizontalSegments; i++)
            {
                AddIndex(0);
                AddIndex(1 + i);
                AddIndex(1 + (i + 1) % horizontalSegments);
            }

            for (int i = 0; i < verticalSegments - 2; i++)
            {
                for (int j = 0; j < horizontalSegments; j++)
                {
                    int nextI = i + 1;
                    int nextJ = (j + 1) % horizontalSegments;

                    AddIndex(1 + i * horizontalSegments + j);
                    AddIndex(1 + nextI * horizontalSegments + j);
                    AddIndex(1 + i * horizontalSegments + nextJ);

                    AddIndex(1 + i * horizontalSegments + nextJ);
                    AddIndex(1 + nextI * horizontalSegments + j);
                    AddIndex(1 + nextI * horizontalSegments + nextJ);
                }
            }

            for (int i = 0; i < horizontalSegments; i++)
            {
                AddIndex(this.NumVertices - 1);
                AddIndex(this.NumVertices - 2 - i);
                AddIndex(this.NumVertices - 2 - (i + 1) % horizontalSegments);
            }
        }
    }
}