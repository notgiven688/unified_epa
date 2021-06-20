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
using OpenTK.Windowing.GraphicsLibraryFramework;
using OpenTK.Windowing.Common;

namespace GJKEPADemo
{
    public class Camera : IDrawableComponent
    {
        public Matrix4 ViewMatrix = Matrix4.Identity;
        public Matrix4 ProjectionMatrix = Matrix4.Identity;

        public Vector3 Position = Vector3.Zero;
        public Vector3 Direction = Vector3.Zero;

        double theta = MathHelper.PiOver2;
        double phi = 0.0d;

        public void Draw(FrameEventArgs e)
        {
            // nothing to do
        }

        public void Load()
        {
            // nothing to do
        }

        public void Update(FrameEventArgs e)
        {
            KeyboardState ks = Showcase.Current.KeyState;
            MouseState ms = Showcase.Current.MouseState;

            if (ms.IsButtonDown(MouseButton.Left))
            {
                phi -= ms.Delta.X * 0.006f;
                theta += ms.Delta.Y * 0.006f;
            }

            if (theta > Math.PI - 0.1d) theta = Math.PI - 0.1d;
            if (theta < 0.1d) theta = 0.1d;

            float z = (float)(Math.Sin(theta) * Math.Cos(phi));
            float x = (float)(Math.Sin(theta) * Math.Sin(phi));
            float y = (float)(Math.Cos(theta));

            Direction = new Vector3(x, y, z);
            Vector3 cright = Vector3.Normalize(Vector3.Cross(Vector3.UnitY, Direction));
            Vector3 cup = Vector3.Cross(Direction, cright);

            Vector3 mv = Vector3.Zero;
            if (ks.IsKeyDown(Keys.W)) mv += Direction;
            if (ks.IsKeyDown(Keys.S)) mv -= Direction;
            if (ks.IsKeyDown(Keys.A)) mv += cright;
            if (ks.IsKeyDown(Keys.D)) mv -= cright;

            if (mv.LengthSquared > 0.1f) mv.Normalize();
            Position += 0.1f * mv;

            float width = (float)Showcase.Current.Window.Size.X;
            float height = (float)Showcase.Current.Window.Size.Y;

            ViewMatrix = Matrix4.LookAt(Position, Position + Direction, Vector3.UnitY);
            ProjectionMatrix = Matrix4.CreatePerspectiveFieldOfView(MathHelper.DegreesToRadians(45.0f), width / height, 0.1f, 100.0f);
        }
    }
}