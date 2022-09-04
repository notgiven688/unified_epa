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

using OpenTK.Graphics.OpenGL;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;

namespace GJKEPADemo
{
    public interface IDrawableComponent
    {
        public void Draw(FrameEventArgs e);
        public void Update(FrameEventArgs e);
        public void Load();
    }

    public sealed class MainWindow : GameWindow
    {
        private Showcase showcase;

        public MainWindow(GameWindowSettings gws, NativeWindowSettings nws) : base(gws, nws)
        {
            showcase = new Showcase(this);
        }

        protected override void OnLoad()
        {
            base.OnLoad();
            showcase.Load();
        }

        protected override void OnResize(ResizeEventArgs e)
        {
            base.OnResize(e);
            GL.Viewport(0, 0, this.Size.X, this.Size.Y);
        }

        protected override void OnUpdateFrame(FrameEventArgs e) { base.OnUpdateFrame(e); showcase.Update(e); }

        protected override void OnRenderFrame(FrameEventArgs e)
        {
            base.OnRenderFrame(e);

            GL.Enable(EnableCap.DepthTest);
            GL.Enable(EnableCap.CullFace);

            GL.ClearColor(63.0f / 255.0f, 66.0f / 255.0f, 73.0f / 255.0f, 1.0f);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            showcase.Draw(e);
            SwapBuffers();
        }
    }

    static class Program
    {

        [STAThread]
        static void Main()
        {

#if TEST
            #pragma warning disable CS0162
            Tests.RunTests();
            return;
#endif


            GameWindowSettings gws = GameWindowSettings.Default;
            NativeWindowSettings nws = NativeWindowSettings.Default;

            nws.Title = "GJK/EPA Demonstration";
            nws.Size = new Vector2i(1024, 768);
            new MainWindow(gws, nws).Run();

        }
    }
}
