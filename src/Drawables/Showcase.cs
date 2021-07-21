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
using System.Reflection;
using System.Collections.Generic;

using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.GraphicsLibraryFramework;

namespace GJKEPADemo
{
    public class Showcase : IDrawableComponent
    {
        public static Showcase Current { private set; get; }

        public Camera Camera { private set; get; }

        public MouseState MouseState { private set; get; }
        public KeyboardState KeyState { private set; get; }

        public MainWindow Window { get; private set; }

        private List<IDrawableComponent> drawableComponents = new List<IDrawableComponent>();

        public ImplicitShape PrimitiveLeft { private set; get; }
        public ImplicitShape PrimitiveRight { private set; get; }
        public Connection Connection { private set; get; }
        public TextOverlay Overlay { private set; get; }

        private float distance = 1.7f;
        private bool autorotate = true;
        private bool advancerotation = false;

        private Random random = new Random();

        public List<Type> AllShapes { get; private set; }

        public Showcase(MainWindow window)
        {
            if (Showcase.Current != null)
                throw new InvalidOperationException("Showcase should be used as a singleton.");

            this.Window = window;

            this.Camera = new Camera();
            this.drawableComponents.Add(Camera);

            Camera.Position = new Vector3(0, 0, -6);

            Showcase.Current = this;

            Connection = new Connection();
            this.drawableComponents.Add(Connection);

            AllShapes = new List<Type>(ShapeAttribute.GetAllTypes());

            PrimitiveLeft = new ImplicitShape(new CubeShape());
            PrimitiveRight = new ImplicitShape(new CubeShape());

            this.drawableComponents.Add(PrimitiveLeft);
            this.drawableComponents.Add(PrimitiveRight);

            Overlay = new TextOverlay();
            Overlay.Position = new Vector2(10, 10);
            this.drawableComponents.Add(Overlay);
        }

        private void NextRandomShapes()
        {
            this.drawableComponents.Remove(PrimitiveLeft);
            this.drawableComponents.Remove(PrimitiveRight);

            PrimitiveLeft.Dispose();
            PrimitiveRight.Dispose();

            int l = random.Next(AllShapes.Count);
            int r = random.Next(AllShapes.Count);

            PrimitiveLeft = new ImplicitShape((ISupportMappable)Activator.CreateInstance(AllShapes[l]));
            PrimitiveRight = new ImplicitShape((ISupportMappable)Activator.CreateInstance(AllShapes[r]));

            PrimitiveLeft.Load();
            PrimitiveRight.Load();

            this.drawableComponents.Insert(2, PrimitiveLeft);
            this.drawableComponents.Insert(2, PrimitiveRight);
        }

        public void Load()
        {
            foreach (var component in drawableComponents) component.Load();
        }

        public void Draw(FrameEventArgs e)
        {
            foreach (var component in drawableComponents) component.Draw(e);
        }

        public void Detect()
        {
            string leftShape = PrimitiveLeft.SupportMap.GetType().GetCustomAttribute<ShapeAttribute>(true).Name;
            string rightShape = PrimitiveRight.SupportMap.GetType().GetCustomAttribute<ShapeAttribute>(true).Name;

            Overlay.Text = $"GJK/EPA Demonstration ({leftShape}/{rightShape})\n\n";
            Overlay.Text += "+ Controls\n\n";
            Overlay.Text += "  change shape distance       (k,l)\n";
            Overlay.Text += "  move the camera             (w,a,s,d,mouse)\n";
            Overlay.Text += "  toggle continuous rotation  (r)\n";
            Overlay.Text += "  rotate single step          (t)\n";
            Overlay.Text += "  switch pair of shapes       (n)\n";
            Overlay.Text += "  exit                        (esc)\n\n";
            Overlay.Text += "+ Statistics\n\n";

            JMatrix orientLeft = PrimitiveLeft.GetJMatrix();
            JMatrix orientRight = PrimitiveRight.GetJMatrix();
            JVector posLeft = PrimitiveLeft.GetJPosition();
            JVector posRight = PrimitiveRight.GetJPosition();

            JVector pl, pr;
            double separation;

            bool success = GJKEPA.Detect(PrimitiveLeft.SupportMap, PrimitiveRight.SupportMap,
                ref orientLeft, ref orientRight, ref posLeft, ref posRight,
                out pl, out pr, out separation);

            if (success)
            {
                Vector3 pA = new Vector3((float)pl.X, (float)pl.Y, (float)pl.Z);
                Vector3 pB = new Vector3((float)pr.X, (float)pr.Y, (float)pr.Z);

                Overlay.Text += $"  Collision:  {(separation < 0.0d).ToString()}\n";
                Overlay.Text += $"  Iterations: {GJKEPA.epaSolver.Statistics.Iterations.ToString()}\n";
                Overlay.Text += $"  Accuracy:   {GJKEPA.epaSolver.Statistics.Accuracy.ToString("0E+00")}\n";
                Overlay.Text += $"  Separation: {separation.ToString("0.###")}\n";

                PrimitiveLeft.Alpha = separation > 0.0d ? 1.0f : 0.5f;
                PrimitiveRight.Alpha = separation > 0.0d ? 1.0f : 0.5f;

                Connection.SetProperties(pA, pB);
            }

            // crude way of saving distances to file
            // if(success) System.IO.File.AppendAllText("data.txt", separation.ToString() + "\n");
            // else System.IO.File.AppendAllText("data.txt", 0.ToString() + "\n");
        }

        private void HandleInput()
        {
            bool IsSwitched(Keys key) =>
                KeyState.IsKeyDown(key) && !KeyState.WasKeyDown(key);

            if (KeyState.IsKeyDown(Keys.K)) distance -= 0.03f;
            if (KeyState.IsKeyDown(Keys.L)) distance += 0.03f;
        
            if (IsSwitched(Keys.R)) autorotate = !autorotate;
            if (IsSwitched(Keys.T)) advancerotation = true;
            if (IsSwitched(Keys.N)) NextRandomShapes();
            if (IsSwitched(Keys.Escape)) this.Window.Close();

            PrimitiveLeft.Position = new Vector3(+distance / 2.0f, 0.0f, 0.0f);
            PrimitiveRight.Position = new Vector3(-distance / 2.0f, 0.3f, 0.1f);
        }

        public void Update(FrameEventArgs e)
        {
            Vector3 deltaAngle = new Vector3(0.006f, 0.007f, 0.008f);

            this.MouseState = this.Window.MouseState;
            this.KeyState = this.Window.KeyboardState;

            if (autorotate || advancerotation)
            {
                (PrimitiveLeft as ImplicitShape).Angles += deltaAngle;
                (PrimitiveRight as ImplicitShape).Angles -= deltaAngle;
                advancerotation = false;
            }

            HandleInput();
            Detect();

            foreach (var component in drawableComponents) component.Update(e);
        }
    }
}