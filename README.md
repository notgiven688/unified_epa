# Unified EPA Demo 

Robust and simple implementation of the **Gilbert-Johnson-Keerthi** (GJK) distance and the **Expanding Polytope Algorithm** (EPA).

The demo is written in C#, with [OpenTK](https://github.com/opentk/opentk) as only dependency.

## Some Details

The implementation itself is contained in [GJKEPA.cs](src/GJKEPA.cs) with ~500 lines of code. The algorithm detects whether two convex objects are separated or colliding and returns the points of closest distance and deepest penetration, respectively.

In contrast to other available implementations we only work with a convex hull with finite volume. The algorithm relies solely on point-triangle distances.

Details of the algorithm are summarized in [notes](notes.pdf).

## Run the demo

The demo should be able to run cross-platform utilizing OpenGL.
```
1. Install the [.NET 5.0 SDK](https://dotnet.microsoft.com/download/dotnet/5.0)
2. git clone https://github.com/notgiven688/unified_epa.git
3. cd unified_epa
4. dotnet run -c Release
```

## Screenshots

[YouTube Video](https://www.youtube.com/watch?v=NMdp7A13EAI)

![alt text](screenshots/gjkepa.png?raw=true)
