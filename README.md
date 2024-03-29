# Unified EPA Demo 
[![uepa_tests](https://github.com/notgiven688/unified_epa/actions/workflows/uepa_tests.yml/badge.svg)](https://github.com/notgiven688/unified_epa/actions/workflows/uepa_tests.yml)

Robust and simple implementation of the **Gilbert-Johnson-Keerthi** (GJK) distance and the **Expanding Polytope Algorithm** (EPA).

The demo is written in C#, with [OpenTK](https://github.com/opentk/opentk) as only dependency.

## Some Details

The implementation itself is contained in [GJKEPA.cs](src/GJKEPA.cs) with ~400 lines of code. The algorithm detects whether two convex objects are separated or colliding and returns the points of closest distance and deepest penetration, respectively.

In contrast to other available implementations we only work with a convex hull with finite volume. Thus the GJK-phase differs from the computationally more efficient "traditional implementations" of GJK. The algorithm relies solely on point-triangle distances.

## Run the demo

The demo should be able to run cross-platform utilizing OpenGL.
```
1. Install the [.NET 8.0 SDK](https://dotnet.microsoft.com/download/dotnet/8.0)
2. git clone https://github.com/notgiven688/unified_epa.git
3. cd unified_epa
4. dotnet run -c Release
```

## Screenshots

[YouTube Video](https://www.youtube.com/watch?v=NMdp7A13EAI)

![alt text](screenshots/gjkepa.png?raw=true)
