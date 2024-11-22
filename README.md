# Unified EPA Demo 
[![uepa_tests](https://github.com/notgiven688/unified_epa/actions/workflows/uepa_tests.yml/badge.svg)](https://github.com/notgiven688/unified_epa/actions/workflows/uepa_tests.yml)

Robust and simple implementation of the **Expanding Polytope Algorithm** (EPA).

The demo is written in C#, with [OpenTK](https://github.com/opentk/opentk) as only dependency.

## Some Details

The implementation itself is contained in [UEPA.cs](src/UEPA.cs) with ~400 lines of code. The algorithm detects whether two convex objects are separated or colliding and returns the points of closest distance and deepest penetration, respectively.

A variant of the expanding polytope algorithm is used also for the separating case - simplifying
the algorithm for collision detection for the general case.

## Run the demo

The demo should be able to run cross-platform utilizing OpenGL.

1. Install the [.NET 9.0 SDK](https://dotnet.microsoft.com/download/dotnet/9.0)
2. git clone https://github.com/notgiven688/unified_epa.git
3. cd unified_epa && dotnet run -c Release

## Screenshots

[YouTube Video](https://www.youtube.com/watch?v=NMdp7A13EAI)

![alt text](screenshots/uepa.png?raw=true)
