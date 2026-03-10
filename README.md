# Unified EPA (UEPA) Demo 
[![uepa_tests](https://github.com/notgiven688/unified_epa/actions/workflows/uepa_tests.yml/badge.svg)](https://github.com/notgiven688/unified_epa/actions/workflows/uepa_tests.yml)

Robust implementation of the **Expanding Polytope Algorithm** (EPA).

### 🌐 [**Try the Interactive Web Demo →**](https://notgiven688.github.io/unified_epa/)

## Some Details

The implementation itself is contained in [UEPA.cs](src/UnifiedEPA/UEPA.cs). The algorithm detects whether two convex objects are separated or colliding and returns the points of closest distance and deepest penetration, respectively.

The expanding polytope algorithm is used also for the separating case - simplifying
the algorithm for collision detection for the general case.

## Run the demo locally

1. Install the [.NET 10.0 SDK](https://dotnet.microsoft.com/download/dotnet/10.0) and the WASM workloads:
   ```
   dotnet workload install wasm-tools wasm-experimental
   ```
2. Clone and run:
   ```
   git clone https://github.com/notgiven688/unified_epa.git
   cd unified_epa/src/UnifiedEPA.WebDemo && ./run
   ```

