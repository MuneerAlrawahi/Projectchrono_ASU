
//  Copyright (c) 2023, Arizona state university 
//


// =============================================================================
// This demo features A box of granular particles and vibration waves to study the 
// the energy dissipation  
// =============================================================================

#include <core/ApiVersion.h>
#include <core/utils/ThreadManager.h>
#include <DEM/API.h>
#include <DEM/HostSideHelpers.hpp>
#include <DEM/utils/Samplers.hpp>

#include <cstdio>
#include <chrono>
#include <filesystem>

using namespace deme;
using namespace std::filesystem;

int main() {


    // Initialize the DEMSolver
    DEMSolver DEMSim;
    DEMSim.SetVerbosity(STEP_METRIC);
    DEMSim.SetOutputFormat(OUTPUT_FORMAT::CSV);
    DEMSim.SetOutputContent(OUTPUT_CONTENT::ABSV);
    DEMSim.SetMeshOutputFormat(MESH_FORMAT::VTK);

    



    
}