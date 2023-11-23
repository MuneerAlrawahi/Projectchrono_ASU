
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

    

    //Define materials propeties 
    // E, nu, CoR, mu, Crr...
    auto mat_type_Plate = DEMSim.LoadMaterial({{"E", 1e8}, {"nu", 0.3}, {"CoR", 0.6}, {"mu", 0.5}, {"Crr", 0.0}});
    auto mat_type_granular = DEMSim.LoadMaterial({{"E", 1e8}, {"nu", 0.3}, {"CoR", 0.6}, {"mu", 0.2}, {"Crr", 0.0}});
    DEMSim.SetMaterialPropertyPair("mu", mat_type_Plate, mat_type_granular, 0.5);



    // Define the simulation domain and boundary conditions 
    float step_size = 1e-5;
    double world_size = 3;
    DEMSim.InstructBoxDomainDimension({0, world_size}, {0, world_size}, {0, world_size});
    DEMSim.InstructBoxDomainBoundingBC("top_open", mat_type_granular);


    // Define the terrain particle templates
    // Calculate its mass and MOI
    float terrain_density = 2.6e3;
    double clump_vol = 5.5886717;
    float mass = terrain_density * clump_vol;
    float3 MOI = make_float3(2.928, 2.6029, 3.9908) * terrain_density;
    // Then load it to system
    std::shared_ptr<DEMClumpTemplate> my_template =
        DEMSim.LoadClumpType(mass, MOI, GetDEMEDataFile("clumps/3_clump.csv"), mat_type_terrain);
    my_template->SetVolume(clump_vol);
    // Decide the scalings of the templates we just created (so that they are... like particles, not rocks)
    double scale = 0.0044;
    my_template->Scale(scale);

    // Sampler to sample
    HCPSampler sampler(scale * 3.);
    float fill_height = 0.5;
    double bottom = -0.5;
    float3 fill_center = make_float3(0, 0, 0);
    //const float fill_radius = soil_bin_diameter / 2. - scale * 3.;
    float sample_halfwidth = world_size / 2 * 0.95;
    auto input_xyz = sampler.SampleBox(fill_center, sample_halfwidth);
    DEMSim.AddClumps(my_template, input_xyz);
    std::cout << "Total num of particles: " << input_xyz.size() << std::endl;


    DEMSim.DisableContactBetweenFamilies(1, 2);
    DEMSim.SetCDUpdateFreq(30);
    DEMSim.SetInitTimeStep(step_size);
    DEMSim.SetGravitationalAcceleration(make_float3(0, 0, -9.81));
    DEMSim.SetCDUpdateFreq(40);



    // Start the sumiluation 
    DEMSim.Initialize();

    path out_dir = current_path();
    out_dir += "/DemoOutput_Trail";
    create_directory(out_dir);

    float sim_time = 2.0;
    unsigned int fps = 20;
    float frame_time = 1.0 / fps;

    std::cout << "Output at " << fps << " FPS" << std::endl;
    unsigned int currframe = 0;


    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    for (float t = 0; t < sim_time; t += frame_time) {
        std::cout << "Frame: " << currframe << std::endl;
        char filename[200], meshfilename[200];
        sprintf(filename, "%s/DEMdemo_output_%04d.csv", out_dir.c_str(), currframe);
        sprintf(meshfilename, "%s/DEMdemo_mesh_%04d.vtk", out_dir.c_str(), currframe);
        DEMSim.WriteSphereFile(std::string(filename));
        DEMSim.WriteMeshFile(std::string(meshfilename));
        currframe++;

        DEMSim.DoDynamicsThenSync(frame_time);
        DEMSim.ShowThreadCollaborationStats();

    }
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_sec = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
    std::cout << time_sec.count() << " seconds (wall time) to finish the simulation" << std::endl;

    DEMSim.ShowTimingStats();
    DEMSim.ShowAnomalies();
    std::cout << "DEMdemo_BallDrop exiting..." << std::endl;
    return 0;


    
}