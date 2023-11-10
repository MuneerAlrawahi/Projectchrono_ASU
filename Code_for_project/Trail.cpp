//  Copyright (c) 2021, SBEL GPU Development Team
//  Copyright (c) 2021, University of Wisconsin - Madison
//
//	SPDX-License-Identifier: BSD-3-Clause

// =============================================================================
// This demo features a mesh-represented bladed mixer interacting with clump-represented
// DEM particles.
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
    DEMSolver DEMSim;
    DEMSim.SetVerbosity(STEP_METRIC);
    // For general use cases, you want to set the verbosity to INFO: It's also a bit faster than STEP_METRIC.
    // DEMSim.SetVerbosity(INFO);
    DEMSim.SetOutputFormat(OUTPUT_FORMAT::CSV);
    DEMSim.SetOutputContent(OUTPUT_CONTENT::ABSV);
    DEMSim.SetMeshOutputFormat(MESH_FORMAT::VTK);

    // If you don't need individual force information, then this option makes the solver run a bit faster.

    // E, nu, CoR, mu, Crr...
    auto mat_type_Plate = DEMSim.LoadMaterial({{"E", 1e8}, {"nu", 0.3}, {"CoR", 0.6}, {"mu", 0.5}, {"Crr", 0.0}});
    auto mat_type_granular = DEMSim.LoadMaterial({{"E", 1e8}, {"nu", 0.3}, {"CoR", 0.6}, {"mu", 0.2}, {"Crr", 0.0}});
    // If you don't have this line, then mu between mixer material and granular material will be 0.35 (average of the
    // two).
    DEMSim.SetMaterialPropertyPair("mu", mat_type_Plate, mat_type_granular, 0.5);

    float step_size = 1e-5;
    double world_size = 3;
    DEMSim.InstructBoxDomainDimension({0, world_size}, {0, world_size}, {0, world_size});
    DEMSim.InstructBoxDomainBoundingBC("top_open", mat_type_granular);

    // Now add a box boundary
    // auto template_granular = DEMSim.LoadSphereType(granular_rad * granular_rad * granular_rad * 2.8e3 * 4 / 3 * 3.14,
    //                                                granular_rad, mat_type_granular);
    // Calculate its mass and MOI

    auto projectile = DEMSim.AddWavefrontMeshObject((GET_DATA_PATH() / "mesh/plate_1by1.obj").string(), mat_type_Plate);
    std::cout << "Total num of triangles: " << projectile->GetNumTriangles() << std::endl;

    projectile->SetInitPos(make_float3(world_size / 2, world_size / 2, -world_size /  2));
    float plate_mass = 7.8e3 ;
    projectile->SetMass(plate_mass);
    projectile->SetMOI(make_float3(plate_mass * 2 / 5, plate_mass * 2 / 5, plate_mass * 2 / 5));
    projectile->SetFamily(2);
    DEMSim.SetFamilyFixed(2);



    float terrain_rad = 0.25;
    auto template_terrain = DEMSim.LoadSphereType(terrain_rad * terrain_rad * terrain_rad * 2.6e3 * 4 / 3 * 3.14,
                                                  terrain_rad, mat_type_terrain);

    // Track the projectile
    auto proj_tracker = DEMSim.Track(projectile);

    float sample_halfheight = world_size / 8;
    float3 sample_center = make_float3(world_size / 2, world_size / 2, sample_halfheight + 0.05);
    float sample_halfwidth = world_size / 2 * 0.95;
    auto input_xyz = DEMBoxHCPSampler(sample_center, make_float3(sample_halfwidth, sample_halfwidth, sample_halfheight),
                                      2.01 * terrain_rad);
    DEMSim.AddClumps(template_terrain, input_xyz);
    std::cout << "Total num of particles: " << input_xyz.size() << std::endl;




    DEMSim.SetInitTimeStep(step_size);
    DEMSim.SetGravitationalAcceleration(make_float3(0, 0, -9.81));
    DEMSim.SetCDUpdateFreq(40);



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