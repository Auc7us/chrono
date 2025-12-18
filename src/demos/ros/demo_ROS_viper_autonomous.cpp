// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Keshav Sharan
// =============================================================================
//
// Demo showing Viper Rover on SCM Terrain with obstacles, sensors, and ROS integration.
//
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include <cmath>
#include <cstdint>
#include <array>
#include <fstream>
#include <chrono>
#include <limits>
#include <stdexcept>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/input_output/ChUtilsInputOutput.h"
#include "chrono/physics/ChMassProperties.h"

#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/filters/ChFilterRadarXYZReturn.h"
#include "chrono_sensor/filters/ChFilterRadarXYZVisualize.h"
#include "chrono_sensor/optix/ChOptixDefinitions.h"  // PointLight

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler.h"
#include "chrono_ros/handlers/robot/viper/ChROSViperSpeedDriverHandler.h"
#include "chrono_ros/handlers/robot/viper/ChROSViperWaypointFollowerHandler.h"
#include "chrono_ros/handlers/robot/viper/ChROSViperWaypointPathHandler.h"
#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"
// #include "chrono_thirdparty/filesystem/path.h"

#include "chrono/assets/ChVisualSystem.h"
// #include "demos/SetChronoSolver.h

#ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
#include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::viper;
using namespace chrono::sensor;
using namespace chrono::ros;

using namespace irr;

// Disable visualization for headless execution.
ChVisualSystem::Type vis_type = ChVisualSystem::Type::NONE;
double mesh_resolution = 0.02;
bool enable_bulldozing = false; // Enable/disable bulldozing effects
bool enable_moving_patch = true; // Enable/disable moving patch feature
bool var_params = true; // If true, use provided callback to change soil properties based on location
ViperWheelType wheel_type = ViperWheelType::RealWheel; // Define Viper rover wheel type

class MySoilParams : public vehicle::SCMTerrain::SoilParametersCallback {
  public:
    virtual void Set(const ChVector3d& loc,
                     double& Bekker_Kphi,
                     double& Bekker_Kc,
                     double& Bekker_n,
                     double& Mohr_cohesion,
                     double& Mohr_friction,
                     double& Janosi_shear,
                     double& elastic_K,
                     double& damping_R) override {
        Bekker_Kphi = 0.82e6;
        Bekker_Kc = 0.14e4;
        Bekker_n = 1.0;
        Mohr_cohesion = 0.017e4;
        Mohr_friction = 35.0;
        Janosi_shear = 1.78e-2;
        elastic_K = 2e8;
        damping_R = 3e4;
    }
};

// Use custom material for the Viper Wheel
bool use_custom_mat = false;

void InitializeApolloTerrain(vehicle::SCMTerrain& terrain, double mesh_resolution) {
    const std::string bmp_file = GetChronoDataFile("robot/viper/terrain/nasa_apollo_site.bmp");

    // Uniform scaling: enforce 50 m terrain length (BMP x-axis span).
    constexpr double kDesiredTerrainLength = 200.0;
    constexpr double kHeightMin = -10.0;
    constexpr double kHeightMax = 10.0;

    std::ifstream bmp_stream(bmp_file, std::ios::binary);
    if (!bmp_stream.is_open()) {
        throw std::runtime_error("Failed to open BMP file: " + bmp_file);
    }

    bmp_stream.seekg(18);
    int32_t bmp_width = 0;
    int32_t bmp_height = 0;
    bmp_stream.read(reinterpret_cast<char*>(&bmp_width), sizeof(bmp_width));
    bmp_stream.read(reinterpret_cast<char*>(&bmp_height), sizeof(bmp_height));
    if (bmp_width <= 0 || bmp_height == 0) {
        throw std::runtime_error("Invalid BMP dimensions in file: " + bmp_file);
    }

    const double meters_per_pixel = kDesiredTerrainLength / static_cast<double>(bmp_width);
    const double terrain_length = kDesiredTerrainLength;
    const double terrain_width = std::abs(static_cast<double>(bmp_height)) * meters_per_pixel;

    terrain.Initialize(bmp_file, terrain_length, terrain_width, kHeightMin, kHeightMax, mesh_resolution);
}

std::shared_ptr<ChContactMaterial> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.1f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChContactMaterial>();
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;
    
    // Simulation Loop Time
    double time_step = 5e-4;

    // Global parameter for moving patch size:
    double wheel_range = 0.5;

    // Create a Chrono physical system and associated collision system
    ChSystemNSC sys;

    // // Set solver and integrator
    // double step_size = 2e-3;
    // auto solver_type = ChSolver::Type::BARZILAIBORWEIN;
    // auto integrator_type = ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED;
    // SetChronoSolver(*sys, solver_type, integrator_type);
    int num_threads_chrono = std::min(4, ChOMP::GetNumProcs());
    int num_threads_collision = std::min(3, ChOMP::GetNumProcs()-num_threads_chrono-1);
    int num_threads_eigen = std::min(2, ChOMP::GetNumProcs()-(num_threads_chrono+num_threads_collision));
    sys.SetNumThreads(num_threads_chrono, num_threads_collision, num_threads_eigen);

    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -1.62));

    // Toggle ROS to isolate performance; when disabled, skip all ROS handlers.
    bool enable_ros = true;
    std::shared_ptr<ChROSManager> ros_manager;
    if (enable_ros) {
        ros_manager = chrono_types::make_shared<ChROSManager>();
        auto clock_handler = chrono_types::make_shared<ChROSClockHandler>();
        ros_manager->RegisterHandler(clock_handler);
    }

    double initial_target_x = -5.0;
    double initial_target_y =  0.0;
    double initial_target_z =  0.0;
    auto driver = chrono_types::make_shared<ViperWaypointFollower>(initial_target_x, initial_target_y, initial_target_z);

    Viper viper(&sys, wheel_type);
    // Viper viper(&sys, ViperWheelType::RealWheel);
    viper.SetDriver(driver);
    if (use_custom_mat) {
        viper.SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
    }

    //
    // THE DEFORMABLE TERRAIN
    //
    vehicle::SCMTerrain terrain(&sys);
    // Position the SCM reference frame so the nominal surface origin is at (17, 0, -3).
    terrain.SetReferenceFrame(ChCoordsys<>(ChVector3d(17, 0, -3)));
    InitializeApolloTerrain(terrain, mesh_resolution);

    // Spawn the rover relative to the actual terrain height to avoid large drop impacts.
    const ChVector3d start_xy(-5.0, 0.0, 0.0);
    const double start_clearance = 0.25;  // small lift above surface
    const double terrain_height = terrain.GetHeight(start_xy);
    const ChFrame<> start_pose(ChVector3d(start_xy.x(), start_xy.y(), terrain_height + start_clearance), QUNIT);
    viper.Initialize(start_pose);

    // Get wheels and bodies to set up SCM patches (after Viper constructed)
    auto Wheel_1 = viper.GetWheel(ViperWheelID::V_LF)->GetBody();
    auto Wheel_2 = viper.GetWheel(ViperWheelID::V_RF)->GetBody();
    auto Wheel_3 = viper.GetWheel(ViperWheelID::V_LB)->GetBody();
    auto Wheel_4 = viper.GetWheel(ViperWheelID::V_RB)->GetBody();
    auto Body_1 = viper.GetChassis()->GetBody();


    if (enable_ros) {
        // Create a subscriber to receive ROS motor commands
        auto driver_inputs_rate = 25;
        auto driver_inputs_topic_name = "~/input/driver_waypoint_update";
        auto driver_inputs_handler =
            chrono_types::make_shared<ChROSViperWaypointFollowerHandler>(driver_inputs_rate, driver, driver_inputs_topic_name);
        ros_manager->RegisterHandler(driver_inputs_handler);

        auto path_topic_name = "~/output/rover/waypoint_path";
        auto path_publish_rate = 10;
        auto waypoint_path_handler =
            chrono_types::make_shared<ChROSViperWaypointPathHandler>(path_publish_rate, driver, path_topic_name);
        ros_manager->RegisterHandler(waypoint_path_handler);

        // Create a publisher for the rover state
        auto rover_state_rate = 25;
        auto rover_state_topic_name = "~/output/rover/state";
        auto rover_state_handler = chrono_types::make_shared<ChROSBodyHandler>(
            rover_state_rate, viper.GetChassis()->GetBody(), rover_state_topic_name);
        ros_manager->RegisterHandler(rover_state_handler);
    }

    std::cout << "Initial velocity before sim loop: " << viper.GetChassis()->GetBody()->GetPosDt() << std::endl;

    // Obstacles
    std::vector<std::shared_ptr<ChBodyAuxRef>> rocks;
    std::shared_ptr<ChContactMaterial> rockSurfaceMaterial = ChContactMaterial::DefaultMaterial(sys.GetContactMethod());

    // Rock material
    auto rock_vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    rock_vis_mat->SetAmbientColor({1,1,1}); //0.65f,0.65f,0.65f
    rock_vis_mat->SetDiffuseColor({1,1,1});
    rock_vis_mat->SetSpecularColor({1,1,1});
    rock_vis_mat->SetUseSpecularWorkflow(true);
    rock_vis_mat->SetRoughness(1.0f);
    rock_vis_mat->SetUseHapke(true);
    rock_vis_mat->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f,23.4f*(CH_PI/180));

    // Rocks' Predefined Positions (XY fixed; Z sampled from terrain), spread along the 200x50 terrain
    // Reduced to ~25 rocks, clustered in small groups of 1-3 while remaining scattered across the domain.
    std::vector<ChVector3d> rock_positions = {
        {12.0,  -6.0, 0.0}, { 5.5,  -5.0, 0.0}, { 2.5,   0.0, 0.0},
        { 8.0,   8.0, 0.0}, {10.0,   9.5, 0.0}, {12.0,   5.5, 0.0}, 
        {20.0,  -5.0, 0.0}, {22.5,  -4.5, 0.0},                  
        {30.0,   9.0, 0.0}, {32.0,   7.0, 0.0}, {33.5,   8.5, 0.0},
        {40.0,  -8.5, 0.0}, {42.0,  -5.5, 0.0},                  
        {50.0,   5.0, 0.0}, {52.5,   6.0, 0.0}, {54.0,   5.5, 0.0}, 
        {60.0,  -5.0, 0.0}, {62.0,  -4.5, 0.0},                   
        {70.0,   9.5, 0.0}, {72.0,   8.0, 0.0}, {74.0,  10.5, 0.0},
        {82.0,  -9.0, 0.0}, {84.0,  -5.0, 0.0},                    
        {92.0,   5.0, 0.0}, {94.0,   8.5, 0.0}                     
    };

    // Fixed, repeatable rock scales (base factors; multiplied below). Values chosen so final scales (with *2.5) match prior sizes.
    std::array<double, 25> rock_scales = {
        0.20, 0.30, 0.25, 0.26, 0.65,
        0.40, 0.70, 0.18, 0.30, 0.50,
        0.65, 0.26, 0.40, 0.50, 0.20,
        0.30, 0.50, 0.18, 0.26, 0.65,
        0.40, 0.20, 0.30, 0.50, 0.70};

    // Place rocks at their predefined positions (no terrain height adjustment)
    for (int i = 0; i < static_cast<int>(rock_positions.size()); i++) {
        std::string rock_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock" + std::to_string(i % 3 + 1) + ".obj");

        auto rock_mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(rock_obj_path, false, true);
        double scale_ratio = rock_scales[i] * 2.5;
        rock_mesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));
        rock_mesh->RepairDuplicateVertexes(1e-9);

        // compute mass inertia from mesh
        double mmass;
        ChVector3d mcog;
        ChMatrix33<> minertia;
        double mdensity = 8000;  // paramsH->bodyDensity;
        rock_mesh->ComputeMassProperties(true, mmass, mcog, minertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector3d principal_I;
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto rock_body = chrono_types::make_shared<ChBodyAuxRef>();
        ChQuaternion<> rock_rot = QuatFromAngleX(CH_PI / 2);

        rock_body->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

        rock_body->SetMass(mmass * mdensity);
        rock_body->SetInertiaXX(mdensity * principal_I);

        // Drop rocks from a higher initial lift so they fall visibly once (computed once at startup).
        const double rock_lift = 0.5;  // 25 cm above local terrain
        const double rock_surface_z = terrain.GetHeight(rock_positions[i]) + rock_lift;
        ChVector3d rock_pos(rock_positions[i].x(), rock_positions[i].y(), rock_surface_z);
        rock_body->SetFrameRefToAbs(ChFrame<>(rock_pos, ChQuaternion<>(rock_rot)));
        sys.Add(rock_body);

        rock_body->SetFixed(false);

        auto rock_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(rockSurfaceMaterial, rock_mesh, false, false, 0.005);
        rock_body->AddCollisionShape(rock_shape);
        rock_body->EnableCollision(true);

        auto rock_vis_mesh = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        rock_vis_mesh->SetMesh(rock_mesh);
        rock_vis_mesh->SetBackfaceCull(true);

        if(rock_vis_mesh->GetNumMaterials() == 0){
            rock_vis_mesh->AddMaterial(rock_vis_mat);
        }
        else{
            rock_vis_mesh->GetMaterials()[0] = rock_vis_mat;
        }

        rock_body->AddVisualShape(rock_vis_mesh);

        // sys.Add(rock_body);
        rocks.push_back(rock_body);
    }
    auto lunar_material = chrono_types::make_shared<ChVisualMaterial>();
    lunar_material->SetAmbientColor({0.0, 0.0, 0.0}); //0.65f,0.65f,0.65f
    lunar_material->SetDiffuseColor({0.7, 0.7, 0.7});
    lunar_material->SetSpecularColor({1.0, 1.0, 1.0});
    lunar_material->SetUseSpecularWorkflow(true);
    lunar_material->SetRoughness(0.8f);
    lunar_material->SetAnisotropy(1.f);
    lunar_material->SetUseHapke(true);
    lunar_material->SetHapkeParameters(0.32357f, 0.23955f, 0.30452f, 1.80238f, 0.07145f, 0.3f,23.4f*(CH_PI/180));
    lunar_material->SetClassID(30000);
    lunar_material->SetInstanceID(20000);
    auto mesh = terrain.GetMesh();

    {
        if(mesh->GetNumMaterials() == 0){
            mesh->AddMaterial(lunar_material);
        }
        else{
            mesh->GetMaterials()[0] = lunar_material;
        }
    }
    // Set the soil terramechanical parameters
    if (var_params) {
        // Here we use the soil callback defined at the beginning of the code
        auto my_params = chrono_types::make_shared<MySoilParams>();
        terrain.RegisterSoilParametersCallback(my_params);
    } else {
        // If var_params is set to be false, these parameters will be used
        terrain.SetSoilParameters(0.2e6,  // Bekker Kphi
                                  0,      // Bekker Kc
                                  1.1,    // Bekker n exponent
                                  0,      // Mohr cohesive limit (Pa)
                                  30,     // Mohr friction limit (degrees)
                                  0.01,   // Janosi shear coefficient (m)
                                  4e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                                  3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
        );
    }

    // Set up bulldozing factors
    if (enable_bulldozing) {
        terrain.EnableBulldozing(true);  // inflate soil at the border of the rut
        terrain.SetBulldozingParameters(
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    }

    // Add active domains around wheels/rocks (API supported in this Chrono version)
    if (enable_moving_patch) {
        double wheel_range = 0.5;
        terrain.AddActiveDomain(Wheel_1, ChVector3d(0, 0, 0), ChVector3d(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddActiveDomain(Wheel_2, ChVector3d(0, 0, 0), ChVector3d(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddActiveDomain(Wheel_3, ChVector3d(0, 0, 0), ChVector3d(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddActiveDomain(Wheel_4, ChVector3d(0, 0, 0), ChVector3d(0.5, 2 * wheel_range, 2 * wheel_range));

        for (int i = 0; i < static_cast<int>(rock_positions.size()); i++) {
            terrain.AddActiveDomain(rocks[i], ChVector3d(0, 0, 0), ChVector3d(0.5, 0.5, 0.5));
        }
    }

    // Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
    terrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 20000);
    terrain.SetMeshWireframe(true);

#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    if (vis_type != ChVisualSystem::Type::NONE) {
        switch (vis_type) {
            case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
                auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
                vis_irr->AttachSystem(&sys);
                vis_irr->SetCameraVertical(CameraVerticalDir::Z);
                vis_irr->SetWindowSize(800, 600);
                vis_irr->SetWindowTitle("Viper Rover on SCM");
                vis_irr->Initialize();
                vis_irr->AddLogo();
                vis_irr->AddSkyBox();
                vis_irr->AddCamera(ChVector3d(1.0, 2.0, 1.4), ChVector3d(0, 0, wheel_range));
                vis_irr->AddTypicalLights();
                vis_irr->AddLightWithShadow(ChVector3d(-5.0, -0.5, 8.0), ChVector3d(-1, 0, 0), 100, 1, 35, 85, 512,
                                            ChColor(0.8f, 0.8f, 0.8f));
                vis_irr->EnableShadows();

                vis = vis_irr;
#endif
                break;
            }
            default:
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
                vis_vsg->AttachSystem(&sys);
                vis_vsg->SetWindowSize(800, 600);
                vis_vsg->SetWindowTitle("Viper Rover on SCM");
                vis_vsg->AddCamera(ChVector3d(1.0, 2.0, 1.4), ChVector3d(0, 0, wheel_range));
                vis_vsg->Initialize();

                vis = vis_vsg;
#endif
                break;
            }
        }
    }

    //
    // SENSOR SIMULATION
    // 

    // Sensor Manager
    auto manager = chrono_types::make_shared<ChSensorManager>(&sys);
    // Restore a gentle ambient term so the scene isn’t fully black away from the headlamp.
    manager->scene->SetAmbientLight({0.0f, 0.0f, 0.0f});
    // Softer global fill.
    manager->scene->AddPointLight({-10.f, 0.f, 50.f}, {1.f, 1.f, 1.f}, 200.f);
    // Headlamp-style rover light, positioned near the stereo rig (slightly below the camera).
    // Moderate headlamp brightness; warm tint with noticeable falloff.
    const float headlamp_range = 20.f;
    const auto headlamp_color = make_float3(2.0f, 1.8f, 1.6f);
    unsigned int rover_light_id = manager->scene->AddPointLight({0.f, 0.f, 0.f}, {headlamp_color.x, headlamp_color.y, headlamp_color.z}, headlamp_range);
    manager->SetVerbose(false);
    Background b;
    // Restore HDR environment map background.
    b.mode = BackgroundMode::ENVIRONMENT_MAP;
    b.env_tex = GetChronoDataFile("sensor/textures/starmap_2020_4k.hdr");
    manager->scene->SetBackground(b);

    // Lidar Sensor
    auto offset_pose = ChFrame<>(ChVector3d(1.5, 0, 0.4), QuatFromAngleZ(0));
    auto offset_pose_stereo_L = ChFrame<>(ChVector3d(1.5,  0.2, 0.4), QuatFromAngleZ(0)); // y +ve is left 
    auto offset_pose_stereo_R = ChFrame<>(ChVector3d(1.5, -0.2, 0.4), QuatFromAngleZ(0));

    // Camera Sensor
    int camera_update_rate = 25;
    int camera_image_width = 960;
    int camera_image_height = 480;
    float camera_fov = (float)CH_PI / 3;

    // Wide observer camera (global view)
    int observer_update_rate = 25;
    int observer_image_width = 1280;
    int observer_image_height = 720;
    float observer_fov = (float)CH_PI / 2;  // wide FOV
    auto observer_pose = ChFrame<>(ChVector3d(-6.0, 0.0, 3.0), QuatFromAngleZ(0));

    auto stereo_L = chrono_types::make_shared<ChCameraSensor>(viper.GetChassis()->GetBody(), // body lidar is attached to
                                                         camera_update_rate,                            // scanning rate in Hz
                                                         offset_pose_stereo_L,                   // offset pose
                                                         camera_image_width,                           // image width
                                                         camera_image_height,                           // image height
                                                         camera_fov);                    // FOV
    stereo_L->SetName("Camera Sensor L");
    stereo_L->SetLag(0.f);
    stereo_L->SetCollectionWindow(0.02f);                                                        
    // No visualization for stereo; ROS only.
    if (enable_ros) {
        stereo_L->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
        auto stereo_L_handler = chrono_types::make_shared<ChROSCameraHandler>(
            camera_update_rate,  // Publish rate
            stereo_L,                   // Camera sensor
            "~/stereo/left"             // ROS topic name
        );
        ros_manager->RegisterHandler(stereo_L_handler);
    }
    manager->AddSensor(stereo_L);
    
    
    auto stereo_R = chrono_types::make_shared<ChCameraSensor>(viper.GetChassis()->GetBody(), // body lidar is attached to
                                                         camera_update_rate,                            // scanning rate in Hz
                                                         offset_pose_stereo_R,                   // offset pose
                                                         camera_image_width,                           // image width
                                                         camera_image_height,                           // image height
                                                         camera_fov);                    // FOV
    stereo_R->SetName("Camera Sensor");
    stereo_R->SetLag(0.f);
    stereo_R->SetCollectionWindow(0.02f);                                                        
    // No visualization for stereo; ROS only.
    if (enable_ros) {
        stereo_R->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
        auto stereo_R_handler = chrono_types::make_shared<ChROSCameraHandler>(
            camera_update_rate,
            stereo_R,
            "~/stereo/right"
        );
        ros_manager->RegisterHandler(stereo_R_handler);
    }
    manager->AddSensor(stereo_R);

    // Add observer camera sensor for a full-scene view
    auto observer_cam = chrono_types::make_shared<ChCameraSensor>(
        viper.GetChassis()->GetBody(),  // mount to chassis for relative tracking
        observer_update_rate,
        observer_pose,
        observer_image_width,
        observer_image_height,
        observer_fov);
    observer_cam->SetName("Observer Camera");
    observer_cam->SetLag(0.f);
    observer_cam->SetCollectionWindow(0.02f);
    observer_cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(observer_image_width, observer_image_height, "Observer View"));
    if (enable_ros) {
        observer_cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
        auto observer_handler = chrono_types::make_shared<ChROSCameraHandler>(
            observer_update_rate,
            observer_cam,
            "~/stereo/observer");
        ros_manager->RegisterHandler(observer_handler);
    }
    manager->AddSensor(observer_cam);


    // Finally, initialize the ros manager
    if (enable_ros) {
        ros_manager->Initialize();
    }

    // Run the simulation headless; keep a finite horizon to avoid infinite loops.
    const double sim_end_time = 900.0;  // seconds
    auto wall_start = std::chrono::steady_clock::now();
    double last_rtf_print_wall = 0.0;
    double last_rtf_print_sim = 0.0;
    const double rtf_print_interval_wall = 1.0;  // seconds (wall)
    while (sys.GetChTime() < sim_end_time) {
        if (vis) {
#if defined(CHRONO_IRRLICHT) || defined(CHRONO_VSG)
            vis->BeginScene();
            vis->SetCameraTarget(Body_1->GetPos());
            vis->Render();
            vis->EndScene();
#endif
        }

        // Update ROS interfaces (clock, rover state, camera topics, etc.)
        if (enable_ros) {
            if (!ros_manager->Update(sys.GetChTime(), time_step)) {
                break;
            }
        }

        // Advance all sensors; each camera decides internally when to render
        // based on its own update rate and the current simulation time.
        // Keep the rover-mounted light attached to the chassis (position only).
        {
            PointLight pl{};
            const ChVector3d light_offset(1.5, 0.0, 0.25);  // near stereo cameras, slightly lower
            ChVector3d light_pos = viper.GetChassis()->GetBody()->GetPos() + light_offset;
            pl.pos = make_float3((float)light_pos.x(), (float)light_pos.y(), (float)light_pos.z());
            pl.color = headlamp_color;
            pl.max_range = headlamp_range;
            manager->scene->ModifyPointLight(rover_light_id, pl);
        }
        manager->Update();

        // Advance the rover and dynamics.
        viper.Update();
        sys.DoStepDynamics(time_step);

        // Report real-time factor (RTF) as a slowdown multiplier: wall/sim (so 2.0 means 2x slower than real-time).
        auto now = std::chrono::steady_clock::now();
        double wall_elapsed = std::chrono::duration<double>(now - wall_start).count();
        if (wall_elapsed - last_rtf_print_wall >= rtf_print_interval_wall) {
            double sim_t = sys.GetChTime();
            double dt_wall = wall_elapsed - last_rtf_print_wall;
            double dt_sim = sim_t - last_rtf_print_sim;
            double inst_rtf = dt_wall > 0.0 ? dt_sim / dt_wall : 0.0;
            double inst_slowdown = inst_rtf > 0.0 ? 1.0 / inst_rtf : std::numeric_limits<double>::infinity();
            double avg_rtf = wall_elapsed > 0.0 ? sim_t / wall_elapsed : 0.0;
            double avg_slowdown = avg_rtf > 0.0 ? 1.0 / avg_rtf : std::numeric_limits<double>::infinity();
            std::cout << "[RTF] sim=" << sim_t << "s wall=" << wall_elapsed
                      << "s inst_rtf=" << inst_rtf << " inst_slowdown=" << inst_slowdown
                      << "x avg_slowdown=" << avg_slowdown << "x" << std::endl;
            last_rtf_print_wall = wall_elapsed;
            last_rtf_print_sim = sim_t;
        }
    }

    return 0;
}
