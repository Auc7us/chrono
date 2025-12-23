// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou, Radu Serbanl, Keshav Sharan
// =============================================================================
//
// NASA VIPER Lunar Rover Model Class.
// This class contains model for NASA's VIPER lunar rover for NASA's 2024 Moon
// exploration mission.
//
// =============================================================================
//
// RADU TODO:
// - Recheck kinematics of mechanism (for negative lift angle)
// - Forces and torques are reported relative to the part's centroidal frame.
//   Likely confusing for a user since all bodies are ChBodyAuxRef!
// - Consider using a torque motor instead of driveshafts
//   (for a driver that uses torque control)
//
// =============================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono/functions/ChFunctionSetpoint.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChShaftBodyConstraint.h"
#include "chrono/physics/ChMassProperties.h"

#include "chrono/utils/ChUtils.h"

#include "chrono_models/robot/viper/Viper.h"

namespace chrono {
namespace viper {

// =============================================================================

const double Viper::m_max_steer_angle = 5 * CH_PI / 18;
const double Viper::max_wheel_speed = CH_PI;

// =============================================================================

// Default contact material for rover parts
std::shared_ptr<ChContactMaterial> DefaultContactMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.0f;   // coefficient of restitution
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

// Add a revolute joint between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
void AddRevoluteJoint(std::shared_ptr<ChBody> body1,
                      std::shared_ptr<ChBody> body2,
                      std::shared_ptr<ViperChassis> chassis,
                      const ChVector3d& rel_pos,
                      const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrameRefToAbs() * ChFrame<>(rel_pos, rel_rot);

    // Create joint (DOF about Z axis of X_GC frame)
    auto joint = chrono_types::make_shared<ChLinkLockRevolute>();
    joint->Initialize(body1, body2, ChFrame<>(X_GC.GetPos(), X_GC.GetRot()));
    chassis->GetBody()->GetSystem()->AddLink(joint);
}

// Add a universal joint between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
void AddUniversalJoint(std::shared_ptr<ChBody> body1,
                       std::shared_ptr<ChBody> body2,
                       std::shared_ptr<ViperChassis> chassis,
                       const ChVector3d& rel_pos,
                       const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrameRefToAbs() * ChFrame<>(rel_pos, rel_rot);

    // Create joint (DOFs about X and Y axes of X_GC frame)
    auto joint = chrono_types::make_shared<ChLinkUniversal>();
    joint->Initialize(body1, body2, X_GC);
    chassis->GetBody()->GetSystem()->AddLink(joint);
}

// Add a rotational speed motor between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkMotorRotationSpeed> AddMotorSpeed(std::shared_ptr<ChBody> body1,
                                                        std::shared_ptr<ChBody> body2,
                                                        std::shared_ptr<ViperChassis> chassis,
                                                        const ChVector3d& rel_pos,
                                                        const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrameRefToAbs() * ChFrame<>(rel_pos, rel_rot);

    // Create motor (actuated DOF about Z axis of X_GC frame)
    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(body1, body2, X_GC);
    chassis->GetBody()->GetSystem()->AddLink(motor);

    return motor;
}

// Add a rotational angle motor between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkMotorRotationAngle> AddMotorAngle(std::shared_ptr<ChBody> body1,
                                                        std::shared_ptr<ChBody> body2,
                                                        std::shared_ptr<ViperChassis> chassis,
                                                        const ChVector3d& rel_pos,
                                                        const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrameRefToAbs() * ChFrame<>(rel_pos, rel_rot);

    // Create motor (actuated DOF about Z axis of X_GC frame)
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->Initialize(body1, body2, X_GC);
    chassis->GetBody()->GetSystem()->AddLink(motor);

    return motor;
}

// Add a rotational torque motor between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkMotorRotationTorque> AddMotorTorque(std::shared_ptr<ChBody> body1,
                                                          std::shared_ptr<ChBody> body2,
                                                          std::shared_ptr<ViperChassis> chassis,
                                                          const ChVector3d& rel_pos,
                                                          const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrameRefToAbs() * ChFrame<>(rel_pos, rel_rot);

    // Create motor (actuated DOF about Z axis of X_GC frame)
    auto motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    motor->Initialize(body1, body2, X_GC);
    chassis->GetBody()->GetSystem()->AddLink(motor);

    return motor;
}

// Add a spring between two bodies connected at the specified points
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkTSDA> AddSuspensionSpring(std::shared_ptr<ChBodyAuxRef> body1,
                                                std::shared_ptr<ChBodyAuxRef> body2,
                                                std::shared_ptr<ViperChassis> chassis,
                                                const ChVector3d& pos1,
                                                const ChVector3d& pos2) {
    const ChFrame<>& X_GP = chassis->GetBody()->GetFrameRefToAbs();
    auto p1 = X_GP.TransformPointLocalToParent(pos1);
    auto p2 = X_GP.TransformPointLocalToParent(pos2);

    std::shared_ptr<ChLinkTSDA> spring;
    spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(body1, body2, false, p1, p2);
    spring->SetSpringCoefficient(800000.0);
    spring->SetDampingCoefficient(10000.0);
    chassis->GetBody()->GetSystem()->AddLink(spring);
    return spring;
}

// =============================================================================

// Base class for all Viper Part
ViperPart::ViperPart(const std::string& name,
                     const ChFrame<>& rel_pos,
                     std::shared_ptr<ChContactMaterial> mat,
                     bool collide)
    : m_name(name), m_pos(rel_pos), m_mat(mat), m_collide(collide), m_visualize(true) {}

void ViperPart::Construct(ChSystem* system) {
    m_body = chrono_types::make_shared<ChBodyAuxRef>();
    m_body->SetName(m_name + "_body");
    m_body->SetMass(m_mass);
    m_body->SetInertiaXX(m_inertia);
    m_body->SetFrameCOMToRef(m_cog);

    // Add visualization shape
    if (m_visualize) {
        auto vis_mesh_file = GetChronoDataFile("robot/viper/obj/" + m_mesh_name + ".obj");
        auto trimesh_vis = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, true, true);
        trimesh_vis->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetRotMat());  // translate/rotate/scale mesh
        trimesh_vis->RepairDuplicateVertexes(1e-9);                               // if meshes are not watertight

        auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh_vis);
        trimesh_shape->SetName(m_mesh_name);
        trimesh_shape->SetMutable(false);
        m_body->AddVisualShape(trimesh_shape);
    }

    // Add collision shape
    if (m_collide) {
        auto col_mesh_file = GetChronoDataFile("robot/viper/col/" + m_mesh_name + ".obj");
        auto trimesh_col = ChTriangleMeshConnected::CreateFromWavefrontFile(col_mesh_file, false, false);
        trimesh_col->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetRotMat());  // translate/rotate/scale mesh
        trimesh_col->RepairDuplicateVertexes(1e-9);                               // if meshes are not watertight

        auto shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(m_mat, trimesh_col, false, false, 0.005);
        m_body->AddCollisionShape(shape);
        m_body->EnableCollision(m_collide);
    }

    system->AddBody(m_body);
}

void ViperPart::CalcMassProperties(double density) {
    auto mesh_filename = GetChronoDataFile("robot/viper/col/" + m_mesh_name + ".obj");
    auto trimesh_col = ChTriangleMeshConnected::CreateFromWavefrontFile(mesh_filename, false, false);
    trimesh_col->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetRotMat());  // translate/rotate/scale mesh
    trimesh_col->RepairDuplicateVertexes(1e-9);                               // if meshes are not watertight

    double vol;
    ChVector3d cog_pos;
    ChMatrix33<> cog_rot;
    ChMatrix33<> inertia;
    trimesh_col->ComputeMassProperties(true, vol, cog_pos, inertia);
    ChInertiaUtils::PrincipalInertia(inertia, m_inertia, cog_rot);
    m_mass = density * vol;
    m_inertia *= density;
    m_cog = ChFrame<>(cog_pos, cog_rot);
}

void ViperPart::Initialize(std::shared_ptr<ChBodyAuxRef> chassis) {
    Construct(chassis->GetSystem());

    // Set absolute position
    ChFrame<> X_GC = chassis->GetFrameRefToAbs() * m_pos;
    m_body->SetFrameRefToAbs(X_GC);
}

// =============================================================================

// Rover Chassis
ViperChassis::ViperChassis(const std::string& name, std::shared_ptr<ChContactMaterial> mat)
    : ViperPart(name, ChFrame<>(VNULL, QUNIT), mat, false) {
    m_mesh_name = "viper_chassis";
    m_color = ChColor(1.0f, 1.0f, 1.0f);
    CalcMassProperties(165);
}

void ViperChassis::Initialize(ChSystem* system, const ChFrame<>& pos) {
    Construct(system);

    m_body->SetFrameRefToAbs(pos);
}

// =============================================================================

// Viper Wheel
ViperWheel::ViperWheel(const std::string& name,
                       const ChFrame<>& rel_pos,
                       std::shared_ptr<ChContactMaterial> mat,
                       ViperWheelType wheel_type)
    : ViperPart(name, rel_pos, mat, true) {
    switch (wheel_type) {
        case ViperWheelType::RealWheel:
            m_mesh_name = "viper_wheel";
            break;
        case ViperWheelType::SimpleWheel:
            m_mesh_name = "viper_simplewheel";
            break;
        case ViperWheelType::CylWheel:
            m_mesh_name = "viper_cylwheel";
            break;
    }

    m_color = ChColor(0.4f, 0.7f, 0.4f);
    CalcMassProperties(800);
}

// =============================================================================

// Viper Upper Suspension Arm
ViperUpperArm::ViperUpperArm(const std::string& name,
                             const ChFrame<>& rel_pos,
                             std::shared_ptr<ChContactMaterial> mat,
                             const int& side)
    : ViperPart(name, rel_pos, mat, false) {
    if (side == 0) {
        m_mesh_name = "viper_L_up_sus";
    } else if (side == 1) {
        m_mesh_name = "viper_R_up_sus";
    }

    m_color = ChColor(0.7f, 0.4f, 0.4f);
    CalcMassProperties(2000);
}

// =============================================================================

// Viper Lower Suspension Arm
ViperLowerArm::ViperLowerArm(const std::string& name,
                             const ChFrame<>& rel_pos,
                             std::shared_ptr<ChContactMaterial> mat,
                             const int& side)
    : ViperPart(name, rel_pos, mat, false) {
    if (side == 0) {
        m_mesh_name = "viper_L_bt_sus";
    } else if (side == 1) {
        m_mesh_name = "viper_R_bt_sus";
    }

    m_color = ChColor(0.7f, 0.4f, 0.4f);
    CalcMassProperties(4500);
}

// =============================================================================

// Viper Upright
ViperUpright::ViperUpright(const std::string& name,
                           const ChFrame<>& rel_pos,
                           std::shared_ptr<ChContactMaterial> mat,
                           const int& side)
    : ViperPart(name, rel_pos, mat, false) {
    if (side == 0) {
        m_mesh_name = "viper_L_steer";
    } else if (side == 1) {
        m_mesh_name = "viper_R_steer";
    }

    m_color = ChColor(0.7f, 0.7f, 0.7f);
    CalcMassProperties(4500);
}

// =============================================================================

// Rover model
Viper::Viper(ChSystem* system, ViperWheelType wheel_type) : m_system(system), m_chassis_fixed(false) {
    // Set default collision model envelope commensurate with model dimensions.
    // Note that an SMC system automatically sets envelope to 0.
    auto contact_method = m_system->GetContactMethod();
    if (contact_method == ChContactMethod::NSC) {
        ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);
        ChCollisionModel::SetDefaultSuggestedMargin(0.005);
    }

    // Create the contact materials
    m_default_material = DefaultContactMaterial(contact_method);
    m_wheel_material = DefaultContactMaterial(contact_method);

    Create(wheel_type);
}

void Viper::Create(ViperWheelType wheel_type) {
    // create rover chassis
    m_chassis = chrono_types::make_shared<ViperChassis>("chassis", m_default_material);

    // initilize rover wheels
    double wx = 0.5618 + 0.08;
    double wy = 0.2067 + 0.32 + 0.0831;
    double wz = 0.0;
    m_wheelbase = 2.0 * wx;  // Distance between front and rear axles
    m_track_width = 2.0 * wy; // Distance between left and right wheels

    m_wheels[V_LF] = chrono_types::make_shared<ViperWheel>("wheel_LF", ChFrame<>(ChVector3d(+wx, +wy, wz), QUNIT),
                                                           m_wheel_material, wheel_type);
    m_wheels[V_RF] = chrono_types::make_shared<ViperWheel>("wheel_RF", ChFrame<>(ChVector3d(+wx, -wy, wz), QUNIT),
                                                           m_wheel_material, wheel_type);
    m_wheels[V_LB] = chrono_types::make_shared<ViperWheel>("wheel_LB", ChFrame<>(ChVector3d(-wx, +wy, wz), QUNIT),
                                                           m_wheel_material, wheel_type);
    m_wheels[V_RB] = chrono_types::make_shared<ViperWheel>("wheel_RB", ChFrame<>(ChVector3d(-wx, -wy, wz), QUNIT),
                                                           m_wheel_material, wheel_type);

    m_wheels[V_LF]->m_mesh_xform = ChFrame<>(VNULL, QuatFromAngleZ(CH_PI));
    m_wheels[V_LB]->m_mesh_xform = ChFrame<>(VNULL, QuatFromAngleZ(CH_PI));

    // create rover upper and lower suspension arms
    double cr_lx = 0.5618 + 0.08;
    double cr_ly = 0.2067;  // + 0.32/2;
    double cr_lz = 0.0525;

    ChVector3d cr_rel_pos_lower[] = {
        ChVector3d(+cr_lx, +cr_ly, -cr_lz),  // LF
        ChVector3d(+cr_lx, -cr_ly, -cr_lz),  // RF
        ChVector3d(-cr_lx, +cr_ly, -cr_lz),  // LB
        ChVector3d(-cr_lx, -cr_ly, -cr_lz)   // RB
    };

    ChVector3d cr_rel_pos_upper[] = {
        ChVector3d(+cr_lx, +cr_ly, cr_lz),  // LF
        ChVector3d(+cr_lx, -cr_ly, cr_lz),  // RF
        ChVector3d(-cr_lx, +cr_ly, cr_lz),  // LB
        ChVector3d(-cr_lx, -cr_ly, cr_lz)   // RB
    };

    for (int i = 0; i < 4; i++) {
        m_lower_arms[i] = chrono_types::make_shared<ViperLowerArm>("lower_arm", ChFrame<>(cr_rel_pos_lower[i], QUNIT),
                                                                   m_default_material, i % 2);
        m_upper_arms[i] = chrono_types::make_shared<ViperUpperArm>("upper_arm", ChFrame<>(cr_rel_pos_upper[i], QUNIT),
                                                                   m_default_material, i % 2);
    }

    // create uprights
    double sr_lx = 0.5618 + 0.08;
    double sr_ly = 0.2067 + 0.32 + 0.0831;
    double sr_lz = 0.0;
    ChVector3d sr_rel_pos[] = {
        ChVector3d(+sr_lx, +sr_ly, -sr_lz),  // LF
        ChVector3d(+sr_lx, -sr_ly, -sr_lz),  // RF
        ChVector3d(-sr_lx, +sr_ly, -sr_lz),  // LB
        ChVector3d(-sr_lx, -sr_ly, -sr_lz)   // RB
    };

    for (int i = 0; i < 4; i++) {
        m_uprights[i] = chrono_types::make_shared<ViperUpright>("upright", ChFrame<>(sr_rel_pos[i], QUNIT),
                                                                m_default_material, i % 2);
    }

    // create drive shafts
    for (int i = 0; i < 4; i++) {
        m_drive_shafts[i] = chrono_types::make_shared<ChShaft>();
    }
}

void Viper::Initialize(const ChFrame<>& pos) {
    assert(m_driver);

    m_chassis->Initialize(m_system, pos);
    m_chassis->GetBody()->SetFixed(m_chassis_fixed);

    for (int i = 0; i < 4; i++) {
        m_wheels[i]->Initialize(m_chassis->GetBody());
        m_upper_arms[i]->Initialize(m_chassis->GetBody());
        m_lower_arms[i]->Initialize(m_chassis->GetBody());
        m_uprights[i]->Initialize(m_chassis->GetBody());
    }

    // add all constraints to the system
    // redefine pos data for constraints
    double sr_lx = 0.5618 + 0.08;
    // double sr_ly = 0.2067 + 0.32 + 0.0831;
    // double sr_lz = 0.0;
    double sr_ly_joint = 0.2067 + 0.32;

    double cr_lx = 0.5618 + 0.08;
    double cr_ly = 0.2067;  // + 0.32/2;
    double cr_lz = 0.0525;

    double w_lx = 0.5618 + 0.08;
    double w_ly = 0.2067 + 0.32 + 0.0831;
    double w_lz = 0.0;

    ChVector3d wheel_rel_pos[] = {
        ChVector3d(+w_lx, +w_ly, w_lz),  // LF
        ChVector3d(+w_lx, -w_ly, w_lz),  // RF
        ChVector3d(-w_lx, +w_ly, w_lz),  // LB
        ChVector3d(-w_lx, -w_ly, w_lz)   // RB
    };

    ChVector3d sr_rel_pos_lower[] = {
        ChVector3d(+sr_lx, +sr_ly_joint, -cr_lz),  // LF
        ChVector3d(+sr_lx, -sr_ly_joint, -cr_lz),  // RF
        ChVector3d(-sr_lx, +sr_ly_joint, -cr_lz),  // LB
        ChVector3d(-sr_lx, -sr_ly_joint, -cr_lz)   // RB
    };

    ChVector3d sr_rel_pos_upper[] = {
        ChVector3d(+sr_lx, +sr_ly_joint, cr_lz),  // LF
        ChVector3d(+sr_lx, -sr_ly_joint, cr_lz),  // RF
        ChVector3d(-sr_lx, +sr_ly_joint, cr_lz),  // LB
        ChVector3d(-sr_lx, -sr_ly_joint, cr_lz)   // RB
    };

    ChVector3d cr_rel_pos_lower[] = {
        ChVector3d(+cr_lx, +cr_ly, -cr_lz),  // LF
        ChVector3d(+cr_lx, -cr_ly, -cr_lz),  // RF
        ChVector3d(-cr_lx, +cr_ly, -cr_lz),  // LB
        ChVector3d(-cr_lx, -cr_ly, -cr_lz)   // RB
    };

    ChVector3d cr_rel_pos_upper[] = {
        ChVector3d(+cr_lx, +cr_ly, cr_lz),  // LF
        ChVector3d(+cr_lx, -cr_ly, cr_lz),  // RF
        ChVector3d(-cr_lx, +cr_ly, cr_lz),  // LB
        ChVector3d(-cr_lx, -cr_ly, cr_lz)   // RB
    };

    // Orientation of steer motors.
    // A positive steering input results in positive (left) front wheel steering and negative (right) rear wheel
    // steering.
    ChQuaternion<> sm_rot[] = {
        QUNIT,                  // LF
        QUNIT,                  // RF
        QuatFromAngleX(CH_PI),  // LB
        QuatFromAngleX(CH_PI)   // RB
    };

    // Orientation of lift motors.
    // A positive lifting input results in rasing the chassis relative to the wheels.
    ChQuaternion<> lm_rot[] = {
        QUNIT,                  // LF
        QuatFromAngleX(CH_PI),  // RF
        QUNIT,                  // LB
        QuatFromAngleX(CH_PI)   // RB
    };

    ChQuaternion<> z2x = QuatFromAngleY(CH_PI_2);

    for (int i = 0; i < 4; i++) {
        AddUniversalJoint(m_lower_arms[i]->GetBody(), m_uprights[i]->GetBody(), m_chassis, sr_rel_pos_lower[i], QUNIT);
        AddUniversalJoint(m_upper_arms[i]->GetBody(), m_uprights[i]->GetBody(), m_chassis, sr_rel_pos_upper[i], QUNIT);

        // Add lifting motors at the connecting points between upper_arm & chassis and lower_arm & chassis
        m_lift_motor_funcs[i] = chrono_types::make_shared<ChFunctionConst>(0.0);
        m_lift_motors[i] = AddMotorAngle(m_chassis->GetBody(), m_lower_arms[i]->GetBody(), m_chassis,
                                         cr_rel_pos_lower[i], z2x * lm_rot[i]);
        m_lift_motors[i]->SetMotorFunction(m_lift_motor_funcs[i]);
        AddRevoluteJoint(m_chassis->GetBody(), m_upper_arms[i]->GetBody(), m_chassis, cr_rel_pos_upper[i], z2x);

        auto steer_rod = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 1000, true, false);
        steer_rod->SetPos(m_wheels[i]->GetPos());
        steer_rod->SetFixed(false);
        m_system->Add(steer_rod);

        ChQuaternion<> z2y;
        z2y.SetFromAngleX(CH_PI_2);

        switch (m_driver->GetDriveMotorType()) {
            case ViperDriver::DriveMotorType::SPEED:
                m_drive_motor_funcs[i] = chrono_types::make_shared<ChFunctionSetpoint>();
                m_drive_motors[i] = AddMotorSpeed(steer_rod, m_wheels[i]->GetBody(), m_chassis, wheel_rel_pos[i], z2y);
                m_drive_motors[i]->SetMotorFunction(m_drive_motor_funcs[i]);
                break;
            case ViperDriver::DriveMotorType::TORQUE:
                AddRevoluteJoint(steer_rod, m_wheels[i]->GetBody(), m_chassis, wheel_rel_pos[i], z2y);
                break;
        }

        m_steer_motor_funcs[i] = chrono_types::make_shared<ChFunctionConst>(0.0);
        m_steer_motors[i] = AddMotorAngle(steer_rod, m_uprights[i]->GetBody(), m_chassis, wheel_rel_pos[i], sm_rot[i]);
        m_steer_motors[i]->SetMotorFunction(m_steer_motor_funcs[i]);

        m_springs[i] = AddSuspensionSpring(m_chassis->GetBody(), m_uprights[i]->GetBody(), m_chassis,
                                           cr_rel_pos_upper[i], sr_rel_pos_lower[i]);
    }

    double J = 0.1;  // shaft rotational inertia
    for (int i = 0; i < 4; i++) {
        m_drive_shafts[i]->SetInertia(J);
        m_system->Add(m_drive_shafts[i]);

        // Connect shaft aligned with the wheel's axis of rotation (local wheel Y).
        // Set connection such that a positive torque applied to the shaft results in forward rover motion.
        auto shaftbody_connection = chrono_types::make_shared<ChShaftBodyRotation>();
        shaftbody_connection->Initialize(m_drive_shafts[i], m_wheels[i]->GetBody(), ChVector3d(0, 0, -1));
        m_system->Add(shaftbody_connection);
    }
}

void Viper::SetDriver(std::shared_ptr<ViperDriver> driver) {
    m_driver = driver;
    m_driver->viper = this;
}

void Viper::SetWheelContactMaterial(std::shared_ptr<ChContactMaterial> mat) {
    for (auto& wheel : m_wheels)
        wheel->m_mat = mat;
}

void Viper::SetChassisFixed(bool fixed) {
    m_chassis_fixed = fixed;
}

void Viper::SetChassisVisualization(bool state) {
    m_chassis->SetVisualize(state);
}

void Viper::SetWheelVisualization(bool state) {
    for (auto& wheel : m_wheels)
        wheel->SetVisualize(state);
}

void Viper::SetSuspensionVisualization(bool state) {
    for (auto& p : m_lower_arms)
        p->SetVisualize(state);
    for (auto& p : m_upper_arms)
        p->SetVisualize(state);
    for (auto& p : m_uprights)
        p->SetVisualize(state);
}

ChVector3d Viper::GetWheelContactForce(ViperWheelID id) const {
    return m_wheels[id]->GetBody()->GetContactForce();
}

ChVector3d Viper::GetWheelContactTorque(ViperWheelID id) const {
    return m_wheels[id]->GetBody()->GetContactTorque();
}

ChVector3d Viper::GetWheelAppliedForce(ViperWheelID id) const {
    return m_wheels[id]->GetBody()->GetAppliedForce();
}

ChVector3d Viper::GetWheelAppliedTorque(ViperWheelID id) const {
    return m_wheels[id]->GetBody()->GetAppliedTorque();
}

double Viper::GetWheelTracTorque(ViperWheelID id) const {
    if (m_driver->GetDriveMotorType() == ViperDriver::DriveMotorType::TORQUE)
        return 0;

    return m_drive_motors[id]->GetMotorTorque();
}

double Viper::GetRoverMass() const {
    double tot_mass = m_chassis->GetBody()->GetMass();
    for (int i = 0; i < 4; i++) {
        tot_mass += m_wheels[i]->GetBody()->GetMass();
        tot_mass += m_upper_arms[i]->GetBody()->GetMass();
        tot_mass += m_lower_arms[i]->GetBody()->GetMass();
        tot_mass += m_uprights[i]->GetBody()->GetMass();
    }
    return tot_mass;
}

double Viper::GetWheelMass() const {
    return m_wheels[0]->GetBody()->GetMass();
}

void Viper::Update() {
    double time = m_system->GetChTime();
    m_driver->Update(time);

    for (int i = 0; i < 4; i++) {
        // Extract driver inputs
        double driving = m_driver->drive_speeds[i];
        double steering = m_driver->steer_angles[i];
        double lifting = m_driver->lift_angles[i];

        // Enforce maximum steering angle
        ChClampValue(steering, -m_max_steer_angle, +m_max_steer_angle);

        // Set motor functions
        m_steer_motor_funcs[i]->SetConstant(steering);
        m_lift_motor_funcs[i]->SetConstant(lifting);
        if (m_driver->GetDriveMotorType() == ViperDriver::DriveMotorType::SPEED)
            m_drive_motor_funcs[i]->SetSetpoint(driving, time);
    }
}

// =============================================================================

ViperDriver::ViperDriver()
    : drive_speeds({0, 0, 0, 0}), steer_angles({0, 0, 0, 0}), lift_angles({0, 0, 0, 0}), viper(nullptr) {}

void ViperDriver::SetSteering(double angle) {
    for (int i = 0; i < 4; i++)
        steer_angles[i] = angle;
}

void ViperDriver::SetSteering(double angle, ViperWheelID id) {
    steer_angles[id] = angle;
}

/// Set current lift input angles.
void ViperDriver::SetLifting(double angle) {
    for (int i = 0; i < 4; i++)
        lift_angles[i] = angle;
}

/// DC Motor Control
ViperDCMotorControl::ViperDCMotorControl()
    : m_stall_torque({300, 300, 300, 300}), m_no_load_speed({CH_PI, CH_PI, CH_PI, CH_PI}) {}

void ViperDCMotorControl::Update(double time) {
    double speed_reading;
    double target_torque;
    for (int i = 0; i < 4; i++) {
        speed_reading = -viper->m_drive_shafts[i]->GetPosDt();

        if (speed_reading > m_no_load_speed[i]) {
            target_torque = 0;
        } else if (speed_reading < 0) {
            target_torque = m_stall_torque[i];
        } else {
            target_torque = m_stall_torque[i] * ((m_no_load_speed[i] - speed_reading) / m_no_load_speed[i]);
        }

        viper->m_drive_shafts[i]->SetAppliedLoad(-target_torque);
    }
}

/// Angular Speed Driver
ViperSpeedDriver::ViperSpeedDriver(double time_ramp, double speed) : m_ramp(time_ramp), m_speed(speed) {}

void ViperSpeedDriver::Update(double time) {
    double speed = m_speed;
    if (time < m_ramp)
        speed = m_speed * (time / m_ramp);
    drive_speeds = {speed, speed, speed, speed};
}

/// Waypoint Follower
ViperWaypointFollower::ViperWaypointFollower(double init_target_x, double init_target_y, double init_target_z)
    : m_target_x(init_target_x),
      m_target_y(init_target_y),
      m_target_z(init_target_z),
      m_waypoint_threshold(0.5),
      m_curvature_limit_rad(60.0 * CH_PI / 180.0),
      m_lookahead_distance(0.5),
      m_reverse_heading_threshold(CH_PI / 2.0),
      m_nominal_speed_ratio(0.8),
      m_goal_slowdown_radius(0.25),
      m_reverse_speed_ratio(0.6),
      m_min_speed_ratio(0.1),
      m_reverse_mode(false) {}

void ViperWaypointFollower::SetTarget(double x, double y, double z) {
    m_target_x = x;
    m_target_y = y;
    m_target_z = z;
    m_reverse_mode = false;

    if (!viper) {
        std::cout << "[ViperWaypointFollower] Received target (" << x << ", " << y << ", " << z
                  << ") but rover handle not yet assigned." << std::endl;
        return;
    }

    auto chassis_body = viper->GetChassis()->GetBody();
    std::cout << "[ViperWaypointFollower] Final Target = (" << x << ", " << y << ", " << z << ")"
              << " | Current Pos: " << chassis_body->GetPos()
              << " | Current Vel: " << chassis_body->GetPosDt() << std::endl;

    auto start_pos = viper->GetChassisPos();
    auto yaw = ExtractYaw(viper->GetChassisRot());
    BuildCurvatureLimitedSpline(start_pos, yaw, ChVector3d(m_target_x, m_target_y, start_pos.z()));
}

void ViperWaypointFollower::Update(double time) {
    (void)time;

    if (!viper)
        return;

    ChVector3d current_position = viper->GetChassisPos();
    ChQuaternion<> q = viper->GetChassisRot();
    double yaw = ExtractYaw(q);
    ChVector3d final_target(m_target_x, m_target_y, current_position.z());
    ChVector3d final_delta(final_target.x() - current_position.x(), final_target.y() - current_position.y(), 0);
    double distance_to_target = final_delta.Length();

    if (distance_to_target < m_waypoint_threshold) {
        drive_speeds = {0.0, 0.0, 0.0, 0.0};
        steer_angles = {0.0, 0.0, 0.0, 0.0};
        m_path_points.clear();
        m_path_index = 0;
        m_reverse_mode = false;
        m_angular_velocity = 0.0;
        return;
    }

    if (m_path_points.empty())
        BuildCurvatureLimitedSpline(current_position, yaw, final_target);

    if (m_path_points.empty()) {
        drive_speeds = {0.0, 0.0, 0.0, 0.0};
        steer_angles = {0.0, 0.0, 0.0, 0.0};
        return;
    }

    size_t closest_idx = FindClosestPathIndex(current_position);
    m_path_index = closest_idx;
    size_t lookahead_idx = SelectLookaheadIndex(closest_idx, m_lookahead_distance);
    lookahead_idx = std::min(lookahead_idx, m_path_points.size() - 1);
    ChVector3d lookahead_point = m_path_points[lookahead_idx];
    double actual_lookahead = (lookahead_point - current_position).Length();
    if (actual_lookahead < 1e-4)
        actual_lookahead = m_lookahead_distance;

    bool reverse_flag = false;
    double curvature = ComputePurePursuitCurvature(lookahead_point, current_position, yaw, actual_lookahead, reverse_flag);
    m_reverse_mode = reverse_flag;

    double wheelbase = viper->GetWheelbase();
    double track_width = viper->GetTrackWidth();
    double steer_limit = viper->GetMaxSteerAngle();

    std::array<double, 4> steering_cmd = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> radii = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                                   std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    double center_angle = 0.0;
    double max_radius = MapCurvatureToSteering(curvature, wheelbase, track_width, steer_limit, steering_cmd, radii,
                                               center_angle);

    double base_speed = viper->GetMaxWheelSpeed() * m_nominal_speed_ratio;
    double speed_command =
        ComputeSpeedCommand(center_angle, steer_limit, distance_to_target, base_speed, reverse_flag);

    if (!std::isfinite(max_radius) || max_radius < 1e-6) {
        for (int i = 0; i < 4; i++) {
            drive_speeds[i] = reverse_flag ? -speed_command : speed_command;
            turn_radius[i] = radii[i];
        }
        m_angular_velocity = 0.0;
    } else {
        m_angular_velocity = speed_command / std::max(max_radius, 1e-6);
        for (int i = 0; i < 4; i++) {
            double scaled_speed = speed_command * (radii[i] / max_radius);
            drive_speeds[i] = reverse_flag ? -scaled_speed : scaled_speed;
            turn_radius[i] = radii[i];
        }
    }

    for (int i = 0; i < 4; i++) {
        steer_angles[i] = steering_cmd[i];
    }

    // Throttle verbose logging to avoid spamming stdout and stalling the main loop.
    // Log on index change or every N iterations.
    static size_t last_idx = std::numeric_limits<size_t>::max();
    static int log_decim = 0;
    if (lookahead_idx != last_idx || (log_decim++ % 50) == 0) {
        last_idx = lookahead_idx;
        std::cout << "[ViperWaypointFollower] Pos: " << current_position << " | Lookahead: " << lookahead_point
                  << " | Reverse: " << (reverse_flag ? "true" : "false") << " | idx " << lookahead_idx << " / "
                  << m_path_points.size() << std::endl;
        std::cout << "[ViperWaypointFollower] Curvature: " << curvature
                  << " | Center steer (deg): " << center_angle * 180.0 / CH_PI << " | Speed cmd: " << speed_command
                  << std::endl;
    }
}

size_t ViperWaypointFollower::FindClosestPathIndex(const ChVector3d& position) const {
    if (m_path_points.empty())
        return 0;

    size_t closest = 0;
    ChVector3d delta0 = m_path_points[0] - position;
    double min_dist2 = delta0.Dot(delta0);

    for (size_t i = 1; i < m_path_points.size(); ++i) {
        ChVector3d delta = m_path_points[i] - position;
        double dist2 = delta.Dot(delta);
        if (dist2 < min_dist2) {
            min_dist2 = dist2;
            closest = i;
        }
    }

    return closest;
}

size_t ViperWaypointFollower::SelectLookaheadIndex(size_t closest_idx, double lookahead_distance) const {
    if (m_path_points.empty())
        return 0;

    size_t start_idx = std::min(closest_idx, m_path_points.size() - 1);
    double accumulated = 0.0;
    size_t idx = start_idx;

    for (size_t i = start_idx; i + 1 < m_path_points.size(); ++i) {
        ChVector3d segment = m_path_points[i + 1] - m_path_points[i];
        accumulated += segment.Length();
        idx = i + 1;
        if (accumulated >= lookahead_distance)
            break;
    }

    return idx;
}

double ViperWaypointFollower::ComputePurePursuitCurvature(const ChVector3d& lookahead_point,
                                                          const ChVector3d& rover_pos,
                                                          double rover_yaw,
                                                          double lookahead_distance,
                                                          bool& reverse_mode) const {
    ChVector3d delta = lookahead_point - rover_pos;
    double heading_to_target = std::atan2(delta.y(), delta.x());
    double heading_error = heading_to_target - rover_yaw;
    heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));

    reverse_mode = std::abs(heading_error) > m_reverse_heading_threshold;

    double Ld = std::max(lookahead_distance, 1e-4);
    return 2.0 * std::sin(heading_error) / Ld;
}

double ViperWaypointFollower::MapCurvatureToSteering(double curvature,
                                                     double wheelbase,
                                                     double track_width,
                                                     double steer_limit,
                                                     std::array<double, 4>& out_angles,
                                                     std::array<double, 4>& out_radii,
                                                     double& center_angle) const {
    const double steering_eps = 1e-6;
    center_angle = std::atan(wheelbase * curvature);
    ChClampValue(center_angle, -steer_limit, steer_limit);

    if (std::abs(center_angle) < steering_eps) {
        out_angles.fill(0.0);
        out_radii.fill(std::numeric_limits<double>::infinity());
        return std::numeric_limits<double>::infinity();
    }

    double icr_y = wheelbase / std::tan(center_angle);
    double omega_sign = (center_angle >= 0.0) ? 1.0 : -1.0;
    double half_wheelbase = 0.5 * wheelbase;
    double half_track = 0.5 * track_width;
    std::array<double, 4> wheel_x = {half_wheelbase, half_wheelbase, -half_wheelbase, -half_wheelbase};
    std::array<double, 4> wheel_y = {half_track, -half_track, half_track, -half_track};

    double max_radius = 0.0;

    for (int i = 0; i < 4; ++i) {
        double delta_y = wheel_y[i] - icr_y;
        double radius = std::sqrt(wheel_x[i] * wheel_x[i] + delta_y * delta_y);
        out_radii[i] = radius;
        if (radius > max_radius)
            max_radius = radius;

        double v_x = -omega_sign * delta_y;
        double v_y = omega_sign * wheel_x[i];
        double wheel_angle = std::atan2(v_y, v_x);
        ChClampValue(wheel_angle, -steer_limit, steer_limit);
        out_angles[i] = wheel_angle;
    }

    return max_radius;
}

double ViperWaypointFollower::ComputeSpeedCommand(double center_angle,
                                                  double steer_limit,
                                                  double distance_to_goal,
                                                  double base_speed,
                                                  bool reverse_mode) const {
    double steer_ratio = (steer_limit > 1e-6) ? std::abs(center_angle) / steer_limit : 0.0;
    double speed_scale = 1.0;

    if (steer_ratio <= 0.7) {
        double normalized = steer_ratio / 0.7;
        speed_scale *= (1.0 - 0.3 * normalized);
    } else {
        speed_scale *= 0.4;
    }

    double slowdown_radius = std::max(m_goal_slowdown_radius, 1e-3);
    if (distance_to_goal < slowdown_radius) {
        double normalized_dist = std::clamp(distance_to_goal / slowdown_radius, 0.0, 1.0);
        double slowdown_factor = 0.1 + 0.9 * normalized_dist;
        speed_scale *= slowdown_factor;
    }

    if (reverse_mode)
        speed_scale *= m_reverse_speed_ratio;

    speed_scale = std::clamp(speed_scale, m_min_speed_ratio, 1.0);
    return base_speed * speed_scale;
}

void ViperWaypointFollower::BuildCurvatureLimitedSpline(const ChVector3d& start_pos,
                                                        double start_yaw,
                                                        const ChVector3d& goal_pos) {
    m_path_points.clear();
    m_path_index = 0;

    ChVector3d start_dir(std::cos(start_yaw), std::sin(start_yaw), 0);
    double start_dir_len = std::sqrt(start_dir.x() * start_dir.x() + start_dir.y() * start_dir.y());
    if (start_dir_len < 1e-6) {
        start_dir = ChVector3d(1, 0, 0);
        start_dir_len = 1.0;
    }
    start_dir /= start_dir_len;

    ChVector3d flattened_goal(goal_pos.x(), goal_pos.y(), start_pos.z());
    ChVector3d chord = flattened_goal - start_pos;
    double distance = chord.Length();
    if (distance < 1e-3) {
        m_path_points.push_back(flattened_goal);
        return;
    }

    ChVector3d end_dir(chord.x(), chord.y(), 0);
    double end_dir_len = std::sqrt(end_dir.x() * end_dir.x() + end_dir.y() * end_dir.y());
    if (end_dir_len < 1e-6) {
        end_dir = start_dir;
        end_dir_len = 1.0;
    }
    end_dir /= end_dir_len;

    double control_scale = std::max(0.05, std::min(distance * 0.25, distance));

    std::array<ChVector3d, 4> control_pts = {
        start_pos,
        start_pos + start_dir * control_scale,
        flattened_goal - end_dir * control_scale,
        flattened_goal};

    size_t base_samples =
        static_cast<size_t>(std::max<double>(20.0, std::ceil(distance / std::max(0.1, m_waypoint_threshold * 0.5))));

    m_path_points.reserve(base_samples + 1);
    for (size_t i = 0; i <= base_samples; ++i) {
        double t = static_cast<double>(i) / static_cast<double>(base_samples);
        m_path_points.push_back(EvaluateBezierPoint(control_pts, t));
    }

    EnsureCurvatureLimit(m_path_points);
}

void ViperWaypointFollower::EnsureCurvatureLimit(std::vector<ChVector3d>& samples) {
    if (samples.size() < 3)
        return;

    bool inserted = true;
    int guard = 0;
    const int guard_limit = 200;

    while (inserted && guard < guard_limit) {
        inserted = false;
        for (size_t i = 1; i + 1 < samples.size(); ++i) {
            ChVector3d prev = samples[i] - samples[i - 1];
            ChVector3d next = samples[i + 1] - samples[i];

            double prev_len = prev.Length();
            double next_len = next.Length();

            if (prev_len < 1e-6 || next_len < 1e-6)
                continue;

            ChVector3d prev_n = prev / prev_len;
            ChVector3d next_n = next / next_len;

            double dot = prev_n.Dot(next_n);
            ChClampValue(dot, -1.0, 1.0);
            double heading_change = std::acos(dot);
            if (heading_change > m_curvature_limit_rad) {
                ChVector3d mid = 0.5 * (samples[i] + samples[i + 1]);
                samples.insert(samples.begin() + i + 1, mid);
                inserted = true;
                break;
            }
        }
        ++guard;
    }
}

ChVector3d ViperWaypointFollower::EvaluateBezierPoint(const std::array<ChVector3d, 4>& control_pts, double t) const {
    double u = 1.0 - t;
    double b0 = u * u * u;
    double b1 = 3 * u * u * t;
    double b2 = 3 * u * t * t;
    double b3 = t * t * t;

    return control_pts[0] * b0 + control_pts[1] * b1 + control_pts[2] * b2 + control_pts[3] * b3;
}

double ViperWaypointFollower::ExtractYaw(const ChQuaternion<>& q) const {
    return std::atan2(2.0 * (q.e0() * q.e3() + q.e1() * q.e2()),
                      1.0 - 2.0 * (q.e2() * q.e2() + q.e3() * q.e3()));
}

std::vector<ChVector3d> ViperWaypointFollower::GetPathPoints() const {
    return m_path_points;
}

}  // namespace viper
}  // namespace chrono

