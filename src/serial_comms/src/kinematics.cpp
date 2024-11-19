#include "kinematics.h"

Pose3 FwdKin(Angles t) {
    // pre-compute sines and cosines
    std::array<double, 6> s;
    std::array<double, 6> c;
    for (int i = 0; i < 6; i++) {
        s[i] = sin(t[i]);
        c[i] = cos(t[i]);
    }

    Mat3 R_3rel0{};
    R_3rel0[0][0] =  c[0]*c[1]*c[2] - c[0]*s[1]*s[2];
    R_3rel0[0][1] = -c[0]*c[1]*s[2] - c[0]*s[1]*c[2];
    R_3rel0[0][2] =  s[0];
    R_3rel0[1][0] =  s[0]*c[1]*c[2] - s[0]*s[1]*s[2];
    R_3rel0[1][1] = -s[0]*c[1]*s[2] - s[0]*s[1]*c[2];
    R_3rel0[1][2] = -c[0];
    R_3rel0[2][0] =  s[1]*c[2] + c[1]*s[2];
    R_3rel0[2][1] = -s[1]*s[2] + c[1]*c[2];
    R_3rel0[2][2] =  0.0;

    Mat3 R_6rel3{};
    R_6rel3[0][0] =  c[3]*c[4]*c[5] - s[3]*s[5];
    R_6rel3[0][1] = -c[3]*c[4]*s[5] - s[3]*c[5];
    R_6rel3[0][2] = -c[3]*s[4];
    R_6rel3[1][0] = -s[4]*c[5];
    R_6rel3[1][1] =  s[4]*s[5];
    R_6rel3[1][2] = -c[4];     
    R_6rel3[2][0] =  s[3]*c[4]*c[5] + c[3]*s[5];
    R_6rel3[2][1] = -s[3]*c[4]*s[5] + c[3]*c[5];
    R_6rel3[2][2] = -s[3]*s[4];

    Mat3 dir = R_3rel0.mul(R_6rel3);

    Vec3 pos = Vec3(
        c[0]*c[1]*armdim::A2 + c[0]*armdim::A1 + s[0]*armdim::D2,
        s[0]*c[1]*armdim::A2 + s[0]*armdim::A1 - c[0]*armdim::D2,
        s[1]*armdim::A2
    );

    pos = pos - R_3rel0.getColumn(1)*armdim::D4
              + dir.getColumn(2)*armdim::CLAW_LENGTH
              + Vec3(0, 0, armdim::BASE_HEIGHT);

    return Pose3{pos, dir};
}

// returns servo angles in radians
// if given target is unreachable, servo angles are clamped and input pose is updated
Angles InvKin(Pose3 & pose, Limits limits) {
    Vec3 & pos = pose.pos;
    Mat3 & dir = pose.dir;
    Angles angles;
    
    // array for sines and cosines to avoid recomputing
    std::array<double, 6> s = {};
    std::array<double, 6> c = {};

    // offset final position to offset base height and arm length
    Vec3 neg_z_axis = -dir.getColumn(2);
    Vec3 z_offset = neg_z_axis * armdim::CLAW_LENGTH;

    // desired position relative to frame of link 0
    Vec3 pos0 = pos + z_offset - Vec3(0, 0, armdim::BASE_HEIGHT);
    
    if (pos0.x + armdim::D2 == 0.0) {
        angles[0] = 3 * M_PI_2;
    } else {
        angles[0] = M_PI_2 + 2*atan(   (pos0.y - armdim::REACH_MULTIPLIER*sqrt(sq(pos0.x) + sq(pos0.y) - sq(armdim::D2)))
                                     /                                 (pos0.x + armdim::D2)                              );
    }

    s[0] = sin(angles[0]);
    c[0] = cos(angles[0]);

    double d = pos0.y*s[0] + pos0.x*c[0] - armdim::A1;

    angles[2] = M_PI_2 - armdim::ELBOW_UP_MULTIPLIER*acos(   (sq(d) + sq(pos0.z) - sq(armdim::A2) - sq(armdim::D4))
                                                           /                (2*armdim::A2*armdim::D4)               );
    
    s[2] = sin(angles[2]);
    c[2] = cos(angles[2]);

    if (d == 0.0 && pos0.z == 0.0) {
        // no point in assigning appropriate value to this
        // servo 2 will be out of range well before this point is reached
        // this is when the target is the base of link 2 / at servo 1 (0-index) output
        angles[1] = 0.0;
    } else {
        angles[1] = atan2(pos0.z, d) + asin(      (armdim::D4*c[2])
                                            / sqrt(sq(d) + sq(pos0.z)) );
    }

    s[1] = sin(angles[1]);
    c[1] = cos(angles[1]);

    Mat3 Rinv_4rel0_t3is0{};
    Rinv_4rel0_t3is0[0][0] = c[0]*c[1]*c[2] - c[0]*s[1]*s[2];
    Rinv_4rel0_t3is0[0][1] = s[0];
    Rinv_4rel0_t3is0[0][2] = c[0]*c[1]*c[2] + c[0]*s[1]*c[2];
    Rinv_4rel0_t3is0[1][0] = s[0]*c[1]*c[2] - s[0]*s[1]*s[2];
    Rinv_4rel0_t3is0[1][1] = -c[0];
    Rinv_4rel0_t3is0[1][2] = s[0]*c[1]*c[2] + s[0]*s[1]*c[2];
    Rinv_4rel0_t3is0[2][0] = s[1]*c[2] + c[1]*s[2];
    Rinv_4rel0_t3is0[2][1] = 0.0;
    Rinv_4rel0_t3is0[2][2] = s[1]*s[2] - c[1]*c[2];

    Mat3 R_6rel4_t3is0 = Rinv_4rel0_t3is0.mul(dir);

    // could be + or -, we choose one then if the servo range doesn't work, we switch
    c[4] = R_6rel4_t3is0[2][2];
    angles[4] = acos(c[4]);
    s[4] = sin(angles[4]);

    // if s4 = 0 then axis 3 and 5 are aligned and there are infinite solutions
    // for now, lazy solution is to set a small value of theta4 and proceed
    double orig_angle = angles[4];
    if (orig_angle == 0.0) {
        angles[4] = 0.0001;
    }

    s[3] = -R_6rel4_t3is0[1][2] / s[4];
    c[3] = -R_6rel4_t3is0[0][2] / s[4];
    angles[3] = atan2(s[3], c[3]);

    s[5] = -R_6rel4_t3is0[2][1] / s[4];
    c[5] =  R_6rel4_t3is0[2][0] / s[4];
    angles[5] = atan2(s[5], c[5]);

    if (angles[3] < -M_PI_2 || angles[3] > M_PI_2) {
        angles[4] = -angles[4];
        angles[3] = atan2(-s[3], -c[3]);
        angles[5] = atan2(-s[5], -c[5]);
    }

    // // flip wrist 180 degrees if out of range
    // if (angles[5] < -M_PI_2) {
    //     angles[5] += M_PI;
    // } else if (angles[5] > M_PI_2) {
    //     angles[5] -= M_PI;
    // }

    if (orig_angle == 0.0) {
        angles[4] = 0.0;
    }

    bool adjust = false;

    // clamp angles
    for (int i = 0; i < 6; i++) {
        if (angles[i] < limits[i].first) {
            angles[i] = limits[i].first;
            adjust = true;
        } else if (angles[i] > limits[i].second) {
            angles[i] = limits[i].second;
            adjust = true;
        }
    }

    // fwdkin and modify input pose
    if (adjust) {
        pose = FwdKin(angles);
    }

    return angles;
}