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

bool OutOfRange(double & angle, std::pair<double, double> limit, bool clamp) {
    if (angle < limit.first || angle > limit.second) {
        if (clamp) {
            angle = std::max(limit.first, std::min(limit.second, angle));
        }
        return true;
    }
    return false;
}

// returns servo angles in radians
// if given target is unreachable, servo angles are clamped and input pose is updated
bool InvKin(Pose3 & pose, Limits limits, Angles & angles) {
    Vec3 & pos = pose.pos;
    Mat3 & dir = pose.dir;
    Angles old_angles = angles;
    
    // array for sines and cosines to avoid recomputing
    std::array<double, 6> s = {};
    std::array<double, 6> c = {};

    // offset final position to offset base height and arm length
    Vec3 neg_z_axis = -dir.getColumn(2);
    Vec3 z_offset = neg_z_axis * armdim::CLAW_LENGTH;

    // desired position relative to frame of link 0
    Vec3 pos0 = pos + z_offset - Vec3(0, 0, armdim::BASE_HEIGHT);

    // if reach, -1 is elbow up
    // if balance, -1 is elbow down
    // default as elbow up
    double elbow_multiplier = -1.0;
    
    if (pos0.x + armdim::D2 == 0.0) {
        // lim atan(x) as x -> 0 is pi/2
        angles[0] = 3 * M_PI_2;
    } else {
        // start with reach, change to balance if turret out of range

        double root = sqrt(sq(pos0.x) + sq(pos0.y) - sq(armdim::D2));
        angles[0] = M_PI_2 + 2*atan(      (pos0.y - root)
                                     / (pos0.x + armdim::D2) );

        if (OutOfRange(angles[0], limits[0], false)) {
            angles[0] = M_PI_2 + 2*atan(      (pos0.y + root)
                                        / (pos0.x + armdim::D2) );

            // using balance now, so elbow up default is -1 multiplier
            elbow_multiplier = 1.0;
        }
    }

    s[0] = sin(angles[0]);
    c[0] = cos(angles[0]);

    double d = pos0.y*s[0] + pos0.x*c[0] - armdim::A1;

    double inside =   (sq(d) + sq(pos0.z) - sq(armdim::A2) - sq(armdim::D4))
                    /               (2*armdim::A2*armdim::D4);
    
    // for now, if we try to reach too far, just freeze
    if (abs(inside) > 1.0) {
        angles = old_angles;
        return false;
    }

    // only considering elbow up for now
    // elbow down is only strictly necessary for some cases where
    // the last 3 servos are out of range but is very likely to cause
    // angles[1] to be out of range too
    angles[2] = M_PI_2 + elbow_multiplier*acos(inside);
    
    s[2] = sin(angles[2]);
    c[2] = cos(angles[2]);

    // if d = 0 and z = 0, angles[2] already broke (inside < -1). Don't need to check here.
    // I also don't think it's possible for the inside of asin here to be out of range
    // without angles[2] already breaking
    angles[1] = atan2(pos0.z, d) + asin(      (armdim::D4*c[2])
                                        / sqrt(sq(d) + sq(pos0.z)) );

    s[1] = sin(angles[1]);
    c[1] = cos(angles[1]);

    Mat3 Rinv_4rel0_t3is0{};
    Rinv_4rel0_t3is0[0][0] =  c[0]*c[1]*c[2] - c[0]*s[1]*s[2];
    Rinv_4rel0_t3is0[0][1] =  s[0];
    Rinv_4rel0_t3is0[0][2] =  c[0]*c[1]*s[2] + c[0]*s[1]*c[2];
    Rinv_4rel0_t3is0[1][0] =  s[0]*c[1]*c[2] - s[0]*s[1]*s[2];
    Rinv_4rel0_t3is0[1][1] = -c[0];
    Rinv_4rel0_t3is0[1][2] =  s[0]*c[1]*s[2] + s[0]*s[1]*c[2];
    Rinv_4rel0_t3is0[2][0] =  s[1]*c[2] + c[1]*s[2];
    Rinv_4rel0_t3is0[2][1] =  0.0;
    Rinv_4rel0_t3is0[2][2] =  s[1]*s[2] - c[1]*c[2];

    // transpose of rotation matrix is its inverse
    Mat3 R_6rel4_t3is0 = Rinv_4rel0_t3is0.transpose().mul(dir);

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

    // std::cout << "initial " << angles[3] << " " << angles[4] << " " << angles[5] << std::endl;

    bool adjust = false;
    // test for out of range direction angles
    for (int i = 3; i < 6; i++) {
        if (angles[i] < limits[i].first || angles[i] > limits[i].second) {
            adjust = true;
            break;
        }
    }

    // scuffed direction flip to try to compensate for initial out of bounds
    if (adjust) {
        angles[4] = -angles[4];
        angles[3] = atan2(-s[3], -c[3]);
        angles[5] = atan2(-s[5], -c[5]);
        // std::cout << "adjusting initial " << angles[3] << " " << angles[4] << " " << angles[5] << std::endl;
        adjust = false;
    }

    if (orig_angle == 0.0) {
        angles[4] = 0.0;
    }

    // clamp all angles
    for (int i = 0; i < 6; i++) {
        adjust = adjust || OutOfRange(angles[i], limits[i], false);
    }

    // fwdkin and modify input pose (using original angles, not attempted angle[4] flip)
    if (adjust) {
        angles = old_angles;
        return false;
        // angles[4] = -angles[4];
        // angles[3] = atan2(s[3], c[3]);
        // angles[5] = atan2(s[5], c[5]);
        // std::cout << "adjusting" << std::endl;
        // pose = FwdKin(angles);
    }

    return true;
}