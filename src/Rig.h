#pragma once
#include "AK/Math.h"

#include <string>
#include <vector>

struct Rig
{
    std::vector<std::string> bone_names = {};
    std::vector<i32> parents = {};
    std::vector<AK::xform> ref_pose = {};
    u32 num_bones = 0;

    void local_to_model(std::vector<AK::xform> const& local_pose, std::vector<AK::xform>& model_pose)
    {
        for (u32 i = 0; i < num_bones; ++i)
        {
            i32 const id_parent = parents[i];
            if (id_parent >= 0)
            {
                model_pose[i] = AK::Math::mul_xforms(model_pose[id_parent], local_pose[i]);
            }
            else
            {
                model_pose[i] = local_pose[i];
            }

            // model_pose[i] = AK::Math::mul_xforms(model_pose[i], ref_pose[i]);
        }

        // Copying tracks will be handled here
        // ...
    }
};
