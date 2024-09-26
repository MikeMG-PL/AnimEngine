#pragma once

#include "AK/Math.h"
#include "assimp/Importer.hpp"
#include "assimp/anim.h"
#include "glm/gtx/quaternion.hpp"

#include <map>
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

struct BoneInfo
{
    i32 id = 0;
    glm::mat4 offset = glm::mat4(1.0f);
};

struct KeyPosition
{
    glm::vec3 position = {0.0f, 0.0f, 0.0f};
    float time_stamp = 0.0f;
};

struct KeyRotation
{
    glm::quat orientation = {1.0f, 0.0f, 0.0f, 0.0f};
    float time_stamp = 0.0f;
};

struct Bone
{
    std::vector<KeyPosition> positions = {};
    std::vector<KeyRotation> rotations = {};

    u32 num_positions = 0;
    u32 num_rotations = 0;

    glm::mat4 local_transform = glm::mat4(1.0f);
    std::string name = "";
    i32 id = -1;

    void init(std::string const& name, i32 id, aiNodeAnim const* channel)
    {
        this->name = name;
        this->id = id;
        num_positions = channel->mNumPositionKeys;

        for (int position_index = 0; position_index < num_positions; ++position_index)
        {
            aiVector3D ai_position = channel->mPositionKeys[position_index].mValue;
            float const time_stamp = channel->mPositionKeys[position_index].mTime;
            KeyPosition data;
            data.position = {ai_position.x, ai_position.y, ai_position.z};
            data.time_stamp = time_stamp;
            positions.push_back(data);
        }

        num_rotations = channel->mNumRotationKeys;

        for (int rotation_index = 0; rotation_index < num_rotations; ++rotation_index)
        {
            aiQuaternion ai_orientation = channel->mRotationKeys[rotation_index].mValue;
            float const time_stamp = channel->mRotationKeys[rotation_index].mTime;
            KeyRotation data;
            data.orientation = {ai_orientation.w, ai_orientation.x, ai_orientation.y, ai_orientation.z};
            data.time_stamp = time_stamp;
            rotations.push_back(data);
        }
    }

    void update(float animation_time)
    {
        glm::mat4 const translation = test_position();
        glm::mat4 const rotation = test_rotation();
        local_transform = translation * rotation * glm::mat4(1.0f); // skalujesz zalujesz
    }

    glm::mat4 test_position()
    {
        return glm::translate(glm::mat4(1.0f), positions[0].position);
    }

    glm::mat4 test_rotation()
    {
        auto const rotation = glm::normalize(rotations[0].orientation);
        return glm::toMat4(rotation);
    }
};

struct AssimpNodeData
{
    glm::mat4 transformation = glm::mat4(1.0f);
    std::string name = "";
    u32 children_count = 0;
    std::vector<AssimpNodeData> children = {};
};

struct Animation
{
    float duration = 0.0f;
    float ticks_per_second = 0.0f;
    std::vector<Bone> bones = {};
    AssimpNodeData root_node = {};
    std::map<std::string, BoneInfo> bone_info_map = {};
};
