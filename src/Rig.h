#pragma once

#include "AK/Math.h"
#include "Globals.h"
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
    glm::mat4 model_transform = glm::mat4(1.0f);
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
        glm::mat4 const translation = interpolate_position(animation_time);
        glm::mat4 const rotation = interpolate_rotation(animation_time);
        local_transform = translation * rotation * glm::mat4(1.0f); // skalujesz zalujesz
    }

    u32 get_position_index(float animation_time)
    {
        for (int index = 0; index < num_positions - 1; ++index)
        {
            if (animation_time < positions[index + 1].time_stamp)
                return index;
        }
        assert(false);
        std::unreachable();
    }

    u32 get_rotation_index(float animation_time)
    {
        for (int index = 0; index < num_rotations - 1; ++index)
        {
            if (animation_time < rotations[index + 1].time_stamp)
                return index;
        }
        assert(false);
        std::unreachable();
    }

    glm::mat4 interpolate_position(float animation_time)
    {
        if (num_positions == 1)
            return glm::translate(glm::mat4(1.0f), positions[0].position);

        auto const p0_index = get_position_index(animation_time);
        auto const p1_index = p0_index + 1;
        glm::vec3 const final_position =
            glm::mix(positions[p0_index].position, positions[p1_index].position,
                     get_scale_factor(positions[p0_index].time_stamp, positions[p1_index].time_stamp, animation_time));

        return glm::translate(glm::mat4(1.0f), final_position);
    }

    glm::mat4 interpolate_rotation(float animation_time)
    {
        if (num_rotations == 1)
        {
            auto const rotation = glm::normalize(rotations[0].orientation);
            return glm::toMat4(rotation);
        }

        auto const p0_index = get_rotation_index(animation_time);
        auto const p1_index = p0_index + 1;
        glm::quat final_rotation =
            glm::slerp(rotations[p0_index].orientation, rotations[p1_index].orientation,
                       get_scale_factor(rotations[p0_index].time_stamp, rotations[p1_index].time_stamp, animation_time));
        final_rotation = glm::normalize(final_rotation);
        return glm::toMat4(final_rotation);
    }

    float get_scale_factor(float last_time_stamp, float next_time_stamp, float animation_time)
    {
        float scaleFactor = 0.0f;
        float const mid_way_length = animation_time - last_time_stamp;
        float const frames_diff = next_time_stamp - last_time_stamp;
        scaleFactor = mid_way_length / frames_diff;
        return scaleFactor;
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
    float current_time = 0.0f;
    glm::vec3 cached_root_offset = glm::vec3(0.0f);
    glm::vec3 wrap_offset = glm::vec3(0.0f);
    bool wrap_extracted_motion = false;
    std::vector<Bone> bones = {};
    AssimpNodeData root_node = {};
    std::map<std::string, BoneInfo> bone_info_map = {};
};
