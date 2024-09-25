#pragma once

#include "AK/Math.h"
#include "SkinnedModel.h"
#include "assimp/Importer.hpp"
#include "assimp/anim.h"
#include "assimp/postprocess.h"
#include "assimp/scene.h"
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

    void init(std::string const& animationPath, SkinnedModel const* model)
    {
        Assimp::Importer importer;
        aiScene const* scene = importer.ReadFile(animationPath, aiProcess_Triangulate);
        assert(scene && scene->mRootNode);
        auto const animation = scene->mAnimations[0];
        duration = animation->mDuration;
        ticks_per_second = animation->mTicksPerSecond;
        read_hierarchy_data(root_node, scene->mRootNode);
        read_missing_bones(animation, *model);
    }

    void read_hierarchy_data(AssimpNodeData& dest, aiNode const* src)
    {
        assert(src);

        dest.name = src->mName.data;
        dest.transformation = AK::Math::ai_matrix_to_glm(src->mTransformation);
        dest.children_count = src->mNumChildren;

        for (int i = 0; i < src->mNumChildren; i++)
        {
            AssimpNodeData newData;
            read_hierarchy_data(newData, src->mChildren[i]);
            dest.children.push_back(newData);
        }
    }

    void read_missing_bones(aiAnimation const* animation, SkinnedModel const& model)
    {
        u32 const size = animation->mNumChannels;

        std::map<std::string, BoneInfo> new_bone_info_map = model.get_bone_info_map(); //getting m_BoneInfoMap from Model class
        u32 bone_count = model.get_bone_count(); //getting the m_BoneCounter from Model class

        //reading channels(bones engaged in an animation and their keyframes)
        for (int i = 0; i < size; i++)
        {
            auto const channel = animation->mChannels[i];
            std::string boneName = channel->mNodeName.data;

            if (!new_bone_info_map.contains(boneName))
            {
                new_bone_info_map[boneName].id = bone_count;
                bone_count++;
            }

            Bone bone;
            bone.init(channel->mNodeName.data, new_bone_info_map[channel->mNodeName.data].id, channel);
            bones.push_back(bone);
        }

        bone_info_map = new_bone_info_map;
    }

    Bone* find_bone(std::string const& name)
    {
        auto const iter = std::ranges::find_if(bones, [&](Bone const& bone) { return bone.name == name; });
        if (iter == bones.end())
            return nullptr;

        return &(*iter);
    }
};
