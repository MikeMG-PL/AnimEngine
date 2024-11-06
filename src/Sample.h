#pragma once
#include "AK/Types.h"
#include "glm/vec3.hpp"

// "Sample" is a sample in context of motion matching.

struct Feature
{
    glm::vec3 position = {};
    glm::vec3 facing_direction = {};
};

struct Sample
{
    Feature past_features[3] = {};
    Feature future_features[3] = {};
    Feature current_feature = {};
    u32 clip_id = 0;
    float clip_local_time = 0.0f;
};
