cbuffer skinning_buffer : register(b4)
{
    float4x4 bones[128];
}

struct VS_Input
{
    float3 pos: POSITION;
    float3 normal : NORMAL;
    float2 UV : TEXCOORD0;
    int4 skin_indices : TEXCOORD1;
    float4 skin_weights : TEXCOORD2;
};

struct VS_Output
{
    float4 pixel_pos : SV_POSITION;
    float3 normal : NORMAL;
    float3 world_pos : POSITION;
    float2 UV : TEXCOORD;
};

struct PS_Output
{
    float4 position : SV_Target0;
    float4 normal : SV_Target1; // Alpha channel holds whether something is blinking or not
    float4 diffuse : SV_Target2;
};

cbuffer object_buffer : register(b0)
{
    float4x4 projection_view_model;
    float4x4 model;
    float4x4 projection_view;
};

cbuffer object_buffer : register(b10)
{
    float4x4 projection_view_model1;
    float4x4 model1;
    float4x4 projection_view1;
    bool is_glowing;
};

Texture2D obj_texture : register(t0);
SamplerState obj_sampler_state : register(s0);

VS_Output vs_main(VS_Input input)
{
    VS_Output output;

    float4 pos_skinned = float4(input.pos, 1.0f);
    /////////////
    // Check if there are bones influencing the vertex
    bool has_influences = false;
    for(int i = 0; i < 4; i++)
    {
        if(input.skin_indices[i] >= 0)
        {
            has_influences = true;
            break;
        }
    }

    if(has_influences)
    {   
        // Perform skinning calculations
        pos_skinned = float4(0.f,0.f,0.f,0.f);
        float4 norm_skinned = float4(0.f,0.f,0.f,0.f);

        float4 input_pos_4 = float4(input.pos, 1.0f);
        float4 input_norm_4 = float4(input.normal, 1.0f);
        
        
        for(int i = 0; i < 4; i++)
        {
            if(input.skin_indices[i] >= 0)
            {
                float4x4 bone = bones[input.skin_indices[i]]; // 128 bones
                float weight = input.skin_weights[i];
                pos_skinned += mul(mul(bone, input_pos_4), weight);            // posSkinned+=(bone*pos)*weight;
                norm_skinned += mul(mul(bone, input_norm_4), weight);           // norm_skinned+=(bone*norm)*weight;
            }
        }
        
        pos_skinned.w = 1.f;
    }
    
    output.world_pos = mul(model, pos_skinned);
    output.UV = input.UV;
    output.normal = normalize(mul(input.normal, (float3x3)model));
    output.pixel_pos = mul(projection_view_model, pos_skinned);
    return output;

}

PS_Output ps_main(VS_Output input)
{
    PS_Output output;
    output.diffuse = obj_texture.Sample(obj_sampler_state, input.UV);
    output.position.xyz = input.world_pos;
    output.normal.xyz = normalize(input.normal);
    output.normal.a = is_glowing ? 1.0f : -1.0f;
    output.position.a = 1.0f;
    
    return output;
}
