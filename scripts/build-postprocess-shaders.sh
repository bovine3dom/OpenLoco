#!/usr/bin/env bash

set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
source_dir="$repo_root/data/shaders/src"
output_dir="$repo_root/data/shaders"
dxc="${DXC:-dxc}"
spirv_cross="${SPIRV_CROSS:-spirv-cross}"

for shader in fxaa smaa_edges smaa_weights smaa_blend; do
    source="$source_dir/$shader.frag.hlsl"

    "$dxc" -T ps_6_0 -E main -O3 -spirv -fspv-target-env=vulkan1.0 \
        -I "$source_dir" -Fo "$output_dir/$shader.frag.spv" "$source"
    "$dxc" -T ps_6_0 -E main -O3 \
        -I "$source_dir" -Fo "$output_dir/$shader.frag.dxil" "$source"
    "$spirv_cross" "$output_dir/$shader.frag.spv" --msl --msl-version 20100 \
        --output "$output_dir/$shader.frag.msl"

    if command -v spirv-val >/dev/null 2>&1; then
        spirv-val --target-env vulkan1.0 "$output_dir/$shader.frag.spv"
    fi
done
