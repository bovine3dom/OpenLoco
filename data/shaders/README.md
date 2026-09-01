# Post-processing shaders

OpenLoco ships offline-compiled fragment shaders for the SDL GPU renderer:

- FXAA 3.11 quality mode;
- SMAA 1x high mode (colour edge detection, blend weights, and neighbourhood
  blending).

The checked-in SPIR-V, DXIL, and MSL files are generated from the HLSL sources
in `src/`. Set `DXC` and `SPIRV_CROSS` if those tools are not on `PATH`, then
run:

```sh
scripts/build-postprocess-shaders.sh
```

The SMAA area/search lookup data comes from the official SMAA repository.
The FXAA header is the variant distributed with Intel's CMAA2 sample.
Third-party license texts are in `licenses/`.
