# Third-party components

The public benchmark contains the simulation package, primitive brick geometry,
product definitions and validation tools. Hardware SDKs and model weights are not
part of this distribution.

- The Franka Panda MJCF files under `src/mj_bridge/mj_bridge` retain their
  Apache-2.0 license.
- FoundationPose is an optional external dependency under NVIDIA's license,
  including a non-commercial research/evaluation limitation. Install its source
  and weights separately under the upstream terms; neither is bundled here.
- The IFM O3P SDK is excluded from the public benchmark.
- Two legacy LEGO/Duplo mesh files in the research workspace lack provenance
  metadata. Public episodes use self-authored box/cylinder primitives instead;
  `scripts/create_public_release.sh` excludes those legacy meshes.
- The release script also excludes model weights, camera captures, build outputs
  and real-robot logs.

The word LEGO is a trademark of the LEGO Group, which does not sponsor,
authorize, or endorse this research environment.
