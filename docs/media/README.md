# Assembly demonstration

[![Looping six-times-speed preview of five-brick Tier 3 assembly](workbenchmark-tier3-preview.gif)](workbenchmark-tier3-demo.mp4)

[Watch or download the MP4](https://raw.githubusercontent.com/ma-haha-hehe/robot_korea/main/docs/media/workbenchmark-tier3-demo.mp4)

| Item | Description |
|---|---|
| Task | Workbenchmark Tier 3, `simple_shape_assembly_056` |
| Structure | Five bricks |
| Controller | Oracle reference controller using simulated poses and contact forces |
| Environment | Panda arm in MuJoCo, physics mode with plastic contact |
| Preview | Looping GIF, 6× playback speed, approximately 15 seconds |
| Full video | H.264 MP4, 1920 × 1080, 24 fps, approximately 88 seconds at 1× simulation time |
| Camera views | Both views show the same recorded physical states |
| Outcome | All five bricks placed; release, support and final stability checks passed |

The preview is labeled as accelerated and links to the full video. The static
PNG remains available as a poster. The full clip includes a short introduction
and a final view of the completed structure. It shows one successful simulation
episode. It does not establish
success across the complete dataset, performance on real hardware or the
generalization of a vision-language-action model.

The video and preview are distributed with this repository under its
[Apache-2.0 license](../../LICENSE). The Panda model retains its upstream
license; see [third-party notices](../../THIRD_PARTY_NOTICES.md). LEGO is a
trademark of the LEGO Group, which does not sponsor, authorize or endorse this
research environment.
