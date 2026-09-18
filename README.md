# ExHist Labs Simulation

[![ExHist Labs — looping 60-second lab promo](simulation/promo_v5/preview.gif)](simulation/promo_v5/ExHist-Labs-Promo-60s.mp4)

[Watch/download the full-quality video](simulation/promo_v5/ExHist-Labs-Promo-60s.mp4) · [Replay and QC details](simulation/promo_v5/README.md)

The preview loops through the full 60-second promo. If it is paused, check your
[GitHub animated-image settings](https://docs.github.com/en/account-and-profile/how-tos/account-settings/managing-accessibility-settings#managing-motion).

A CAD-based histology laboratory in MuJoCo, with LeHisto stations, Nori A3, incubators, staining/coverslipping, scanning, and slide send-out/QC.

**Research simulation / development snapshot.** The live demo validates a bounded LeHisto rack loop. The promo is staged kinematic playback, not a physically validated autonomous lab.

## Browse by purpose

| Looking for | Open |
| --- | --- |
| Install, run, and tour every station | [Full illustrated guide](simulation/README.md) |
| Latest 60-second video and portable playback | [Revision 5: close-ups and continuous departure](simulation/promo_v5/) |
| Promo instructions and limitations | [Promo guide](simulation/docs/PROMO.md) |
| CAD STEP/STP and STL files | [CAD folder](simulation/cad/) |
| Static asset XMLs | [Asset models](simulation/models/assets/) |
| Station and asset pictures | [Image gallery](simulation/docs/images/) |
| Original/pinned source assets | [Source folder](simulation/source/) |
| Simulator mesh and texture assets | [Runtime assets](simulation/assets/) |
| Validation and distribution notes | [Validation record](simulation/docs/DISTRIBUTION_VALIDATION.md) |
| Repository organization | [Folder map](docs/REPOSITORY_LAYOUT.md) |
| Licensing and attribution | [License notice](simulation/LICENSE-NOTICE.md) |

## Run

From the repository root, using the documented Windows/Conda setup:

```powershell
cd simulation
conda env create -f environment.yml
conda activate exhist-sim
python tools/unpack_assets.py
python tools/check_install.py
python lab_rail_loop.py
```

After setup, `./launch.ps1` from the repository root starts the live demo. The viewer starts paused; press Space to run. See the full guide for controls and the separate Nori equipment-access environment.

The simulation bundle stays under `simulation/`, with the latest promo, portable
scene/playback, looping preview and scoped evidence grouped in `simulation/promo_v5/`.
Revision 5 includes the left parallel/right stock-claw swap, upright rack
transfers, gripper close-ups, smooth camera, supplied scanner screen and logo,
and Nori continuing away through the ending. The previous v4 bundle remains
available as history; the separate contact-tested live LeHisto demo is unchanged.
