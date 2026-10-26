# TI IWR1843Boost vital signs detection and estimation demo

## About

Heartrate and breathing rate detection and estimation signal processing chain realised completely on the IWR1843Boost evaluation board. The demo will not run on 16xx/68xx boards out of the box.

***

## System requirements

In order to run the demo the following tools need to be installed on your system:

1. mmWave SDK version 03.06.02.00-LTS

2. TI ARM CGT version 16.9.6.LTS

3. TI C6000 CGT version 8.3.3

4. XDCtools version 3.50.08.24-core

5. SYS/BIOS version 6.73.01.01

6. DSPLIB C674X version 3.4.0.0

7. DSPLIB C64P+ version 3.4.0.0

8. MATHLIB C674x version 3.1.2.1

9. TI UniFlash

To execute the `send_config.py` and `vital_signs_interface.py` you also need `python3` installed. To execute `vital_signs_interface.py`, `tkinter` is also necessary.

***

## Point the environment at the SDK

**setenv.sh** must be sourced from its own directory (it calls ./setenv.sh by relative path):

```
cd <your_install_path>/packages/scripts/unix
source ./setenv.sh
```

Then select a device:

```
export MMWAVE_SDK_DEVICE=iwr18xx
```

***

## Build

```
cd <your_repo>
make clean
make mmwDemo
```

`mmwDemo` builds MSS(Cortex-R4F) and DSS(C674x) images and then runs the metaimage generator, producing the flashable `xwr18xx_mmw_demo.bin`

***

## Flash

1. Power down the EVM

2. Set the SOP jumpers to **flashing mode**

3. Power up and connect USB

4. In UniFlash, select **IWR1843BOOST**, enter your serial connection on the **CLI port** at 115200 and load `xwr18xx_mmw_demo.bin` into the **Meta image** slot

5. Flash

6. Power down the EVM, return the SOP jumpers to **functional mode**, power up the board

***

## Run the project

In the project folder

```
python3 send_config.py profiles/profile_2d.cfg
python3 vital_signs_interface.py
```

`send_config.py` takes an optional second argument for the CLI port. The GUI currently hard-codes `/dev/ttyACM1` - change it for a different port or for Windows `COMx`

***

## Regarding the measurements

To see chirp/frame specifications, go to profiles/profile_2d.cfg and refer to the **MMWAVE SDK User Guide** as a reference.

1. The search for the breathing and heartbeat amplitude is limited from bin 3 to bin 8 - approximately 0.5-1.4m. Tests discovered that **~0.7m is the optimal distance**

2. The test subject should be in a still position and **not move closer/farther from the radar** 

3. The filters in the signal processing chain need time to settle to produce adequate output values and the FFT buffer needs ~26 seconds to be filled with values. The table below shows the stages, frames and approximate time.

|**Stage**           |**Frames**   |**Time**  |
|---                 |---          |---       |
|Filter warm-up      |300          |30s       |
|Range-bin selection |100          |10s       |
|FFT window filling  |256          |25.6s     |
|**Total**           |**656**      |**65.6s** |

***