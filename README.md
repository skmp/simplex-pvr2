# polly2-rtl
![daytona-broken](https://drk.emudev.org/images/polly2-rtl.jpg)

polly2-rtl is a reimplementation for PowerVR2 / CLX2 CORE in verilog, targeting the de10-nano (mister) board.

It is largely based on / ported from refsw2, and the project is currently in a very exploratory phase to figure out constraints and architecture.

This will not result to a full Dreamcast core for MiSTer, as this sub part of the gpu takes over most of the FPGA. A software emulator will have to feed polly2-rtl, in a hybrid SW/FPGA setup.

Even when optimized, this will be cut down compared to the real hardware to make things fit.

This will never run fullspeed on DE10-nano / MiSTer, as the software emulator can't get fullspeed on its own. Best estimate is something like 15-30 fps when fully optimized. Performance will vary wildly per game.

This repository is just the CORE part, without a mister core wrapper.

Many thanks to ElectronAsh and Corn!

### Videos from pc simulator
[![Daytona](https://img.youtube.com/vi/Mt4AAAL1QkU/0.jpg)](https://www.youtube.com/watch?v=Mt4AAAL1QkU)

### Videos from de10-nano
[![Jet Set Radio](https://img.youtube.com/vi/asii_UoAHI0/0.jpg)](https://www.youtube.com/watch?v=asii_UoAHI0)

[![Sonic Adventure](https://img.youtube.com/vi/PMoPsqOetqg/0.jpg)](https://www.youtube.com/watch?v=PMoPsqOetqg)

Some more details can be found in [my blog](https://drk.emudev.org/blog/hello-polly2-rtl).