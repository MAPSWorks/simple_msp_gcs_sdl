# simple_msp_gcs
simple Multiwii Serial Protocol gcs

This project is simple GCS for our custom drone.
receive video from drone via UDP and also send and receive MSP serial protocol via serial(like bluetooth module)

This project use OpenCV's computer vision, NanoGUI_SDL's GUI library.

To build this project, you should install below
* ffmpeg 2.8.13
* OpenCV 2.4.13.5
* SDL 2.0

## Install ffmpeg 2.8.13
[See this wiki]()

## Install OpenCV 2.4.13.5
[See this wiki]()

## Install SDL 2.0

```
sudo apt-get install libsdl2-2.0
sudo apt-get install libsdl2-dev
```

## Install NanoGUI's dependencies

```
apt-get install cmake xorg-dev libglu1-mesa-dev
```

## How to compile

### Clone this repo

Move to workspace you want and clone.
Also this repo you submodules from another repo please check submodules with submodule command

```
git clone https://github.com/whdlgp/simple_msp_gcs.git
cd simple_msp_gcs
git submodule update --init --recursive
```

### Compile GUI library

This project need .so file from nanogui-sdl.  
you can see the directory 'nanogui-sdl'.
move to this directory and build library
```
cd nanogui-sdl
cmake .
make
```
while compile this, maybe some errors appear. But after end of compile, you can see 'libnanogui.so' file

Also you can test the library. You can see the file named 'example1' and 'example2'. 
Just click these files. if you can see some GUI displays and buttons, It means library works

### compile main App

Now go to top directory of this project, 'simple_msp_gcs'.
and type this

```
cmake .
make
```

If you can see 'simple_msp_gcs' binary file. The building process done.

![](https://i.imgur.com/XR4FuMW.png)
