#Copyright (c) 2015 Benjamin Pylko

# Jeff Gibbons; 22mar2021 --
# This Makefile should be put into the directory underneath the
# root directory of a project, at the same directory level as
# the xxx/src/ directory, and with a symbolic filesystem link
# to the cpp directory where the Robot.cpp file is.  For example:
# /home/team4918/Desktop/Roborio2021/FRCMakeProject/Makefile
# This Makefile will build and download an FRC Robotics program
# to a Roborio, without using any graphics IDE or any proprietary
# code or settings from VSCode or Gradle.
#
# Use "make"            to build the project,
#     "make clean"      to remove the files built during the build,
#     "make edit"       to edit the C++ sourcecode files.
#     "make deploy"     to download the program to the Roborio, and
#     "make kill-robot" to stop the program on the Roborio.
# 
# I adapted this Makefile from the 2015 version by Benjamin Pylko,
# and adjusted for the new VSCode/Gradle build environment.
# I found most of the compiler/linker settings in:
#### OLD: /home/team4918/Roboctopi/testc/testc/build/tmp/
#### OLD:  compileFrcUserProgramExecutableFrcUserProgramCpp/options.txt
#### OLD:  linkFrcUserProgramExecutable/options.txt
# /home/team4918/Desktop/Roborio2024/build/tmp/
#  compileFrcUserProgramReleaseExecutableFrcUserProgramCpp/options.txt
# and in:
# /home/team4918/Desktop/Roborio2024/build/tmp/
#  linkFrcUserProgramReleaseExecutable/options.txt
# and in 2021 and 2022, in these two files:
# the "allLibHeaders" definition in ../build/vscodeconfig.json:
# ../build/tmp/linkFrcUserProgramReleaseExecutable/options.txt
# (trying to use the libraries from DebugExecutables/options.txt
#  resulted in programs which didn't run in the Roborio because
#  they had missing libraries there).

#This program is free software: you can redistribute it and/or modify
#it under the terms of the GNU General Public License as published by
#the Free Software Foundation, either version 3 of the License, or
#(at your option) any later version.

#This program is distributed in the hope that it will be useful,
#but WITHOUT ANY WARRANTY; without even the implied warranty of
#MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#GNU General Public License for more details.

#You should have received a copy of the GNU General Public License
#along with this program.  If not, see <http://www.gnu.org/licenses/>.

# jag; 12jan2023: build faster on multi-cpu machines:
#MAKEFLAGS += -j 2
MAKEFLAGS += -j

# Comment this line out to get a small executable, with no debugging info:
## DEBUG= YES

AT = @
INCDIR  = -I../src/main/include

INCDIR += -I/home/team4918/wpilib/2024/roborio/arm-nilrt-linux-gnueabi/sysroot/usr/include/c++/12

# The rest of these INCDIRs came from the "allLibHeaders" definition in
# the  "buildType": "release" section of ../build/vscodeconfig.json:
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/7165392b19accac70407ca2d99987820/transformed/wpilibNewCommands-cpp-2024.3.2-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/cb15d96176868be90f9d434752694508/transformed/REVLib-cpp-2024.2.0-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/40e147915a4d1c50761375fe4c935161/transformed/REVLib-driver-2024.2.0-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/82e706d647e3ec77e68a1ff04b4e9764/transformed/wpiapi-cpp-5.33.0-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/99839636be4f210ff45767dae4c11ac4/transformed/api-cpp-5.33.0-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/40d06b99bf432c53fd131951f212ca0c/transformed/cci-5.33.0-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/fdac311019f3873f6cf9f34474a0ab82/transformed/wpiapi-cpp-24.1.0-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/8fc3321cbbc7fd89a1d10664b0963494/transformed/tools-24.1.0-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/1246b9f7fd509e8351f9131d62e93366/transformed/wpilibc-cpp-2024.3.2-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/f246118f30887941db0fdc5a4491a816/transformed/ntcore-cpp-2024.3.2-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/5f132877f9382d8e3fed05364da04d20/transformed/hal-cpp-2024.3.2-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/256a0df102fc4e987ac2eed827b625df/transformed/wpimath-cpp-2024.3.2-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/1ac89878ab63a895e8dec9552976c214/transformed/wpinet-cpp-2024.3.2-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/a7bd31a0e738292fb67237168744afa9/transformed/wpiutil-cpp-2024.3.2-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/df12404738d68c4b0849727f2bdde023/transformed/netcomm-2024.2.1-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/e2285a26d7abab37bc3879310b668db9/transformed/chipobject-2024.2.1-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/8a75f965af77f951bca5dd657aff871a/transformed/visa-2024.2.1-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/89bca649c2287ce56c9b8bab38fd2550/transformed/cameraserver-cpp-2024.3.2-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/e786e36a132d400523fe2f917723a5f6/transformed/cscore-cpp-2024.3.2-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/1d85d9447ea39a8b09f5375dd973005e/transformed/apriltag-cpp-2024.3.2-headers
INCDIR += -I/home/team4918/.gradle/caches/transforms-3/e7fafbc3e371bd18dcbfbe20a56b8be3/transformed/opencv-cpp-4.8.0-2-headers

INCDIR += -I/home/team4918/Desktop/Roborio2024/src/main/include

## jag; 21mar2024: added new include directives from options.txt
INCDIR += -isystem /home/team4918/wpilib/2024/roborio/arm-nilrt-linux-gnueabi/sysroot/usr/lib/gcc/arm-nilrt-linux-gnueabi/12/include
INCDIR += -isystem /home/team4918/wpilib/2024/roborio/arm-nilrt-linux-gnueabi/sysroot/usr/lib/gcc/arm-nilrt-linux-gnueabi/12/include-fixed
INCDIR += -isystem /home/team4918/wpilib/2024/roborio/arm-nilrt-linux-gnueabi/sysroot/usr/include/c++/12
INCDIR += -isystem /home/team4918/wpilib/2024/roborio/arm-nilrt-linux-gnueabi/sysroot/usr/include/c++/12/arm-nilrt-linux-gnueabi
INCDIR += -isystem /home/team4918/wpilib/2024/roborio/arm-nilrt-linux-gnueabi/sysroot/usr/include/c++/12/backward
INCDIR += -isystem /home/team4918/wpilib/2024/roborio/arm-nilrt-linux-gnueabi/sysroot/usr/include


## jag; 12jan2023: See the definitions in vscodeconfig.json:
CPPFLAGS += -x c++ -c
CPPFLAGS += -Wall -Wextra
CPPFLAGS += -Wno-deprecated-enum-enum-conversion
CPPFLAGS += $(INCDIR)
CPPFLAGS += -std=c++20 -Wformat=2 -pedantic -Wno-psabi
CPPFLAGS += -Wno-unused-parameter
## jag; 21mar2024: added new flag from options.txt
CPPFLAGS += -nostdinc

# jag; I added the next two lines myself, to eliminate a ton
# #    of warnings from the Phoenix5/6 libraries
CPPFLAGS += -Wno-deprecated-declarations
CPPFLAGS += -Wno-error=deprecated-declarations

CPPFLAGS += -Wno-error=deprecated-enum-enum-conversion
CPPFLAGS += -fPIC -pthread
##CPPFLAGS += -Og
CPPFLAGS += -O2
ifeq ($(DEBUG),YES)
  CPPFLAGS += -g
endif

## This is the C++ compiler used in 2023 (g++ version 12.1.0)
CXX = /home/team4918/wpilib/2024/roborio/bin/arm-frc2024-linux-gnueabi-g++

TEAM = 4918

# These LFLAGS came from 
# ../build/tmp/linkFrcUserProgramReleaseExecutable/options.txt
# Be careful not to get the debug versions in the
# linkFrcUserProgramDebugExecutable directory; if you do, the program
# will # build here but will fail to run on the Roborio, because
# it won't be able to find the libraries (the debug libraries aren't
# on the Roborio).

## LFLAGS += -L/usr/local/frc/third-party/lib

LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/68279316f039aa32db741744c82e5255/transformed/wpilibNewCommands-cpp-2024.3.2-linuxathena/linux/athena/shared
LFLAGS += -lwpilibNewCommands
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/a6eb2644a7115c6195cb7a4b6cc8563c/transformed/REVLib-cpp-2024.2.0-linuxathenastatic/linux/athena/static
LFLAGS += -l:libREVLib.a
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/2096962cb34bba98691055ccb1e9ed44/transformed/REVLib-driver-2024.2.0-linuxathenastatic/linux/athena/static
LFLAGS += -l:libREVLibDriver.a
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/6a26ab639249911ed2a3a1a6f73ea038/transformed/wpiapi-cpp-5.33.0-linuxathena/linux/athena/shared
LFLAGS += -lCTRE_Phoenix_WPI
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/2be3b9e73b50a9dcd303cb0b396837c8/transformed/api-cpp-5.33.0-linuxathena/linux/athena/shared
LFLAGS += -lCTRE_Phoenix
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/885762c350cbe1abebbdf090a0bb9145/transformed/cci-5.33.0-linuxathena/linux/athena/shared
LFLAGS += -lCTRE_PhoenixCCI
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/974c396ef9016ceaf756a960825e1e9b/transformed/wpiapi-cpp-24.1.0-linuxathena/linux/athena/shared
LFLAGS += -lCTRE_Phoenix6_WPI
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/d535c1f5cbc638d7de4abe8dee69cb04/transformed/tools-24.1.0-linuxathena/linux/athena/shared
LFLAGS += -lCTRE_PhoenixTools

LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/4af82a1b4ad50289b0f47b54f01fb575/transformed/wpilibc-cpp-2024.3.2-linuxathena/linux/athena/shared
LFLAGS += -lwpilibc
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/7c3bc9723d91eee20add62c053e539a2/transformed/ntcore-cpp-2024.3.2-linuxathena/linux/athena/shared
LFLAGS += -lntcore
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/f6083895b447d0e69fd5fce592ffd847/transformed/hal-cpp-2024.3.2-linuxathena/linux/athena/shared
LFLAGS += -lwpiHal
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/14ef87392e574ab5f0cc2d4f64705eb4/transformed/wpimath-cpp-2024.3.2-linuxathena/linux/athena/shared
LFLAGS += -lwpimath
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/db93050b651a3c8dec3734ff79bf45b6/transformed/wpinet-cpp-2024.3.2-linuxathena/linux/athena/shared
LFLAGS += -lwpinet
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/ce7a2346ed7230487c08e2b345df8d87/transformed/wpiutil-cpp-2024.3.2-linuxathena/linux/athena/shared
LFLAGS += -lwpiutil
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/ecfba5e2092df256a1ed9965a7e64690/transformed/netcomm-2024.2.1-linuxathena/linux/athena/shared
LFLAGS += -l:libFRC_NetworkCommunication.so.24.0.0
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/86a888182ed794b6ea50f3bc1f8bccba/transformed/chipobject-2024.2.1-linuxathena/linux/athena/shared
LFLAGS += -l:libRoboRIO_FRC_ChipObject.so.24.0.0
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/2eae536710917e0398157afe9b3b0aab/transformed/visa-2024.2.1-linuxathena/linux/athena/shared
LFLAGS += -l:libvisa.so.23.3.0
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/2a66f661e785372a799ffd7745826f45/transformed/runtime-2024.2.1-linuxathena/linux/athena/shared
LFLAGS += -lfpgalvshim
LFLAGS += -lembcanshim
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/0a0a39f950d33fba83ca84392bd04aed/transformed/cameraserver-cpp-2024.3.2-linuxathena/linux/athena/shared
LFLAGS += -lcameraserver
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/e763ab10cda2079a2d21329e4bd87de1/transformed/cscore-cpp-2024.3.2-linuxathena/linux/athena/shared
LFLAGS += -lcscore
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/0716c1735aed8c87ad8c47f9173afc34/transformed/apriltag-cpp-2024.3.2-linuxathena/linux/athena/shared
LFLAGS += -lapriltag
LFLAGS += -L/home/team4918/.gradle/caches/transforms-3/a793a41de9d3d272beade27703414ef1/transformed/opencv-cpp-4.8.0-2-linuxathena/linux/athena/shared
LFLAGS += -l:libopencv_photo.so.4.8
LFLAGS += -l:libopencv_video.so.4.8
LFLAGS += -l:libopencv_core.so.4.8
LFLAGS += -l:libopencv_features2d.so.4.8
LFLAGS += -l:libopencv_imgproc.so.4.8
LFLAGS += -l:libopencv_highgui.so.4.8
LFLAGS += -l:libopencv_ml.so.4.8
LFLAGS += -l:libopencv_gapi.so.4.8
LFLAGS += -l:libopencv_stitching.so.4.8
LFLAGS += -l:libopencv_objdetect.so.4.8
LFLAGS += -l:libopencv_imgcodecs.so.4.8
LFLAGS += -l:libopencv_calib3d.so.4.8
LFLAGS += -l:libopencv_videoio.so.4.8
LFLAGS += -l:libopencv_aruco.so.4.8
LFLAGS += -l:libopencv_flann.so.4.8
LFLAGS += -l:libopencv_dnn.so.4.8

LFLAGS += -rdynamic
LFLAGS += -pthread
LFLAGS += -ldl
LFLAGS += -latomic

ifeq ($(DEBUG),YES)
  LFLAGS += -g
endif

RMCOMMAND = rm -f
##DEPLOYTARGET = roboRIO-$(TEAM)-FRC.local
## 10.49.18.2 is the Roborio's IP address when the laptop is connected to the Roborio's
## access point.  So the laptop must either have 2 separate WiFi interfaces, or it must
## disconnect from the AP it is on, so it can connect to the Roborio's 10.49.18.xxx AP.
DEPLOYTARGET = 10.49.18.2
## DEPLOYTARGETB is the IP address of the Roborio when connected with the USB
## cable, and when the programming laptop has had its wired USB network
## configured for "link-local", and enabled.
##DEPLOYTARGETB = 169.254.248.24   Woodbot
##DEPLOYTARGETB = 172.22.11.2      E-Z P-Z
DEPLOYTARGETB = 169.254.248.24

SOURCES = $(wildcard cpp/*.cpp)
HEADERS = $(wildcard cpp/*.hpp include/*.h)
OBJECTS = $(patsubst cpp/%.cpp,cpp/%.o,$(wildcard cpp/*.cpp))

all build:	bin/frcUserProgram


bin/frcUserProgram:	$(OBJECTS)
	@echo "Linking The Program..."
	@##@echo "SOURCES = $(SOURCES)."
	@##@echo "HEADERS = $(HEADERS)."
	@##echo "OBJECTS = $(OBJECTS)."
	@test -d cpp/ || ln -s ../src/main/cpp
	@test -d bin/ || mkdir -p bin/
	@$(CXX) $(OBJECTS) $(LFLAGS) -o bin/frcUserProgram
	@# Uncomment the following "rm cpp/Robot.o" line to force a recompile/rebuild every time
	@# make is run.
	@#rm cpp/Robot.o
	@echo "Done."

%.o:	%.cpp $(HEADERS)
	@echo "Compiling Sourcecode file $< ..."
	@##@echo "SOURCES = $(SOURCES)."
	@##@echo "HEADERS = $(HEADERS)."
	@##echo "OBJECTS = $(OBJECTS)."
	@test -d cpp/ || ln -s ../src/main/cpp
	@test -d bin/ || mkdir -p bin/
	@$(CXX) $(CPPFLAGS) $< -o $@

verbose:	
	$(CXX) $(CPPFLAGS) cpp/Robot.cpp -o Robot.o
	$(CXX) $(OBJECTS) $(LFLAGS) -o bin/frcUserProgram


#
	@# ctags allows an editor to go immediately to a function
	@# or object, without knowing what file it is in.  For example:
	@# :tag IterativeRobot
ctags:	
	ctags cpp/*.cpp ../build/tmp/expandedArchives/*sources*/*cpp \
        ../build/tmp/expandedArchives/*sources*/*/*cpp   \
        ../build/tmp/expandedArchives/*sources*/*/*/*cpp

clean:
	for dir in cpp; do find $$dir/ -name \*.o -exec $(RMCOMMAND) {} \; ; done
	$(RMCOMMAND) bin/*

deploy:		bin/frcUserProgram
	# Kill the old program on the Roborio
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET) ". /etc/profile.d/natinst-path.sh; \
					/usr/local/frc/bin/frcKillRobot.sh -t 2> /dev/null"
	# Remove the old program from the Roborio
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET) "rm /home/lvuser/frcUserProgram || true"
	# Should I copy the robotCommand script to the Roborio, with:?
	# ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET) "echo ' \"/home/lvuser/frcUserProgram\" ' > /home/lvuser/robotCommand"
	# ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET) "chmod +x /home/lvuser/robotCommand; chown lvuser /home/lvuser/robotCommand"
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET) "sync"
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET) "sleep 1"
	# copy the new program to the Roborio
	# ( see https://wpilib.screenstepslive.com/s/currentCS/m/cpp/l/
	#   145320-building-and-downloading-a-robot-project-to-the-roborio )
	# I could use scp for copying these files, instead of sftp
	sftp -o StrictHostKeyChecking=no -oBatchMode=no -b sftpbatchfile lvuser$(AT)$(DEPLOYTARGET)
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET) 'chmod +x "/home/lvuser/frcUserProgram"; chown lvuser "/home/lvuser/frcUserProgram"'
	echo "Logging in as admin user for setcap"
	ssh -o StrictHostKeyChecking=no admin$(AT)$(DEPLOYTARGET) 'setcap cap_sys_nice+eip "/home/lvuser/frcUserProgram"'
	# Delay a bit, and make sure the file is there and settled...
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET) "sync"
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET) "sleep 1"
	# kill the existing program (if it is running), make sure the new
	# copy is executable, then restart it
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET) ". /etc/profile.d/natinst-path.sh; \
		 chmod a+x    /home/lvuser/robotCommand /home/lvuser/frcUserProgram ; \
		 chown lvuser /home/lvuser/robotCommand /home/lvuser/frcUserProgram ; \
		  /usr/local/frc/bin/frcKillRobot.sh -t -r 2> /dev/null"


deployusb:		bin/frcUserProgram
	# Kill the old program on the Roborio
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGETB) ". /etc/profile.d/natinst-path.sh; \
					/usr/local/frc/bin/frcKillRobot.sh -t 2> /dev/null"
	# Remove the old program from the Roborio
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGETB) "rm /home/lvuser/frcUserProgram || true"
	# Should I copy the robotCommand script to the Roborio, with:?
	# ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGETB) "echo ' \"/home/lvuser/frcUserProgram\" ' > /home/lvuser/robotCommand"
	# ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGETB) "chmod +x /home/lvuser/robotCommand; chown lvuser /home/lvuser/robotCommand"
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGETB) "sync"
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGETB) "sleep 1"
	# copy the new program to the Roborio
	# ( see https://wpilib.screenstepslive.com/s/currentCS/m/cpp/l/
	#   145320-building-and-downloading-a-robot-project-to-the-roborio )
	# I could use scp for copying these files, instead of sftp
	sftp -o StrictHostKeyChecking=no -oBatchMode=no -b sftpbatchfile lvuser$(AT)$(DEPLOYTARGETB)
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGETB) 'chmod +x "/home/lvuser/frcUserProgram"; chown lvuser "/home/lvuser/frcUserProgram"'
	echo "Logging in as admin user for setcap"
	ssh -o StrictHostKeyChecking=no admin$(AT)$(DEPLOYTARGETB) 'setcap cap_sys_nice+eip "/home/lvuser/frcUserProgram"'
	# Delay a bit, and make sure the file is there and settled...
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGETB) "sync"
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGETB) "sleep 1"
	# kill the existing program (if it is running), make sure the new
	# copy is executable, then restart it
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGETB) ". /etc/profile.d/natinst-path.sh; \
		 chmod a+x    /home/lvuser/robotCommand /home/lvuser/frcUserProgram ; \
		 chown lvuser /home/lvuser/robotCommand /home/lvuser/frcUserProgram ; \
		  /usr/local/frc/bin/frcKillRobot.sh -t -r 2> /dev/null"

login:
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET)

loginusb:
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGETB)

log:
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET)  tail -f /var/local/natinst/log/FRC_UserProgram.log

tiny login_to_tiny login_to_host logintiny loginhost:
	ssh -o StrictHostKeyChecking=no jgibbons$(AT)10.0.2.2

kill-robot:
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET) "/usr/local/frc/bin/frcKillRobot.sh"

get-roborio-files:	
	scp -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET):/usr/local/frc/bin/frcKillRobot.sh .
	scp -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET):/etc/profile.d/natinst-path.sh .

seelog seelogperiodic see-roborio-log:	
	@echo Hit ctrl-C to end this log display.
	@#ssh -o StrictHostKeyChecking=no team4918@192.168.100.245 tail -f RoborioLogTeleop2024.txt
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET) tail -f /home/lvuser/RoborioLogTeleop2024.txt

seelogauto see-roborio-logauto:	
	@echo Hit ctrl-C to end this log display.
	@#ssh -o StrictHostKeyChecking=no team4918@192.168.100.245 tail -f RoborioLogAUTOnomous2024.txt
	ssh -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET) tail -f /home/lvuser/RoborioLogAUTOnomous2024.txt

getlog getlogs get-roborio-log:	
	@#scp -o StrictHostKeyChecking=no team4918$(AT)192.168.100.245:/home/team4918/RoborioLog*.txt .
	@#scp -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET):/home/lvuser/RoborioLogTeleop2024.txt .
	@#scp -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET):/home/lvuser/RoborioLogAUTOnomous2024.txt .
	scp -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET):/var/local/natinst/log/FRC_UserProgram.log .
	scp -o StrictHostKeyChecking=no lvuser$(AT)$(DEPLOYTARGET):/home/lvuser/RoborioLog*.txt .

edit:	cpp/Robot.cpp
	#vi cpp/Robot.cpp
	gvim cpp/Robot.cpp

gitstatusshort:	
	cd cpp; git status -s; cd -

gitstatus:	
	cd cpp; git status; cd -

gitdiffbasic:	
	cd cpp; git diff Robot.cpp; cd -

gitdiff gitdifftool:	
	cd cpp; git difftool -t tkdiff Robot.cpp; cd -

gitup:	Robot.cpp
	cd cpp; git add Robot.cpp; cd -
	cd cpp; git commit; cd -
	cd cpp; git push origin master; cd -

gitadd:	Robot.cpp
	cd cpp; git add Robot.cpp; cd -

gitcommit:	
	cd cpp; git commit; cd -

gitpush:	
	cd cpp; git push origin master; cd -

gitpull:	
	cd cpp; git pull; cd -

# See "man date" for details about this next command:
SAVEDIR := $(shell date +%Y%m%d_%H%M%S)

save:	
	mkdir archive/${SAVEDIR}/
	cp -p cpp/Robot.cpp           archive/${SAVEDIR}/
	cp -p cpp/Drivetrain.cpp      archive/${SAVEDIR}/
	cp -p cpp/SwerveModule.cpp    archive/${SAVEDIR}/
	cp -p include/Drivetrain.h    archive/${SAVEDIR}/
	cp -p include/SwerveModule.h  archive/${SAVEDIR}/
	@echo
	@echo Sourcecode has been saved into archive/${SAVEDIR}/

##clean:
##	## rm -f $(ROBOTS) $(LIBROBOTS_OBJS) robots.a robotserver $(MAIN_OBJ)

